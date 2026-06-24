#include "L1_Orchestration/foc_app.h"

#include <stdio.h>

#include "L3/foc_platform_api.h"
#include "L2/Control/foc_ctrl_executor.h"
#include "L2/Control/foc_ctrl_cfg.h"
#include "L3/foc_sensor.h"
#include "L3/foc_svpwm.h"
#include "L2/Protocol/foc_protocol_handler.h"
#include "L2/Runtime/foc_task_scheduler.h"
#include "L2/Runtime/foc_debug_stream.h"
#include "L1_Orchestration/foc_service_handler.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 全局实例
 * ================================================================ */

static foc_motor_t g_motor;

/* 系统级状态 */
static volatile uint8_t g_service_task_pending = 0U;
static volatile uint8_t g_monitor_task_pending = 0U;

/* 通讯脉冲计数器 */
static uint16_t g_led_comm_pulse_counter = 0U;

/* ================================================================
 * 内部工具
 * ================================================================ */

static void FOC_App_UpdateIndicators(void)
{
    static uint8_t s_led_run_on = 0U;
    static uint16_t s_led_run_blink_counter = 0U;
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    if (g_motor.state.system_fault != 0U)
    {
        led_run_on = 0U;
        led_fault_on = 1U;
        s_led_run_on = 0U;
        s_led_run_blink_counter = 0U;
    }
    else if (g_motor.state.system_running != 0U)
    {
        if (s_led_run_blink_counter >= (FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS - 1U))
        {
            s_led_run_blink_counter = 0U;
            s_led_run_on = (s_led_run_on == 0U) ? 1U : 0U;
        }
        else
        {
            s_led_run_blink_counter++;
        }
        led_run_on = s_led_run_on;
    }
    else
    {
        led_run_on = 0U;
        s_led_run_on = 0U;
        s_led_run_blink_counter = 0U;
    }

    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_run_on);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, led_fault_on);

    if (g_led_comm_pulse_counter > 0U)
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
        g_led_comm_pulse_counter--;
    }
    else
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 0U);
    }
}

/* ================================================================
 * 初始化
 * ================================================================ */

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, 1U);

    /* 调度器初始化 */
    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();
    FOC_Platform_SetControlTickCallback(ControlScheduler_RunTick);

    ControlScheduler_SetCallback(FOC_TASK_RATE_SERVICE, FOC_App_ServiceTrigger);
    ControlScheduler_SetCallback(FOC_TASK_RATE_FAST_CONTROL, FOC_App_ControlTrigger);
    ControlScheduler_SetCallback(FOC_TASK_RATE_MONITOR, FOC_App_MonitorTrigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    /* 通讯初始化 */
    FOC_Platform_CommInit();

    /* 协议初始化 */
    FOC_Protocol_Init();

    /* 调试流初始化 */
    DebugStream_Init();

    /* 传感器初始化（包含 snapshot 初始化） */
    Sensor_InitSnapshot(&g_motor.sensor);
    Sensor_InitSnapshot(&g_motor.sensor_fast);
    Sensor_Init(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    Sensor_SetZeroOffset(&g_motor);
    Sensor_ReadAll(&g_motor);

    /* PWM 初始化 */
    SVPWM_Init(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    /* 快速电流环初始化 */
    FOC_ControlExecutor_Init(&g_motor);

    /* 电机硬件初始化（含控制初始化、标定等） */
    FOC_Service_InitMotor(&g_motor);
    FOC_Control_ApplyConfig(&g_motor);

    /* 初始化检查完整性校验 */
    FOC_Service_VerifyInitChecks(&g_motor, &g_motor.sensor);

    /* 启动信息输出 */
    {
        char startup_info[160];
        snprintf(startup_info, sizeof(startup_info),
                 "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, set_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
                 (double)g_motor.mech_angle_at_elec_zero_rad,
                 (int)g_motor.direction,
                 (int)g_motor.pole_pairs,
                 (double)g_motor.vbus_voltage,
                 (double)g_motor.set_voltage,
                 (double)(g_motor.vbus_voltage > 0.0f ? g_motor.set_voltage / g_motor.vbus_voltage : 0.0f),
                 (double)g_motor.sensor.vbus_voltage_filtered);
        FOC_Platform_WriteDebugText(startup_info);
    }

    FOC_App_UpdateIndicators();
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlRuntimeInterrupts(1U);
}

/* ================================================================
 * 主循环（由 main() 持续调用）
 * ================================================================ */

void FOC_App_Loop(void)
{
    /* Monitor 任务：调试流输出 */
    if (g_monitor_task_pending != 0U)
    {
        g_monitor_task_pending = 0U;

#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
        DebugStream_SetExecutionCycles(ControlScheduler_GetExecutionCycles());
        DebugStream_Process(&g_motor.sensor, &g_motor,
                            FOC_Protocol_GetTelemetry());
#endif
    }

    /* Service 任务 */
    if (g_service_task_pending != 0U)
    {
        g_service_task_pending = 0U;

        if (FOC_Service_Process(&g_motor) != 0U)
        {
            g_led_comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;
        }
    }
}

/* ================================================================
 * 回调桥接
 * ================================================================ */

void FOC_App_ServiceTrigger(void)
{
    FOC_App_UpdateIndicators();
    g_service_task_pending = 1U;
}

void FOC_App_MonitorTrigger(void)
{
    g_monitor_task_pending = 1U;
}

void FOC_App_ControlTrigger(void)
{
    FOC_ControlExecutor_RunCycle(&g_motor, FOC_CONTROL_DT_SEC);
}

void FOC_App_OnPwmUpdateISR(void)
{
    FOC_ControlExecutor_RunISR(&g_motor);
}
