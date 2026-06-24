#include "L1_Orchestration/foc_app.h"

#include <stdio.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L1_Orchestration/foc_service_handler.h"
#include "L1_Orchestration/foc_output_mgr.h"
#include "L3/foc_platform_api.h"
#include "L2/Control/foc_ctrl_executor.h"
#include "L2/Control/foc_ctrl_cfg.h"
#include "L3/foc_sensor.h"
#include "L3/foc_svpwm.h"
#include "L2/Protocol/foc_protocol_handler.h"
#include "L2/Runtime/foc_task_scheduler.h"
#include "L2/Runtime/foc_debug_stream.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 全局实例
 * ================================================================ */

static foc_system_t g_sys;

/* ================================================================
 * 内部工具
 * ================================================================ */

/* 调度器 tick 回调桥接（C 函数指针，无参数） */
static void FOC_App_SchedTickBridge(void)
{
    ControlScheduler_RunTick(&g_sys.runtime.scheduler);
}

static void FOC_App_UpdateIndicators(void)
{
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    if (g_sys.motor.state.system_fault != 0U)
    {
        led_run_on = 0U;
        led_fault_on = 1U;
        g_sys.runtime.indicator.led_run_on = 0U;
        g_sys.runtime.indicator.led_run_blink_counter = 0U;
    }
    else if (g_sys.motor.state.system_running != 0U)
    {
        if (g_sys.runtime.indicator.led_run_blink_counter >= (FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS - 1U))
        {
            g_sys.runtime.indicator.led_run_blink_counter = 0U;
            g_sys.runtime.indicator.led_run_on = (g_sys.runtime.indicator.led_run_on == 0U) ? 1U : 0U;
        }
        else
        {
            g_sys.runtime.indicator.led_run_blink_counter++;
        }
        led_run_on = g_sys.runtime.indicator.led_run_on;
    }
    else
    {
        led_run_on = 0U;
        g_sys.runtime.indicator.led_run_on = 0U;
        g_sys.runtime.indicator.led_run_blink_counter = 0U;
    }

    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_run_on);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, led_fault_on);

    if (g_sys.runtime.indicator.comm_pulse_counter > 0U)
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
        g_sys.runtime.indicator.comm_pulse_counter--;
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

    /* 初始化系统运行时 */
    g_sys.runtime.service_task_pending = 0U;
    g_sys.runtime.monitor_task_pending = 0U;
    g_sys.runtime.indicator.comm_pulse_counter = 0U;
    g_sys.runtime.indicator.led_run_on = 0U;
    g_sys.runtime.indicator.led_run_blink_counter = 0U;

    /* 调度器初始化 */
    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init(&g_sys.runtime.scheduler);
    FOC_Platform_SetControlTickCallback(FOC_App_SchedTickBridge);

    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_SERVICE, FOC_App_ServiceTrigger);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_FAST_CONTROL, FOC_App_ControlTrigger);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_MONITOR, FOC_App_MonitorTrigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    /* 通讯初始化 */
    FOC_Platform_CommInit();

    /* 输出管理器初始化 */
    FOC_OutputMgr_Init(&g_sys);

    /* 协议初始化（传入系统遥测策略地址） */
    FOC_Protocol_Init(&g_sys.cfg.telemetry);

    /* 调试流初始化 */
    DebugStream_Init(&g_sys.runtime.debug_stream);

    /* 传感器初始化（包含 snapshot 初始化） */
    Sensor_InitSnapshot(&g_sys.motor.sensor);
    Sensor_InitSnapshot(&g_sys.motor.sensor_fast);
    Sensor_Init(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    Sensor_SetZeroOffset(&g_sys.motor);
    Sensor_ReadAll(&g_sys.motor);

    /* PWM 初始化 */
    SVPWM_Init(&g_sys.motor, FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    /* 快速电流环初始化 */
    FOC_ControlExecutor_Init(&g_sys.motor);

    /* 电机硬件初始化（含控制初始化、标定等） */
    FOC_Service_InitMotor(&g_sys.motor);
    FOC_Control_ApplyConfig(&g_sys.motor);

    /* 初始化检查完整性校验 */
    FOC_Service_VerifyInitChecks(&g_sys.motor, &g_sys.motor.sensor);

    /* 启动信息输出（直写通道） */
    {
        char startup_info[160];
        snprintf(startup_info, sizeof(startup_info),
                 "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, set_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
                 (double)g_sys.motor.mech_angle_at_elec_zero_rad,
                 (int)g_sys.motor.direction,
                 (int)g_sys.motor.pole_pairs,
                 (double)g_sys.motor.vbus_voltage,
                 (double)g_sys.motor.set_voltage,
                 (double)(g_sys.motor.vbus_voltage > 0.0f ? g_sys.motor.set_voltage / g_sys.motor.vbus_voltage : 0.0f),
                 (double)g_sys.motor.sensor.vbus_voltage_filtered);
        FOC_OutputMgr_WriteDirect(&g_sys, startup_info);
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
    if (g_sys.runtime.monitor_task_pending != 0U)
    {
        g_sys.runtime.monitor_task_pending = 0U;

#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
        DebugStream_SetExecutionCycles(&g_sys.runtime.debug_stream,
                                       ControlScheduler_GetExecutionCycles(&g_sys.runtime.scheduler));
        DebugStream_Process(&g_sys.runtime.debug_stream,
                            &g_sys.motor.sensor, &g_sys.motor,
                            FOC_Protocol_GetTelemetry());
#endif
    }

    /* Service 任务 */
    if (g_sys.runtime.service_task_pending != 0U)
    {
        g_sys.runtime.service_task_pending = 0U;

        if (FOC_Service_Process(&g_sys.motor) != 0U)
        {
            g_sys.runtime.indicator.comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;
        }
    }

    /* 输出队列消费任务（主循环唯一阻塞点） */
    FOC_OutputMgr_FlushQueue(&g_sys);
}

/* ================================================================
 * 回调桥接
 * ================================================================ */

void FOC_App_ServiceTrigger(void)
{
    FOC_App_UpdateIndicators();
    g_sys.runtime.service_task_pending = 1U;
}

void FOC_App_MonitorTrigger(void)
{
    g_sys.runtime.monitor_task_pending = 1U;
}

void FOC_App_ControlTrigger(void)
{
    FOC_ControlExecutor_RunCycle(&g_sys.motor, FOC_CONTROL_DT_SEC);
}

void FOC_App_OnPwmUpdateISR(void)
{
    FOC_ControlExecutor_RunISR(&g_sys.motor);
}