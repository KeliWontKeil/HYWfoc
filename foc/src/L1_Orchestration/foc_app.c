#include "L1_Orchestration/foc_app.h"

#include <stdio.h>
#include <math.h>

#include "L2/Control/foc_ctrl_entry.h"
#include "L2/Control/foc_ctrl_init.h"
#include "L2/Control/foc_ctrl_cfg.h"
#include "L2/Control/foc_sensor.h"
#include "L2/Control/foc_svpwm.h"
#include "L2/Protocol/foc_protocol_handler.h"
#include "L2/Runtime/foc_task_scheduler.h"
#include "L2/Runtime/foc_debug_stream.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define FOC_APP_COMM_FRAMES_PER_STEP 1U

static void Service_Task_Trigger(void);
static void Motor_Control_Loop(void);
static void Monitor_Task_Trigger(void);
static void FOC_App_OnPwmUpdateISR(void);
static void FOC_App_StopFastCurrentLoop(void);
static void FOC_App_ReInitMotor(void);
static void FOC_App_InitMotorHardware(void);
static void FOC_App_EnterSafeOutputState(uint8_t report_skip);

/* === 全局状态 === */
static foc_motor_t g_motor;
static sensor_data_t g_sensor_snapshot;
static sensor_data_t g_fast_current_sensor_snapshot;

/* 电流环快速路径（在PWM ISR中使用） */
static volatile uint8_t g_fast_current_loop_enabled = 0U;
static volatile uint8_t g_fast_current_loop_div_counter = 0U;
static volatile float g_fast_current_loop_iq_target = 0.0f;
static volatile float g_fast_current_loop_electrical_angle = 0.0f;

/* 系统级状态 */
static volatile uint8_t g_service_task_pending = 0U;
static volatile uint8_t g_monitor_task_pending = 0U;
static volatile uint8_t g_reinit_in_progress = 0U;
static uint16_t g_led_comm_pulse_counter = 0U;
static uint8_t g_prev_control_mode_check = 0xFFU;

/* ================================================================ */

static void FOC_App_StopFastCurrentLoop(void)
{
    g_fast_current_loop_enabled = 0U;
    g_fast_current_loop_iq_target = 0.0f;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_div_counter = 0U;
}

static void FOC_App_InitMotorHardware(void)
{
    FOC_MotorInit(&g_motor,
                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);

    /* 应用配置：PID 初始化在 FOC_MotorInit -> FOC_Control_Init / FOC_ControlConfigResetDefault 中完成 */
    FOC_Control_ApplyConfig(&g_motor);

    g_prev_control_mode_check = 0xFFU;
}

static void FOC_App_ReInitMotor(void)
{
    FOC_Platform_WriteDebugText("\r\n=== ReInitMotor started ===\r\n");

    FOC_App_StopFastCurrentLoop();
    FOC_Control_OpenLoopStep(&g_motor, 0.0f, 0.0f);
    SVPWM_ApplyDirectDuty(0U, 0.0f, 0.0f, 0.0f);

    g_reinit_in_progress = 1U;

    FOC_App_InitMotorHardware();

    g_reinit_in_progress = 0U;

    g_motor.state.reinit_pending = 0U;

    char info[120];
    snprintf(info, sizeof(info),
             "reinit done: mech_zero=%.4f rad, dir=%d, poles=%d, vbus=%.2fV\r\n",
             (double)g_motor.mech_angle_at_elec_zero_rad,
             (int)g_motor.direction,
             (int)g_motor.pole_pairs,
             (double)g_sensor_snapshot.vbus_voltage_filtered);
    FOC_Platform_WriteDebugText(info);
}

static void FOC_App_EnterSafeOutputState(uint8_t report_skip)
{
    FOC_App_StopFastCurrentLoop();
    FOC_Control_OpenLoopStep(&g_motor, 0.0f, 0.0f);
    SVPWM_ApplyDirectDuty(0U, 0.0f, 0.0f, 0.0f);

    if (report_skip != 0U)
    {
        g_motor.state.control_skip_count++;
    }
}

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

    ControlScheduler_SetCallback(FOC_TASK_RATE_SERVICE, Service_Task_Trigger);
    ControlScheduler_SetCallback(FOC_TASK_RATE_FAST_CONTROL, Motor_Control_Loop);
    ControlScheduler_SetCallback(FOC_TASK_RATE_MONITOR, Monitor_Task_Trigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    /* 通讯初始化 */
    FOC_Platform_CommInit();

    /* 协议初始化 */
    FOC_Protocol_Init();

    /* 调试流初始化 */
    DebugStream_Init();

    /* 传感器初始化 */
    Sensor_Init(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    Sensor_ReadAll();
    Sensor_CopyData(&g_sensor_snapshot);

    /* PWM 初始化 */
    SVPWM_Init(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    /* 电机硬件初始化（含控制初始化、标定等） */
    FOC_App_InitMotorHardware();
    FOC_Control_ApplyConfig(&g_motor);

    /* === 初始化 init_check 状态（必须放在 FOC_MotorInit 之后，因其会清零 state） === */
    g_motor.state.init_check_mask = RUNTIME_INIT_CHECK_COMMAND |
                                    RUNTIME_INIT_CHECK_COMM |
                                    RUNTIME_INIT_CHECK_PROTOCOL |
                                    RUNTIME_INIT_CHECK_DEBUG |
                                    RUNTIME_INIT_CHECK_PWM |
                                    RUNTIME_INIT_CHECK_MOTOR;

    if ((g_sensor_snapshot.adc_valid != 0U) && (g_sensor_snapshot.encoder_valid != 0U))
    {
        g_motor.state.init_check_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }
    else
    {
        g_motor.state.init_fail_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (g_sensor_snapshot.vbus_voltage_filtered > FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        g_motor.state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
    else
    {
        g_motor.state.init_fail_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
#else
    g_motor.state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
#endif

    /* 检查初始化是否全部通过 */
    {
        uint16_t missing = (uint16_t)(RUNTIME_INIT_CHECK_VBUS |
                           RUNTIME_INIT_CHECK_MOTOR |
                           RUNTIME_INIT_CHECK_PWM |
                           RUNTIME_INIT_CHECK_SENSOR |
                           RUNTIME_INIT_CHECK_DEBUG |
                           RUNTIME_INIT_CHECK_COMMAND |
                           RUNTIME_INIT_CHECK_PROTOCOL |
                           RUNTIME_INIT_CHECK_COMM) & (~g_motor.state.init_check_mask);

        if ((g_motor.state.init_check_mask != 0U) &&
            (g_motor.state.init_fail_mask == 0U) &&
            (missing == 0U))
        {
            g_motor.state.system_running = 1U;
            g_motor.state.system_fault = 0U;
            g_motor.state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
            FOC_Platform_WriteDebugText("init: all checks passed\r\n");
        }
        else
        {
            g_motor.state.system_running = 0U;
            g_motor.state.system_fault = 1U;
            g_motor.state.last_fault_code = (uint8_t)FOC_FAULT_INIT_FAILED;
            FOC_Platform_WriteDebugText("init: checks failed or missing\r\n");
        }
    }

    char startup_info[160];
    snprintf(startup_info, sizeof(startup_info),
             "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, set_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
             (double)g_motor.mech_angle_at_elec_zero_rad,
             (int)g_motor.direction,
             (int)g_motor.pole_pairs,
             (double)g_motor.vbus_voltage,
             (double)g_motor.set_voltage,
             (double)(g_motor.vbus_voltage > 0.0f ? g_motor.set_voltage / g_motor.vbus_voltage : 0.0f),
             (double)g_sensor_snapshot.vbus_voltage_filtered);
    FOC_Platform_WriteDebugText(startup_info);

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
        DebugStream_Process(&g_sensor_snapshot, &g_motor,
                            FOC_Protocol_GetTelemetry());
#endif
    }

    /* Service 任务：协议处理 + 系统命令 */
    if (g_service_task_pending != 0U)
    {
        g_service_task_pending = 0U;

        /* 重初始化处理 */
        if (g_motor.state.reinit_pending != 0U)
        {
            FOC_App_ReInitMotor();
            FOC_Control_ApplyConfig(&g_motor);
        }

        /* 协议处理：读取帧→解析→修改 motor */
        if (FOC_Protocol_Process(&g_motor, FOC_APP_COMM_FRAMES_PER_STEP) != 0U)
        {
            g_led_comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;
        }

        /* 配置脏检查：协议修改参数后应用 */
        if (g_motor.state.cfg_dirty != 0U)
        {
            FOC_Control_ApplyConfig(&g_motor);
            Sensor_ADCSampleTimeOffset(g_motor.cfg.sensor_sample_offset_percent);
            FOC_Protocol_Commit(&g_motor);
        }

        /* 系统命令处理（Y通道） */
        if (g_motor.state.pending_system_action != FOC_SYSACTION_NONE)
        {
            switch (g_motor.state.pending_system_action)
            {
            case FOC_SYSACTION_COGGING_START:
                FOC_Control_CoggingCalibRequestStart(&g_motor);
                break;
            case FOC_SYSACTION_COGGING_DUMP:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
                g_motor.cogging_calib_state.request_dump = 1U;
#endif
                break;
            case FOC_SYSACTION_COGGING_EXPORT:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
                g_motor.cogging_calib_state.request_export = 1U;
#endif
                break;
            default:
                break;
            }
            g_motor.state.pending_system_action = FOC_SYSACTION_NONE;
        }

        /* 齿槽标定数据导出 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        if (g_motor.cogging_calib_state.request_dump != 0U)
        {
            g_motor.cogging_calib_state.request_dump = 0U;
            FOC_Control_CoggingCalibDumpTable(&g_motor);
        }
        if (g_motor.cogging_calib_state.request_export != 0U)
        {
            g_motor.cogging_calib_state.request_export = 0U;
            FOC_Control_CoggingCalibExportTable(&g_motor);
        }
#endif
    }
}

/* ================================================================
 * PWM ISR：电流环 + SVPWM 插值
 * ================================================================ */

static void FOC_App_OnPwmUpdateISR(void)
{
    uint8_t divider;
    float current_loop_dt_sec;
    const sensor_data_t *current_sensor = 0;

    if (g_reinit_in_progress != 0U) return;
    if (g_fast_current_loop_enabled == 0U) return;

    SVPWM_InterpolationISR();

    divider = (FOC_CURRENT_LOOP_ISR_DIVIDER == 0U) ? 1U : (uint8_t)FOC_CURRENT_LOOP_ISR_DIVIDER;
    g_fast_current_loop_div_counter++;
    if (g_fast_current_loop_div_counter < divider) return;
    g_fast_current_loop_div_counter = 0U;

    g_motor.iq_target = g_fast_current_loop_iq_target;
    current_loop_dt_sec = (FOC_PWM_FREQ_KHZ == 0U) ? FOC_CONTROL_DT_SEC
                          : ((float)divider / ((float)FOC_PWM_FREQ_KHZ * 1000.0f));

    if (FOC_Control_CurrentLoopRequiresSample() != 0U)
    {
        Sensor_ReadCurrentOnly();
        Sensor_CopyData(&g_fast_current_sensor_snapshot);

        if (g_fast_current_sensor_snapshot.adc_valid == 0U) return;
        current_sensor = &g_fast_current_sensor_snapshot;
    }

    FOC_Control_CurrentLoop(&g_motor, current_sensor,
                            g_fast_current_loop_electrical_angle,
                            current_loop_dt_sec);
}

/* ================================================================
 * 调度器回调
 * ================================================================ */

static void Service_Task_Trigger(void)
{
    FOC_App_UpdateIndicators();
    g_service_task_pending = 1U;
}

static void Monitor_Task_Trigger(void)
{
    g_monitor_task_pending = 1U;
}

static void Motor_Control_Loop(void)
{
    if (g_reinit_in_progress != 0U) return;

    if (g_motor.state.system_fault != 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    Sensor_ReadAll();
    Sensor_CopyData(&g_sensor_snapshot);

    /* 传感器有效性检查 → 更新电机故障状态 */
    if ((g_sensor_snapshot.adc_valid != 0U) && (g_sensor_snapshot.encoder_valid != 0U))
    {
        g_motor.state.sensor_invalid_consecutive = 0U;
        if (g_motor.state.system_fault == 0U)
        {
            g_motor.state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
        }
    }
    else
    {
        g_motor.state.sensor_invalid_consecutive++;
        g_motor.state.control_skip_count++;

        if (g_sensor_snapshot.adc_valid == 0U)
        {
            g_motor.state.last_fault_code = (uint8_t)FOC_FAULT_SENSOR_ADC_INVALID;
        }
        else
        {
            g_motor.state.last_fault_code = (uint8_t)FOC_FAULT_SENSOR_ENCODER_INVALID;
        }

        if (g_motor.state.sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
        {
            if (g_motor.state.system_fault == 0U)
            {
                g_motor.state.system_fault = 1U;
                FOC_Platform_WriteDebugText("sensor invalid threshold reached\r\n");
            }
        }

        FOC_App_EnterSafeOutputState(0U);
        return;
    }

    /* 欠压保护 */
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (g_sensor_snapshot.vbus_voltage_filtered < FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        g_motor.state.last_fault_code = (uint8_t)FOC_FAULT_UNDERVOLTAGE;
        g_motor.state.system_fault = 1U;
        FOC_App_EnterSafeOutputState(0U);
        return;
    }
#endif

    if (g_motor.state.motor_enabled == 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    /* 检查控制模式变化（用于 PID 重置，foc_ctrl_entry.c 内部已处理） */
    if (g_motor.state.control_mode != g_prev_control_mode_check)
    {
        g_prev_control_mode_check = g_motor.state.control_mode;
    }

    /* 齿槽标定模式下特殊处理 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    if (FOC_Control_CoggingCalibIsBusy(&g_motor) != 0U)
    {
        g_motor.current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        g_motor.current_soft_switch_status.enabled = 0U;

        FOC_Control_CoggingCalibSampleStep(&g_motor, &g_sensor_snapshot, FOC_CONTROL_DT_SEC);

        g_fast_current_loop_iq_target = g_motor.iq_target;
        g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
        g_fast_current_loop_enabled = 1U;
    }
    else
#endif
    {
        /* 正常控制外环 */
        FOC_Control_Run(&g_motor, &g_sensor_snapshot, FOC_CONTROL_DT_SEC);

        /* 更新电流环路径变量 */
        g_fast_current_loop_iq_target = g_motor.iq_target;
        g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
        g_fast_current_loop_enabled = 1U;
    }
}