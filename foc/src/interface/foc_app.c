#include "interface/foc_app.h"

#include <stdio.h>

#include "interface/foc_platform_api.h"
#include "interface/control_scheduler.h"
#include "algorithm/foc_control.h"
#include "algorithm/foc_control_init.h"
#include "algorithm/sensor.h"
#include "interface/protocol_parser.h"
#include "interface/command_manager.h"
#include "interface/debug_stream.h"
#include "algorithm/svpwm.h"
#include "config/foc_config.h"

/* Internal function prototypes */
static void Service_Task_Trigger(void);
static void Motor_Control_Loop(void);
static void Monitor_Task_Trigger(void);
static void FOC_App_OnPwmUpdateISR(void);
static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data);
static void FOC_App_StopFastCurrentLoop(void);
static void FOC_App_EnterSafeOutputState(uint8_t report_skip);
static void FOC_App_ApplyPidRuntimeParams(void);
static void FOC_App_ProcessCommStep(void);
static void FOC_App_UpdateIndicators(void);
static void FOC_App_TriggerCommIndicatorPulse(void);
static uint8_t FOC_App_IsUndervoltageFaultActive(void);

#define FOC_APP_COMM_FRAMES_PER_STEP 1U

static foc_motor_t g_motor;
static sensor_data_t g_sensor_snapshot;
static foc_pid_t g_torque_current_pid;
static foc_pid_t g_angle_pid;
static foc_pid_t g_speed_pid;
#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
static sensor_data_t g_fast_current_sensor_snapshot;
#endif
static volatile uint8_t g_service_task_pending = 0U;
static volatile uint8_t g_monitor_task_pending = 0U;
static volatile uint8_t g_fast_current_loop_enabled = 0U;
static volatile uint8_t g_fast_current_loop_div_counter = 0U;
static volatile float g_fast_current_loop_iq_target = 0.0f;
static volatile float g_fast_current_loop_electrical_angle = 0.0f;
static uint8_t g_led_run_on = 1U;
static uint8_t g_app_init_completed = 0U;
static uint16_t g_led_run_blink_counter = 0U;
static uint16_t g_led_comm_pulse_counter = 0U;
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
static uint8_t g_prev_control_mode = COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE;
#endif
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
static uint8_t g_undervoltage_fault_latched = 0U;
#endif

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, 1U);

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();

    FOC_Platform_SetControlTickCallback(ControlScheduler_RunTick);
    ControlScheduler_SetCallback(FOC_TASK_RATE_SERVICE, Service_Task_Trigger);
    ControlScheduler_SetCallback(FOC_TASK_RATE_FAST_CONTROL, Motor_Control_Loop);
    ControlScheduler_SetCallback(FOC_TASK_RATE_MONITOR, Monitor_Task_Trigger);

    FOC_Platform_CommInit();

    CommandManager_Init();
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_COMMAND, 1U);
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_COMM, 1U);
    FOC_Platform_WriteDebugText("\r\n=== FOC System Started ===\r\n");
    FOC_Platform_WriteDebugText("USART1 telemetry channel enabled\r\n");
    FOC_Platform_WriteDebugText("PWM update ISR interpolation path enabled\r\n");
    FOC_Platform_WriteDebugText("PWM bridge running in center-aligned complementary mode\r\n");
    FOC_Platform_WriteDebugText("Control scheduler running at configured tick rate\r\n");
    FOC_Platform_WriteDebugText("Magnetic encoder feedback enabled\r\n");
    FOC_Platform_WriteDebugText("Current sampling pipeline enabled\r\n");
    FOC_Platform_WriteDebugText("Control debug telemetry enabled\r\n");
    FOC_Platform_WriteDebugText("Init feedback pipeline...\r\n\r\n");

    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_PROTOCOL, 1U);

    DebugStream_Init();
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_DEBUG, 1U);

    Sensor_Init(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    Sensor_ReadAll();
    if (Sensor_CopyData(&g_sensor_snapshot) != 0U)
    {
        CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_SENSOR,
                                       (uint8_t)((g_sensor_snapshot.adc_valid != 0U) &&
                                                 (g_sensor_snapshot.encoder_valid != 0U)));
    }
    else
    {
        CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_SENSOR, 0U);
    }

    /* Initialize SVPWM output and interpolation callback. */
    SVPWM_Init(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_PWM, 1U);

    /* Initialize motor model and targets. */
    FOC_MotorInit(&g_motor,
                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_MOTOR, 1U);

    /* Initialize current-loop PID states (safe defaults for future closed-loop enabling). */
    FOC_PIDInit(&g_torque_current_pid,
                CommandManager_GetCurrentPidKp(),
                CommandManager_GetCurrentPidKi(),
                CommandManager_GetCurrentPidKd(),
                -g_motor.set_voltage,
                g_motor.set_voltage);
    FOC_PIDInit(&g_angle_pid,
                CommandManager_GetAnglePidKp(),
                CommandManager_GetAnglePidKi(),
                CommandManager_GetAnglePidKd(),
                -g_motor.set_voltage,
                g_motor.set_voltage);
    FOC_PIDInit(&g_speed_pid,
                CommandManager_GetSpeedPidKp(),
                CommandManager_GetSpeedPidKi(),
                CommandManager_GetSpeedPidKd(),
                -g_motor.set_voltage,
                g_motor.set_voltage);

    char startup_info[128];
    snprintf(startup_info,
            sizeof(startup_info),
            "mech zero at elec0: %.4f rad, direction: %d ,pole pairs: %d\r\n",
            g_motor.mech_angle_at_elec_zero_rad,
            g_motor.direction,
            g_motor.pole_pairs);
    FOC_Platform_WriteDebugText(startup_info);

    CommandManager_FinalizeInitDiagnostics();
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    g_prev_control_mode = CommandManager_GetControlMode();
#endif
    g_app_init_completed = 1U;
    FOC_App_UpdateIndicators();

    //FOC_Platform_WaitMs(1000U);
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlRuntimeInterrupts(1U);
}

void FOC_App_Loop(void)
{
    if (ProtocolParser_IsParsePending() != 0U)
    {
        FOC_App_ProcessCommStep();
    }

    if (g_service_task_pending != 0U)
    {
        g_service_task_pending = 0U;
        FOC_App_UpdateIndicators();
    }

    if (g_monitor_task_pending != 0U)
    {
        g_monitor_task_pending = 0U;
        /* Debug stream cadence is bounded by monitor task trigger rate. */
        DebugStream_Process(&g_sensor_snapshot, &g_motor);
    }
}

static void FOC_App_ProcessCommStep(void)
{
    uint8_t consumed = 0U;

    while ((ProtocolParser_IsParsePending() != 0U) &&
           (consumed < FOC_APP_COMM_FRAMES_PER_STEP))
    {
        ProtocolParser_Process();
        if (ProtocolParser_GetLastResult() == PROTOCOL_PARSER_RESULT_FRAME_ERROR)
        {
            CommandManager_ReportProtocolFrameError();
        }
        CommandManager_Process();
        if (CommandManager_GetRuntimeState()->last_exec_ok != 0U)
        {
            FOC_App_TriggerCommIndicatorPulse();
        }
        consumed++;
    }
}

static void Service_Task_Trigger(void)
{
    g_service_task_pending = 1U;
}

static void Monitor_Task_Trigger(void)
{
    g_monitor_task_pending = 1U;
}

static void FOC_App_StopFastCurrentLoop(void)
{
    g_fast_current_loop_enabled = 0U;
    g_fast_current_loop_iq_target = 0.0f;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_div_counter = 0U;
}

static void FOC_App_EnterSafeOutputState(uint8_t report_skip)
{
    FOC_App_StopFastCurrentLoop();
    FOC_ControlResetCurrentSoftSwitchState();
    FOC_OpenLoopStep(&g_motor, 0.0f, 0.0f);

    if (report_skip != 0U)
    {
        CommandManager_ReportControlLoopSkip();
    }
}

static void FOC_App_OnPwmUpdateISR(void)
{
    uint8_t divider;
    float current_loop_dt_sec;
    const sensor_data_t *current_sensor = 0;

    /* Keep interpolation update at each PWM update interrupt. */
    SVPWM_InterpolationISR();

    if (g_fast_current_loop_enabled == 0U)
    {
        return;
    }

    divider = (FOC_CURRENT_LOOP_ISR_DIVIDER == 0U) ? 1U : (uint8_t)FOC_CURRENT_LOOP_ISR_DIVIDER;
    g_fast_current_loop_div_counter++;
    if (g_fast_current_loop_div_counter < divider)
    {
        return;
    }
    g_fast_current_loop_div_counter = 0U;

    g_motor.iq_target = g_fast_current_loop_iq_target;
    if (FOC_PWM_FREQ_KHZ == 0U)
    {
        current_loop_dt_sec = FOC_CONTROL_DT_SEC;
    }
    else
    {
        current_loop_dt_sec = (float)divider / ((float)FOC_PWM_FREQ_KHZ * 1000.0f);
    }

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    Sensor_ReadCurrentOnly();
    if (Sensor_CopyData(&g_fast_current_sensor_snapshot) == 0U)
    {
        return;
    }

    if (g_fast_current_sensor_snapshot.adc_valid == 0U)
    {
        return;
    }
    current_sensor = &g_fast_current_sensor_snapshot;
#endif

    FOC_CurrentControlStep(&g_motor,
                           &g_torque_current_pid,
                           current_sensor,
                           g_fast_current_loop_electrical_angle,
                           current_loop_dt_sec);
}

static void FOC_App_UpdateIndicators(void)
{
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;
    const command_manager_runtime_state_t *state = CommandManager_GetRuntimeState();

    if (g_app_init_completed == 0U)
    {
        led_run_on = 1U;
        g_led_run_on = 1U;
        g_led_run_blink_counter = 0U;
    }
    else if ((state != 0) && (state->system_state == COMMAND_MANAGER_SYSTEM_FAULT))
    {
        led_run_on = 0U;
        led_fault_on = 1U;
        g_led_run_on = 0U;
        g_led_run_blink_counter = 0U;
    }
    else if ((state != 0) && (state->system_state == COMMAND_MANAGER_SYSTEM_RUNNING))
    {
        if (g_led_run_blink_counter >= (FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS - 1U))
        {
            g_led_run_blink_counter = 0U;
            g_led_run_on = (g_led_run_on == 0U) ? 1U : 0U;
        }
        else
        {
            g_led_run_blink_counter++;
        }
        led_run_on = g_led_run_on;
    }
    else
    {
        led_run_on = 0U;
        g_led_run_on = 0U;
        g_led_run_blink_counter = 0U;
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

static void FOC_App_TriggerCommIndicatorPulse(void)
{
    g_led_comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;
}

static void Motor_Control_Loop(void)
{
    const command_manager_runtime_state_t *state;

    state = CommandManager_GetRuntimeState();

    if ((state != 0) && (state->system_state == COMMAND_MANAGER_SYSTEM_FAULT))
    {
        /* Stop runtime I2C read path in fault state to avoid repeated bus timeout load. */
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    Sensor_ReadAll();
    if (Sensor_CopyData(&g_sensor_snapshot) == 0U)
    {
        FOC_App_EnterSafeOutputState(0U);
        CommandManager_ReportRuntimeSensorState(0U, 0U);
        return;
    }

    CommandManager_ReportRuntimeSensorState(g_sensor_snapshot.adc_valid,
                                            g_sensor_snapshot.encoder_valid);

    if ((g_sensor_snapshot.adc_valid == 0U) || (g_sensor_snapshot.encoder_valid == 0U))
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if ((state != 0) && (state->params_dirty != 0U))
    {
        /* Apply sampling offset before motor/fault gates so protocol updates work in disabled state. */
        Sensor_ADCSampleTimeOffset(CommandManager_GetSensorSampleOffsetPercent());
    }

    if (FOC_App_IsUndervoltageFaultActive() != 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if (CommandManager_IsMotorEnabled() == COMMAND_MANAGER_ENABLED_DISABLE)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if ((state != 0) && (state->params_dirty != 0U))
    {
        FOC_App_ApplyPidRuntimeParams();
        CommandManager_ClearDirtyFlag();
    }

    FOC_App_RunControlAlgorithm(&g_sensor_snapshot);
}

static uint8_t FOC_App_IsUndervoltageFaultActive(void)
{
    const float vbus_voltage = g_motor.vbus_voltage;
    FOC_Platform_UndervoltageProtect(vbus_voltage);

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if ((g_undervoltage_fault_latched != 0U) &&
        (vbus_voltage >= FOC_UNDERVOLTAGE_RECOVER_VBUS_DEFAULT))
    {
        g_undervoltage_fault_latched = 0U;
    }

    if (g_undervoltage_fault_latched != 0U)
    {
        return 1U;
    }

    if (vbus_voltage < FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        g_undervoltage_fault_latched = 1U;
        CommandManager_ReportUndervoltageFault(vbus_voltage);
        return 1U;
    }

    return 0U;
#else
    (void)vbus_voltage;
    return 0U;
#endif
}

static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data)
{
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)
    FOC_SpeedOuterLoopStep(&g_motor,
                           &g_speed_pid,
                           CommandManager_GetSpeedOnlyRadS(),
                           sensor_data,
                           FOC_CONTROL_DT_SEC);
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)
    FOC_SpeedAngleOuterLoopStep(&g_motor,
                                &g_speed_pid,
                                &g_angle_pid,
                                CommandManager_GetTargetAngleRad(),
                                CommandManager_GetAngleSpeedRadS(),
                                sensor_data,
                                FOC_CONTROL_DT_SEC);
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    const uint8_t control_mode = CommandManager_GetControlMode();

    if (control_mode != g_prev_control_mode)
    {
        if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
        {
            FOC_ControlRebaseMechanicalAngleAccum(&g_motor, sensor_data->mech_angle_rad.output_value);
        }

        g_torque_current_pid.integral = 0.0f;
        g_torque_current_pid.prev_error = 0.0f;
        g_angle_pid.integral = 0.0f;
        g_angle_pid.prev_error = 0.0f;
        g_speed_pid.integral = 0.0f;
        g_speed_pid.prev_error = 0.0f;
        FOC_ControlResetSpeedLoopState();
        g_prev_control_mode = control_mode;
    }

    /* FULL build keeps both parallel algorithms and runtime mode switching. */
    if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(&g_motor,
                               &g_speed_pid,
                               CommandManager_GetSpeedOnlyRadS(),
                               sensor_data,
                               FOC_CONTROL_DT_SEC);
    }
    else if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
    {
        FOC_SpeedAngleOuterLoopStep(&g_motor,
                                    &g_speed_pid,
                                    &g_angle_pid,
                                    CommandManager_GetTargetAngleRad(),
                                    CommandManager_GetAngleSpeedRadS(),
                                    sensor_data,
                                    FOC_CONTROL_DT_SEC);
    }
    else
    {
        FOC_App_StopFastCurrentLoop();
        return;
    }
#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif

    /* Current algorithm always runs in PWM ISR; task loop only publishes fast-loop targets. */
    g_fast_current_loop_iq_target = g_motor.iq_target;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_enabled = 1U;
}

static void FOC_App_ApplyPidRuntimeParams(void)
{
    g_torque_current_pid.kp = CommandManager_GetCurrentPidKp();
    g_torque_current_pid.ki = CommandManager_GetCurrentPidKi();
    g_torque_current_pid.kd = CommandManager_GetCurrentPidKd();

    g_angle_pid.kp = CommandManager_GetAnglePidKp();
    g_angle_pid.ki = CommandManager_GetAnglePidKi();
    g_angle_pid.kd = CommandManager_GetAnglePidKd();

    g_speed_pid.kp = CommandManager_GetSpeedPidKp();
    g_speed_pid.ki = CommandManager_GetSpeedPidKi();
    g_speed_pid.kd = CommandManager_GetSpeedPidKd();

    g_torque_current_pid.out_min = -g_motor.set_voltage;
    g_torque_current_pid.out_max = g_motor.set_voltage;
    g_angle_pid.out_min = -g_motor.set_voltage;
    g_angle_pid.out_max = g_motor.set_voltage;
    g_speed_pid.out_min = -g_motor.set_voltage;
    g_speed_pid.out_max = g_motor.set_voltage;

    FOC_ControlSetCurrentSoftSwitchMode(CommandManager_GetCurrentSoftSwitchMode());
    FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(CommandManager_GetCurrentSoftSwitchAutoOpenIqA());
    FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(CommandManager_GetCurrentSoftSwitchAutoClosedIqA());
    FOC_ControlSetCurrentSoftSwitchEnable(CommandManager_IsCurrentSoftSwitchEnabled());
    FOC_ControlResetCurrentSoftSwitchState();
}
