#include "L1_Orchestration/foc_app.h"

#include <stdio.h>

#include "L1_Orchestration/control_scheduler.h"
#include "L2_Service/debug_stream.h"
#include "L2_Service/runtime_c1_entry.h"
#include "L2_Service/motor_control_service.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* Internal function prototypes */
static void Service_Task_Trigger(void);
static void Motor_Control_Loop(void);
static void Monitor_Task_Trigger(void);
static void FOC_App_OnPwmUpdateISR(void);
static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data);
static void FOC_App_StopFastCurrentLoop(void);
static void FOC_App_EnterSafeOutputState(uint8_t report_skip);
static void FOC_App_ProcessCommStep(void);
static void FOC_App_UpdateIndicators(void);
static void FOC_App_TriggerCommIndicatorPulse(void);
static uint8_t FOC_App_IsUndervoltageFaultActive(void);
static void FOC_App_RefreshL2Snapshot(void);

#define FOC_APP_COMM_FRAMES_PER_STEP 1U

static foc_motor_t g_motor;
static sensor_data_t g_sensor_snapshot;
static foc_pid_t g_torque_current_pid;
static foc_pid_t g_angle_pid;
static foc_pid_t g_speed_pid;
static sensor_data_t g_fast_current_sensor_snapshot;
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
static runtime_snapshot_t g_l2_snapshot;
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
static uint8_t g_undervoltage_fault_latched = 0U;
#endif

void FOC_App_Init(void)
{
    uint16_t init_check_pass_mask = 0U;
    uint16_t init_check_fail_mask = 0U;
    runtime_c1_step_input_t init_step = {0};

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
    /* Keep control-related IRQs disabled until FOC_App_Start() enters runtime. */
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    FOC_Platform_CommInit();

    RuntimeC1_Init();
    FOC_App_RefreshL2Snapshot();
    init_check_pass_mask = (uint16_t)(init_check_pass_mask |
                                      RUNTIME_INIT_CHECK_COMMAND |
                                      RUNTIME_INIT_CHECK_COMM);

    MotorControlService_ResetControlConfigDefault(&g_motor);
    FOC_Platform_WriteDebugText("\r\n=== FOC System Started ===\r\n");
    FOC_Platform_WriteDebugText("USART1 telemetry channel enabled\r\n");
    FOC_Platform_WriteDebugText("PWM update ISR interpolation path enabled\r\n");
    FOC_Platform_WriteDebugText("PWM bridge running in center-aligned complementary mode\r\n");
    FOC_Platform_WriteDebugText("Control scheduler running at configured tick rate\r\n");
    FOC_Platform_WriteDebugText("Magnetic encoder feedback enabled\r\n");
    FOC_Platform_WriteDebugText("Current sampling pipeline enabled\r\n");
    FOC_Platform_WriteDebugText("Control debug telemetry enabled\r\n");
    FOC_Platform_WriteDebugText("Init feedback pipeline...\r\n\r\n");

    init_check_pass_mask = (uint16_t)(init_check_pass_mask | RUNTIME_INIT_CHECK_PROTOCOL);

#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    DebugStream_Init();
#endif
    init_check_pass_mask = (uint16_t)(init_check_pass_mask | RUNTIME_INIT_CHECK_DEBUG);

    MotorControlService_InitSensorInput(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    if (MotorControlService_ReadAllSensorSnapshot(&g_sensor_snapshot) != 0U)
    {
        if ((g_sensor_snapshot.adc_valid != 0U) && (g_sensor_snapshot.encoder_valid != 0U))
        {
            init_check_pass_mask = (uint16_t)(init_check_pass_mask | RUNTIME_INIT_CHECK_SENSOR);
        }
        else
        {
            init_check_fail_mask = (uint16_t)(init_check_fail_mask | RUNTIME_INIT_CHECK_SENSOR);
        }
    }
    else
    {
        init_check_fail_mask = (uint16_t)(init_check_fail_mask | RUNTIME_INIT_CHECK_SENSOR);
    }

    /* Initialize SVPWM output and interpolation callback. */
    MotorControlService_InitPwmOutput(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);
    init_check_pass_mask = (uint16_t)(init_check_pass_mask | RUNTIME_INIT_CHECK_PWM);

    /* Initialize motor model and targets. */
    MotorControlService_InitMotor(&g_motor,
                                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);
    init_check_pass_mask = (uint16_t)(init_check_pass_mask | RUNTIME_INIT_CHECK_MOTOR);

    /* PID initialization and runtime config applying are managed by L2 service. */
    MotorControlService_InitPidControllers(&g_motor,
                                           &g_torque_current_pid,
                                           &g_speed_pid,
                                           &g_angle_pid,
                                           &g_l2_snapshot.control_cfg);

    char startup_info[128];
    snprintf(startup_info,
            sizeof(startup_info),
            "mech zero at elec0: %.4f rad, direction: %d ,pole pairs: %d\r\n",
            g_motor.mech_angle_at_elec_zero_rad,
            g_motor.direction,
            g_motor.pole_pairs);
    FOC_Platform_WriteDebugText(startup_info);

    init_step.init_checks_pass_mask = init_check_pass_mask;
    init_step.init_checks_fail_mask = init_check_fail_mask;
    init_step.finalize_init = 1U;
    (void)RuntimeC1_RunStep(0U, &init_step);

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
    FOC_App_ProcessCommStep();

    if (g_service_task_pending != 0U)
    {
        g_service_task_pending = 0U;
        FOC_App_UpdateIndicators();
    }

    if (g_monitor_task_pending != 0U)
    {
        g_monitor_task_pending = 0U;
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
        /* Debug stream cadence is bounded by monitor task trigger rate. */
        FOC_App_RefreshL2Snapshot();
        DebugStream_SetExecutionCycles(ControlScheduler_GetExecutionCycles());
        DebugStream_Process(&g_sensor_snapshot,
                            &g_motor,
                            &g_l2_snapshot.runtime,
                            &g_l2_snapshot.telemetry);
#endif
    }
}

static void FOC_App_RefreshL2Snapshot(void)
{
    RuntimeC1_GetSnapshot(&g_l2_snapshot);
}

static void FOC_App_ProcessCommStep(void)
{
    if (RuntimeC1_RunStep(FOC_APP_COMM_FRAMES_PER_STEP, 0) != 0U)
    {
        FOC_App_TriggerCommIndicatorPulse();
    }

    RuntimeC1_GetSnapshot(&g_l2_snapshot);
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
    motor_control_service_task_args_t task_args;

    FOC_App_StopFastCurrentLoop();
    MotorControlService_ResetCurrentSoftSwitchState(&g_motor);

    task_args.sensor = 0;
    task_args.control_mode = 0U;
    task_args.speed_only_rad_s = 0.0f;
    task_args.target_angle_rad = 0.0f;
    task_args.angle_position_speed_rad_s = 0.0f;
    task_args.electrical_angle = 0.0f;
    task_args.open_loop_voltage = 0.0f;
    task_args.open_loop_turn_speed = 0.0f;
    task_args.dt_sec = 0.0f;
    (void)MotorControlService_RunControlTask(MOTOR_CONTROL_SERVICE_TASK_OPEN_LOOP,
                                             &g_motor,
                                             0,
                                             0,
                                             0,
                                             &task_args);

    if (report_skip != 0U)
    {
        runtime_c1_step_input_t step_input = {0};
        step_input.control_loop_skipped = 1U;
        (void)RuntimeC1_RunStep(0U, &step_input);
    }
}

static void FOC_App_OnPwmUpdateISR(void)
{
    uint8_t divider;
    float current_loop_dt_sec;
    const sensor_data_t *current_sensor = 0;
    motor_control_service_task_args_t task_args;

    /* Keep interpolation update at each PWM update interrupt. */
    MotorControlService_RunPwmInterpolationIsr();

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

    if (MotorControlService_RequiresCurrentSample() != 0U)
    {
        if (MotorControlService_ReadCurrentSensorSnapshot(&g_fast_current_sensor_snapshot) == 0U)
        {
            return;
        }

        if (g_fast_current_sensor_snapshot.adc_valid == 0U)
        {
            return;
        }
        current_sensor = &g_fast_current_sensor_snapshot;
    }

    task_args.sensor = current_sensor;
    task_args.control_mode = 0U;
    task_args.speed_only_rad_s = 0.0f;
    task_args.target_angle_rad = 0.0f;
    task_args.angle_position_speed_rad_s = 0.0f;
    task_args.electrical_angle = g_fast_current_loop_electrical_angle;
    task_args.open_loop_voltage = 0.0f;
    task_args.open_loop_turn_speed = 0.0f;
    task_args.dt_sec = current_loop_dt_sec;
    (void)MotorControlService_RunControlTask(MOTOR_CONTROL_SERVICE_TASK_CURRENT_LOOP,
                                             &g_motor,
                                             &g_torque_current_pid,
                                             0,
                                             0,
                                             &task_args);
}

static void FOC_App_UpdateIndicators(void)
{
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    FOC_App_RefreshL2Snapshot();

    if (g_app_init_completed == 0U)
    {
        led_run_on = 1U;
        g_led_run_on = 1U;
        g_led_run_blink_counter = 0U;
    }
    else if (g_l2_snapshot.runtime.system_fault != 0U)
    {
        led_run_on = 0U;
        led_fault_on = 1U;
        g_led_run_on = 0U;
        g_led_run_blink_counter = 0U;
    }
    else if (g_l2_snapshot.runtime.system_running != 0U)
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
    FOC_App_RefreshL2Snapshot();

    if (g_l2_snapshot.runtime.system_fault != 0U)
    {
        /* Stop runtime I2C read path in fault state to avoid repeated bus timeout load. */
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if (MotorControlService_ReadAllSensorSnapshot(&g_sensor_snapshot) == 0U)
    {
        runtime_c1_step_input_t step_input = {0};
        step_input.sensor_state_updated = 1U;
        step_input.adc_valid = 0U;
        step_input.encoder_valid = 0U;
        (void)RuntimeC1_RunStep(0U, &step_input);

        FOC_App_EnterSafeOutputState(0U);
        return;
    }

    {
        runtime_c1_step_input_t step_input = {0};
        step_input.sensor_state_updated = 1U;
        step_input.adc_valid = g_sensor_snapshot.adc_valid;
        step_input.encoder_valid = g_sensor_snapshot.encoder_valid;
        (void)RuntimeC1_RunStep(0U, &step_input);
    }

    if ((g_sensor_snapshot.adc_valid == 0U) || (g_sensor_snapshot.encoder_valid == 0U))
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if (g_l2_snapshot.runtime.params_dirty != 0U)
    {
        /* Apply sampling offset before motor/fault gates so protocol updates work in disabled state. */
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
        MotorControlService_SetSensorSampleOffsetPercent(g_l2_snapshot.control_cfg.sensor_sample_offset_percent);
#endif
    }

    if (FOC_App_IsUndervoltageFaultActive() != 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if (g_l2_snapshot.control_cfg.motor_enabled == 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if (g_l2_snapshot.runtime.params_dirty != 0U)
    {
        MotorControlService_ApplyConfigSnapshot(&g_motor,
                                                &g_torque_current_pid,
                                                &g_speed_pid,
                                                &g_angle_pid,
                                                &g_l2_snapshot.control_cfg);
        RuntimeC1_Commit();
    }

    FOC_App_RunControlAlgorithm(&g_sensor_snapshot);
}

static uint8_t FOC_App_IsUndervoltageFaultActive(void)
{
    const float vbus_voltage = g_motor.vbus_voltage;

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    FOC_Platform_UndervoltageProtect(vbus_voltage);

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
        runtime_c1_step_input_t step_input = {0};
        g_undervoltage_fault_latched = 1U;
        step_input.undervoltage_fault = 1U;
        step_input.undervoltage_vbus = vbus_voltage;
        (void)RuntimeC1_RunStep(0U, &step_input);
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
    motor_control_service_task_args_t task_args;

    task_args.sensor = sensor_data;
    task_args.control_mode = g_l2_snapshot.control_cfg.control_mode;
    task_args.speed_only_rad_s = g_l2_snapshot.control_cfg.speed_only_rad_s;
    task_args.target_angle_rad = g_l2_snapshot.control_cfg.target_angle_rad;
    task_args.angle_position_speed_rad_s = g_l2_snapshot.control_cfg.angle_position_speed_rad_s;
    task_args.electrical_angle = 0.0f;
    task_args.open_loop_voltage = 0.0f;
    task_args.open_loop_turn_speed = 0.0f;
    task_args.dt_sec = FOC_CONTROL_DT_SEC;

    if (MotorControlService_RunControlTask(MOTOR_CONTROL_SERVICE_TASK_OUTER_LOOP,
                                           &g_motor,
                                           &g_torque_current_pid,
                                           &g_speed_pid,
                                           &g_angle_pid,
                                           &task_args) == 0U)
    {
        FOC_App_StopFastCurrentLoop();
        return;
    }

    /* Current algorithm always runs in PWM ISR; task loop only publishes fast-loop targets. */
    g_fast_current_loop_iq_target = g_motor.iq_target;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_enabled = 1U;
}



