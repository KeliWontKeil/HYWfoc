#include "L1_Orchestration/foc_app.h"

#include <stdio.h>

#include "L1_Orchestration/control_scheduler.h"
#include "L2_Service/debug_stream.h"
#include "L2_Service/runtime_c1_entry.h"
#include "L2_Service/motor_control_service.h"
#include "L3_Algorithm/foc_control_c11_entry.h"
#include "L3_Algorithm/foc_control_c24_compensation.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define FOC_APP_COMM_FRAMES_PER_STEP 1U

static void Service_Task_Trigger(void);
static void Motor_Control_Loop(void);
static void Monitor_Task_Trigger(void);
static void FOC_App_OnPwmUpdateISR(void);
static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data);
static void FOC_App_StopFastCurrentLoop(void);
static void FOC_App_EnterSafeOutputState(uint8_t report_skip);

static foc_motor_t g_motor;
static sensor_data_t g_sensor_snapshot;
static sensor_data_t g_fast_current_sensor_snapshot;
static foc_pid_t g_torque_current_pid;
static foc_pid_t g_angle_pid;
static foc_pid_t g_speed_pid;
static runtime_snapshot_t g_StateSnapshot;

static volatile uint8_t g_service_task_pending = 0U;
static volatile uint8_t g_monitor_task_pending = 0U;

static volatile uint8_t g_fast_current_loop_enabled = 0U;
static volatile uint8_t g_fast_current_loop_div_counter = 0U;
static volatile float g_fast_current_loop_iq_target = 0.0f;
static volatile float g_fast_current_loop_electrical_angle = 0.0f;

static uint16_t g_led_comm_pulse_counter = 0U;

static void FOC_App_UpdateIndicators(void)
{
    static uint8_t s_led_run_on = 0U;
    static uint16_t s_led_run_blink_counter = 0U;
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    if (g_StateSnapshot.runtime.system_fault != 0U)
    {
        led_run_on = 0U;
        led_fault_on = 1U;
        s_led_run_on = 0U;
        s_led_run_blink_counter = 0U;
    }
    else if (g_StateSnapshot.runtime.system_running != 0U)
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
    MotorControlService_ResetCurrentSoftSwitchState(&g_motor);

    MotorControlService_RunOpenLoopControlTask(&g_motor, 0.0f, 0.0f);

    if (report_skip != 0U)
    {
        runtime_step_signal_t step_input = {0};
        step_input.control_loop_skipped = 1U;
        Runtime_UpdateSignals(&step_input);
    }
}

static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data)
{
    static uint8_t s_last_control_mode = 0xFFU;
    uint8_t cur_mode;
    float dt_sec;

    cur_mode = g_StateSnapshot.control_cfg.control_mode;

    /* Reset current soft-switch blend state when control mode changes. */
    if (cur_mode != s_last_control_mode)
    {
        MotorControlService_ResetCurrentSoftSwitchState(&g_motor);
        s_last_control_mode = cur_mode;
    }

    dt_sec = FOC_CONTROL_DT_SEC;
    MotorControlService_RunOuterLoopControlTask(&g_motor,
                                                &g_torque_current_pid,
                                                &g_speed_pid,
                                                &g_angle_pid,
                                                sensor_data,
                                                cur_mode,
                                                g_StateSnapshot.control_cfg.speed_only_rad_s,
                                                g_StateSnapshot.control_cfg.target_angle_rad,
                                                g_StateSnapshot.control_cfg.angle_position_speed_rad_s,
                                                dt_sec);

    /* Run calibration sample step (handles deferred start/dump/export requests when idle). */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    (void)FOC_CoggingCalibSampleStep(&g_motor, sensor_data, dt_sec);
#endif

    FOC_ControlCompensationStep(&g_motor, sensor_data);

    /* Current algorithm always runs in PWM ISR; task loop only publishes fast-loop targets. */
    g_fast_current_loop_iq_target = g_motor.iq_target;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_enabled = 1U;
}

//////////////////////////////////////////////////////////////////////////////////////////

void FOC_App_Init(void)
{
    runtime_step_signal_t init_step = {0};

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

    Runtime_Init();
    Runtime_GetSnapshot(&g_StateSnapshot);
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask |
                                      RUNTIME_INIT_CHECK_COMMAND |
                                      RUNTIME_INIT_CHECK_COMM);

    MotorControlService_ResetControlConfigDefault(&g_motor);
    FOC_Platform_WriteDebugText("\r\n=== FOC System Started ===\r\n");
    FOC_Platform_WriteDebugText("Init motor,please wait...\r\n\r\n");

    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_PROTOCOL);

    DebugStream_Init();

    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_DEBUG);

    MotorControlService_InitSensorInput(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    MotorControlService_ReadAllSensorSnapshot(&g_sensor_snapshot);

    if ((g_sensor_snapshot.adc_valid != 0U) && (g_sensor_snapshot.encoder_valid != 0U))
    {
        init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_SENSOR);
    }
    else
    {
        init_step.init_checks_fail_mask = (uint16_t)(init_step.init_checks_fail_mask | RUNTIME_INIT_CHECK_SENSOR);
    }

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)

    if(g_sensor_snapshot.vbus_voltage_filtered > FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_VBUS);
    }
    else
    {
        init_step.init_checks_fail_mask = (uint16_t)(init_step.init_checks_fail_mask | RUNTIME_INIT_CHECK_VBUS);
    }

#else
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_VBUS);
#endif

    /* Initialize SVPWM output and interpolation callback. */
    MotorControlService_InitPwmOutput(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_PWM);

    /* Initialize motor model and targets. */
    MotorControlService_InitMotor(&g_motor,
                                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_MOTOR);


    /* PID initialization and runtime config applying are managed by L2 service. */
    MotorControlService_InitPidControllers(&g_motor,
                                           &g_torque_current_pid,
                                           &g_speed_pid,
                                           &g_angle_pid,
                                           &g_StateSnapshot.control_cfg);

    char startup_info[160];
    snprintf(startup_info,
            sizeof(startup_info),
            "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, set_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
            (double)g_motor.mech_angle_at_elec_zero_rad,
            (int)g_motor.direction,
            (int)g_motor.pole_pairs,
            (double)g_motor.vbus_voltage,
            (double)g_motor.set_voltage,
            (double)(g_motor.vbus_voltage > 0.0f ? g_motor.set_voltage / g_motor.vbus_voltage : 0.0f),
            (double)g_sensor_snapshot.vbus_voltage_filtered);
    FOC_Platform_WriteDebugText(startup_info);
    init_step.finalize_init = 1U;

    Runtime_UpdateSignals(&init_step);

    FOC_App_UpdateIndicators();
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlRuntimeInterrupts(1U);
}

void FOC_App_Loop(void)
{
    if (g_monitor_task_pending != 0U)
    {
        g_monitor_task_pending = 0U;
    #if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
        /* Debug stream cadence is bounded by monitor task trigger rate. */
        DebugStream_SetExecutionCycles(ControlScheduler_GetExecutionCycles());
        DebugStream_Process(&g_sensor_snapshot,
                            &g_motor,
                            &g_StateSnapshot.runtime,
                            &g_StateSnapshot.telemetry);
    #endif
    }

    if (g_service_task_pending != 0U)
    {
        g_service_task_pending = 0U;
			
        if (Runtime_FrameRunStep(FOC_APP_COMM_FRAMES_PER_STEP) != 0U)
            g_led_comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;

        if (g_StateSnapshot.runtime.params_dirty != 0U)
        {
            MotorControlService_ApplyConfigSnapshot(&g_motor,
                                                        &g_torque_current_pid,
                                                        &g_speed_pid,
                                                        &g_angle_pid,
                                                        &g_StateSnapshot.control_cfg);
            MotorControlService_SetSensorSampleOffsetPercent(g_StateSnapshot.control_cfg.sensor_sample_offset_percent);
            Runtime_Commit();
        }

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    /*
     * Deferred cogging LUT dump/export output at service-task rate,
     * one frame per call to avoid blocking the service loop.
     * Protocol functions Y:D and Y:T set the request flags; the actual
     * serial output happens here outside of control ISR context.
     */
    if (FOC_CoggingCalibIsDumpPending() != 0U)
    {
        FOC_CoggingCalibClearDumpPending();
        FOC_CoggingCalibDumpTable(&g_motor);
    }
    else if (FOC_CoggingCalibIsExportPending() != 0U)
    {
        FOC_CoggingCalibClearExportPending();
        FOC_CoggingCalibExportTable(&g_motor);
    }
#endif
    }
}

static void FOC_App_OnPwmUpdateISR(void)
{
    uint8_t divider;
    float current_loop_dt_sec;
    const sensor_data_t *current_sensor = 0;

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
        MotorControlService_ReadCurrentSensorSnapshot(&g_fast_current_sensor_snapshot);

        if (g_fast_current_sensor_snapshot.adc_valid == 0U)
        {
            return;
        }
        current_sensor = &g_fast_current_sensor_snapshot;
    }

    MotorControlService_RunCurrentLoopControlTask(&g_motor,
                                                  &g_torque_current_pid,
                                                  current_sensor,
                                                  g_fast_current_loop_electrical_angle,
                                                  current_loop_dt_sec);
}

static void Service_Task_Trigger(void)
{
    FOC_App_UpdateIndicators();
	Runtime_GetSnapshot(&g_StateSnapshot);

    g_service_task_pending = 1U;
}

static void Monitor_Task_Trigger(void)
{
    g_monitor_task_pending = 1U;
}

static void Motor_Control_Loop(void)
{
		
    if (g_StateSnapshot.runtime.system_fault != 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    MotorControlService_ReadAllSensorSnapshot(&g_sensor_snapshot);

    runtime_step_signal_t step_input = {0};
    step_input.sensor_state_updated = 1U;
    step_input.adc_valid = g_sensor_snapshot.adc_valid;
    step_input.encoder_valid = g_sensor_snapshot.encoder_valid;
    step_input.undervoltage_vbus = g_sensor_snapshot.vbus_voltage_filtered;

    Runtime_UpdateSignals(&step_input);

    if (g_StateSnapshot.control_cfg.motor_enabled == 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    if (FOC_CoggingCalibIsBusy(&g_motor) != 0U)
    {
        /* ------ Calibration active: pure open-loop self-drive ------ */

        /* Ensure PWM ISR runs open-loop (no current PID, no soft-switch). */
        g_motor.current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        g_motor.current_soft_switch_status.enabled = 0U;
        
        FOC_CoggingCalibSampleStep(&g_motor, &g_sensor_snapshot, FOC_CONTROL_DT_SEC);

        /* Publish targets for the fast current loop (PWM ISR). */
        g_fast_current_loop_iq_target = g_motor.iq_target;
        g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
        g_fast_current_loop_enabled = 1U;
    }
    else
    {
        FOC_App_RunControlAlgorithm(&g_sensor_snapshot);
    }
}
