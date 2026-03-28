#include "foc_app.h"

#include <stdio.h>

#include "foc_platform_api.h"
#include "control_scheduler.h"
#include "foc_control.h"
#include "foc_control_init.h"
#include "sensor.h"
#include "protocol_parser.h"
#include "command_manager.h"
#include "debug_stream.h"
#include "svpwm.h"

/* Internal function prototypes */
static void LED_Blink_1Hz(void);
static void Motor_Control_Loop(void);
static void Monitor_Task_Trigger(void);
static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data);
static void FOC_App_ApplyPidRuntimeParams(void);
static void FOC_App_ProcessCommStep(void);

#define FOC_APP_COMM_FRAMES_PER_STEP 1U

static foc_motor_t g_motor;
static sensor_data_t *sensor;
static foc_pid_t g_torque_current_pid;
static foc_pid_t g_angle_pid;
static foc_pid_t g_speed_pid;
static volatile uint8_t g_monitor_task_pending = 0U;

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetHeartbeatIndicator(1U);
    FOC_Platform_HighRateClockInit(FOC_APP_PWM_FREQ_KHZ);

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();

    FOC_Platform_BindControlTickCallback(ControlScheduler_RunTick);
    ControlScheduler_SetCallback(FOC_TASK_RATE_HEARTBEAT, LED_Blink_1Hz);
    ControlScheduler_SetCallback(FOC_TASK_RATE_FAST_CONTROL, Motor_Control_Loop);
    ControlScheduler_SetCallback(FOC_TASK_RATE_MONITOR, Monitor_Task_Trigger);

    {
        FOC_Platform_CommConfig_t comm_config;

        comm_config.source_mask = (uint8_t)FOC_PLATFORM_COMM_SOURCE_ALL;
        comm_config.arbitration_policy = (uint8_t)FOC_PLATFORM_COMM_ARB_ROUND_ROBIN;
        FOC_Platform_CommInit(&comm_config);
    }

    CommandManager_Init();
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_COMMAND, 1U);
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_COMM, 1U);
    FOC_Platform_DebugOutput("\r\n=== FOC System Started ===\r\n");
    FOC_Platform_DebugOutput("USART1 telemetry channel enabled\r\n");
    FOC_Platform_DebugOutput("High-rate modulation clock active at configured PWM frequency\r\n");
    FOC_Platform_DebugOutput("PWM bridge running in center-aligned complementary mode\r\n");
    FOC_Platform_DebugOutput("Control scheduler running at 1kHz with heartbeat callback\r\n");
    FOC_Platform_DebugOutput("Magnetic encoder feedback enabled\r\n");
    FOC_Platform_DebugOutput("Current sampling pipeline enabled\r\n");
    FOC_Platform_DebugOutput("Control debug telemetry enabled\r\n");
    FOC_Platform_DebugOutput("Init feedback pipeline...\r\n\r\n");

    ProtocolParser_Init();
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_PROTOCOL, 1U);

    DebugStream_Init();
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_DEBUG, 1U);

    Sensor_Init(FOC_APP_SENSOR_SAMPLE_FREQ_KHZ, FOC_APP_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    Sensor_ReadAll();
    sensor = Sensor_GetData();
    if (sensor != 0)
    {
        CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_SENSOR,
                                       (uint8_t)((sensor->adc_valid != 0U) && (sensor->encoder_valid != 0U)));
    }
    else
    {
        CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_SENSOR, 0U);
    }

    /* Initialize SVPWM output and interpolation callback. */
    SVPWM_Init(FOC_APP_PWM_FREQ_KHZ, FOC_APP_SVPWM_DEADTIME_PERCENT_DEFAULT);
    CommandManager_ReportInitCheck(COMMAND_MANAGER_INIT_CHECK_PWM, 1U);

    FOC_Platform_RegisterHighRateCallback(SVPWM_InterpolationISR);
    FOC_Platform_StartHighRateClock();

    /* Initialize motor model and targets. */
    FOC_MotorInit(&g_motor,
                  FOC_APP_MOTOR_INIT_VBUS_DEFAULT,
                  FOC_APP_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                  FOC_APP_MOTOR_INIT_PHASE_RES_DEFAULT,
                  FOC_APP_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                  FOC_APP_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                  FOC_APP_MOTOR_INIT_DIRECTION_DEFAULT);
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
        FOC_Platform_DebugOutput(startup_info);

    CommandManager_FinalizeInitDiagnostics();

    //FOC_Platform_WaitMs(1000U);
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
}

void FOC_App_Loop(void)
{
    if (ProtocolParser_IsParsePending() != 0U)
    {
        FOC_App_ProcessCommStep();
    }

    if (g_monitor_task_pending != 0U)
    {
        g_monitor_task_pending = 0U;
        DebugStream_Process(sensor, &g_motor);
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
        consumed++;
    }
}

static void Monitor_Task_Trigger(void)
{
    g_monitor_task_pending = 1U;
}

static void LED_Blink_1Hz(void)
{
    static uint8_t led3_state = 0;

    if (led3_state == 0)
    {
        FOC_Platform_SetHeartbeatIndicator(1U);
        led3_state = 1;
    }
    else
    {
        FOC_Platform_SetHeartbeatIndicator(0U);
        led3_state = 0;
    }
}

static void Motor_Control_Loop(void)
{
    const command_manager_runtime_state_t *state;

    Sensor_ReadAll();

    sensor = Sensor_GetData();
    state = CommandManager_GetRuntimeState();

    if (sensor == 0)
    {
        CommandManager_ReportRuntimeSensorState(0U, 0U);
        return;
    }

    CommandManager_ReportRuntimeSensorState(sensor->adc_valid, sensor->encoder_valid);

    if ((sensor->adc_valid == 0U) || (sensor->encoder_valid == 0U))
    {
        return;
    }

    if ((state != 0) && (state->system_state == COMMAND_MANAGER_SYSTEM_FAULT))
    {
        FOC_OpenLoopStep(&g_motor, 0.0f, 0.0f);
        CommandManager_ReportControlLoopSkip();
        return;
    }

    if (CommandManager_IsMotorEnabled() == COMMAND_MANAGER_ENABLED_DISABLE)
    {
        FOC_OpenLoopStep(&g_motor, 0.0f, 0.0f);
        CommandManager_ReportControlLoopSkip();
        return;
    }

    if ((state != 0) && (state->params_dirty != 0U))
    {
        FOC_App_ApplyPidRuntimeParams();
        CommandManager_ClearDirtyFlag();
    }

    FOC_App_RunControlAlgorithm(sensor);
}

static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data)
{
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)
    FOC_SpeedControlStep(&g_motor,
                         &g_speed_pid,
                         &g_torque_current_pid,
                         CommandManager_GetAngleSpeedRadS(),
                         sensor_data,
                         FOC_APP_CONTROL_DT_SEC,
                         FOC_TORQUE_MODE_CURRENT_PID);
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)
    FOC_SpeedAngleControlStep(&g_motor,
                              &g_speed_pid,
                              &g_angle_pid,
                              &g_torque_current_pid,
                              CommandManager_GetTargetAngleRad(),
                              CommandManager_GetAngleSpeedRadS(),
                              sensor_data,
                              FOC_APP_CONTROL_DT_SEC,
                              FOC_TORQUE_MODE_CURRENT_PID);
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    /* FULL build keeps both parallel algorithms and runtime mode switching. */
    if (CommandManager_GetControlMode() == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedControlStep(&g_motor,
                             &g_speed_pid,
                             &g_torque_current_pid,
                             CommandManager_GetAngleSpeedRadS(),
                             sensor_data,
                             FOC_APP_CONTROL_DT_SEC,
                             FOC_TORQUE_MODE_CURRENT_PID);
    }
    else
    {
        FOC_SpeedAngleControlStep(&g_motor,
                                  &g_speed_pid,
                                  &g_angle_pid,
                                  &g_torque_current_pid,
                                  CommandManager_GetTargetAngleRad(),
                                  CommandManager_GetAngleSpeedRadS(),
                                  sensor_data,
                                  FOC_APP_CONTROL_DT_SEC,
                                  FOC_TORQUE_MODE_CURRENT_PID);
    }
#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif
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
}
