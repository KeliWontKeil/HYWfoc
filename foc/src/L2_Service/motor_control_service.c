#include "L2_Service/motor_control_service.h"

#include "L2_Service/command_manager.h"
#include "L3_Algorithm/control_config_iface.h"
#include "L3_Algorithm/motion_control_iface.h"
#include "L3_Algorithm/motor_init_iface.h"
#include "L3_Algorithm/sensor.h"
#include "L3_Algorithm/svpwm.h"

void MotorControlService_ResetControlConfigDefault(void)
{
    FOC_ControlConfigResetDefault();
}

void MotorControlService_InitMotor(foc_motor_t *motor,
                                   float vbus_voltage,
                                   float set_voltage,
                                   float phase_resistance,
                                   uint8_t pole_pairs,
                                   float mech_angle_at_elec_zero_rad,
                                   int8_t direction)
{
    FOC_MotorInit(motor,
                  vbus_voltage,
                  set_voltage,
                  phase_resistance,
                  pole_pairs,
                  mech_angle_at_elec_zero_rad,
                  direction);
}

void MotorControlService_InitPwmOutput(uint16_t pwm_freq_khz, uint8_t deadtime_percent)
{
    SVPWM_Init(pwm_freq_khz, deadtime_percent);
}

void MotorControlService_RunPwmInterpolationIsr(void)
{
    SVPWM_InterpolationISR();
}

void MotorControlService_InitSensorInput(uint8_t pwm_freq_khz, float sample_offset_percent)
{
    Sensor_Init(pwm_freq_khz, sample_offset_percent);
}

void MotorControlService_SetSensorSampleOffsetPercent(float sample_offset_percent)
{
    Sensor_ADCSampleTimeOffset(sample_offset_percent);
}

uint8_t MotorControlService_ReadAllSensorSnapshot(sensor_data_t *snapshot)
{
    Sensor_ReadAll();
    return Sensor_CopyData(snapshot);
}

uint8_t MotorControlService_ReadCurrentSensorSnapshot(sensor_data_t *snapshot)
{
    Sensor_ReadCurrentOnly();
    return Sensor_CopyData(snapshot);
}

uint8_t MotorControlService_RequiresCurrentSample(void)
{
    return FOC_ControlRequiresCurrentSample();
}

void MotorControlService_ResetCurrentSoftSwitchState(void)
{
    FOC_ControlResetCurrentSoftSwitchState();
}

uint8_t MotorControlService_RunControlTask(motor_control_service_task_t task,
                                           foc_motor_t *motor,
                                           foc_pid_t *current_pid,
                                           foc_pid_t *speed_pid,
                                           foc_pid_t *angle_hold_pid,
                                           const motor_control_service_task_args_t *args)
{
    switch (task)
    {
        case MOTOR_CONTROL_SERVICE_TASK_OUTER_LOOP:
            return FOC_ControlOuterLoopStep(motor,
                                            current_pid,
                                            speed_pid,
                                            angle_hold_pid,
                                            args->sensor,
                                            args->control_mode,
                                            args->speed_only_rad_s,
                                            args->target_angle_rad,
                                            args->angle_position_speed_rad_s,
                                            args->dt_sec);

        case MOTOR_CONTROL_SERVICE_TASK_CURRENT_LOOP:
            FOC_CurrentControlStep(motor,
                                   current_pid,
                                   args->sensor,
                                   args->electrical_angle,
                                   args->dt_sec);
            return 1U;

        case MOTOR_CONTROL_SERVICE_TASK_OPEN_LOOP:
            FOC_OpenLoopStep(motor, args->open_loop_voltage, args->open_loop_turn_speed);
            return 1U;

        default:
            return 0U;
    }
}

void MotorControlService_RunOpenLoop(foc_motor_t *motor, float voltage, float turn_speed)
{
    motor_control_service_task_args_t args;

    args.sensor = 0;
    args.control_mode = 0U;
    args.speed_only_rad_s = 0.0f;
    args.target_angle_rad = 0.0f;
    args.angle_position_speed_rad_s = 0.0f;
    args.electrical_angle = 0.0f;
    args.open_loop_voltage = voltage;
    args.open_loop_turn_speed = turn_speed;
    args.dt_sec = 0.0f;

    (void)MotorControlService_RunControlTask(MOTOR_CONTROL_SERVICE_TASK_OPEN_LOOP,
                                             motor,
                                             0,
                                             0,
                                             0,
                                             &args);
}

uint8_t MotorControlService_RunOuterLoop(foc_motor_t *motor,
                                         foc_pid_t *current_pid,
                                         foc_pid_t *speed_pid,
                                         foc_pid_t *angle_hold_pid,
                                         const sensor_data_t *sensor,
                                         uint8_t control_mode,
                                         float speed_only_rad_s,
                                         float target_angle_rad,
                                         float angle_position_speed_rad_s,
                                         float dt_sec)
{
    motor_control_service_task_args_t args;

    args.sensor = sensor;
    args.control_mode = control_mode;
    args.speed_only_rad_s = speed_only_rad_s;
    args.target_angle_rad = target_angle_rad;
    args.angle_position_speed_rad_s = angle_position_speed_rad_s;
    args.electrical_angle = 0.0f;
    args.open_loop_voltage = 0.0f;
    args.open_loop_turn_speed = 0.0f;
    args.dt_sec = dt_sec;

    return MotorControlService_RunControlTask(MOTOR_CONTROL_SERVICE_TASK_OUTER_LOOP,
                                              motor,
                                              current_pid,
                                              speed_pid,
                                              angle_hold_pid,
                                              &args);
}

void MotorControlService_RunCurrentLoop(foc_motor_t *motor,
                                        foc_pid_t *current_pid,
                                        const sensor_data_t *sensor,
                                        float electrical_angle,
                                        float dt_sec)
{
    motor_control_service_task_args_t args;

    args.sensor = sensor;
    args.control_mode = 0U;
    args.speed_only_rad_s = 0.0f;
    args.target_angle_rad = 0.0f;
    args.angle_position_speed_rad_s = 0.0f;
    args.electrical_angle = electrical_angle;
    args.open_loop_voltage = 0.0f;
    args.open_loop_turn_speed = 0.0f;
    args.dt_sec = dt_sec;

    (void)MotorControlService_RunControlTask(MOTOR_CONTROL_SERVICE_TASK_CURRENT_LOOP,
                                             motor,
                                             current_pid,
                                             0,
                                             0,
                                             &args);
}

void MotorControlService_InitPidControllers(foc_motor_t *motor,
                                            foc_pid_t *current_pid,
                                            foc_pid_t *speed_pid,
                                            foc_pid_t *angle_hold_pid)
{
    if ((motor == 0) || (current_pid == 0) || (speed_pid == 0) || (angle_hold_pid == 0))
    {
        return;
    }

    FOC_PIDInit(current_pid,
                CommandManager_GetCurrentPidKp(),
                CommandManager_GetCurrentPidKi(),
                CommandManager_GetCurrentPidKd(),
                -motor->set_voltage,
                motor->set_voltage);
    FOC_PIDInit(angle_hold_pid,
                CommandManager_GetAnglePidKp(),
                CommandManager_GetAnglePidKi(),
                CommandManager_GetAnglePidKd(),
                -motor->set_voltage,
                motor->set_voltage);
    FOC_PIDInit(speed_pid,
                CommandManager_GetSpeedPidKp(),
                CommandManager_GetSpeedPidKi(),
                CommandManager_GetSpeedPidKd(),
                -motor->set_voltage,
                motor->set_voltage);

    MotorControlService_ApplyPendingConfig(motor,
                                           current_pid,
                                           speed_pid,
                                           angle_hold_pid);
}

void MotorControlService_ApplyPendingConfig(foc_motor_t *motor,
                                            foc_pid_t *current_pid,
                                            foc_pid_t *speed_pid,
                                            foc_pid_t *angle_hold_pid)
{
    if ((motor == 0) || (current_pid == 0) || (speed_pid == 0) || (angle_hold_pid == 0))
    {
        return;
    }

    current_pid->kp = CommandManager_GetCurrentPidKp();
    current_pid->ki = CommandManager_GetCurrentPidKi();
    current_pid->kd = CommandManager_GetCurrentPidKd();

    angle_hold_pid->kp = CommandManager_GetAnglePidKp();
    angle_hold_pid->ki = CommandManager_GetAnglePidKi();
    angle_hold_pid->kd = CommandManager_GetAnglePidKd();

    speed_pid->kp = CommandManager_GetSpeedPidKp();
    speed_pid->ki = CommandManager_GetSpeedPidKi();
    speed_pid->kd = CommandManager_GetSpeedPidKd();

    current_pid->out_min = -motor->set_voltage;
    current_pid->out_max = motor->set_voltage;
    angle_hold_pid->out_min = -motor->set_voltage;
    angle_hold_pid->out_max = motor->set_voltage;
    speed_pid->out_min = -motor->set_voltage;
    speed_pid->out_max = motor->set_voltage;

    FOC_ControlSetMinMechAngleAccumDeltaRad(CommandManager_GetControlMinMechAngleAccumDeltaRad());
    FOC_ControlSetAngleHoldIntegralLimit(CommandManager_GetControlAngleHoldIntegralLimit());
    FOC_ControlSetAngleHoldPidDeadbandRad(CommandManager_GetControlAngleHoldPidDeadbandRad());
    FOC_ControlSetSpeedAngleTransitionStartRad(CommandManager_GetControlSpeedAngleTransitionStartRad());
    FOC_ControlSetSpeedAngleTransitionEndRad(CommandManager_GetControlSpeedAngleTransitionEndRad());

    FOC_ControlSetCurrentSoftSwitchMode(CommandManager_GetCurrentSoftSwitchMode());
    FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(CommandManager_GetCurrentSoftSwitchAutoOpenIqA());
    FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(CommandManager_GetCurrentSoftSwitchAutoClosedIqA());
    FOC_ControlSetCurrentSoftSwitchEnable(CommandManager_IsCurrentSoftSwitchEnabled());
    FOC_ControlResetCurrentSoftSwitchState();
}
