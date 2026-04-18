#include "L2_Service/motor_control_service.h"

#include "L2_Service/command_manager.h"
#include "L3_Algorithm/foc_control_c11_entry.h"
#include "L3_Algorithm/foc_control_c12_init.h"
#include "L3_Algorithm/foc_control_c24_compensation.h"
#include "L3_Algorithm/sensor.h"
#include "L3_Algorithm/svpwm.h"

void MotorControlService_ResetControlConfigDefault(foc_motor_t *motor)
{
    FOC_ControlConfigResetDefault(motor);
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
    return FOC_ControlCurrentLoopRequiresSample();
}

void MotorControlService_ResetCurrentSoftSwitchState(foc_motor_t *motor)
{
    FOC_ControlResetCurrentSoftSwitchState(motor);
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
        {
            uint8_t run_ok = FOC_ControlOuterLoopStep(motor,
                                                      current_pid,
                                                      speed_pid,
                                                      angle_hold_pid,
                                                      args->sensor,
                                                      args->control_mode,
                                                      args->speed_only_rad_s,
                                                      args->target_angle_rad,
                                                      args->angle_position_speed_rad_s,
                                                      args->dt_sec);
            if (run_ok == 0U)
            {
                return 0U;
            }

            if (args->sensor != 0)
            {
                FOC_ControlApplyCoggingCompensation(motor,
                                                    args->sensor->mech_angle_rad.output_value,
                                                    motor->cogging_speed_ref_rad_s);
            }

            return 1U;
        }

        case MOTOR_CONTROL_SERVICE_TASK_CURRENT_LOOP:
            FOC_ControlCurrentLoopStep(motor,
                                       current_pid,
                                       args->sensor,
                                       args->electrical_angle,
                                       args->dt_sec);
            return 1U;

        case MOTOR_CONTROL_SERVICE_TASK_OPEN_LOOP:
            FOC_ControlOpenLoopStep(motor, args->open_loop_voltage, args->open_loop_turn_speed);
            return 1U;

        default:
            return 0U;
    }
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

    FOC_ControlSetMinMechAngleAccumDeltaRad(motor, CommandManager_GetControlMinMechAngleAccumDeltaRad());
    FOC_ControlSetAngleHoldIntegralLimit(motor, CommandManager_GetControlAngleHoldIntegralLimit());
    FOC_ControlSetAngleHoldPidDeadbandRad(motor, CommandManager_GetControlAngleHoldPidDeadbandRad());
    FOC_ControlSetSpeedAngleTransitionStartRad(motor, CommandManager_GetControlSpeedAngleTransitionStartRad());
    FOC_ControlSetSpeedAngleTransitionEndRad(motor, CommandManager_GetControlSpeedAngleTransitionEndRad());

    FOC_ControlSetCurrentSoftSwitchMode(motor, CommandManager_GetCurrentSoftSwitchMode());
    FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(motor, CommandManager_GetCurrentSoftSwitchAutoOpenIqA());
    FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(motor, CommandManager_GetCurrentSoftSwitchAutoClosedIqA());
    FOC_ControlSetCurrentSoftSwitchEnable(motor, CommandManager_IsCurrentSoftSwitchEnabled());
    FOC_ControlSetCoggingCompEnable(motor, FOC_COGGING_COMP_ENABLE);
    FOC_ControlResetCurrentSoftSwitchState(motor);
}
