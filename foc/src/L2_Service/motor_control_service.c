#include "L2_Service/motor_control_service.h"

#include "L3_Algorithm/foc_control_c11_entry.h"
#include "L3_Algorithm/foc_control_c12_init.h"
#include "L3_Algorithm/sensor.h"
#include "L3_Algorithm/svpwm.h"
#include "LS_Config/foc_config.h"

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
                FOC_ControlCompensationStep(motor, args->sensor);
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
                                            foc_pid_t *angle_hold_pid,
                                            const l2_control_config_snapshot_t *control_cfg)
{
    if ((motor == 0) || (current_pid == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (control_cfg == 0))
    {
        return;
    }

    FOC_PIDInit(current_pid,
                control_cfg->pid_current_kp,
                control_cfg->pid_current_ki,
                control_cfg->pid_current_kd,
                -motor->set_voltage,
                motor->set_voltage);
    FOC_PIDInit(angle_hold_pid,
                control_cfg->pid_angle_kp,
                control_cfg->pid_angle_ki,
                control_cfg->pid_angle_kd,
                -motor->set_voltage,
                motor->set_voltage);
    FOC_PIDInit(speed_pid,
                control_cfg->pid_speed_kp,
                control_cfg->pid_speed_ki,
                control_cfg->pid_speed_kd,
                -motor->set_voltage,
                motor->set_voltage);

    MotorControlService_ApplyConfigSnapshot(motor,
                                            current_pid,
                                            speed_pid,
                                            angle_hold_pid,
                                            control_cfg);
}

void MotorControlService_ApplyConfigSnapshot(foc_motor_t *motor,
                                             foc_pid_t *current_pid,
                                             foc_pid_t *speed_pid,
                                             foc_pid_t *angle_hold_pid,
                                             const l2_control_config_snapshot_t *control_cfg)
{
    if ((motor == 0) || (current_pid == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (control_cfg == 0))
    {
        return;
    }

    current_pid->kp = control_cfg->pid_current_kp;
    current_pid->ki = control_cfg->pid_current_ki;
    current_pid->kd = control_cfg->pid_current_kd;

    angle_hold_pid->kp = control_cfg->pid_angle_kp;
    angle_hold_pid->ki = control_cfg->pid_angle_ki;
    angle_hold_pid->kd = control_cfg->pid_angle_kd;

    speed_pid->kp = control_cfg->pid_speed_kp;
    speed_pid->ki = control_cfg->pid_speed_ki;
    speed_pid->kd = control_cfg->pid_speed_kd;

    current_pid->out_min = -motor->set_voltage;
    current_pid->out_max = motor->set_voltage;
    angle_hold_pid->out_min = -motor->set_voltage;
    angle_hold_pid->out_max = motor->set_voltage;
    speed_pid->out_min = -motor->set_voltage;
    speed_pid->out_max = motor->set_voltage;

    FOC_ControlSetMinMechAngleAccumDeltaRad(motor, control_cfg->cfg_min_mech_angle_accum_delta_rad);
    FOC_ControlSetAngleHoldIntegralLimit(motor, control_cfg->cfg_angle_hold_integral_limit);
    FOC_ControlSetAngleHoldPidDeadbandRad(motor, control_cfg->cfg_angle_hold_pid_deadband_rad);
    FOC_ControlSetSpeedAngleTransitionStartRad(motor, control_cfg->cfg_speed_angle_transition_start_rad);
    FOC_ControlSetSpeedAngleTransitionEndRad(motor, control_cfg->cfg_speed_angle_transition_end_rad);

    FOC_ControlSetCurrentSoftSwitchMode(motor, control_cfg->current_soft_switch_mode);
    FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(motor, control_cfg->current_soft_switch_auto_open_iq_a);
    FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(motor, control_cfg->current_soft_switch_auto_closed_iq_a);
    FOC_ControlSetCurrentSoftSwitchEnable(motor, control_cfg->current_soft_switch_enable);
    FOC_ControlSetCoggingCompEnable(motor, FOC_COGGING_COMP_ENABLE);
    FOC_ControlResetCurrentSoftSwitchState(motor);
}
