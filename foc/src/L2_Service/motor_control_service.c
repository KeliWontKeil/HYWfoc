#include "L2_Service/motor_control_service.h"

#include <math.h>

#include "L3_Algorithm/foc_control_c11_entry.h"
#include "L3_Algorithm/foc_control_c12_init.h"
#include "L3_Algorithm/foc_control_c13_cfg_state.h"
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

void MotorControlService_ReadAllSensorSnapshot(sensor_data_t *snapshot)
{
    Sensor_ReadAll();
    Sensor_CopyData(snapshot);
}

void MotorControlService_ReadCurrentSensorSnapshot(sensor_data_t *snapshot)
{
    Sensor_ReadCurrentOnly();
    Sensor_CopyData(snapshot);
}

uint8_t MotorControlService_RequiresCurrentSample(void)
{
    return FOC_ControlCurrentLoopRequiresSample();
}

void MotorControlService_ResetCurrentSoftSwitchState(foc_motor_t *motor)
{
    FOC_ControlResetCurrentSoftSwitchState(motor);
}

void MotorControlService_RunOuterLoopControlTask(foc_motor_t *motor,
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
    FOC_ControlOuterLoopStep(motor,
                                current_pid,
                                speed_pid,
                                angle_hold_pid,
                                sensor,
                                control_mode,
                                speed_only_rad_s,
                                target_angle_rad,
                                angle_position_speed_rad_s,
                                dt_sec);
}

void MotorControlService_RunCurrentLoopControlTask(foc_motor_t *motor,
                                                    foc_pid_t *current_pid,
                                                    const sensor_data_t *sensor,
                                                    float electrical_angle,
                                                    float dt_sec)
{
    FOC_ControlCurrentLoopStep(motor,
                                current_pid,
                                sensor,
                                electrical_angle,
                                dt_sec);
}

void MotorControlService_RunOpenLoopControlTask(foc_motor_t *motor,
                                                float open_loop_voltage,
                                                float open_loop_turn_speed)
{
    FOC_ControlOpenLoopStep(motor, open_loop_voltage, open_loop_turn_speed);
}

void MotorControlService_InitPidControllers(foc_motor_t *motor,
                                            foc_pid_t *current_pid,
                                            foc_pid_t *speed_pid,
                                            foc_pid_t *angle_hold_pid,
                                            const control_config_snapshot_t *control_cfg)
{
    float phase_res;
    float i_max;

    if ((motor == 0) || (current_pid == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (control_cfg == 0))
    {
        return;
    }

    /* Outer-loop (speed/angle) PID output is iq_target in amperes.
     * Use set_voltage / phase_resistance as a coarse clamp to prevent
     * commanding an unachievable current target that would saturate the
     * voltage-limited current loop. */
    phase_res = (fabsf(motor->phase_resistance) > 1e-6f) ? fabsf(motor->phase_resistance) : 1e-6f;
    i_max = motor->set_voltage / phase_res;
    if (i_max < 0.0f)
    {
        i_max = 0.0f;
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
                -i_max,
                i_max);
    FOC_PIDInit(speed_pid,
                control_cfg->pid_speed_kp,
                control_cfg->pid_speed_ki,
                control_cfg->pid_speed_kd,
                -i_max,
                i_max);

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
                                             const control_config_snapshot_t *control_cfg)
{
    if ((motor == 0) || (current_pid == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (control_cfg == 0))
    {
        return;
    }

#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    current_pid->kp = control_cfg->pid_current_kp;
    current_pid->ki = control_cfg->pid_current_ki;
    current_pid->kd = control_cfg->pid_current_kd;
#endif

#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    angle_hold_pid->kp = control_cfg->pid_angle_kp;
    angle_hold_pid->ki = control_cfg->pid_angle_ki;
    angle_hold_pid->kd = control_cfg->pid_angle_kd;
#endif

#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    speed_pid->kp = control_cfg->pid_speed_kp;
    speed_pid->ki = control_cfg->pid_speed_ki;
    speed_pid->kd = control_cfg->pid_speed_kd;
#endif

    current_pid->out_min = -motor->set_voltage;
    current_pid->out_max = motor->set_voltage;

    {
        float phase_res;
        float i_max;

        /* Outer-loop (speed/angle) PID output is iq_target in amperes.
         * Recompute coarse clamp from latest set_voltage and phase_resistance
         * so protocol-driven parameter updates take effect immediately. */
        phase_res = (fabsf(motor->phase_resistance) > 1e-6f) ? fabsf(motor->phase_resistance) : 1e-6f;
        i_max = motor->set_voltage / phase_res;
        if (i_max < 0.0f)
        {
            i_max = 0.0f;
        }

        angle_hold_pid->out_min = -i_max;
        angle_hold_pid->out_max = i_max;
        speed_pid->out_min = -i_max;
        speed_pid->out_max = i_max;
    }

#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    FOC_ControlSetMinMechAngleAccumDeltaRad(motor, control_cfg->cfg_min_mech_angle_accum_delta_rad);
    FOC_ControlSetAngleHoldIntegralLimit(motor, control_cfg->cfg_angle_hold_integral_limit);
    FOC_ControlSetAngleHoldPidDeadbandRad(motor, control_cfg->cfg_angle_hold_pid_deadband_rad);
    FOC_ControlSetSpeedAngleTransitionStartRad(motor, control_cfg->cfg_speed_angle_transition_start_rad);
    FOC_ControlSetSpeedAngleTransitionEndRad(motor, control_cfg->cfg_speed_angle_transition_end_rad);
#endif

    FOC_ControlSetCurrentSoftSwitchMode(motor, control_cfg->current_soft_switch_mode);
    FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(motor, control_cfg->current_soft_switch_auto_open_iq_a);
    FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(motor, control_cfg->current_soft_switch_auto_closed_iq_a);
    FOC_ControlSetCurrentSoftSwitchEnable(motor, control_cfg->current_soft_switch_enable);
    FOC_ControlResetCurrentSoftSwitchState(motor);

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    FOC_ControlSetCoggingCompEnable(motor, control_cfg->cogging_comp_enable);
#else
    /* Keep feature default when protocol cogging chain is trimmed out. */
#endif
    FOC_ControlSetCoggingCompIqLimitA(motor, control_cfg->cogging_comp_iq_limit_a);
    FOC_ControlSetCoggingCompSpeedGateRadS(motor, control_cfg->cogging_comp_speed_gate_rad_s);
    FOC_ControlSetCoggingCalibGainK(motor, control_cfg->cogging_calib_gain_k);
#endif /* FOC_COGGING_COMP_ENABLE */
}
