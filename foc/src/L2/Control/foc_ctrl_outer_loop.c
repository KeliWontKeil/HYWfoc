#include "L2/Control/foc_ctrl_outer_loop.h"

#include <math.h>

#include "L2/Control/foc_ctrl_actuation.h"
#include "L3/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

static float FOC_NormalizeDt(float dt_sec)
{
    return (dt_sec > 0.0f) ? dt_sec : FOC_CONTROL_DT_SEC;
}

static float FOC_PIDRunCore(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    if (pid == 0) return 0.0f;

    dt_sec = FOC_NormalizeDt(dt_sec);
    error = target - measurement;
    pid->integral += error * dt_sec;
    derivative = (error - pid->prev_error) / dt_sec;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if ((pid->ki > 1e-6f) &&
        ((output <= pid->out_min + 1e-6f) || (output >= pid->out_max - 1e-6f)))
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = Math_ClampFloat(pid->integral, i_min, i_max);
    }
    pid->prev_error = error;
    return output;
}

static float FOC_AngleHoldPIDRun(foc_pid_t *pid, const foc_motor_t *motor,
                                 float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    if ((pid == 0) || (motor == 0)) return 0.0f;

    dt_sec = FOC_NormalizeDt(dt_sec);
    error = target - measurement;
    if (fabsf(error) <= motor->angle_hold_pid_deadband_rad)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        return 0.0f;
    }

    pid->integral += error * dt_sec;
    pid->integral = Math_ClampFloat(pid->integral, -motor->angle_hold_integral_limit, motor->angle_hold_integral_limit);
    derivative = (error - pid->prev_error) / dt_sec;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = Math_ClampFloat(pid->integral, i_min, i_max);
        pid->integral = Math_ClampFloat(pid->integral, -motor->angle_hold_integral_limit, motor->angle_hold_integral_limit);
    }
    pid->prev_error = error;
    return output;
}

static void FOC_ResetPIDState(foc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void FOC_ResetSpeedState(foc_motor_t *motor)
{
    motor->outer_loop_state.speed_err_accum_rad = 0.0f;
    motor->outer_loop_state.prev_mech_signed_rad = 0.0f;
    motor->outer_loop_state.speed_state_valid = 0U;
}

static void FOC_UpdateAccumulatedMechanicalAngle(foc_motor_t *motor, float mech_angle_rad)
{
    float delta;
    if (motor == 0) return;

    if (motor->mech_angle_prev_valid == 0U)
    {
        motor->mech_angle_prev_rad = mech_angle_rad;
        motor->mech_angle_accum_rad = mech_angle_rad;
        motor->mech_angle_prev_valid = 1U;
        return;
    }

    delta = Math_WrapRadDelta(mech_angle_rad - motor->mech_angle_prev_rad);
    if (fabsf(delta) >= motor->min_mech_angle_accum_delta_rad)
    {
        motor->mech_angle_accum_rad += delta;
    }
    motor->mech_angle_prev_rad = mech_angle_rad;
}

static float FOC_UpdateSpeedAngleError(foc_motor_t *motor, float mech_angle_rad,
                                       float speed_ref_rad_s, float dt_sec)
{
    float mech_signed_rad;
    float mech_delta_rad;
    float speed_cmd_delta_rad;

    if (motor == 0) return 0.0f;
    dt_sec = FOC_NormalizeDt(dt_sec);
    mech_signed_rad = motor->direction * mech_angle_rad;

    if (motor->outer_loop_state.speed_state_valid == 0U)
    {
        motor->outer_loop_state.prev_mech_signed_rad = mech_signed_rad;
        motor->outer_loop_state.speed_err_accum_rad = 0.0f;
        motor->outer_loop_state.speed_state_valid = 1U;
        return 0.0f;
    }

    mech_delta_rad = Math_WrapRadDelta(mech_signed_rad - motor->outer_loop_state.prev_mech_signed_rad);
    motor->outer_loop_state.prev_mech_signed_rad = mech_signed_rad;
    speed_cmd_delta_rad = speed_ref_rad_s * dt_sec;
    motor->outer_loop_state.speed_err_accum_rad += speed_cmd_delta_rad - mech_delta_rad;
    motor->outer_loop_state.speed_err_accum_rad = Math_ClampFloat(motor->outer_loop_state.speed_err_accum_rad,
                                                                   -FOC_SPEED_ERR_ACCUM_LIMIT_RAD,
                                                                   FOC_SPEED_ERR_ACCUM_LIMIT_RAD);
    return motor->outer_loop_state.speed_err_accum_rad;
}

void FOC_ControlRebaseMechanicalAngleAccum(foc_motor_t *motor, float mech_angle_rad)
{
    if (motor == 0) return;
    motor->mech_angle_accum_rad = mech_angle_rad;
    motor->mech_angle_prev_rad = mech_angle_rad;
    motor->mech_angle_prev_valid = 1U;
}

void FOC_ControlResetSpeedLoopState(foc_motor_t *motor)
{
    if (motor == 0) return;
    FOC_ResetSpeedState(motor);
}

void FOC_SpeedOuterLoopStep(foc_motor_t *motor, foc_pid_t *speed_pid,
                            float speed_ref_rad_s, const sensor_data_t *sensor, float dt_sec)
{
    float speed_angle_error_rad;
    if ((motor == 0) || (speed_pid == 0) || (sensor == 0)) return;

    dt_sec = FOC_NormalizeDt(dt_sec);
    speed_angle_error_rad = FOC_UpdateSpeedAngleError(motor, sensor->mech_angle_rad.output_value,
                                                       speed_ref_rad_s, dt_sec);
    motor->iq_target = FOC_PIDRunCore(speed_pid, speed_angle_error_rad, 0.0f, dt_sec);
    motor->cogging_speed_ref_rad_s = speed_ref_rad_s;
    motor->electrical_phase_angle = FOC_ControlMechanicalToElectricalAngle(motor, sensor->mech_angle_rad.output_value);
}

void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor, foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad, float angle_position_speed_rad_s,
                                 const sensor_data_t *sensor, float dt_sec)
{
    float torque_ref_speed;
    float torque_ref_hold;
    float mech_signed_total_rad;
    float angle_error_rad;
    float abs_angle_error_rad;
    float transition_span_rad;
    float speed_blend;
    float speed_ref_rad_s;
    float speed_angle_error_rad;

    if ((motor == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (sensor == 0)) return;

    dt_sec = FOC_NormalizeDt(dt_sec);
    angle_ref_rad *= motor->direction;

    FOC_UpdateAccumulatedMechanicalAngle(motor, sensor->mech_angle_rad.output_value);
    mech_signed_total_rad = motor->direction * motor->mech_angle_accum_rad;
    angle_error_rad = angle_ref_rad - mech_signed_total_rad;
    abs_angle_error_rad = fabsf(angle_error_rad);

    transition_span_rad = motor->speed_angle_transition_end_rad - motor->speed_angle_transition_start_rad;
    if (transition_span_rad < 1e-6f) transition_span_rad = 1e-6f;

    if (abs_angle_error_rad <= motor->speed_angle_transition_start_rad)
        speed_blend = 0.0f;
    else if (abs_angle_error_rad >= motor->speed_angle_transition_end_rad)
        speed_blend = 1.0f;
    else
        speed_blend = (abs_angle_error_rad - motor->speed_angle_transition_start_rad) / transition_span_rad;

    speed_ref_rad_s = ((angle_error_rad >= 0.0f) ? fabsf(angle_position_speed_rad_s) : -fabsf(angle_position_speed_rad_s)) * speed_blend;

    if (speed_blend < 1e-4f)
    {
        FOC_ResetPIDState(speed_pid);
        FOC_ResetSpeedState(motor);
        torque_ref_speed = 0.0f;
    }
    else
    {
        speed_angle_error_rad = FOC_UpdateSpeedAngleError(motor, sensor->mech_angle_rad.output_value,
                                                           speed_ref_rad_s, dt_sec);
        torque_ref_speed = FOC_PIDRunCore(speed_pid, speed_angle_error_rad, 0.0f, dt_sec);
    }

    torque_ref_hold = FOC_AngleHoldPIDRun(angle_hold_pid, motor,
                                          angle_ref_rad, mech_signed_total_rad, dt_sec);

    motor->iq_target = (1.0f - speed_blend) * torque_ref_hold + speed_blend * torque_ref_speed;
    motor->cogging_speed_ref_rad_s = speed_ref_rad_s;
    motor->electrical_phase_angle = FOC_ControlMechanicalToElectricalAngle(motor, sensor->mech_angle_rad.output_value);
}