#include "L2_Core/foc_motor_aggregate.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"

#include <math.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 加速器：全速域速度参考斜率限制 + 区域上限钳位
 * ================================================================ */

float FOC_Accel_ApplySpeedLimit(foc_outer_loop_private_t *state,
                                float target_mech_speed_rad_s,
                                float ramp_rate_rad_s2,
                                float speed_limit_rad_s,
                                float dt_sec)
{
    float current;
    float max_step;

    current = state->ramped_speed_rad_s;

    max_step = ramp_rate_rad_s2 * fmaxf(dt_sec, 0.0f);
    if (target_mech_speed_rad_s > current + max_step)
    {
        current += max_step;
    }
    else if (target_mech_speed_rad_s < current - max_step)
    {
        current -= max_step;
    }
    else
    {
        current = target_mech_speed_rad_s;
    }

    if (current > speed_limit_rad_s)  current = speed_limit_rad_s;
    if (current < -speed_limit_rad_s) current = -speed_limit_rad_s;

    state->ramped_speed_rad_s = current;
    return current;
}

static float FOC_NormalizeDt(float dt_sec)
{
    return (dt_sec > 0.0f) ? dt_sec : FOC_CONTROL_DT_SEC;
}

static float FOC_PIDRunCore(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

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

static float FOC_AngleHoldPIDRun(foc_pid_t *pid,
                                 const foc_control_cfg_t *cfg,
                                 float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    dt_sec = FOC_NormalizeDt(dt_sec);
    error = target - measurement;
    if (fabsf(error) <= cfg->angle_hold_pid_deadband_rad)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        return 0.0f;
    }

    pid->integral += error * dt_sec;
    pid->integral = Math_ClampFloat(pid->integral, -cfg->angle_hold_integral_limit, cfg->angle_hold_integral_limit);
    derivative = (error - pid->prev_error) / dt_sec;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = Math_ClampFloat(pid->integral, i_min, i_max);
        pid->integral = Math_ClampFloat(pid->integral, -cfg->angle_hold_integral_limit, cfg->angle_hold_integral_limit);
    }
    pid->prev_error = error;
    return output;
}

static void FOC_ResetPIDState(foc_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void FOC_ResetSpeedState(foc_outer_loop_private_t *state)
{
    state->speed_err_accum_rad = 0.0f;
    state->prev_mech_signed_rad = 0.0f;
    state->speed_state_valid = 0U;
}

static void FOC_UpdateAccumulatedMechanicalAngle(foc_outer_loop_private_t *state,
                                                  const foc_control_cfg_t *cfg,
                                                  float mech_angle_rad)
{
    float delta;

    if (state->prev_valid == 0U)
    {
        state->prev_rad = mech_angle_rad;
        state->accum_rad = mech_angle_rad;
        state->prev_valid = 1U;
        return;
    }

    delta = Math_WrapRadDelta(mech_angle_rad - state->prev_rad);
    if (fabsf(delta) >= cfg->min_mech_angle_accum_delta_rad)
    {
        state->accum_rad += delta;
    }
    state->prev_rad = mech_angle_rad;
}

static float FOC_UpdateSpeedAngleError(foc_outer_loop_private_t *state,
                                       const foc_motor_params_t *params,
                                       float mech_angle_rad,
                                       float speed_ref_rad_s, float dt_sec)
{
    float mech_signed_rad;
    float mech_delta_rad;
    float speed_cmd_delta_rad;

    dt_sec = FOC_NormalizeDt(dt_sec);
    mech_signed_rad = params->direction * mech_angle_rad;

    if (state->speed_state_valid == 0U)
    {
        state->prev_mech_signed_rad = mech_signed_rad;
        state->speed_err_accum_rad = 0.0f;
        state->speed_state_valid = 1U;
        return 0.0f;
    }

    mech_delta_rad = Math_WrapRadDelta(mech_signed_rad - state->prev_mech_signed_rad);
    state->prev_mech_signed_rad = mech_signed_rad;
    speed_cmd_delta_rad = speed_ref_rad_s * dt_sec;
    state->speed_err_accum_rad += speed_cmd_delta_rad - mech_delta_rad;
    state->speed_err_accum_rad = Math_ClampFloat(state->speed_err_accum_rad,
                                                    -FOC_SPEED_ERR_ACCUM_LIMIT_RAD,
                                                    FOC_SPEED_ERR_ACCUM_LIMIT_RAD);
    return state->speed_err_accum_rad;
}

void FOC_SpeedOuterLoopStep(foc_outer_loop_private_t *state,
                            foc_pid_t *speed_pid,
                            foc_control_runtime_t *ctrl,
                            const foc_active_source_state_t *active,
                            const foc_control_cfg_t *cfg,
                            const foc_motor_params_t *params,
                            uint8_t control_region,
                            float speed_ref_rad_s, float dt_sec)
{
    float speed_angle_error_rad;
    float mech_angle_rad;
    float ramp_rate;
    float speed_limit;

    if ((state == 0) || (speed_pid == 0) || (ctrl == 0) || (active == 0) || (cfg == 0) || (params == 0)) return;

    dt_sec = FOC_NormalizeDt(dt_sec);

    /* 加速器：斜坡限幅 + 区域上限钳位（单点入口 FOC_Accel_ApplySpeedLimit） */
    if (control_region == FOC_CONTROL_REGION_LOW)
    {
        ramp_rate   = FOC_ACCEL_RAMP_RATE_LOW_RAD_S2;
        speed_limit = FOC_ACCEL_SPEED_LIMIT_LOW_RAD_S;
    }
    else
    {
        ramp_rate   = FOC_ACCEL_RAMP_RATE_HIGH_RAD_S2;
        speed_limit = FOC_ACCEL_SPEED_LIMIT_HIGH_RAD_S;
    }

    speed_ref_rad_s = FOC_Accel_ApplySpeedLimit(state, speed_ref_rad_s, ramp_rate, speed_limit, dt_sec);

    mech_angle_rad = active->mech_angle_rad;
    speed_angle_error_rad = FOC_UpdateSpeedAngleError(state, params, mech_angle_rad,
                                                       speed_ref_rad_s, dt_sec);
    ctrl->iq_target = FOC_PIDRunCore(speed_pid, speed_angle_error_rad, 0.0f, dt_sec);
    (void)cfg;
}

void FOC_SpeedAngleOuterLoopStep(foc_outer_loop_private_t *state,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 foc_control_runtime_t *ctrl,
                                 const foc_active_source_state_t *active,
                                 const foc_control_cfg_t *cfg,
                                 const foc_motor_params_t *params,
                                 float angle_ref_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec)
{
    float mech_angle_rad;
    float torque_ref_speed;
    float torque_ref_hold;
    float mech_signed_total_rad;
    float angle_error_rad;
    float abs_angle_error_rad;
    float transition_span_rad;
    float speed_blend;
    float speed_ref_rad_s;
    float speed_angle_error_rad;

    if ((state == 0) || (speed_pid == 0) || (angle_hold_pid == 0) ||
        (ctrl == 0) || (active == 0) || (cfg == 0) || (params == 0)) return;

    dt_sec = FOC_NormalizeDt(dt_sec);
    mech_angle_rad = active->mech_angle_rad;
    angle_ref_rad *= params->direction;

    FOC_UpdateAccumulatedMechanicalAngle(state, cfg, mech_angle_rad);
    mech_signed_total_rad = params->direction * state->accum_rad;
    angle_error_rad = angle_ref_rad - mech_signed_total_rad;
    abs_angle_error_rad = fabsf(angle_error_rad);

    transition_span_rad = cfg->speed_angle_transition_end_rad - cfg->speed_angle_transition_start_rad;
    if (transition_span_rad < 1e-6f) transition_span_rad = 1e-6f;

    if (abs_angle_error_rad <= cfg->speed_angle_transition_start_rad)
        speed_blend = 0.0f;
    else if (abs_angle_error_rad >= cfg->speed_angle_transition_end_rad)
        speed_blend = 1.0f;
    else
        speed_blend = (abs_angle_error_rad - cfg->speed_angle_transition_start_rad) / transition_span_rad;

    speed_ref_rad_s = ((angle_error_rad >= 0.0f) ? fabsf(angle_position_speed_rad_s) : -fabsf(angle_position_speed_rad_s)) * speed_blend;

    if (speed_blend < 1e-4f)
    {
        FOC_ResetPIDState(speed_pid);
        FOC_ResetSpeedState(state);
        torque_ref_speed = 0.0f;
    }
    else
    {
        speed_angle_error_rad = FOC_UpdateSpeedAngleError(state, params, mech_angle_rad,
                                                           speed_ref_rad_s, dt_sec);
        torque_ref_speed = FOC_PIDRunCore(speed_pid, speed_angle_error_rad, 0.0f, dt_sec);
    }

    torque_ref_hold = FOC_AngleHoldPIDRun(angle_hold_pid, cfg,
                                          angle_ref_rad, mech_signed_total_rad, dt_sec);

    ctrl->iq_target = (1.0f - speed_blend) * torque_ref_hold + speed_blend * torque_ref_speed;
}