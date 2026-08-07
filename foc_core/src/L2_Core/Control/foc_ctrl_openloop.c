#include "L2_Core/Control/foc_ctrl_openloop.h"

#include <math.h>

#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)

void FOC_OpenLoop_Init(foc_openloop_state_t *state,
                       const foc_motor_params_t *params,
                       const foc_control_cfg_t *cfg)
{
    if ((state == 0) || (params == 0) || (cfg == 0)) return;

    state->phase = FOC_OPENLOOP_STATE_RUNNING;
    state->virtual_angle_rad = 0.0f;
    state->virtual_speed_rad_s = 0.0f;
    state->mech_speed_rad_s = 0.0f;

    state->ramp_rate_rad_s2 = FOC_ACCEL_RAMP_RATE_LOW_RAD_S2
                              * (float)params->pole_pairs;

    {
        float default_elec_speed = 0.0f;
        if (params->pole_pairs > 0U)
        {
            default_elec_speed = cfg->speed_only_rad_s * (float)params->pole_pairs;
            float elec_limit = FOC_ACCEL_SPEED_LIMIT_LOW_RAD_S * (float)params->pole_pairs;
            if (default_elec_speed > elec_limit) default_elec_speed = elec_limit;
            if (default_elec_speed < -elec_limit) default_elec_speed = -elec_limit;
        }
        state->target_speed_rad_s = default_elec_speed;
    }
}

uint8_t FOC_OpenLoop_RunStep(foc_openloop_state_t *state,
                             foc_control_runtime_t *ctrl,
                             const foc_motor_params_t *params,
                             const foc_control_cfg_t *cfg,
                             float dt_sec,
                             float *mech_speed_rad_s)
{
    float target_elec;
    float elec_limit;
    float virtual_speed;
    float virtual_angle;

    if ((state == 0) || (ctrl == 0) || (params == 0) || (cfg == 0) || (mech_speed_rad_s == 0)) return 0U;
    if (state->phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
    if ((params->pole_pairs == 0U) || (dt_sec <= 0.0f)) return 0U;

    target_elec = cfg->speed_only_rad_s * (float)params->pole_pairs;
    elec_limit = FOC_ACCEL_SPEED_LIMIT_LOW_RAD_S * (float)params->pole_pairs;
    if (target_elec > elec_limit) target_elec = elec_limit;
    if (target_elec < -elec_limit) target_elec = -elec_limit;
    state->target_speed_rad_s = target_elec;

    /* 开环电流按目标方向取符号，负向时反向牵引 */
    ctrl->iq_target = (target_elec >= 0.0f) ? FOC_OPENLOOP_CURRENT_A : -FOC_OPENLOOP_CURRENT_A;

    /* 对称双向斜坡逼近目标速度 */
    virtual_speed = state->virtual_speed_rad_s;
    {
        float speed_step = state->ramp_rate_rad_s2 * dt_sec;
        if (virtual_speed < target_elec)
        {
            virtual_speed += speed_step;
            if (virtual_speed > target_elec) virtual_speed = target_elec;
        }
        else
        {
            virtual_speed -= speed_step;
            if (virtual_speed < target_elec) virtual_speed = target_elec;
        }
    }
    state->virtual_speed_rad_s = virtual_speed;

    virtual_angle = state->virtual_angle_rad + virtual_speed * dt_sec;
    virtual_angle = Math_WrapRad(virtual_angle);
    state->virtual_angle_rad = virtual_angle;

    state->mech_speed_rad_s = virtual_speed / (float)params->pole_pairs;

    *mech_speed_rad_s = state->mech_speed_rad_s;

    return 1U;
}

#endif