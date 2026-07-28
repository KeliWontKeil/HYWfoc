#include "L2_Core/Control/foc_ctrl_openloop.h"

#include <math.h>

#include "LS_Config/foc_config.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)

static float OpenLoop_Wrap2Pi(float angle_rad)
{
    while (angle_rad > FOC_MATH_TWO_PI) angle_rad -= FOC_MATH_TWO_PI;
    while (angle_rad < 0.0f) angle_rad += FOC_MATH_TWO_PI;
    return angle_rad;
}

void FOC_OpenLoop_Init(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->openloop_state.phase = FOC_OPENLOOP_STATE_RUNNING;
    motor->openloop_state.virtual_angle_rad = 0.0f;
    motor->openloop_state.virtual_speed_rad_s = 0.0f;
    motor->openloop_state.mech_speed_rad_s = 0.0f;

    motor->openloop_state.ramp_rate_rad_s2 = FOC_ACCEL_RAMP_RATE_LOW_RAD_S2
                                             * (float)motor->params.pole_pairs;

    {
        float default_elec_speed = 0.0f;
        if (motor->params.pole_pairs > 0U)
        {
            default_elec_speed = motor->cfg.speed_only_rad_s * (float)motor->params.pole_pairs;
            float elec_limit = FOC_ACCEL_SPEED_LIMIT_LOW_RAD_S * (float)motor->params.pole_pairs;
            if (default_elec_speed > elec_limit) default_elec_speed = elec_limit;
            if (default_elec_speed < 0.0f) default_elec_speed = 0.0f;
        }
        motor->openloop_state.target_speed_rad_s = default_elec_speed;
    }
}

void FOC_OpenLoop_RunStep(foc_motor_t *motor, float dt_sec)
{
    float target_elec;
    float elec_limit;
    float virtual_speed;
    float virtual_angle;

    if (motor == 0) return;
    if (motor->openloop_state.phase == FOC_OPENLOOP_STATE_FAILED) return;
    if ((motor->params.pole_pairs == 0U) || (dt_sec <= 0.0f)) return;

    motor->ctrl.iq_target = FOC_OPENLOOP_CURRENT_A;

    target_elec = motor->cfg.speed_only_rad_s * (float)motor->params.pole_pairs;
    elec_limit = FOC_ACCEL_SPEED_LIMIT_LOW_RAD_S * (float)motor->params.pole_pairs;
    if (target_elec > elec_limit) target_elec = elec_limit;
    if (target_elec < 0.0f) target_elec = 0.0f;
    motor->openloop_state.target_speed_rad_s = target_elec;

    motor->openloop_state.virtual_speed_rad_s +=
        motor->openloop_state.ramp_rate_rad_s2 * dt_sec;
    virtual_speed = motor->openloop_state.virtual_speed_rad_s;
    if (virtual_speed > target_elec)
    {
        virtual_speed = target_elec;
    }
    motor->openloop_state.virtual_speed_rad_s = virtual_speed;

    virtual_angle = motor->openloop_state.virtual_angle_rad + virtual_speed * dt_sec;
    virtual_angle = OpenLoop_Wrap2Pi(virtual_angle);
    motor->openloop_state.virtual_angle_rad = virtual_angle;

    motor->openloop_state.mech_speed_rad_s = virtual_speed / (float)motor->params.pole_pairs;
}

#endif
