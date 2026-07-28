#include "L2_Core/Control/foc_ctrl_estim.h"

#include <math.h>

#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)

#define FOC_ESTIM_SMO_CROSS_MIN 1e-9f

static float wrap_2pi(float angle_rad)
{
    while (angle_rad > FOC_MATH_TWO_PI)  angle_rad -= FOC_MATH_TWO_PI;
    while (angle_rad < 0.0f)             angle_rad += FOC_MATH_TWO_PI;
    return angle_rad;
}

void FOC_EstimSMO_Init(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->estim_smo_state.ialpha_est     = 0.0f;
    motor->estim_smo_state.ibeta_est      = 0.0f;
    motor->estim_smo_state.bemf_alpha     = 0.0f;
    motor->estim_smo_state.bemf_beta      = 0.0f;
    motor->estim_smo_state.z_alpha        = 0.0f;
    motor->estim_smo_state.z_beta         = 0.0f;
    motor->estim_smo_state.pll_angle_rad  = 0.0f;
    motor->estim_smo_state.pll_speed_rad_s = 0.0f;
    motor->estim_smo_state.pll_integral   = 0.0f;
    motor->estim_smo_state.k_slide        = FOC_ESTIM_SMO_K_SLIDE_DEFAULT;
    motor->estim_smo_state.phase_comp_rad = 0.0f;
    motor->estim_smo_state.prev_z_alpha   = 0.0f;
    motor->estim_smo_state.prev_z_beta    = 0.0f;
    motor->estim_smo_state.converge_counter = 0U;
    motor->estim_smo_state.lock_counter   = 0U;
    motor->estim_smo_state.rot_dir_counter = 0U;
    motor->estim_smo_state.rot_dir_last   = 0U;

    if ((motor->params.direction != FOC_DIR_UNDEFINED) &&
        (motor->params.mech_angle_at_elec_zero_rad != FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) &&
        (motor->params.pole_pairs > 0U))
    {
        float mech_zero = motor->params.mech_angle_at_elec_zero_rad;
        if (motor->params.direction == FOC_DIR_REVERSED)
        {
            mech_zero = FOC_MATH_TWO_PI - mech_zero;
        }
        motor->estim_smo_state.pll_angle_rad = mech_zero * (float)motor->params.pole_pairs;
        motor->estim_smo_state.pll_angle_rad = wrap_2pi(motor->estim_smo_state.pll_angle_rad);
        motor->estim_smo_state.initialized   = 1U;
    }
    else
    {
        motor->estim_smo_state.pll_angle_rad = 0.0f;
        motor->estim_smo_state.initialized   = 0U;
    }
}

void FOC_EstimSMO_Step(foc_motor_t *motor, float dt_sec)
{
    float i_alpha_meas, i_beta_meas;
    float u_alpha, u_beta;
    float err_alpha, err_beta;
    float Rs, Ls, inv_L;
    float e_theta;
    float bemf_mag;
    float cross_z;
    float theta_bemf;
    uint8_t valid_dt;

    if (motor == 0) return;

    theta_bemf = (motor->source_mgr_state.active_source == FOC_SOURCE_TYPE_OPENLOOP)
        ? motor->ctrl.electrical_angle_rad
        : motor->estim_smo_state.pll_angle_rad;

    Math_ClarkeTransform(motor->sensor.current_a.output_value,
                         motor->sensor.current_b.output_value,
                         motor->sensor.current_c.output_value,
                         &i_alpha_meas, &i_beta_meas);

    u_alpha = motor->ctrl.ud * cosf(theta_bemf) - motor->ctrl.uq * sinf(theta_bemf);
    u_beta  = motor->ctrl.ud * sinf(theta_bemf) + motor->ctrl.uq * cosf(theta_bemf);

    Rs    = fabsf(motor->params.phase_resistance);
    Ls    = fabsf(motor->params.stator_inductance);
    if (Ls < 1e-9f) Ls = 1e-9f;
    inv_L = 1.0f / Ls;

    valid_dt = (dt_sec > 0.0f) ? 1U : 0U;
    if (valid_dt == 0U) return;

    err_alpha = motor->estim_smo_state.ialpha_est - i_alpha_meas;
    err_beta  = motor->estim_smo_state.ibeta_est  - i_beta_meas;

    motor->estim_smo_state.prev_z_alpha = motor->estim_smo_state.z_alpha;
    motor->estim_smo_state.prev_z_beta  = motor->estim_smo_state.z_beta;

    motor->estim_smo_state.z_alpha = (err_alpha >= 0.0f)
        ?  motor->estim_smo_state.k_slide
        : -motor->estim_smo_state.k_slide;
    motor->estim_smo_state.z_beta  = (err_beta >= 0.0f)
        ?  motor->estim_smo_state.k_slide
        : -motor->estim_smo_state.k_slide;

    motor->estim_smo_state.ialpha_est += dt_sec * inv_L *
        (u_alpha - Rs * motor->estim_smo_state.ialpha_est - motor->estim_smo_state.z_alpha);
    motor->estim_smo_state.ibeta_est  += dt_sec * inv_L *
        (u_beta  - Rs * motor->estim_smo_state.ibeta_est  - motor->estim_smo_state.z_beta);

    {
        float cos_theta = cosf(motor->estim_smo_state.pll_angle_rad);
        float sin_theta = sinf(motor->estim_smo_state.pll_angle_rad);

        e_theta = -motor->estim_smo_state.z_alpha * cos_theta
                  - motor->estim_smo_state.z_beta  * sin_theta;

        motor->estim_smo_state.pll_integral += FOC_ESTIM_SMO_PLL_KI_DEFAULT * e_theta * dt_sec;
        motor->estim_smo_state.pll_speed_rad_s =
            FOC_ESTIM_SMO_PLL_KP_DEFAULT * e_theta + motor->estim_smo_state.pll_integral;

        motor->estim_smo_state.pll_angle_rad += motor->estim_smo_state.pll_speed_rad_s * dt_sec;
        motor->estim_smo_state.pll_angle_rad = wrap_2pi(motor->estim_smo_state.pll_angle_rad);
    }

    bemf_mag = sqrtf(motor->estim_smo_state.z_alpha * motor->estim_smo_state.z_alpha
                   + motor->estim_smo_state.z_beta  * motor->estim_smo_state.z_beta);

    cross_z = motor->estim_smo_state.prev_z_alpha * motor->estim_smo_state.z_beta
            - motor->estim_smo_state.prev_z_beta  * motor->estim_smo_state.z_alpha;

    if (fabsf(cross_z) > FOC_ESTIM_SMO_CROSS_MIN)
    {
        uint8_t dir = (cross_z > 0.0f) ? 1U : 0U;
        if (dir == motor->estim_smo_state.rot_dir_last)
        {
            motor->estim_smo_state.rot_dir_counter++;
        }
        else
        {
            motor->estim_smo_state.rot_dir_counter = 0U;
            motor->estim_smo_state.rot_dir_last    = dir;
        }
    }

    /*
     * 预收敛速度门限：LOW 区域且机械速度低于门限时不累计收敛计数，
     * 避免停转或低速时 SMO 误收敛。核心观测器/PLL 始终运行，
     * 仅收敛状态机被门控。
     */
    uint8_t allow_converge = 1U;
    if (motor->source_mgr_state.control_region == FOC_CONTROL_REGION_LOW)
    {
        float mech_speed = (motor->params.pole_pairs > 0U) ?
            fabsf(motor->estim_smo_state.pll_speed_rad_s / (float)motor->params.pole_pairs) : 0.0f;
        if (mech_speed < FOC_SMO_ACCEL_PRECONV_SPEED_THRESHOLD_RAD_S)
        {
            allow_converge = 0U;
        }
    }

    if ((allow_converge != 0U) &&
        (bemf_mag > FOC_ESTIM_SMO_CONVERGE_BEMF_V) &&
        (motor->estim_smo_state.rot_dir_counter >= FOC_ESTIM_SMO_ROT_DIR_CONSECUTIVE))
    {
        motor->estim_smo_state.converge_counter++;
        motor->estim_smo_state.lock_counter = 0U;
    }
    else
    {
        motor->estim_smo_state.converge_counter = 0U;
        motor->estim_smo_state.lock_counter++;
    }

    /* 机械速度（Source Manager Select 使用） */
    if (motor->params.pole_pairs > 0U)
    {
        motor->estim_smo_state.mech_speed_rad_s = motor->estim_smo_state.pll_speed_rad_s / (float)motor->params.pole_pairs;
    }
    else
    {
        motor->estim_smo_state.mech_speed_rad_s = 0.0f;
    }
}

#endif
