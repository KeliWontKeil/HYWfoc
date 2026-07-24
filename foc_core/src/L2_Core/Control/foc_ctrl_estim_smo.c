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

    /* 5.4节 PLL初始角度策略 */
    if ((motor->direction != FOC_DIR_UNDEFINED) &&
        (motor->mech_angle_at_elec_zero_rad != FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) &&
        (motor->pole_pairs > 0U))
    {
        float mech_zero = motor->mech_angle_at_elec_zero_rad;
        if (motor->direction == FOC_DIR_REVERSED)
        {
            mech_zero = FOC_MATH_TWO_PI - mech_zero;
        }
        motor->estim_smo_state.pll_angle_rad = mech_zero * (float)motor->pole_pairs;
        motor->estim_smo_state.pll_angle_rad = wrap_2pi(motor->estim_smo_state.pll_angle_rad);
        motor->estim_smo_state.initialized   = 1U;
    }
    else
    {
        motor->estim_smo_state.pll_angle_rad = 0.0f;
        motor->estim_smo_state.initialized   = 0U;
    }
}

void FOC_EstimSMO_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec)
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

    if ((motor == 0) || (out == 0)) return;

    /* Step 0: 确定反Park角度（5.5节） */
    theta_bemf = (motor->state.control_phase == (uint8_t)FOC_CONTROL_PHASE_STARTUP)
        ? motor->electrical_phase_angle
        : motor->estim_smo_state.pll_angle_rad;

    /* Step 1: Clarke + 反Park */
    Math_ClarkeTransform(motor->sensor_fast.current_a.output_value,
                         motor->sensor_fast.current_b.output_value,
                         motor->sensor_fast.current_c.output_value,
                         &i_alpha_meas, &i_beta_meas);

    u_alpha = motor->ud * cosf(theta_bemf) - motor->uq * sinf(theta_bemf);
    u_beta  = motor->ud * sinf(theta_bemf) + motor->uq * cosf(theta_bemf);

    /* 参数 */
    Rs    = fabsf(motor->phase_resistance);
    Ls    = fabsf(motor->stator_inductance);
    if (Ls < 1e-9f) Ls = 1e-9f;
    inv_L = 1.0f / Ls;

    valid_dt = (dt_sec > 0.0f) ? 1U : 0U;
    if (valid_dt == 0U) return;

    /* Step 2: SMO电流估计 */
    err_alpha = motor->estim_smo_state.ialpha_est - i_alpha_meas;
    err_beta  = motor->estim_smo_state.ibeta_est  - i_beta_meas;

    /* 保存上一周期 z 用于旋转方向判断 */
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

    /* Step 3: PLL（无LPF，直接使用z，公式 5.3.3） */
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

    /* ========== Step 4: 多指标收敛检测（5.6节） ========== */

    /* --- 4a. BEMF幅度 --- */
    bemf_mag = sqrtf(motor->estim_smo_state.z_alpha * motor->estim_smo_state.z_alpha
                   + motor->estim_smo_state.z_beta  * motor->estim_smo_state.z_beta);

    /* --- 4b. BEMF旋转一致性（叉积方向） --- */
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

    /* --- 收敛判决 --- */
    if ((bemf_mag > FOC_ESTIM_SMO_CONVERGE_BEMF_V) &&
        (motor->estim_smo_state.rot_dir_counter >= FOC_ESTIM_SMO_ROT_DIR_CONSECUTIVE))
    {
        /* BEMF≥阈值 + 旋转一致 → 递增收敛计数 */
        motor->estim_smo_state.converge_counter++;
        motor->estim_smo_state.lock_counter = 0U;               /* lock失败次数清零 */
    }
    else
    {
        /* 一旦任一必要条件不满足，回退到 INIT（计数归零） */
        motor->estim_smo_state.converge_counter = 0U;
        motor->estim_smo_state.lock_counter++;
        if (motor->estim_smo_state.lock_counter > FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
        {
            out->state = FOC_ESTIMATOR_STATE_DIVERGED;
        }
        else
        {
            out->state = FOC_ESTIMATOR_STATE_INIT;
        }

        /* 直接跳转到 Step 5 输出（不进入 LOCKED 判定） */
        goto output;
    }

    /* 收敛计数 - 状态判定 */
    if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_LOCK_CONSECUTIVE)
    {
        /* 超100次 → 已达 LOCKED（可考虑速度一致性判定升级） */
        out->state = FOC_ESTIMATOR_STATE_LOCKED;
    }
    else if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
    {
        out->state = FOC_ESTIMATOR_STATE_CONVERGING;
    }
    else
    {
        out->state = FOC_ESTIMATOR_STATE_INIT;
    }

output:
    /* Step 5: 输出到 est_state */
    out->elec_angle_rad   = motor->estim_smo_state.pll_angle_rad;
    out->elec_speed_rad_s = motor->estim_smo_state.pll_speed_rad_s;

    if (motor->pole_pairs > 0U)
    {
        out->mech_angle_rad  = motor->estim_smo_state.pll_angle_rad / (float)motor->pole_pairs;
        out->mech_speed_rad_s = motor->estim_smo_state.pll_speed_rad_s / (float)motor->pole_pairs;
    }
    else
    {
        out->mech_angle_rad  = 0.0f;
        out->mech_speed_rad_s = 0.0f;
    }

    out->valid  = (out->state >= FOC_ESTIMATOR_STATE_CONVERGING) ? 1U : 0U;
    out->source = FOC_ESTIMATOR_TYPE_SMO;
}

#endif