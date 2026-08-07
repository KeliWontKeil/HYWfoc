#include "L2_Core/Control/foc_ctrl_estim_smo.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_filter_gate.h"
#include "LS_Config/foc_config.h"

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)

static float wrap_2pi(float angle_rad)
{
    while (angle_rad > FOC_MATH_TWO_PI)  angle_rad -= FOC_MATH_TWO_PI;
    while (angle_rad < 0.0f)             angle_rad += FOC_MATH_TWO_PI;
    return angle_rad;
}

static float smo_lpf_alpha(float fc_hz, float dt_sec)
{
    float alpha = FOC_MATH_TWO_PI * fc_hz * dt_sec;
    return Math_ClampFloat(alpha, 0.0f, 1.0f);
}

static float smo_sat(float value)
{
    return Math_ClampFloat(value, -1.0f, 1.0f);
}

#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)

static float smo_pll_speed_limit_elec(const foc_motor_params_t *params)
{
    float limit = 100.0f;

    if ((params != 0) && (params->pole_pairs > 0U))
    {
        limit = FOC_ACCEL_SPEED_LIMIT_HIGH_RAD_S * (float)params->pole_pairs * 2.0f;
        if (limit < 100.0f)
        {
            limit = 100.0f;
        }
    }

    return limit;
}

#endif

void FOC_EstimSMO_Init(foc_estim_smo_state_t *state, const foc_motor_params_t *params)
{
    state->ialpha_est     = 0.0f;
    state->ibeta_est      = 0.0f;
    state->bemf_alpha     = 0.0f;
    state->bemf_beta      = 0.0f;
    state->z_alpha        = 0.0f;
    state->z_beta         = 0.0f;
    state->pll_angle_rad  = 0.0f;
    state->pll_speed_rad_s = 0.0f;
    state->k_slide        = FOC_ESTIM_SMO_K_SLIDE_DEFAULT;
    state->converge_counter = 0U;
    state->lock_counter   = 0U;
    state->rot_dir_counter = 0U;
    state->rot_dir_last   = 0U;
    state->converged_once   = 0U;
    state->speed_window_pos   = 0U;
    state->speed_window_count = 0U;
    state->angle_history_idx = 0U;
    state->speed_dt_accum = 0.0f;

#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
    state->pll_integral   = 0.0f;
#endif
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    state->phase_comp_rad = 0.0f;
#endif

    {
        uint8_t hi;
        for (hi = 0U; hi < FOC_SMO_ANGLE_HISTORY_SIZE; hi++)
            state->pll_angle_history[hi] = state->pll_angle_rad;
    }
#if (FOC_FILTER_SMO_SPEED == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanInit(&state->smo_speed_filter,
                              FOC_FILTER_SMO_SPEED_KALMAN_MEAS_ERR,
                              FOC_FILTER_SMO_SPEED_KALMAN_EST_ERR,
                              FOC_FILTER_SMO_SPEED_KALMAN_PROC_NOISE,
                              FOC_FILTER_SMO_SPEED_KALMAN_INIT);
#elif (FOC_FILTER_SMO_SPEED == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&state->smo_speed_filter, 0.0f);
#else
    state->smo_speed_filter.output_value = 0.0f;
#endif

    if ((params->direction != FOC_DIR_UNDEFINED) &&
        (params->mech_angle_at_elec_zero_rad != FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) &&
        (params->pole_pairs > 0U))
    {
        float mech_zero = params->mech_angle_at_elec_zero_rad;
        if (params->direction == FOC_DIR_REVERSED)
        {
            mech_zero = FOC_MATH_TWO_PI - mech_zero;
        }
        state->pll_angle_rad = mech_zero * (float)params->pole_pairs;
        state->pll_angle_rad = wrap_2pi(state->pll_angle_rad);
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
        state->phase_comp_rad = state->pll_angle_rad;
#endif
        state->initialized   = 1U;
    }
    else
    {
        state->pll_angle_rad = 0.0f;
        state->initialized   = 0U;
    }
}

/* ================================================================
 * 观测器核心：电流方程 + BEMF LPF + 收敛状态机 + 旋转方向检测。
 * ================================================================ */
static void EstimSMO_StepCore(foc_estim_smo_state_t *state,
                              const foc_motor_params_t *params,
                              const sensor_data_t *sensor,
                              const foc_applied_output_state_t *applied,
                              const foc_control_runtime_t *ctrl,
                              uint8_t active_source,
                              uint8_t control_region,
                              float dt_sec,
                              float *bemf_mag_out)
{
    float i_alpha_meas, i_beta_meas;
    float u_alpha, u_beta;
    float err_alpha, err_beta;
    float z_alpha_raw, z_beta_raw;
    float sat_current;
    float Rs, Ls, inv_L;
    float theta_voltage;
    float ud_voltage;
    float uq_voltage;

    *bemf_mag_out = 0.0f;

    if (applied->valid != 0U)
    {
        ud_voltage = applied->ud;
        uq_voltage = applied->uq;
        theta_voltage = applied->electrical_angle_rad;
    }
    else
    {
        ud_voltage = ctrl->ud;
        uq_voltage = ctrl->uq;
        theta_voltage = (active_source == FOC_SOURCE_TYPE_OPENLOOP)
            ? ctrl->electrical_angle_rad
            : state->pll_angle_rad;
    }

#if (FOC_CURRENT_SENSE_PHASES == 2U)
    {
        float ic_comp = -(sensor->current_a.output_value + sensor->current_b.output_value);
        Math_ClarkeTransform(sensor->current_a.output_value,
                             sensor->current_b.output_value,
                             ic_comp,
                             &i_alpha_meas, &i_beta_meas);
    }
#else
    Math_ClarkeTransform(sensor->current_a.output_value,
                         sensor->current_b.output_value,
                         sensor->current_c.output_value,
                         &i_alpha_meas, &i_beta_meas);
#endif
    u_alpha = ud_voltage * FOC_MathLut_Sin(theta_voltage + 0.5f * FOC_MATH_PI)
            - uq_voltage * FOC_MathLut_Sin(theta_voltage);
    u_beta  = ud_voltage * FOC_MathLut_Sin(theta_voltage)
            + uq_voltage * FOC_MathLut_Sin(theta_voltage + 0.5f * FOC_MATH_PI);

    Rs    = fabsf(params->phase_resistance);
    Ls    = fabsf(params->stator_inductance);
    if (Ls < 1e-9f) Ls = 1e-9f;
    inv_L = 1.0f / Ls;

    if (dt_sec <= 0.0f) return;

    err_alpha = state->ialpha_est - i_alpha_meas;
    err_beta  = state->ibeta_est  - i_beta_meas;

    sat_current = fabsf(FOC_ESTIM_SMO_SAT_CURRENT_A);
    if (sat_current < 1e-6f) sat_current = 1e-6f;

    z_alpha_raw = state->k_slide *
        smo_sat(err_alpha / sat_current);
    z_beta_raw = state->k_slide *
        smo_sat(err_beta / sat_current);

    state->z_alpha = z_alpha_raw;
    state->z_beta  = z_beta_raw;

    state->ialpha_est += dt_sec * inv_L *
        (u_alpha - Rs * state->ialpha_est - state->z_alpha);
    state->ibeta_est  += dt_sec * inv_L *
        (u_beta  - Rs * state->ibeta_est  - state->z_beta);

    /* BEMF LPF: 实际滑模注入 z 与物理 BEMF 反极性，这里统一输出物理 BEMF(=-z) */
    {
        float lpf_alpha = smo_lpf_alpha(FOC_ESTIM_SMO_BEMF_LPF_FC, dt_sec);

        state->bemf_alpha += lpf_alpha *
            (-state->z_alpha - state->bemf_alpha);
        state->bemf_beta  += lpf_alpha *
            (-state->z_beta  - state->bemf_beta);
    }

    *bemf_mag_out = sqrtf(state->bemf_alpha * state->bemf_alpha
                        + state->bemf_beta  * state->bemf_beta);

    /* 旋转方向检测 */
    if (fabsf(state->pll_speed_rad_s) > 1e-6f)
    {
        uint8_t dir = (state->pll_speed_rad_s > 0.0f) ? 1U : 0U;
        if (dir == state->rot_dir_last && state->rot_dir_counter <= FOC_ESTIM_SMO_ROT_DIR_CONSECUTIVE)
            state->rot_dir_counter++;
        else if(dir != state->rot_dir_last)
        {
            state->rot_dir_counter = 0U;
            state->rot_dir_last    = dir;
        }
    }

    /*
     * 预收敛速度门限：LOW 区域且机械速度低于门限时不累计收敛计数，
     * 避免停转或低速时 SMO 误收敛。核心观测器/PLL 始终运行，
     * 仅收敛状态机被门控。
     */
    {
        uint8_t allow_converge = 1U;
        if (control_region == FOC_CONTROL_REGION_LOW)
        {
            float mech_speed = (params->pole_pairs > 0U) ?
                fabsf(state->pll_speed_rad_s / (float)params->pole_pairs) : 0.0f;
            if (mech_speed < FOC_SMO_ACCEL_PRECONV_SPEED_THRESHOLD_RAD_S)
            {
                allow_converge = 0U;
            }
        }

        if ((allow_converge != 0U) &&
            (*bemf_mag_out > FOC_ESTIM_SMO_CONVERGE_BEMF_V) &&
            (state->rot_dir_counter >= FOC_ESTIM_SMO_ROT_DIR_CONSECUTIVE))
        {
            state->converge_counter++;
            state->lock_counter = 0U;
            state->converged_once = 1U;
        }
        else
        {
            state->converge_counter = 0U;
            if (state->converged_once != 0U && state->lock_counter < FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
            {
                state->lock_counter++;
            }
        }
    }
}

/* ================================================================
 * PLL 角度提取（仅 PLL 方法编译）：跟踪 bemf 角，私有字段 pll_integral。
 * ================================================================ */
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
static void EstimSMO_ExtractAnglePLL(foc_estim_smo_state_t *state,
                                     const foc_motor_params_t *params,
                                     float dt_sec, float bemf_mag)
{
    float e_theta;
    float pll_speed_limit;
    float cos_theta = FOC_MathLut_Sin(state->pll_angle_rad + 0.5f * FOC_MATH_PI);
    float sin_theta = FOC_MathLut_Sin(state->pll_angle_rad);

    if (bemf_mag > 1e-6f)
    {
        /* bemf 为物理 BEMF，锁相需 sin 相关取负使 θe 为稳定平衡点 */
        e_theta = (float)params->direction * (-state->bemf_alpha * cos_theta
                   - state->bemf_beta  * sin_theta) / bemf_mag;
        e_theta = Math_ClampFloat(e_theta, -1.0f, 1.0f);
    }
    else
    {
        e_theta = 0.0f;
    }

    state->pll_integral += FOC_ESTIM_SMO_PLL_KI_DEFAULT * e_theta * dt_sec;
    pll_speed_limit = smo_pll_speed_limit_elec(params);
    state->pll_integral =
        Math_ClampFloat(state->pll_integral,
                        -pll_speed_limit,
                         pll_speed_limit);
    state->pll_speed_rad_s =
        FOC_ESTIM_SMO_PLL_KP_DEFAULT * e_theta + state->pll_integral;
    state->pll_speed_rad_s =
        Math_ClampFloat(state->pll_speed_rad_s,
                        -pll_speed_limit,
                         pll_speed_limit);

    state->pll_angle_rad += state->pll_speed_rad_s * dt_sec;
    state->pll_angle_rad = wrap_2pi(state->pll_angle_rad);
}
#endif

/* ================================================================
 * LPF_ATAN2 角度提取（仅 ANGLE_METHOD_LPF_ATAN2 方法编译）：atan2 + 差分测速，
 * 私有字段 phase_comp_rad。
 * ================================================================ */
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
static void EstimSMO_ExtractAngleAtan2(foc_estim_smo_state_t *state,
                                       const foc_motor_params_t *params,
                                       float dt_sec, float bemf_mag)
{
    if (bemf_mag > 1e-6f)
    {
        /* bemf 为物理 BEMF，atan2 标准式直接得物理电角 θe；
         * 乘以 direction 归一化到 ctrl 参考系，与 encoder/电流环/源切换同系 */
        float angle = FOC_MathLut_Atan2(-state->bemf_alpha,
                                        state->bemf_beta);

        angle = angle * (float)params->direction;
        angle = wrap_2pi(angle);
        state->pll_angle_rad = angle;
        if (bemf_mag > FOC_ESTIM_SMO_CONVERGE_BEMF_V)
        {
            float delta = Math_WrapRadDelta(angle - state->phase_comp_rad);
            float raw_speed = delta / dt_sec;
            float speed_alpha = smo_lpf_alpha(FOC_ESTIM_SMO_SPEED_LPF_FC, dt_sec);

            state->pll_speed_rad_s += speed_alpha *
                (raw_speed - state->pll_speed_rad_s);
        }
        else
        {
            state->pll_speed_rad_s = 0.0f;
        }
        state->phase_comp_rad = angle;
    }
    else
    {
        state->pll_speed_rad_s = 0.0f;
    }
}
#endif

/* ================================================================
 * 测速尾链：环形缓冲 + 机械速度滤波（两路径共用）。
 * ================================================================ */
static void EstimSMO_StepTail(foc_estim_smo_state_t *state,
                              const foc_motor_params_t *params,
                              float dt_sec)
{
    uint8_t idx;
    float oldest_angle;

    idx = state->angle_history_idx;
    oldest_angle = state->pll_angle_history[idx];
    state->pll_angle_history[idx] = state->pll_angle_rad;
    idx++;
    if (idx >= FOC_SMO_ANGLE_HISTORY_SIZE) idx = 0U;
    state->angle_history_idx = idx;

    state->speed_dt_accum += dt_sec;

    if (idx == 0U && params->pole_pairs > 0U)
    {
        float newest_angle = state->pll_angle_history[FOC_SMO_ANGLE_HISTORY_SIZE - 1U];
        float delta_elec = Math_WrapRadDelta(newest_angle - oldest_angle);
        float total_time_sec = state->speed_dt_accum;
        state->speed_dt_accum = 0.0f;

        if (total_time_sec > 0.0f)
        {
            float raw_mech_speed = (delta_elec / total_time_sec) / (float)params->pole_pairs;

            state->speed_window[state->speed_window_pos] = raw_mech_speed;
            state->speed_window_pos++;
            if (state->speed_window_pos >= FOC_SMO_SPEED_WINDOW_SIZE)
            {
                state->speed_window_pos = 0U;
            }
            if (state->speed_window_count < FOC_SMO_SPEED_WINDOW_SIZE)
            {
                state->speed_window_count++;
            }

            if (state->speed_window_count > 0U)
            {
                uint8_t i;
                float sum = 0.0f;
                for (i = 0U; i < state->speed_window_count; i++)
                {
                    sum += state->speed_window[i];
                }
                float avg_speed = sum / (float)state->speed_window_count;
                state->mech_speed_rad_s =
                    FOC_FilterGate_SMOSpeed(&state->smo_speed_filter, avg_speed);
            }
        }
    }
}

void FOC_EstimSMO_Step(foc_estim_smo_state_t *state,
                       const foc_motor_params_t *params,
                       const sensor_data_t *sensor,
                       const foc_applied_output_state_t *applied,
                       const foc_control_runtime_t *ctrl,
                       uint8_t active_source,
                       uint8_t control_region,
                       float dt_sec)
{
    float bemf_mag;

    if (dt_sec <= 0.0f) return;

    /* 观测器核心 */
    EstimSMO_StepCore(state, params, sensor, applied, ctrl, active_source,
                      control_region, dt_sec, &bemf_mag);

    /* 角度提取（编译期互斥） */
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
    EstimSMO_ExtractAnglePLL(state, params, dt_sec, bemf_mag);
#elif (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    EstimSMO_ExtractAngleAtan2(state, params, dt_sec, bemf_mag);
#else
#error "Unsupported FOC_ESTIM_SMO_ANGLE_METHOD"
#endif

    /* 测速尾链 */
    EstimSMO_StepTail(state, params, dt_sec);
}

#endif
