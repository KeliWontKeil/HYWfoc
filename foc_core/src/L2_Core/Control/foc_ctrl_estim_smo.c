#include "L2_Core/Control/foc_ctrl_estim_smo.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_filter_gate.h"
#include "LS_Config/foc_config.h"

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)

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
    float ls;

    state->ialpha_est     = 0.0f;
    state->ibeta_est      = 0.0f;
    state->bemf_alpha     = 0.0f;
    state->bemf_beta      = 0.0f;
    state->z_alpha        = 0.0f;
    state->z_beta         = 0.0f;
    state->pll_angle_rad  = 0.0f;
    state->pll_speed_rad_s = 0.0f;
    state->k_slide        = FOC_ESTIM_SMO_K_SLIDE_DEFAULT;
    state->converged      = 0U;
    state->lost_count     = 0U;

    /* 运行期不变派生量缓存（替代每周期重算的除法/乘加） */
    state->rs_ohms = fabsf(params->phase_resistance);
    ls = fabsf(params->stator_inductance);
    if (ls < 1e-9f)
    {
        ls = 1e-9f;
    }
    state->inv_l_1h = 1.0f / ls;
    state->sat_current_a = fabsf(FOC_ESTIM_SMO_SAT_CURRENT_A);
    if (state->sat_current_a < 1e-6f)
    {
        state->sat_current_a = 1e-6f;
    }
    state->bemf_lpf_alpha = smo_lpf_alpha(FOC_ESTIM_SMO_BEMF_LPF_FC, FOC_CURRENT_LOOP_DT_SEC);
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    state->speed_lpf_alpha = smo_lpf_alpha(FOC_ESTIM_SMO_SPEED_LPF_FC, FOC_CURRENT_LOOP_DT_SEC);
#endif

#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
    state->pll_integral   = 0.0f;
#endif
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    state->phase_comp_rad = 0.0f;
#endif

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
        /* pll_angle 存储控制系电角：定位后电机位于 mech_zero，控制系电角=0 */
        state->pll_angle_rad = 0.0f;
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
                              const foc_control_runtime_t *ctrl,
                              float u_alpha,
                              float u_beta,
                              uint8_t active_source,
                              uint8_t control_region,
                              float dt_sec,
                              float *bemf_mag_out,
                              uint8_t *sample_ok_out)
{
    float i_alpha_meas, i_beta_meas;
    float err_alpha, err_beta;
    float z_alpha_raw, z_beta_raw;
    float sat_current;
    float inv_L;
    uint8_t sample_ok;

    *bemf_mag_out = 0.0f;
    (void)active_source;

    /* 电流 αβ 与电压 αβ 均由上游单点计算/发布，此处直接消费（消除重复 Clarke/逆 Park） */
    i_alpha_meas = ctrl->ialpha;
    i_beta_meas  = ctrl->ibeta;

    /* Rs/Ls/inv_L/sat_current 为运行期不变派生量，Init 时缓存（消除每周期重算） */
    inv_L = state->inv_l_1h;

    if (dt_sec <= 0.0f) return;

    err_alpha = state->ialpha_est - i_alpha_meas;
    err_beta  = state->ibeta_est  - i_beta_meas;

    sat_current = state->sat_current_a;

    z_alpha_raw = state->k_slide *
        smo_sat(err_alpha / sat_current);
    z_beta_raw = state->k_slide *
        smo_sat(err_beta / sat_current);

    state->z_alpha = z_alpha_raw;
    state->z_beta  = z_beta_raw;

    state->ialpha_est += dt_sec * inv_L *
        (u_alpha - state->rs_ohms * state->ialpha_est - state->z_alpha);
    state->ibeta_est  += dt_sec * inv_L *
        (u_beta  - state->rs_ohms * state->ibeta_est  - state->z_beta);

    /* BEMF LPF: 标准滑模观测器等效控制 z ≈ BEMF（同号），直接作为 BEMF 输出 */
    {
        float lpf_alpha = state->bemf_lpf_alpha;

        state->bemf_alpha += lpf_alpha *
            (state->z_alpha - state->bemf_alpha);
        state->bemf_beta  += lpf_alpha *
            (state->z_beta  - state->bemf_beta);
    }

    *bemf_mag_out = sqrtf(state->bemf_alpha * state->bemf_alpha
                        + state->bemf_beta  * state->bemf_beta);

    /*
     * 收敛/失锁判定（允许偶发毛刺）：
     * - sample_ok：bemf 幅值充足 且 与估计速度一致 且 非 LOW 低速门控。
     *   一致性判据（bemf_mag ≥ RATIO·|pll_speed|）区分"真收敛"（bemf=Ke·ω）与
     *   "假收敛"（堵转时 bemf 极小但 pll_speed 虚高），不依赖外部参考源。
     * - 每拍 sample_ok 置 converged 并清零 lost_count；否则已收敛后 lost_count
     *   连续递增，达到 LOST_THRESHOLD 才判失效（源管理据此降级）。
     * - 观测器/PLL 始终运行，仅状态判定被门控。
     */
    {
        sample_ok = 1U;

        if (control_region == FOC_CONTROL_REGION_LOW)
        {
            float mech_speed = (params->pole_pairs > 0U) ?
                fabsf(state->pll_speed_rad_s / (float)params->pole_pairs) : 0.0f;
            if (mech_speed < FOC_SMO_ACCEL_PRECONV_SPEED_THRESHOLD_RAD_S)
            {
                sample_ok = 0U;
            }
        }
        if ((sample_ok != 0U) && (*bemf_mag_out <= FOC_ESTIM_SMO_CONVERGE_BEMF_V))
        {
            sample_ok = 0U;
        }
        /* 假收敛检测：bemf 相对估计速度异常小则判无效（宽松下限，始终启用） */
        if ((sample_ok != 0U) &&
            (*bemf_mag_out < FOC_ESTIM_SMO_BEMF_SPEED_RATIO * fabsf(state->pll_speed_rad_s)))
        {
            sample_ok = 0U;
        }

        if (sample_ok != 0U)
        {
            state->lost_count = 0U;
            state->converged = 1U;
        }
        else
        {
            if ((state->converged != 0U) &&
                (state->lost_count < FOC_ESTIM_SMO_LOST_THRESHOLD))
            {
                state->lost_count++;
            }
        }
    }

    if (sample_ok_out != 0)
    {
        *sample_ok_out = sample_ok;
    }
}

/* ================================================================
 * PLL 角度提取（仅 PLL 方法编译）：跟踪 bemf 角，私有字段 pll_integral。
 * ================================================================ */
static void EstimSMO_NormalizeBemf(const foc_estim_smo_state_t *state,
                                   float *bemf_alpha_n, float *bemf_beta_n,
                                   float speed_cmd_rad_s)
{
    float sign = (speed_cmd_rad_s < 0.0f) ? -1.0f : 1.0f;
    *bemf_alpha_n = sign * state->bemf_alpha;
    *bemf_beta_n  = sign * state->bemf_beta;
}

#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
static void EstimSMO_ExtractAnglePLL(foc_estim_smo_state_t *state,
                                     const foc_motor_params_t *params,
                                     float speed_cmd_rad_s,
                                     float dt_sec, float bemf_mag,
                                     uint8_t sample_ok)
{
    float pll_speed_limit;
    uint8_t isolate;

    /* 数据隔离仅针对"已收敛后的偶发失效窗口"；未收敛锁相与降级后重新锁相不隔离，
     * 否则 PLL 冻结会使 SMO 永远无法收敛/恢复（自锁）。 */
    isolate = (uint8_t)((state->converged != 0U) &&
                        (sample_ok == 0U) &&
                        (state->lost_count < FOC_ESTIM_SMO_LOST_THRESHOLD));

    if (isolate == 0U)
    {
        float e_theta;
        float na, nb;
        float cos_theta;
        float sin_theta;

        FOC_MathLut_SinCos(state->pll_angle_rad, &sin_theta, &cos_theta);

        EstimSMO_NormalizeBemf(state, &na, &nb, speed_cmd_rad_s);
        if (bemf_mag > 1e-6f)
        {
            e_theta = (-na * cos_theta
                       - nb  * sin_theta) / bemf_mag;
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
    }

    /* 好样本由上面锁相积分更新；失效窗口内积分/速度冻结，角度仅按最后有效速度惯性外推，
     * 坏样本不拉偏角度，从源头切断"错误角度→错误电压→观测再恶化"的正反馈。 */
    state->pll_angle_rad += state->pll_speed_rad_s * dt_sec;
    state->pll_angle_rad = Math_WrapRad(state->pll_angle_rad);
}
#endif

/* ================================================================
 * LPF_ATAN2 角度提取（仅 ANGLE_METHOD_LPF_ATAN2 方法编译）：atan2 + 差分测速，
 * 私有字段 phase_comp_rad。
 * ================================================================ */
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
static void EstimSMO_ExtractAngleAtan2(foc_estim_smo_state_t *state,
                                       const foc_motor_params_t *params,
                                       float speed_cmd_rad_s,
                                       float dt_sec, float bemf_mag,
                                       uint8_t sample_ok)
{
    float na, nb;
    uint8_t isolate;
    (void)params;

    /* 与 PLL 分支一致：仅"已收敛后的偶发失效窗口"隔离，避免未收敛锁相/降级后恢复自锁 */
    isolate = (uint8_t)((state->converged != 0U) &&
                        (sample_ok == 0U) &&
                        (state->lost_count < FOC_ESTIM_SMO_LOST_THRESHOLD));

    if (isolate != 0U)
    {
        /* 失效窗口：无速度记忆，角度保持（避免 bemf 尖峰直跳角），速度衰减向 0 */
        state->pll_speed_rad_s = 0.0f;
        return;
    }

    EstimSMO_NormalizeBemf(state, &na, &nb, speed_cmd_rad_s);
    if (bemf_mag > 1e-6f)
    {
        /* pll_angle 存储物理系电角（atan2 标准式直接得物理 BEMF 角），
         * 控制系转换在只读窗接口层统一完成 */
        float angle = FOC_MathLut_Atan2(-na, nb);

        angle = Math_WrapRad(angle);
        state->pll_angle_rad = angle;
        if (bemf_mag > FOC_ESTIM_SMO_CONVERGE_BEMF_V)
        {
            float delta = Math_WrapRadDelta(angle - state->phase_comp_rad);
            float raw_speed = delta / dt_sec;
            float speed_alpha = state->speed_lpf_alpha;

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

void FOC_EstimSMO_Step(foc_estim_smo_state_t *state,
                       const foc_motor_params_t *params,
                       const sensor_data_t *sensor,
                       const foc_applied_output_state_t *applied,
                       const foc_alpha_beta_phase_t *alpha_beta,
                       const foc_control_runtime_t *ctrl,
                       uint8_t active_source,
                       uint8_t control_region,
                       float speed_cmd_rad_s,
                       float dt_sec)
{
    float bemf_mag;
    float u_alpha;
    float u_beta;
    uint8_t sample_ok;

    if (dt_sec <= 0.0f) return;
    (void)sensor;
    (void)applied;

    u_alpha = (alpha_beta != 0) ? alpha_beta->alpha : 0.0f;
    u_beta  = (alpha_beta != 0) ? alpha_beta->beta  : 0.0f;

    /* 观测器核心 */
    EstimSMO_StepCore(state, params, ctrl, u_alpha, u_beta, active_source,
                      control_region, dt_sec, &bemf_mag, &sample_ok);

    /* 角度提取（编译期互斥）：坏样本被隔离，仅好样本拉偏角度，切断正反馈 */
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
    EstimSMO_ExtractAnglePLL(state, params, speed_cmd_rad_s, dt_sec, bemf_mag, sample_ok);
#elif (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    EstimSMO_ExtractAngleAtan2(state, params, speed_cmd_rad_s, dt_sec, bemf_mag, sample_ok);
#else
#error "Unsupported FOC_ESTIM_SMO_ANGLE_METHOD"
#endif

    /* 测速：直接采用 PLL/ATAN2 自带电速（除以极对数）；bemf 不足时输入 0，经 LPF 衰减向 0 */
    {
        float raw_elec_speed = 0.0f;
        if (bemf_mag > FOC_ESTIM_SMO_CONVERGE_BEMF_V)
        {
            raw_elec_speed = state->pll_speed_rad_s;
        }
        {
            float raw_mech = (params->pole_pairs > 0U) ?
                (raw_elec_speed / (float)params->pole_pairs) : 0.0f;
            state->mech_speed_rad_s =
                FOC_FilterGate_SMOSpeed(&state->smo_speed_filter, raw_mech);
        }
    }
}

#endif
