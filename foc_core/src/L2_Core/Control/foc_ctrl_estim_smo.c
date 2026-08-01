#include "L2_Core/Control/foc_ctrl_estim.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_filter_gate.h"
#include "LS_Config/foc_config.h"

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)

/*
 * This project's equivalent sliding injection has the opposite polarity to
 * the physical BEMF angle convention. Keep the observer unchanged and
 * restore physical polarity only for angle extraction.
 */
static float smo_bemf_for_angle(float bemf_component)
{
    return -bemf_component;
}

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

static float smo_pll_speed_limit_elec(const foc_motor_t *motor)
{
    float limit = 100.0f;

    if ((motor != 0) && (motor->params.pole_pairs > 0U))
    {
        limit = FOC_ACCEL_SPEED_LIMIT_HIGH_RAD_S * (float)motor->params.pole_pairs * 2.0f;
        if (limit < 100.0f)
        {
            limit = 100.0f;
        }
    }

    return limit;
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
    motor->estim_smo_state.converged_once   = 0U;
    motor->estim_smo_state.speed_window_pos   = 0U;
    motor->estim_smo_state.speed_window_count = 0U;
    motor->estim_smo_state.angle_history_idx = 0U;
    {
        uint8_t hi;
        for (hi = 0U; hi < FOC_SMO_ANGLE_HISTORY_SIZE; hi++)
            motor->estim_smo_state.pll_angle_history[hi] = motor->estim_smo_state.pll_angle_rad;
    }
#if (FOC_FILTER_SMO_SPEED == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanInit(&motor->estim_smo_state.smo_speed_filter,
                              FOC_FILTER_SMO_SPEED_KALMAN_MEAS_ERR,
                              FOC_FILTER_SMO_SPEED_KALMAN_EST_ERR,
                              FOC_FILTER_SMO_SPEED_KALMAN_PROC_NOISE,
                              FOC_FILTER_SMO_SPEED_KALMAN_INIT);
#elif (FOC_FILTER_SMO_SPEED == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&motor->estim_smo_state.smo_speed_filter, 0.0f);
#else
    motor->estim_smo_state.smo_speed_filter.output_value = 0.0f;
#endif

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
        motor->estim_smo_state.phase_comp_rad = motor->estim_smo_state.pll_angle_rad;
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
    float z_alpha_raw, z_beta_raw;
    float sat_current;
    float Rs, Ls, inv_L;
    float bemf_mag;
    float theta_voltage;
    float ud_voltage;
    float uq_voltage;
    uint8_t valid_dt;

    if (motor == 0) return;

    if (motor->applied_output.valid != 0U)
    {
        ud_voltage = motor->applied_output.ud;
        uq_voltage = motor->applied_output.uq;
        theta_voltage = motor->applied_output.electrical_angle_rad;
    }
    else
    {
        ud_voltage = motor->ctrl.ud;
        uq_voltage = motor->ctrl.uq;
        theta_voltage = (motor->source_mgr_state.active_source == FOC_SOURCE_TYPE_OPENLOOP)
            ? motor->ctrl.electrical_angle_rad
            : motor->estim_smo_state.pll_angle_rad;
    }

#if (FOC_CURRENT_SENSE_PHASES == 2U)
    {
        float ic_comp = -(motor->sensor.current_a.output_value + motor->sensor.current_b.output_value);
        Math_ClarkeTransform(motor->sensor.current_a.output_value,
                             motor->sensor.current_b.output_value,
                             ic_comp,
                             &i_alpha_meas, &i_beta_meas);
    }
#else
    Math_ClarkeTransform(motor->sensor.current_a.output_value,
                         motor->sensor.current_b.output_value,
                         motor->sensor.current_c.output_value,
                         &i_alpha_meas, &i_beta_meas);
#endif
    u_alpha = ud_voltage * FOC_MathLut_Sin(theta_voltage + 0.5f * FOC_MATH_PI)
            - uq_voltage * FOC_MathLut_Sin(theta_voltage);
    u_beta  = ud_voltage * FOC_MathLut_Sin(theta_voltage)
            + uq_voltage * FOC_MathLut_Sin(theta_voltage + 0.5f * FOC_MATH_PI);

    Rs    = fabsf(motor->params.phase_resistance);
    Ls    = fabsf(motor->params.stator_inductance);
    if (Ls < 1e-9f) Ls = 1e-9f;
    inv_L = 1.0f / Ls;

    valid_dt = (dt_sec > 0.0f) ? 1U : 0U;
    if (valid_dt == 0U) return;

    err_alpha = motor->estim_smo_state.ialpha_est - i_alpha_meas;
    err_beta  = motor->estim_smo_state.ibeta_est  - i_beta_meas;

    sat_current = fabsf(FOC_ESTIM_SMO_SAT_CURRENT_A);
    if (sat_current < 1e-6f) sat_current = 1e-6f;

    z_alpha_raw = motor->estim_smo_state.k_slide *
        smo_sat(err_alpha / sat_current);
    z_beta_raw = motor->estim_smo_state.k_slide *
        smo_sat(err_beta / sat_current);

    motor->estim_smo_state.z_alpha = z_alpha_raw;
    motor->estim_smo_state.z_beta  = z_beta_raw;

    motor->estim_smo_state.ialpha_est += dt_sec * inv_L *
        (u_alpha - Rs * motor->estim_smo_state.ialpha_est - motor->estim_smo_state.z_alpha);
    motor->estim_smo_state.ibeta_est  += dt_sec * inv_L *
        (u_beta  - Rs * motor->estim_smo_state.ibeta_est  - motor->estim_smo_state.z_beta);

    /* BEMF LPF: 两种角度方法共用 */
    {
        float lpf_alpha = smo_lpf_alpha(FOC_ESTIM_SMO_BEMF_LPF_FC, dt_sec);

        motor->estim_smo_state.bemf_alpha += lpf_alpha *
            (motor->estim_smo_state.z_alpha - motor->estim_smo_state.bemf_alpha);
        motor->estim_smo_state.bemf_beta  += lpf_alpha *
            (motor->estim_smo_state.z_beta  - motor->estim_smo_state.bemf_beta);
    }

#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    {
        bemf_mag = sqrtf(motor->estim_smo_state.bemf_alpha * motor->estim_smo_state.bemf_alpha
                       + motor->estim_smo_state.bemf_beta  * motor->estim_smo_state.bemf_beta);

        if (bemf_mag > 1e-6f)
        {
            float bemf_alpha_angle = smo_bemf_for_angle(motor->estim_smo_state.bemf_alpha);
            float bemf_beta_angle = smo_bemf_for_angle(motor->estim_smo_state.bemf_beta);
            float angle = FOC_MathLut_Atan2(-bemf_alpha_angle, bemf_beta_angle);

            angle = wrap_2pi(angle);
            motor->estim_smo_state.pll_angle_rad = angle;
            if (bemf_mag > FOC_ESTIM_SMO_CONVERGE_BEMF_V)
            {
                float delta = Math_WrapRadDelta(angle - motor->estim_smo_state.phase_comp_rad);
                float raw_speed = delta / dt_sec;
                float speed_alpha = smo_lpf_alpha(FOC_ESTIM_SMO_SPEED_LPF_FC, dt_sec);

                motor->estim_smo_state.pll_speed_rad_s += speed_alpha *
                    (raw_speed - motor->estim_smo_state.pll_speed_rad_s);
            }
            else
            {
                motor->estim_smo_state.pll_speed_rad_s = 0.0f;
            }
            motor->estim_smo_state.phase_comp_rad = angle;
        }
        else
        {
            motor->estim_smo_state.pll_speed_rad_s = 0.0f;
            bemf_mag = 0.0f;
        }
        motor->estim_smo_state.pll_integral    = 0.0f;
    }
#else
    {
        float e_theta;
        float pll_speed_limit;
        float bemf_alpha_angle;
        float bemf_beta_angle;
        float cos_theta = FOC_MathLut_Sin(motor->estim_smo_state.pll_angle_rad + 0.5f * FOC_MATH_PI);
        float sin_theta = FOC_MathLut_Sin(motor->estim_smo_state.pll_angle_rad);

        bemf_mag = sqrtf(motor->estim_smo_state.bemf_alpha * motor->estim_smo_state.bemf_alpha
                       + motor->estim_smo_state.bemf_beta  * motor->estim_smo_state.bemf_beta);

        if (bemf_mag > 1e-6f)
        {
            bemf_alpha_angle = smo_bemf_for_angle(motor->estim_smo_state.bemf_alpha);
            bemf_beta_angle = smo_bemf_for_angle(motor->estim_smo_state.bemf_beta);
            e_theta = -(bemf_alpha_angle * cos_theta
                      + bemf_beta_angle  * sin_theta) / bemf_mag;
            e_theta = Math_ClampFloat(e_theta, -1.0f, 1.0f);
        }
        else
        {
            e_theta = 0.0f;
        }

        motor->estim_smo_state.pll_integral += FOC_ESTIM_SMO_PLL_KI_DEFAULT * e_theta * dt_sec;
        pll_speed_limit = smo_pll_speed_limit_elec(motor);
        motor->estim_smo_state.pll_integral =
            Math_ClampFloat(motor->estim_smo_state.pll_integral,
                            -pll_speed_limit,
                             pll_speed_limit);
        motor->estim_smo_state.pll_speed_rad_s =
            FOC_ESTIM_SMO_PLL_KP_DEFAULT * e_theta + motor->estim_smo_state.pll_integral;
        motor->estim_smo_state.pll_speed_rad_s =
            Math_ClampFloat(motor->estim_smo_state.pll_speed_rad_s,
                            -pll_speed_limit,
                             pll_speed_limit);

        motor->estim_smo_state.pll_angle_rad += motor->estim_smo_state.pll_speed_rad_s * dt_sec;
        motor->estim_smo_state.pll_angle_rad = wrap_2pi(motor->estim_smo_state.pll_angle_rad);
    }
#endif

    /* 旋转方向检测 */
    if (fabsf(motor->estim_smo_state.pll_speed_rad_s) > 1e-6f)
    {
        uint8_t dir = (motor->estim_smo_state.pll_speed_rad_s > 0.0f) ? 1U : 0U;
			if (dir == motor->estim_smo_state.rot_dir_last && motor->estim_smo_state.rot_dir_counter <= FOC_ESTIM_SMO_ROT_DIR_CONSECUTIVE)
            motor->estim_smo_state.rot_dir_counter++;
        else if(dir != motor->estim_smo_state.rot_dir_last)
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
        motor->estim_smo_state.converged_once = 1U;
    }
    else
    {
        motor->estim_smo_state.converge_counter = 0U;
			if (motor->estim_smo_state.converged_once != 0U && motor->estim_smo_state.lock_counter < FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
        {
            motor->estim_smo_state.lock_counter++;
        }
    }

    /* S1: 每次 ISR 写入电气角度到环形缓冲（覆盖前捕获旧值作 oldest） */
    {
        uint8_t idx = motor->estim_smo_state.angle_history_idx;
        float oldest_angle = motor->estim_smo_state.pll_angle_history[idx];
        motor->estim_smo_state.pll_angle_history[idx] = motor->estim_smo_state.pll_angle_rad;
        idx++;
        if (idx >= FOC_SMO_ANGLE_HISTORY_SIZE) idx = 0U;
        motor->estim_smo_state.angle_history_idx = idx;

        /* S2: 每 8 个 ISR 步累积电气角度求机械速度 */
        if (idx == 0U && motor->params.pole_pairs > 0U)
        {
            float newest_angle = motor->estim_smo_state.pll_angle_history[FOC_SMO_ANGLE_HISTORY_SIZE - 1U];
            float delta_elec = Math_WrapRadDelta(newest_angle - oldest_angle);
            float total_time_sec = (float)(FOC_SMO_ANGLE_HISTORY_SIZE) / ((float)FOC_CURRENT_LOOP_ISR_FREQ * 1000.0f);
            float raw_mech_speed = (delta_elec / total_time_sec) / (float)motor->params.pole_pairs;

            motor->estim_smo_state.speed_window[motor->estim_smo_state.speed_window_pos] = raw_mech_speed;
            motor->estim_smo_state.speed_window_pos++;
            if (motor->estim_smo_state.speed_window_pos >= FOC_SMO_SPEED_WINDOW_SIZE)
            {
                motor->estim_smo_state.speed_window_pos = 0U;
            }
            if (motor->estim_smo_state.speed_window_count < FOC_SMO_SPEED_WINDOW_SIZE)
            {
                motor->estim_smo_state.speed_window_count++;
            }

            if (motor->estim_smo_state.speed_window_count > 0U)
            {
                uint8_t i;
                float sum = 0.0f;
                for (i = 0U; i < motor->estim_smo_state.speed_window_count; i++)
                {
                    sum += motor->estim_smo_state.speed_window[i];
                }
                float avg_speed = sum / (float)motor->estim_smo_state.speed_window_count;
                motor->estim_smo_state.mech_speed_rad_s =
                    FOC_FilterGate_SMOSpeed(&motor->estim_smo_state.smo_speed_filter, avg_speed);
            }
        }
    }
}

#endif
