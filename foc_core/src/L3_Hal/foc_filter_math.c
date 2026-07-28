#include "L3_Hal/foc_filter_math.h"

#include <math.h>

#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

/* ====== Kalman filter (standard 1-D) ====== */

void FOC_FilterMath_KalmanInit(foc_filter_kalman_t *f, float meas_err,
                               float est_err, float proc_noise, float init_val)
{
    if (f == 0) return;

    f->raw_value = init_val;
    f->filtered_value = init_val;
    f->kalman_gain = 0.9f;
    f->estimate_error = est_err;
    f->measurement_error = meas_err;
    f->process_noise = proc_noise;
    f->zero_offset = 0.0f;
    f->output_value = init_val;
}

float FOC_FilterMath_KalmanStep(foc_filter_kalman_t *f, float input)
{
    float denominator;

    if (f == 0) return input;

    f->raw_value = input;
    f->estimate_error += f->process_noise;

    denominator = f->estimate_error + f->measurement_error;
    if (denominator < 1e-6f)
    {
        f->filtered_value = input;
        f->kalman_gain = 1.0f;
        f->estimate_error = 0.0f;
        f->output_value = f->filtered_value;
        return f->filtered_value;
    }

    f->kalman_gain = f->estimate_error / denominator;
    f->filtered_value = f->filtered_value + f->kalman_gain * (input - f->filtered_value);
    f->estimate_error = (1.0f - f->kalman_gain) * f->estimate_error;
    f->output_value = f->filtered_value;

    return f->filtered_value;
}

/* ====== Angle Kalman filter (handles 0/2pi wrap) ====== */

void FOC_FilterMath_KalmanAngleInit(foc_filter_kalman_t *f, float meas_err,
                                     float est_err, float proc_noise, float init_val)
{
    if (f == 0) return;

    f->raw_value = init_val;
    f->filtered_value = Math_WrapRad(init_val);
    f->kalman_gain = 0.9f;
    f->estimate_error = est_err;
    f->measurement_error = meas_err;
    f->process_noise = proc_noise;
    f->zero_offset = 0.0f;
    f->output_value = f->filtered_value;
}

float FOC_FilterMath_KalmanAngleStep(foc_filter_kalman_t *f, float input)
{
    float denominator;
    float err_direct;
    float err_plus_turn;
    float err_minus_turn;
    float selected_measurement;

    if (f == 0) return input;

    err_direct = fabsf(input - f->filtered_value);
    err_plus_turn = fabsf((input + FOC_MATH_TWO_PI) - f->filtered_value);
    err_minus_turn = fabsf((input - FOC_MATH_TWO_PI) - f->filtered_value);

    selected_measurement = input;
    if ((err_plus_turn < err_direct) && (err_plus_turn <= err_minus_turn))
    {
        selected_measurement = input + FOC_MATH_TWO_PI;
    }
    else if ((err_minus_turn < err_direct) && (err_minus_turn < err_plus_turn))
    {
        selected_measurement = input - FOC_MATH_TWO_PI;
    }

    f->estimate_error += f->process_noise;
    denominator = f->estimate_error + f->measurement_error;

    if (denominator < 1e-6f)
    {
        f->filtered_value = selected_measurement;
        f->kalman_gain = 1.0f;
        f->estimate_error = 0.0f;
    }
    else
    {
        f->kalman_gain = f->estimate_error / denominator;
        f->filtered_value = f->filtered_value +
                            f->kalman_gain * (selected_measurement - f->filtered_value);
        f->estimate_error = (1.0f - f->kalman_gain) * f->estimate_error;
    }

    f->filtered_value = Math_WrapRad(f->filtered_value);
    f->output_value = f->filtered_value;

    return f->filtered_value;
}

/* ====== First-order LPF (standard) ====== */

void FOC_FilterMath_Lpf1Init(foc_filter_lpf1_t *f, float init_val)
{
    if (f == 0) return;

    f->raw_value = init_val;
    f->state = init_val;
    f->output_value = init_val;
    f->valid = 0U;
}

float FOC_FilterMath_Lpf1Step(foc_filter_lpf1_t *f, float input, float alpha)
{
    if (f == 0) return input;

    f->raw_value = input;

    if (f->valid == 0U)
    {
        f->state = input;
        f->output_value = input;
        f->valid = 1U;
        return f->output_value;
    }

    f->state = Math_FirstOrderLpf(input, &f->state, alpha, &f->valid);
    f->output_value = f->state;

    return f->output_value;
}

/* ====== First-order LPF for angle (handles 0/2pi wrap via three-way measurement unwrap) ====== */

float FOC_FilterMath_Lpf1AngleStep(foc_filter_lpf1_t *f, float input, float alpha)
{
    float best_input;
    float err_direct;
    float err_plus;
    float err_minus;

    if (f == 0) return input;

    f->raw_value = input;

    if (f->valid == 0U)
    {
        f->state = Math_WrapRad(input);
        f->output_value = f->state;
        f->valid = 1U;
        return f->output_value;
    }

    /* 三路测量展开：选择离当前状态最近的 input 投影，消除 0/2π 边界双稳态 */
    err_direct = fabsf(input - f->state);
    err_plus   = fabsf((input + FOC_MATH_TWO_PI) - f->state);
    err_minus  = fabsf((input - FOC_MATH_TWO_PI) - f->state);

    best_input = input;
    if (err_plus < err_direct && err_plus <= err_minus)
        best_input = input + FOC_MATH_TWO_PI;
    else if (err_minus < err_direct && err_minus < err_plus)
        best_input = input - FOC_MATH_TWO_PI;

    /* 标准一阶 LPF，然后卷回 [0, 2π) */
    f->state = Math_WrapRad(f->state + alpha * (best_input - f->state));
    f->output_value = f->state;

    return f->output_value;
}

/* ====== Biquad IIR filter (skeleton, not yet implemented) ====== */

void FOC_FilterMath_BiquadInit(foc_filter_biquad_t *f, const float *coeff)
{
    if (f == 0) return;

    f->b0 = coeff[0];
    f->b1 = coeff[1];
    f->b2 = coeff[2];
    f->a1 = coeff[3];
    f->a2 = coeff[4];
    f->x1 = 0.0f;
    f->x2 = 0.0f;
    f->y1 = 0.0f;
    f->y2 = 0.0f;
    f->valid = 0U;
    f->output_value = 0.0f;
}

float FOC_FilterMath_BiquadStep(foc_filter_biquad_t *f, float input)
{
    /* skeleton: passthrough, algorithm not implemented */
    if (f == 0) return input;

    f->output_value = input;

    return f->output_value;
}
