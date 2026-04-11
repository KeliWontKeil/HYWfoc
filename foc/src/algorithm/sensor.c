#include "algorithm/sensor.h"

#include <math.h>

#include "interface/foc_platform_api.h"
#include "config/foc_config.h"
#include "algorithm/math_transforms.h"

/* Private variables */
static sensor_data_t sensor_data;
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_sensor_angle_lpf_state_valid = 0U;
static float g_sensor_angle_lpf_state = 0.0f;
#endif

/* Private function prototypes */
static void Sensor_ReadADC(uint8_t use_fast_window);
static void Sensor_ReadEncoder(void);
static void Kalman_Init(kalman_filter_t* filter,
                        float measurement_error,
                        float estimate_error,
                        float process_noise,
                        float initial_value);
#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update(kalman_filter_t* filter, float measurement);
#endif
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update_Angle(kalman_filter_t* filter, float measurement);
#endif
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static float Sensor_UpdateAngleLpf(float measurement);
#endif

/*!
    \brief      Initialize sensor module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Sensor_Init(uint8_t pwm_freq_kHz,float adc_sample_offset_percent)
{
    FOC_Platform_SensorInputInit(pwm_freq_kHz);

    /* Initialize scalar Kalman filters: (R, P0, Q, x0). */
    Kalman_Init(&sensor_data.current_a,
                FOC_SENSOR_KALMAN_CURRENT_A_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_A_INIT);
    Kalman_Init(&sensor_data.current_b,
                FOC_SENSOR_KALMAN_CURRENT_B_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_B_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_B_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_B_INIT);
    Kalman_Init(&sensor_data.mech_angle_rad,
                FOC_SENSOR_KALMAN_ANGLE_MEAS_ERR,
                FOC_SENSOR_KALMAN_ANGLE_EST_ERR,
                FOC_SENSOR_KALMAN_ANGLE_PROC_NOISE,
                FOC_SENSOR_KALMAN_ANGLE_INIT);

    /* Initialize status flags */
    sensor_data.adc_valid = 0;
    sensor_data.encoder_valid = 0;

    Sensor_SetZeroOffset();
    Sensor_ADCSampleTimeOffset(adc_sample_offset_percent);

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
    g_sensor_angle_lpf_state = FOC_SENSOR_KALMAN_ANGLE_INIT;
    g_sensor_angle_lpf_state_valid = 0U;
#endif

}

void Sensor_SetZeroOffset(void)
{
    uint16_t i;
    uint16_t valid_samples = 0U;
    float current_a = 0.0f;
    float current_b = 0.0f;
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float avg_a;
    float avg_b;
    float min_a = 1e9f;
    float max_a = -1e9f;
    float min_b = 1e9f;
    float max_b = -1e9f;
    float spread_a;
    float spread_b;

    for (i = 0U; i < SENSOR_ZERO_CALIB_SAMPLES; i++)
    {
        if (FOC_Platform_ReadPhaseCurrentAB(&current_a, &current_b) != 0U)
        {
            sum_a += current_a;
            sum_b += current_b;
            valid_samples++;

            if (current_a < min_a)
            {
                min_a = current_a;
            }
            if (current_a > max_a)
            {
                max_a = current_a;
            }
            if (current_b < min_b)
            {
                min_b = current_b;
            }
            if (current_b > max_b)
            {
                max_b = current_b;
            }
        }
        FOC_Platform_WaitMs(1U);
    }

    if (valid_samples < SENSOR_ZERO_CALIB_MIN_VALID_SAMPLES)
    {
        sensor_data.current_a.zero_offset = 0.0f;
        sensor_data.current_b.zero_offset = 0.0f;
        return;
    }

    avg_a = sum_a / (float)valid_samples;
    avg_b = sum_b / (float)valid_samples;
    spread_a = max_a - min_a;
    spread_b = max_b - min_b;

    /*
     * Accept startup zero calibration when samples are stable and within a safe
     * absolute range. This avoids false rejection caused by sensor bias.
     */
    if ((fabsf(avg_a) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT) &&
        (fabsf(avg_b) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT) &&
        (spread_a <= SENSOR_ZERO_CALIB_MAX_SPREAD_CURRENT) &&
        (spread_b <= SENSOR_ZERO_CALIB_MAX_SPREAD_CURRENT))
    {
        sensor_data.current_a.zero_offset = avg_a;
        sensor_data.current_b.zero_offset = avg_b;
    }
    else
    {
        sensor_data.current_a.zero_offset = 0.0f;
        sensor_data.current_b.zero_offset = 0.0f;
    }

}

/*!
    \brief      Read all sensor data (ADC + Encoder)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Sensor_ReadAll(void)
{
    Sensor_ReadADC(0U);
    Sensor_ReadEncoder();
}

void Sensor_ReadCurrentOnly(void)
{
    Sensor_ReadADC(1U);
}

/*!
    \brief      Read ADC current measurements
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void Sensor_ReadADC(uint8_t use_fast_window)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
    float common_mode;
    uint8_t read_ok;

    if (use_fast_window != 0U)
    {
        read_ok = FOC_Platform_ReadPhaseCurrentABFast(&current_a, &current_b);
    }
    else
    {
        read_ok = FOC_Platform_ReadPhaseCurrentAB(&current_a, &current_b);
    }

    if (read_ok != 0U)
    {
#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
        /* Apply Kalman filtering */
        Kalman_Update(&sensor_data.current_a, current_a);
        Kalman_Update(&sensor_data.current_b, current_b);
#else
        sensor_data.current_a.raw_value = current_a;
        sensor_data.current_a.filtered_value = current_a;
        sensor_data.current_b.raw_value = current_b;
        sensor_data.current_b.filtered_value = current_b;
#endif
        /* For 3-phase, we can estimate phase C as -(A + B). */
        sensor_data.current_c.filtered_value = -(sensor_data.current_a.filtered_value + sensor_data.current_b.filtered_value);

        common_mode = (sensor_data.current_a.filtered_value +
                   sensor_data.current_b.filtered_value +
                   sensor_data.current_c.filtered_value) / 3.0f;
        sensor_data.current_a.filtered_value -= common_mode;
        sensor_data.current_b.filtered_value -= common_mode;
        sensor_data.current_c.filtered_value -= common_mode;

        sensor_data.current_a.output_value = sensor_data.current_a.filtered_value - sensor_data.current_a.zero_offset;
        sensor_data.current_b.output_value = sensor_data.current_b.filtered_value - sensor_data.current_b.zero_offset;
        sensor_data.current_c.output_value = -(sensor_data.current_a.output_value + sensor_data.current_b.output_value);

        sensor_data.adc_valid = 1;
    }
    else
    {
        sensor_data.adc_valid = 0;
    }
}

/*!
    \brief      Read encoder angle
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void Sensor_ReadEncoder(void)
{
    float angle_rad;
    float angle_for_output;

    /* Read encoder angle register via decoupled AS5600 interface. */
    if (FOC_Platform_ReadMechanicalAngleRad(&angle_rad) != 0U)
    {
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
        /* Apply Kalman filtering with wraparound compensation for 0-360 discontinuity. */
        Kalman_Update_Angle(&sensor_data.mech_angle_rad, angle_rad);
        angle_for_output = sensor_data.mech_angle_rad.filtered_value;
#else
        sensor_data.mech_angle_rad.raw_value = angle_rad;
        sensor_data.mech_angle_rad.filtered_value = Math_WrapRad(angle_rad);
        angle_for_output = sensor_data.mech_angle_rad.filtered_value;
#endif

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
        angle_for_output = Sensor_UpdateAngleLpf(angle_for_output);
#endif

        sensor_data.mech_angle_rad.output_value = angle_for_output;
        sensor_data.encoder_valid = 1;
    }
    else
    {
        sensor_data.encoder_valid = 0;
    }
}

/*!
    \brief      Get sensor data pointer
    \param[in]  none
    \param[out] none
    \retval     sensor_data_t*: Pointer to sensor data structure
*/
uint8_t Sensor_CopyData(sensor_data_t *out_data)
{
    if (out_data == 0)
    {
        return 0U;
    }

    *out_data = sensor_data;
    return 1U;
}

void Sensor_ADCSampleTimeOffset(float percent)
{
    FOC_Platform_SetSensorSampleOffsetPercent(percent);
}

/*!
    \brief      Initialize Kalman filter
    \param[in]  filter: Kalman filter structure pointer
    \param[in]  measurement_error: Measurement noise
    \param[in]  estimate_error: Initial estimate error
    \param[in]  initial_value: Initial value
    \param[out] none
    \retval     none
*/
static void Kalman_Init(kalman_filter_t* filter, float measurement_error, float estimate_error, float process_noise, float initial_value)
{
    filter->raw_value = initial_value;
    filter->filtered_value = initial_value;
    filter->kalman_gain = 0.9f;
    filter->estimate_error = estimate_error;
    filter->measurement_error = measurement_error;
    filter->process_noise = process_noise;
    filter->zero_offset = 0.0f;  /* Initialize zero offset to 0 */
    filter->output_value = initial_value;
}

/*!
    \brief      Update Kalman filter with new measurement
    \param[in]  filter: Kalman filter structure pointer
    \param[in]  measurement: New measurement value
    \param[out] none
    \retval     none
*/
#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update(kalman_filter_t* filter, float measurement)
{
    float denominator;

    /* Store raw value */
    filter->raw_value = measurement;

    /* Predict covariance: P(k|k-1) = P(k-1|k-1) + Q */
    filter->estimate_error += filter->process_noise;

    /* Calculate denominator for Kalman gain */
    denominator = filter->estimate_error + filter->measurement_error;

    /* Prevent division by zero or very small numbers */
    if (denominator < 1e-6f)
    {
        /* If measurement error is effectively zero, use measurement directly */
        filter->filtered_value = measurement;
        filter->kalman_gain = 1.0f;
        filter->estimate_error = 0.0f;  /* No uncertainty */
        filter->output_value = filter->filtered_value - filter->zero_offset;
        return;
    }

    /* Kalman gain calculation */
    filter->kalman_gain = filter->estimate_error / denominator;

    /* Update estimate */
    filter->filtered_value = filter->filtered_value + filter->kalman_gain * (measurement - filter->filtered_value);

    /* Update estimate error */
    filter->estimate_error = (1.0f - filter->kalman_gain) * filter->estimate_error;

    filter->output_value = filter->filtered_value - filter->zero_offset;  /* Apply zero offset to get final output */
}
#endif

/*!
    \brief      Update Kalman filter for angle with wraparound handling (0-2pi discontinuity).
    \param[in]  filter: Kalman filter structure pointer
    \param[in]  measurement: New measurement value (angle in radians)
    \param[out] none
    \retval     none
    \note       Uses Math_WrapRadDelta to handle 2pi discontinuity without jitter.
*/
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update_Angle(kalman_filter_t* filter, float measurement)
{
    float denominator;
    float err_direct;
    float err_plus_turn;
    float err_minus_turn;
    float selected_measurement;

    if (filter == 0)
    {
        return;
    }

    /*
     * Select the nearest equivalent measurement on the circle to avoid
     * 0/2pi discontinuity (no skip-update branch, so output won't latch at 0).
     */
    err_direct = fabsf(measurement - filter->filtered_value);
    err_plus_turn = fabsf((measurement + FOC_MATH_TWO_PI) - filter->filtered_value);
    err_minus_turn = fabsf((measurement - FOC_MATH_TWO_PI) - filter->filtered_value);

    selected_measurement = measurement;
    if ((err_plus_turn < err_direct) && (err_plus_turn <= err_minus_turn))
    {
        selected_measurement = measurement + FOC_MATH_TWO_PI;
    }
    else if ((err_minus_turn < err_direct) && (err_minus_turn < err_plus_turn))
    {
        selected_measurement = measurement - FOC_MATH_TWO_PI;
    }

    filter->estimate_error += filter->process_noise;
    denominator = filter->estimate_error + filter->measurement_error;

    if (denominator < 1e-6f)
    {
        filter->filtered_value = selected_measurement;
        filter->kalman_gain = 1.0f;
        filter->estimate_error = 0.0f;
    }
    else
    {
        filter->kalman_gain = filter->estimate_error / denominator;
        filter->filtered_value = filter->filtered_value +
                                 filter->kalman_gain * (selected_measurement - filter->filtered_value);
        filter->estimate_error = (1.0f - filter->kalman_gain) * filter->estimate_error;
    }

    /* Publish angle in principal range expected by upper control layers. */
    filter->filtered_value = Math_WrapRad(filter->filtered_value);
    filter->output_value = filter->filtered_value;
}
#endif

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
/*
 * Angle LPF on circular domain: update state by wrapped delta to avoid
 * jumps when crossing 0/2pi.
 */
static float Sensor_UpdateAngleLpf(float measurement)
{
    float alpha;
    float wrapped_measurement;
    float delta;

    alpha = Math_ClampFloat(FOC_SENSOR_ANGLE_LPF_ALPHA, 0.0f, 1.0f);
    wrapped_measurement = Math_WrapRad(measurement);

    if (g_sensor_angle_lpf_state_valid == 0U)
    {
        g_sensor_angle_lpf_state = wrapped_measurement;
        g_sensor_angle_lpf_state_valid = 1U;
        return g_sensor_angle_lpf_state;
    }

    delta = Math_WrapRadDelta(wrapped_measurement - g_sensor_angle_lpf_state);
    g_sensor_angle_lpf_state = Math_WrapRad(g_sensor_angle_lpf_state + alpha * delta);

    return g_sensor_angle_lpf_state;
}
#endif
