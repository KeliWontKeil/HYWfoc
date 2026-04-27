#include "L3_Algorithm/sensor.h"

#include <math.h>

#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* Private variables */
static sensor_data_t sensor_data;
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_sensor_angle_lpf_state_valid = 0U;
static float g_sensor_angle_lpf_state = 0.0f;
#endif

/*
 * DC offset drift tracker (slow LPF / HPF-equivalent).
 *
 * The startup calibration (zero_offset) captures the initial ADC offset.
 * During operation, the analog offset may drift due to temperature or PWM
 * switching noise. This tracker continuously tracks the residual DC on each
 * phase's output (after startup zero_offset removal) and subtracts it.
 *
 * A very low alpha ensures only sub-Hz drift is tracked, leaving the
 * fundamental AC current unaffected. Time constant ≈ dt / alpha.
 *   dt ≈ 83 us (12 kHz current loop)
 *   alpha = 1e-4f → tau ≈ 0.83 s, fc ≈ 0.19 Hz
 *   alpha = 1e-5f → tau ≈ 8.3 s, fc ≈ 0.019 Hz
 */
#define FOC_DC_DRIFT_TRACK_ALPHA 5e-5f
static float g_dc_drift_a = 0.0f;
static float g_dc_drift_b = 0.0f;
static uint8_t g_dc_drift_initialized = 0U;

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
    Sensor_ReadVBUS();
}

void Sensor_ReadCurrentOnly(void)
{
    Sensor_ReadADC(1U);
}

float Sensor_MedianFiltrer(float *Input_Values, uint16_t len)
{
    uint16_t i, j;
    float temp = 0.0f;
    float values[len];

    /* Simple bubble sort to find the median */
    for (i = 0; i < len - 1; i++)
    {
        for (j = 0; j < len - i - 1; j++)
        {
            if (values[j] > values[j + 1])
            {
                temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }

    return values[len / 2];
}

#define SENSOR_MEDIAN_FILTER_LEN 5
float current[2][SENSOR_MEDIAN_FILTER_LEN];

/*Read ADC current measurements*/
static void Sensor_ReadADC(uint8_t use_fast_window)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
    uint8_t read_ok;

    if (use_fast_window != 0U)
    {
        read_ok = FOC_Platform_ReadPhaseCurrentABFast(&current_a, &current_b);
        for(uint8_t i = SENSOR_MEDIAN_FILTER_LEN - 1; i > 0; i--)
        {
            current[0][i] = current[0][i - 1];
            current[1][i] = current[1][i - 1];
        }
        current[0][0] = current_a;
        current[1][0] = current_b;
    }
    else
    {
        read_ok = FOC_Platform_ReadPhaseCurrentAB(&current_a, &current_b);
    }

    //current_a = Sensor_MedianFiltrer(current[0], SENSOR_MEDIAN_FILTER_LEN);
    //current_b = Sensor_MedianFiltrer(current[1], SENSOR_MEDIAN_FILTER_LEN);


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

        /*
         * Apply startup zero_offset calibration.
         *
         * NOTE: The original common-mode cancellation using
         *   common_mode = (A + B + C) / 3
         * is REMOVED because C is reconstructed as -(A+B), making the sum
         * identically zero. The common-mode term was always zero regardless
         * of actual ADC offset — rendering the subtraction completely
         * ineffective as a DC-blocking mechanism.
         */
        sensor_data.current_a.output_value = sensor_data.current_a.filtered_value - sensor_data.current_a.zero_offset;
        sensor_data.current_b.output_value = sensor_data.current_b.filtered_value - sensor_data.current_b.zero_offset;

        /*
         * DC drift tracker (slow LPF on output samples).
         *
         * The startup zero_offset captures the initial ADC bias, but the bias
         * may drift over time (temperature, PWM noise). This tracker continuously
         * estimates the residual DC on each phase using a first-order LPF with
         * a very low alpha, then subtracts it from the output.
         *
         * Time constant: tau ≈ dt/alpha ≈ 83us/5e-5 ≈ 1.66 s  (fc ≈ 0.1 Hz)
         * Only sub-Hz drift is tracked; the motor's fundamental AC is unaffected.
         */
        if (g_dc_drift_initialized == 0U)
        {
            g_dc_drift_a = sensor_data.current_a.output_value;
            g_dc_drift_b = sensor_data.current_b.output_value;
            g_dc_drift_initialized = 1U;
        }
        else
        {
            g_dc_drift_a += FOC_DC_DRIFT_TRACK_ALPHA * (sensor_data.current_a.output_value - g_dc_drift_a);
            g_dc_drift_b += FOC_DC_DRIFT_TRACK_ALPHA * (sensor_data.current_b.output_value - g_dc_drift_b);
        }

        sensor_data.current_a.output_value -= g_dc_drift_a;
        sensor_data.current_b.output_value -= g_dc_drift_b;

        /* Reconstruct phase C from corrected A and B. */
        sensor_data.current_c.output_value = -(sensor_data.current_a.output_value + sensor_data.current_b.output_value);

        sensor_data.adc_valid = 1;
    }
    else
    {
        sensor_data.adc_valid = 0;
    }
}

/* Read encoder angle */
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

/* VBUS voltage sampling with moving average filter */
void Sensor_ReadVBUS(void)
{
    float vbus_raw;

    if (FOC_Platform_ReadVbusVoltage(&vbus_raw) != 0U)
    {
        sensor_data.vbus_voltage_raw = vbus_raw;
        if(sensor_data.vbus_valid == 0U)
        {
            sensor_data.vbus_voltage_filtered = vbus_raw;
            sensor_data.vbus_valid = 1U;
        }        

        sensor_data.vbus_voltage_filtered = Math_FirstOrderLpf(vbus_raw,
                                                                &sensor_data.vbus_voltage_filtered,
                                                                0.1F,
                                                                &sensor_data.vbus_valid);
    }
}

void Sensor_CopyData(sensor_data_t *out_data)
{
    *out_data = sensor_data;
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

float Sensor_GetVBUSVoltage(void)
{
    return sensor_data.vbus_voltage_filtered;
}

