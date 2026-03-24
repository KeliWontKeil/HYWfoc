#include "sensor.h"

#define SENSOR_ANGLE_TO_RAD (2.0f * 3.1415926535f / 4096.0f)

/* Private variables */
static sensor_data_t sensor_data;

/* Private function prototypes */
static void Sensor_ReadADC(void);
static void Sensor_ReadEncoder(void);

/*!
    \brief      Initialize sensor module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Sensor_Init(uint8_t pwm_freq_kHz)
{
    FOC_Platform_SensorInputInit(pwm_freq_kHz);

    /* Initialize scalar Kalman filters: (R, P0, Q, x0). */
    Kalman_Init(&sensor_data.current_a, 0.15f, 0.0f, 0.025f, 0.0f);
    Kalman_Init(&sensor_data.current_b, 0.15f, 0.0f, 0.02f, 0.0f);
    Kalman_Init(&sensor_data.mech_angle_rad, 1.0f, 5.0f, 0.05f, 0.0f);

    /* Initialize status flags */
    sensor_data.adc_valid = 0;
    sensor_data.encoder_valid = 0;

    Sensor_SetZeroOffset();
    Sensor_ADCSampleTimeOffset(96.0f);

}

void Sensor_SetZeroOffset(void)
{
    uint16_t i;
    float current_a = 0.0f;
    float current_b = 0.0f;
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float avg_a;
    float avg_b;

    for (i = 0; i < SENSOR_ZERO_CALIB_SAMPLES; i++)
    {
        if (FOC_Platform_ReadPhaseCurrentAB(&current_a, &current_b) != 0U)
        {
            sum_a += current_a;
            sum_b += current_b;
        }
        FOC_Platform_WaitMs(1U);
    }

    avg_a = sum_a / (float)SENSOR_ZERO_CALIB_SAMPLES;
    avg_b = sum_b / (float)SENSOR_ZERO_CALIB_SAMPLES;

    /* Avoid calibrating with real current present at startup. */
    if ((fabsf(avg_a) <= SENSOR_ZERO_CALIB_NEAR_ZERO_CURRENT) &&
        (fabsf(avg_b) <= SENSOR_ZERO_CALIB_NEAR_ZERO_CURRENT))
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
    Sensor_ReadADC();
    Sensor_ReadEncoder();
}

/*!
    \brief      Read ADC current measurements
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void Sensor_ReadADC(void)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
    float zero_offset;

    /* Use averaged ADC values over one control period (default 24 samples at 24kHz). */
    if (FOC_Platform_ReadPhaseCurrentAB(&current_a, &current_b) != 0U)
    {
        /* Apply Kalman filtering */
        Kalman_Update(&sensor_data.current_a, current_a);
        Kalman_Update(&sensor_data.current_b, current_b);
        /* For 3-phase, we can estimate phase C as -(A + B) */
        sensor_data.current_c.output_value = -(sensor_data.current_a.output_value + sensor_data.current_b.output_value);

        zero_offset = sensor_data.current_a.filtered_value + sensor_data.current_b.filtered_value + sensor_data.current_c.filtered_value;
        sensor_data.current_a.filtered_value -= (zero_offset / 3.0f);
        sensor_data.current_b.filtered_value -= (zero_offset / 3.0f);
        sensor_data.current_c.filtered_value -= (zero_offset / 3.0f);

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
    uint16_t angle;
    float angle_rad;

    /* Read encoder angle register via decoupled AS5600 interface. */
    if (FOC_Platform_ReadEncoderRawAngle(&angle) != 0U)
    {
        angle_rad = (float)angle * SENSOR_ANGLE_TO_RAD;

        /* Apply Kalman filtering */
        //Kalman_Update(&sensor_data.mech_angle_rad, angle_rad);
        angle_rad -= fmodf(angle_rad, 0.002);
        sensor_data.mech_angle_rad.raw_value = angle_rad;
        sensor_data.mech_angle_rad.filtered_value = angle_rad;
        sensor_data.mech_angle_rad.output_value = angle_rad;

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
sensor_data_t* Sensor_GetData(void)
{
    return &sensor_data;
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
void Kalman_Init(kalman_filter_t* filter, float measurement_error, float estimate_error, float process_noise, float initial_value)
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
void Kalman_Update(kalman_filter_t* filter, float measurement)
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
