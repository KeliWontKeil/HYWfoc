#include "sensor.h"
#include "adc.h"
#include "as5600.h"

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
void Sensor_Init(void)
{
    /* Initialize Kalman filters for ADC currents */
    Kalman_Init(&sensor_data.current_a, 0.1f, 1.0f, 0.0f);
    Kalman_Init(&sensor_data.current_b, 0.1f, 1.0f, 0.0f);
    Kalman_Init(&sensor_data.current_c, 0.1f, 1.0f, 0.0f);

    /* Initialize Kalman filters for encoder */
    Kalman_Init(&sensor_data.angle_raw, 2.0f, 10.0f, 0.0f);
    Kalman_Init(&sensor_data.angle_degrees, 1.0f, 5.0f, 0.0f);

    /* Initialize status flags */
    sensor_data.adc_valid = 0;
    sensor_data.encoder_valid = 0;
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
    float adc_currents[2];

    /* Read ADC current values */
    if (ADC_GetSample(adc_currents, CURRENT) == ADC_STATUS_OK)
    {
        /* Apply Kalman filtering */
        Kalman_Update(&sensor_data.current_a, adc_currents[0]);
        Kalman_Update(&sensor_data.current_b, adc_currents[1]);
        /* For 3-phase, we can estimate phase C as -(A + B) */
        Kalman_Update(&sensor_data.current_c, -(adc_currents[0] + adc_currents[1]));

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
    uint16_t raw_angle;

    /* Read encoder */
    if (AS5600_ReadAngle(&raw_angle) == I2C_OK)
    {
        float angle_raw = (float)raw_angle;
        float angle_degrees = (angle_raw / 4096.0f) * 360.0f;

        /* Apply Kalman filtering */
        Kalman_Update(&sensor_data.angle_raw, angle_raw);
        Kalman_Update(&sensor_data.angle_degrees, angle_degrees);

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

/*!
    \brief      Output sensor data for debugging
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Sensor_DebugOutput(void)
{
    /* This will be called from UART debug module */
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
void Kalman_Init(kalman_filter_t* filter, float measurement_error, float estimate_error, float initial_value)
{
    filter->raw_value = initial_value;
    filter->filtered_value = initial_value;
    filter->kalman_gain = 1.0f;
    filter->estimate_error = estimate_error;
    filter->measurement_error = measurement_error;
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
    /* Store raw value */
    filter->raw_value = measurement;

    /* Kalman gain calculation */
    filter->kalman_gain = filter->estimate_error / (filter->estimate_error + filter->measurement_error);

    /* Update estimate */
    filter->filtered_value = filter->filtered_value + filter->kalman_gain * (measurement - filter->filtered_value);

    /* Update estimate error */
    filter->estimate_error = (1.0f - filter->kalman_gain) * filter->estimate_error;
}