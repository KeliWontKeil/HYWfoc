#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "gd32f30x.h"

#define SENSOR_ZERO_CALIB_SAMPLES            200U
#define SENSOR_ZERO_CALIB_NEAR_ZERO_CURRENT  0.15f

/* Sensor data structures */
typedef struct {
    float raw_value;
    float filtered_value;
    float kalman_gain;
    float estimate_error;
    float measurement_error;
    float process_noise;
    float zero_offset;  /* Zero-point calibration offset */
    float output_value;  /* Value after applying zero offset */
} kalman_filter_t;

typedef struct {
    /* ADC current measurements */
    kalman_filter_t current_a;
    kalman_filter_t current_b;
    kalman_filter_t current_c;

    /* AS5600 encoder */
    kalman_filter_t angle_degrees;

    /* Status flags */
    uint8_t adc_valid;
    uint8_t encoder_valid;
} sensor_data_t;

/* Function prototypes */
void Sensor_Init(void);
void Sensor_ReadAll(void);
void Sensor_SetZeroOffset(void);
sensor_data_t* Sensor_GetData(void);
void Sensor_DebugOutput(void);

/* Kalman filter functions */
void Kalman_Init(kalman_filter_t* filter, float measurement_error, float estimate_error, float process_noise, float initial_value);
void Kalman_Update(kalman_filter_t* filter, float measurement);

#endif /* _SENSOR_H_ */

