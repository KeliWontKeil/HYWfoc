#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "foc_shared_types.h"
#include <math.h>


#define SENSOR_ZERO_CALIB_SAMPLES            200U
#define SENSOR_ZERO_CALIB_NEAR_ZERO_CURRENT  0.15f
#define SENSOR_ZERO_CALIB_MIN_VALID_SAMPLES  160U
#define SENSOR_ZERO_CALIB_MAX_ABS_CURRENT    2.50f
#define SENSOR_ZERO_CALIB_MAX_SPREAD_CURRENT 0.80f

/* Function prototypes */
void Sensor_Init(uint8_t pwm_freq_kHz,float adc_sample_offset_percent);
void Sensor_ReadAll(void);
void Sensor_SetZeroOffset(void);
void Sensor_ADCSampleTimeOffset(float percent);
sensor_data_t* Sensor_GetData(void);

/* Kalman filter functions */
void Kalman_Init(kalman_filter_t* filter, float measurement_error, float estimate_error, float process_noise, float initial_value);
void Kalman_Update(kalman_filter_t* filter, float measurement);

#endif /* _SENSOR_H_ */

