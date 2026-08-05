#ifndef FOC_SENSOR_H

#define FOC_SENSOR_H


#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_filter_types.h"

typedef struct foc_motor_t foc_motor_t;

/* ========== Sensor data snapshot ========== */
typedef struct {
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_A) current_a;
    float current_a_zero_offset;
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_B) current_b;
    float current_b_zero_offset;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_C) current_c;
    float current_c_zero_offset;
#endif
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_ANGLE)     mech_angle_rad;
    float prev_mech_angle_rad;
    float mech_speed_rad_s;
    uint8_t mech_speed_valid;
    FOC_FILTER_TYPEDEF(FOC_FILTER_ENCODER_SPEED) encoder_speed_filter;
    float speed_window[FOC_ENCODER_SPEED_WINDOW_SIZE];
    uint8_t speed_window_pos;
    uint8_t speed_window_count;
    struct {
        float raw;
        float filtered;
    } vbus;
    uint8_t adc_valid;
    uint8_t encoder_valid;
    uint8_t vbus_valid;
} sensor_data_t;

void Sensor_Init(uint8_t pwm_freq_kHz, float adc_sample_offset_percent);
void Sensor_InitSnapshot(sensor_data_t *out);
void Sensor_ReadCurrent(sensor_data_t *out);
void Sensor_ReadEncoder(sensor_data_t *out, float dt_sec);
void Sensor_ReadVBUS(sensor_data_t *out);
void Sensor_SetZeroOffset(sensor_data_t *out);
void Sensor_ADCSampleTimeOffset(float percent);

#endif /* FOC_SENSOR_H */