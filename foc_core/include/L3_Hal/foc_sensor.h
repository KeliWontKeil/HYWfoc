#ifndef FOC_SENSOR_H

#define FOC_SENSOR_H


#include "L2_Core/foc_ctrl_types.h"
#include "LS_Config/foc_config.h"


void Sensor_Init(uint8_t pwm_freq_kHz, float adc_sample_offset_percent);
void Sensor_InitSnapshot(sensor_data_t *out);
void Sensor_ReadCurrent(foc_motor_t *motor);
void Sensor_ReadEncoder(foc_motor_t *motor, sensor_data_t *out, float dt_sec);
void Sensor_ReadVBUS(sensor_data_t *out);
void Sensor_SetZeroOffset(foc_motor_t *motor);
void Sensor_AccumulateEcycle(foc_motor_t *motor, const sensor_data_t *current_snapshot);
void Sensor_ADCSampleTimeOffset(float percent);

float Sensor_GetVBUSVoltage(const sensor_data_t *snapshot);
uint8_t Sensor_IsVBUSValid(const sensor_data_t *snapshot);

#endif /* FOC_SENSOR_H */
