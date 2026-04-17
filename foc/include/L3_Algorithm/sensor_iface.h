#ifndef _SENSOR_IFACE_H_
#define _SENSOR_IFACE_H_

#include "LS_Config/foc_shared_types.h"

void Sensor_Init(uint8_t pwm_freq_kHz, float adc_sample_offset_percent);
void Sensor_ReadAll(void);
void Sensor_ReadCurrentOnly(void);
void Sensor_SetZeroOffset(void);
void Sensor_ADCSampleTimeOffset(float percent);
uint8_t Sensor_CopyData(sensor_data_t *out_data);

#endif /* _SENSOR_IFACE_H_ */
