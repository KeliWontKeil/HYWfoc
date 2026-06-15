#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "LS_Config/foc_motor_types.h"


#define SENSOR_ZERO_CALIB_SAMPLES            200U
#define SENSOR_ZERO_CALIB_NEAR_ZERO_CURRENT  0.15f
#define SENSOR_ZERO_CALIB_MIN_VALID_SAMPLES  160U
#define SENSOR_ZERO_CALIB_MAX_ABS_CURRENT    2.50f
#define SENSOR_ZERO_CALIB_MAX_SPREAD_CURRENT 2.50f

/* Function prototypes */
void Sensor_Init(uint8_t pwm_freq_kHz,float adc_sample_offset_percent);
void Sensor_ReadAll(void);
void Sensor_ReadCurrentOnly(void);
void Sensor_SetZeroOffset(void);
void Sensor_ADCSampleTimeOffset(float percent);
void Sensor_CopyData(sensor_data_t *out_data);
void Sensor_ReadVBUS(void);
float Sensor_GetVBUSVoltage(void);
uint8_t Sensor_IsVBUSValid(void);

/* Two-phase zero-offset compensation (only applicable to two-phase sampling).
 * For two-phase systems, phase C is reconstructed as -(Ia+Ib) after compensation.
 * Parameters include electrical-cycle dynamic offsets for runtime drift correction. */
void Sensor_CompensateTwoPhaseZeroOffset(float ia_raw, float ib_raw,
                                         float ecycle_off_a, float ecycle_off_b,
                                         uint8_t ecycle_valid,
                                         float *ia_out, float *ib_out,
                                         float *ic_out);

/* Three-phase zero-offset compensation (placeholder).
 * Reserved for future use. Currently a no-op because the compensation
 * strategy for three-phase sampling is not yet determined. */
void Sensor_CompensateThreePhaseZeroOffset(float ia_raw, float ib_raw, float ic_raw,
                                           float *ia_out, float *ib_out,
                                           float *ic_out);

#endif /* _SENSOR_H_ */