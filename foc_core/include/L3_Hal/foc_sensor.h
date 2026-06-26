#ifndef FOC_SENSOR_H

#define FOC_SENSOR_H


#include "L2_Core/foc_ctrl_types.h"
#include "LS_Config/foc_config.h"


/* Function prototypes */

/*
 * Sensor_Init: configure PWM frequency and sample timing.
 * Motor's sensor snapshots must already be initialized via Sensor_InitSnapshot.
 */
void Sensor_Init(uint8_t pwm_freq_kHz, float adc_sample_offset_percent);

/*
 * Sensor_InitSnapshot: initialize Kalman/state fields in a caller-provided sensor_data_t buffer.
 * Must be called once per snapshot buffer before first use.
 * Typically called for motor->sensor and motor->sensor_fast after motor allocation.
 */
void Sensor_InitSnapshot(sensor_data_t *out);

/*
 * Sensor_ReadCurrent: read phase currents from ADC and write to motor->sensor_fast.
 * This is the single current-sampling entry point, called from PWM ISR only.
 * Zero offsets are applied internally.
 */
void Sensor_ReadCurrent(foc_motor_t *motor);

/*
 * Sensor_ReadEncoder: read mechanical angle from encoder.
 * Results written to the caller-provided snapshot (out).
 * When FOC_SENSOR_ANGLE_FAST_ENABLE is ENABLE, called from PWM ISR;
 * otherwise called from Control ISR.
 */
void Sensor_ReadEncoder(foc_motor_t *motor, sensor_data_t *out);

/*
 * Sensor_ReadVBUS: read VBUS voltage.
 * Always called from Control ISR (slow-rate channel).
 * Results written to the caller-provided snapshot.
 */
void Sensor_ReadVBUS(sensor_data_t *out);

/*
 * Sensor_SetZeroOffset: sample stationary phase currents to obtain DC offset.
 * Stores the result in motor->sensor_zero_offset_*.
 */
void Sensor_SetZeroOffset(foc_motor_t *motor);

/*
 * Sensor_AccumulateEcycle: accumulate phase-current samples for electrical-cycle
 * drift offset estimation. Called after each fast current read (ISR path).
 * Angle reference:
 *   - When FOC_SENSOR_ANGLE_FAST_ENABLE: taken from current_snapshot->mech_angle_rad
 *   - Otherwise: taken from motor->ecycle_ref_angle_rad (volatile bridge)
 * When FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE, this is a no-op.
 */
void Sensor_AccumulateEcycle(foc_motor_t *motor, const sensor_data_t *current_snapshot);

/*
 * Sensor_SyncCurrentSnapshot: copy sensor_fast current values to motor->sensor.
 * Direct copy only — no filtering or averaging.  Intended for consumers that
 * need a readable current snapshot outside the ISR (e.g. DebugStream).
 */
void Sensor_SyncCurrentSnapshot(foc_motor_t *motor);

void Sensor_ADCSampleTimeOffset(float percent);

float Sensor_GetVBUSVoltage(const sensor_data_t *snapshot);
uint8_t Sensor_IsVBUSValid(const sensor_data_t *snapshot);

#endif /* FOC_SENSOR_H */
