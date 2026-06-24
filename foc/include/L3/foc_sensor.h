#ifndef FOC_SENSOR_H

#define FOC_SENSOR_H


#include "LS_Config/foc_motor_types.h"
#include "LS_Config/foc_config.h"


#define SENSOR_ZERO_CALIB_SAMPLES            200U
#define SENSOR_ZERO_CALIB_NEAR_ZERO_CURRENT  0.15f
#define SENSOR_ZERO_CALIB_MIN_VALID_SAMPLES  160U
#define SENSOR_ZERO_CALIB_MAX_ABS_CURRENT    2.50f
#define SENSOR_ZERO_CALIB_MAX_SPREAD_CURRENT 2.50f

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
 * Sensor_ReadAll: read all sensors (current slow + encoder + VBUS) into motor->sensor.
 * Zero offsets and angle LPF state are read from the motor struct.
 */
void Sensor_ReadAll(foc_motor_t *motor);

/*
 * Sensor_ReadCurrentSlow: read phase currents with slow (control-cycle) averaging window.
 * Results written to motor->sensor.  Uses motor->sensor_zero_offset_* for correction.
 */
void Sensor_ReadCurrentSlow(foc_motor_t *motor);

/*
 * Sensor_ReadCurrentFast: read phase currents with fast (current-loop ISR) averaging window.
 * Results written to motor->sensor_fast.  Uses motor->sensor_zero_offset_* for correction.
 */
void Sensor_ReadCurrentFast(foc_motor_t *motor);

/*
 * Sensor_SetZeroOffset: sample stationary phase currents to obtain DC offset.
 * Stores the result in motor->sensor_zero_offset_*.
 */
void Sensor_SetZeroOffset(foc_motor_t *motor);

/*
 * Sensor_AccumulateEcycle: accumulate phase-current samples for electrical-cycle
 * drift offset estimation.  Called after each fast current read (ISR path).
 * Uses motor->sensor.encoder_valid to detect rotation; the current snapshot
 * provides the per-frame filtered values.
 * When FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE, this is a no-op.
 */
void Sensor_AccumulateEcycle(foc_motor_t *motor, const sensor_data_t *current_snapshot);

void Sensor_ADCSampleTimeOffset(float percent);

float Sensor_GetVBUSVoltage(const sensor_data_t *snapshot);
uint8_t Sensor_IsVBUSValid(const sensor_data_t *snapshot);

#endif /* FOC_SENSOR_H */
