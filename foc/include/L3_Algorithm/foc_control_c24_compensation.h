#ifndef FOC_CONTROL_C24_COMPENSATION_H
#define FOC_CONTROL_C24_COMPENSATION_H

#include "LS_Config/foc_shared_types.h"

/* Lookup and apply compensation from the cogging LUT. */
float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float mech_angle_rad,
                                 float speed_ref_rad_s);

void FOC_ControlApplyCoggingCompensation(foc_motor_t *motor,
                                         float mech_angle_rad,
                                         float speed_ref_rad_s);

/* Load an externally-prepared Q15 table into the motor compensation LUT. */
uint8_t FOC_ControlLoadCoggingCompTableQ15(foc_motor_t *motor,
                                           const int16_t *table_q15,
                                           uint16_t point_count,
                                           float iq_lsb_a,
                                           uint8_t source);

/* Mark compensation as unavailable with a given reason code. */
void FOC_ControlSetCoggingCompUnavailable(foc_motor_t *motor, uint8_t source);

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

/*
 * Request-start the runtime cogging calibration (deferred).
 *
 * Sets a static request flag consumed by FOC_CoggingCalibSampleStep()
 * on the next control tick.
 */
void FOC_CoggingCalibRequestStart(void);

/*
 * Start the user-triggered runtime cogging calibration (direct).
 *
 * Resets accumulators and enters START phase.
 * Calibration is self-driven: maintains its own electrical angle and drives
 * Uq = I_calib * R_phase directly (no outer-loop PID, no current-loop PID).
 * The caller must set current_soft_switch to OPEN mode with enabled=0
 * before the next PWM ISR tick.
 *
 * Returns 1 on acceptance, 0 if already busy.
 */
uint8_t FOC_CoggingCalibStart(foc_motor_t *motor);

/*
 * Run one step of the calibration sample collection state machine.
 *
 * Must be called from the control task when FOC_CoggingCalibIsBusy() is true.
 * This function:
 *   1. Consumes deferred request flags (start/dump/export).
 *   2. When busy, runs the open-loop drive step AND advances the
 *      START/SETTLE/SCAN/CHECK/FINISH state machine.
 *   3. In SCAN phase: computes Δθ = θ_actual - θ_expected, maps to
 *      LUT bin by actual mechanical angle, accumulates position error.
 *   4. Self-drives motor->electrical_phase_angle, motor->uq, motor->iq_target
 *      and calls FOC_ControlApplyElectricalAngleRuntime() for SVPWM output.
 *   5. Does NOT call FOC_App_RunControlAlgorithm() - the caller must
 *      bypass the outer-loop PID when this function is active.
 *
 * Returns 1 while busy, 0 when idle/finished.
 */
uint8_t FOC_CoggingCalibSampleStep(foc_motor_t *motor,
                                   const sensor_data_t *sensor,
                                   float dt_sec);

/* Returns 1 if calibration is currently in progress. */
static inline uint8_t FOC_CoggingCalibIsBusy(const foc_motor_t *motor)
{
    return motor->cogging_comp_status.calib_in_progress;
}

/* Returns progress percentage [0..100]. */
static inline uint8_t FOC_CoggingCalibGetProgressPercent(const foc_motor_t *motor)
{
    return motor->cogging_comp_status.calib_progress_percent;
}

/*
 * Dump the current cogging compensation table via debug text stream.
 */
void FOC_CoggingCalibDumpTable(void);

/*
 * Export the current cogging compensation LUT as a C-code initializer string.
 */
void FOC_CoggingCalibExportTable(void);

#endif /* FOC_COGGING_CALIB_ENABLE */

#endif /* FOC_CONTROL_C24_COMPENSATION_H */
