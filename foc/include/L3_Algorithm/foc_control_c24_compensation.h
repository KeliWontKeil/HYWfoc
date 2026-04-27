#ifndef FOC_CONTROL_C24_COMPENSATION_H
#define FOC_CONTROL_C24_COMPENSATION_H

#include "LS_Config/foc_shared_types.h"

/* Compensation source identifiers. */
#define FOC_COGGING_COMP_SOURCE_DISABLED 0U
#define FOC_COGGING_COMP_SOURCE_NONE 1U
#define FOC_COGGING_COMP_SOURCE_STATIC 2U
#define FOC_COGGING_COMP_SOURCE_CALIB 3U

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
 * This is the protocol-safe entry point: it sets a static request flag
 * so that the next call to FOC_CoggingCalibProcess() (which has the
 * motor pointer) can call FOC_CoggingCalibStart(motor) internally.
 *
 * The actual start validation happens on the control tick.
 */
void FOC_CoggingCalibRequestStart(void);

/*
 * Start the user-triggered runtime cogging calibration (direct).
 *
 * The motor must already be spinning at low speed in speed-closed / current-open
 * loop (velocity mode).  This start function resets the state machine; actual
 * work is done in FOC_CoggingCalibProcess() which should be called from the
 * fast control loop (FOC_TASK_RATE_FAST_CONTROL).
 *
 * Returns 1 on acceptance, 0 if already busy or conditions not met.
 */
uint8_t FOC_CoggingCalibStart(foc_motor_t *motor);

/*
 * Run one step of the calibration state machine.
 *
 * Must be called from the fast control loop while FOC_CoggingCalibIsBusy()
 * returns true.  The calibration:
 *   1. Opens the current loop and applies a fixed Iq baseline.
 *   2. Sweeps through each LUT point by injecting a small velocity offset.
 *   3. Averages the measured Iq at each mechanical angle to build the LUT.
 *   4. Repeats for FOC_COGGING_CALIB_NUM_PASSES passes.
 *   5. When complete, loads the averaged table into motor->cogging_comp_table_q15
 *      and sets status.source = FOC_COGGING_COMP_SOURCE_CALIB.
 *
 * Returns 1 while busy, 0 when finished.
 */
uint8_t FOC_CoggingCalibProcess(foc_motor_t *motor);

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
 * Output format is the same as the calibration completion auto-dump.
 * Safe to call from L2 protocol layer (uses FOC_Platform_WriteDebugText).
 */
void FOC_CoggingCalibDumpTable(void);

#endif /* FOC_COGGING_CALIB_ENABLE */

#endif /* FOC_CONTROL_C24_COMPENSATION_H */