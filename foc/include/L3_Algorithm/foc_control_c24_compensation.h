#ifndef FOC_CONTROL_C24_COMPENSATION_H
#define FOC_CONTROL_C24_COMPENSATION_H

#include "LS_Config/foc_motor_types.h"

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)

/*
 * =====================================================================
 *  Compensation lookup & application (only present when COMP_ENABLE)
 * =====================================================================
 */
float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float mech_angle_rad,
                                 float speed_ref_rad_s);

void FOC_ControlApplyCoggingCompensation(foc_motor_t *motor,
                                         float mech_angle_rad,
                                         float speed_ref_rad_s);

uint8_t FOC_ControlLoadCoggingCompTableQ15(foc_motor_t *motor,
                                           const int16_t *table_q15,
                                           uint16_t point_count,
                                           float iq_lsb_a,
                                           uint8_t source);

void FOC_ControlSetCoggingCompUnavailable(foc_motor_t *motor, uint8_t source);

#endif /* FOC_COGGING_COMP_ENABLE */

/*
 * =====================================================================
 *  Calibration API – always declared (stubs when CALIB disabled)
 * =====================================================================
 */

/* 1 if calibration in progress (safe inline for all build configs). */
static inline uint8_t FOC_CoggingCalibIsBusy(const foc_motor_t *motor)
{
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    return motor->cogging_calib_state.in_progress;
#else
    (void)motor;
    return 0U;
#endif
}

/* Progress percentage [0..100] (safe inline for all build configs). */
static inline uint8_t FOC_CoggingCalibGetProgressPercent(const foc_motor_t *motor)
{
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    return motor->cogging_calib_state.progress_percent;
#else
    (void)motor;
    return 0U;
#endif
}

/*
 * Calibration sampling step.
 * Always compiled (stub returns 0 when CALIB disabled) so callers
 * in foc_app.c do not need #if guards.
 */
uint8_t FOC_CoggingCalibSampleStep(foc_motor_t *motor,
                                   const sensor_data_t *sensor,
                                   float dt_sec);

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

void FOC_CoggingCalibRequestStart(void);
uint8_t FOC_CoggingCalibStart(foc_motor_t *motor);

uint8_t FOC_CoggingCalibIsDumpPending(void);
uint8_t FOC_CoggingCalibIsExportPending(void);
void   FOC_CoggingCalibClearDumpPending(void);
void   FOC_CoggingCalibClearExportPending(void);

void FOC_CoggingCalibRequestDump(void);
void FOC_CoggingCalibRequestExport(void);
void FOC_CoggingCalibDumpTable(const foc_motor_t *motor);
void FOC_CoggingCalibExportTable(const foc_motor_t *motor);

#endif /* FOC_COGGING_CALIB_ENABLE */

#endif /* FOC_CONTROL_C24_COMPENSATION_H */
