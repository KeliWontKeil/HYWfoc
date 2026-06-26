#ifndef FOC_CTRL_COMPENSATION_H
#define FOC_CTRL_COMPENSATION_H

#include "L2_Core/foc_ctrl_types.h"

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
 *  inline queries: delegates to focussed calibration module header
 *  Full calibration module is at foc_ctrl_cogging_calib.h/.c
 * =====================================================================
 */

/* Forward include for the inline definitions */
#include "L2_Core/Control/foc_ctrl_cogging_calib.h"
#endif /* FOC_CONTROL_C24_COMPENSATION_H */
