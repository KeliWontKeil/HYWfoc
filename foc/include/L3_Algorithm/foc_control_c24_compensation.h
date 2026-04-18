#ifndef FOC_CONTROL_C24_COMPENSATION_H
#define FOC_CONTROL_C24_COMPENSATION_H

#include "LS_Config/foc_shared_types.h"

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

void FOC_ControlInitCoggingCompensation(foc_motor_t *motor);

#endif /* FOC_CONTROL_C24_COMPENSATION_H */
