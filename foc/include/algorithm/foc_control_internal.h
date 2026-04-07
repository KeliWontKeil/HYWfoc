#ifndef _FOC_CONTROL_INTERNAL_H_
#define _FOC_CONTROL_INTERNAL_H_

#include "config/foc_shared_types.h"

float FOC_ControlMechanicalToElectricalAngle(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlApplyElectricalAngle(foc_motor_t *motor, float electrical_angle);

#endif /* _FOC_CONTROL_INTERNAL_H_ */
