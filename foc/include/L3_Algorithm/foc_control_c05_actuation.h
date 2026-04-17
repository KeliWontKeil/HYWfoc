#ifndef FOC_CONTROL_C05_ACTUATION_H
#define FOC_CONTROL_C05_ACTUATION_H

#include "LS_Config/foc_shared_types.h"

float FOC_ControlMechanicalToElectricalAngle(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle);
void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle);

#endif /* FOC_CONTROL_C05_ACTUATION_H */
