#ifndef FOC_CONTROL_C31_ACTUATION_H
#define FOC_CONTROL_C31_ACTUATION_H

#include "LS_Config/foc_shared_types.h"

float FOC_ControlMechanicalToElectricalAngle(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle);
void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle);
uint8_t FOC_SampleLockedMechanicalAngle(foc_motor_t *motor,
                                        float electrical_angle,
                                        uint16_t settle_ms,
                                        uint16_t sample_count,
                                        float *mech_angle_rad);

#endif /* FOC_CONTROL_C31_ACTUATION_H */
