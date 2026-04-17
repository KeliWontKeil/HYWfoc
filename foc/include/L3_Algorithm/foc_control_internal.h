#ifndef _FOC_CONTROL_INTERNAL_H_
#define _FOC_CONTROL_INTERNAL_H_

#include <stdint.h>

#include "L3_Algorithm/foc_control.h"
#include "LS_Config/foc_shared_types.h"

float FOC_ControlMechanicalToElectricalAngle(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle);
void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle);

foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatusMutable(void);
uint8_t *FOC_ControlGetCurrentSoftSwitchBlendInitFlag(void);
void FOC_ControlGetCoggingCompContext(const foc_cogging_comp_status_t **status,
									  const int16_t **table_q15);

#endif /* _FOC_CONTROL_INTERNAL_H_ */
