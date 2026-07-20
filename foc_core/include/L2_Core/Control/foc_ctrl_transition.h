#ifndef FOC_CTRL_TRANSITION_H
#define FOC_CTRL_TRANSITION_H

#include <stdint.h>
#include "L2_Core/foc_ctrl_types.h"

void FOC_Transition_Init(foc_motor_t *motor);
void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec);
uint8_t FOC_Transition_Request(foc_motor_t *motor, uint8_t target_estimator_type);

#endif /* FOC_CTRL_TRANSITION_H */