#ifndef FOC_CTRL_TRANSITION_H
#define FOC_CTRL_TRANSITION_H

#include <stdint.h>
#include "L2_Core/foc_ctrl_types.h"

void FOC_Transition_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source);
void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec);

#endif /* FOC_CTRL_TRANSITION_H */
