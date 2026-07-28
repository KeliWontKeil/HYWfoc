#ifndef FOC_CTRL_TRANSITION_H
#define FOC_CTRL_TRANSITION_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

void FOC_Transition_OnSourceSwitch(foc_motor_t *motor, uint8_t new_source, uint8_t old_source);
void FOC_Transition_OnModeSwitch(foc_motor_t *motor, uint8_t new_mode, uint8_t old_mode);

#endif /* FOC_CTRL_TRANSITION_H */
