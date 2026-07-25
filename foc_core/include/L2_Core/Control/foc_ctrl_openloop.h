#ifndef FOC_CTRL_OPENLOOP_H
#define FOC_CTRL_OPENLOOP_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
void FOC_OpenLoopLowSpeedPolicy_Init(foc_motor_t *motor);
void FOC_OpenLoopLowSpeedPolicy_RunStep(foc_motor_t *motor, float dt_sec);
uint8_t FOC_OpenLoopLowSpeedPolicy_IsComplete(const foc_motor_t *motor);
void FOC_OpenLoopLowSpeedPolicy_Abort(foc_motor_t *motor);
#endif

#endif /* FOC_CTRL_OPENLOOP_H */
