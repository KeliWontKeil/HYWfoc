#ifndef FOC_CTRL_SOURCE_OPENLOOP_H
#define FOC_CTRL_SOURCE_OPENLOOP_H

#include "L2_Core/foc_ctrl_types.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
void FOC_OpenLoopSource_Init(foc_motor_t *motor);
void FOC_OpenLoopSource_Step(foc_motor_t *motor, foc_active_source_state_t *out, float dt_sec);
#endif

#endif /* FOC_CTRL_SOURCE_OPENLOOP_H */
