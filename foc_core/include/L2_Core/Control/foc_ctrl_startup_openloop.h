#ifndef FOC_CTRL_STARTUP_OPENLOOP_H
#define FOC_CTRL_STARTUP_OPENLOOP_H

#include "L2_Core/Control/foc_ctrl_startup.h"

#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)
void FOC_StartupOpenLoop_RunStep(foc_motor_t *motor, float dt_sec);
#endif

#endif /* FOC_CTRL_STARTUP_OPENLOOP_H */
