#ifndef FOC_CTRL_STARTUP_H
#define FOC_CTRL_STARTUP_H

#include <stdint.h>
#include "L2_Core/foc_ctrl_types.h"

#define FOC_STARTUP_PHASE_IDLE      0U
#define FOC_STARTUP_PHASE_RUNNING   1U
#define FOC_STARTUP_PHASE_DONE      2U
#define FOC_STARTUP_PHASE_FAILED    3U

void FOC_Startup_Init(foc_motor_t *motor);
void FOC_Startup_RunStep(foc_motor_t *motor, float dt_sec);
uint8_t FOC_Startup_IsComplete(const foc_motor_t *motor);
void FOC_Startup_Abort(foc_motor_t *motor);

#endif /* FOC_CTRL_STARTUP_H */