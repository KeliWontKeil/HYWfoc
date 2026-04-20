#ifndef RUNTIME_C31_RUNTIME_FSM_H
#define RUNTIME_C31_RUNTIME_FSM_H

#include <stdint.h>

#include "L2_Service/runtime_internal_types.h"
#include "L2_Service/runtime_c32_command_router.h"

void RuntimeStateMachine_Init(void);
void RuntimeStateMachine_UpdateSignals(const runtime_step_signal_t *signal);
uint8_t RuntimeStateMachine_HandleCommand(const protocol_command_t *cmd);
void RuntimeStateMachine_ReportFrameError(void);

#endif /* RUNTIME_C31_RUNTIME_FSM_H */

