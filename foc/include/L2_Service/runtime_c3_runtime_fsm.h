#ifndef RUNTIME_C3_RUNTIME_FSM_H
#define RUNTIME_C3_RUNTIME_FSM_H

#include <stdint.h>

#include "LS_Config/foc_protocol_types.h"
#include "LS_Config/foc_runtime_types.h"
#include "L2_Service/runtime_snapshot.h"

void RuntimeC3_UpdateSignals(const runtime_step_signal_t *signal);
uint8_t RuntimeC3_HandleCommand(const protocol_command_t *cmd);
void RuntimeC3_Init(void);
void RuntimeC3_ReportFrameError(void);
void RuntimeC3_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeC3_Commit(void);

#endif /* RUNTIME_C3_RUNTIME_FSM_H */


