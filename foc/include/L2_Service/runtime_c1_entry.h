#ifndef RUNTIME_C1_ENTRY_H
#define RUNTIME_C1_ENTRY_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"
#include "L2_Service/runtime_internal_types.h"

void Runtime_Init(void);
uint8_t Runtime_FrameRunStep(uint8_t frame_budget);
void Runtime_UpdateSignals(const runtime_step_signal_t *signals);
void Runtime_GetSnapshot(runtime_snapshot_t *snapshot);
void Runtime_Commit(void);

#endif /* RUNTIME_C1_ENTRY_H */


