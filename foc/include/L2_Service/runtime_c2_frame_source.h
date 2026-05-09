#ifndef RUNTIME_C2_FRAME_SOURCE_H
#define RUNTIME_C2_FRAME_SOURCE_H

#include <stdint.h>

#include "LS_Config/foc_runtime_types.h"
#include "L2_Service/runtime_snapshot.h"

uint8_t RuntimeC2_ProcessOneFrame(void);

void RuntimeC2_Init(void);
void RuntimeC2_UpdateSignals(const runtime_step_signal_t *signal);
void RuntimeC2_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeC2_Commit(void);
void RuntimeC2_ClearReinit(void);

#endif /* RUNTIME_C2_FRAME_SOURCE_H */


