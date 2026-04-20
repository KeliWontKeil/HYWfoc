#ifndef RUNTIME_C21_FRAME_SOURCE_H
#define RUNTIME_C21_FRAME_SOURCE_H

#include <stdint.h>

#include "L2_Service/runtime_internal_types.h"
#include "L2_Service/runtime_snapshot.h"

void RuntimeFramePipeline_Init(void);
void RuntimeFramePipeline_UpdateSignals(const runtime_step_signal_t *signal);
uint8_t RuntimeFramePipeline_ProcessOneFrame(void);
void RuntimeFramePipeline_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeFramePipeline_Commit(void);

#endif /* RUNTIME_C21_FRAME_SOURCE_H */

