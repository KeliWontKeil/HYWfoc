#ifndef RUNTIME_CHAIN_INTERNAL_H
#define RUNTIME_CHAIN_INTERNAL_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"

uint8_t RuntimeChain_ProcessCommStep(uint8_t max_frames);
void RuntimeChain_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeChain_CommitAppliedConfig(void);

#endif /* RUNTIME_CHAIN_INTERNAL_H */