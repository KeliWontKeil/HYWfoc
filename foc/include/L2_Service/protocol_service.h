#ifndef PROTOCOL_SERVICE_H
#define PROTOCOL_SERVICE_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"

uint8_t ProtocolService_ProcessStep(uint8_t max_frames);
void ProtocolService_BuildSnapshot(runtime_snapshot_t *snapshot);
void ProtocolService_CommitAppliedConfig(void);

#endif /* PROTOCOL_SERVICE_H */
