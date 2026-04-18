#ifndef PROTOCOL_SERVICE_H
#define PROTOCOL_SERVICE_H

#include <stdint.h>

#include "L2_Service/l2_service_contract.h"

uint8_t ProtocolService_ProcessStep(uint8_t max_frames);
void ProtocolService_BuildSnapshot(l2_service_snapshot_t *snapshot);
void ProtocolService_CommitAppliedConfig(void);

#endif /* PROTOCOL_SERVICE_H */
