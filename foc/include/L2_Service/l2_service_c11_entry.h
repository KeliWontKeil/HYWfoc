#ifndef L2_SERVICE_C11_ENTRY_H
#define L2_SERVICE_C11_ENTRY_H

#include <stdint.h>

#include "L2_Service/l2_service_contract.h"

uint8_t L2_ServiceC11_ProcessCommStep(uint8_t max_frames, l2_service_snapshot_t *snapshot);
void L2_ServiceC11_RefreshSnapshot(l2_service_snapshot_t *snapshot);
void L2_ServiceC11_CommitAppliedConfig(void);

#endif /* L2_SERVICE_C11_ENTRY_H */