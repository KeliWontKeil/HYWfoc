#ifndef DEBUG_STREAM_H
#define DEBUG_STREAM_H

#include <stdint.h>

#include "L2_Service/l2_service_contract.h"
#include "LS_Config/foc_shared_types.h"
#include "LS_Config/foc_config.h"

/*
 * Unused-interface audit tag: REFACTOR_RESIDUAL_CANDIDATE.
 * Kept for explicit state reset entry; evaluate deletion after one release.
 */
void DebugStream_Init(void);
void DebugStream_SetExecutionCycles(uint32_t exec_cycles);
void DebugStream_Process(const sensor_data_t *sensor,
                         const foc_motor_t *motor,
                         const l2_runtime_status_snapshot_t *runtime,
                         const l2_telemetry_policy_snapshot_t *telemetry);

#endif /* DEBUG_STREAM_H */
