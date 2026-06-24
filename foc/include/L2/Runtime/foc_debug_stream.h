#ifndef FOC_DEBUG_STREAM_H

#define FOC_DEBUG_STREAM_H


#include <stdint.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L2/foc_ctrl_types.h"
#include "L2/Protocol/foc_snapshot_types.h"
#include "LS_Config/foc_config.h"

void DebugStream_Init(debug_stream_state_t *ds);
void DebugStream_SetExecutionCycles(debug_stream_state_t *ds, uint32_t exec_cycles);
void DebugStream_Process(debug_stream_state_t *ds,
                         const sensor_data_t *sensor,
                         const foc_motor_t *motor,
                         const telemetry_policy_snapshot_t *telemetry);

#endif /* DEBUG_STREAM_H */