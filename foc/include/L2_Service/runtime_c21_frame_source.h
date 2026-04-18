#ifndef RUNTIME_C31_FRAME_SOURCE_H
#define RUNTIME_C31_FRAME_SOURCE_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"

typedef struct {
    uint16_t init_checks_pass_mask;
    uint16_t init_checks_fail_mask;
    uint8_t finalize_init;

    uint8_t sensor_state_updated;
    uint8_t adc_valid;
    uint8_t encoder_valid;

    uint8_t control_loop_skipped;

    uint8_t undervoltage_fault;
    float undervoltage_vbus;
} runtime_frame_pipeline_signal_t;

void RuntimeFramePipeline_Init(void);
void RuntimeFramePipeline_UpdateSignals(const runtime_frame_pipeline_signal_t *signal);
uint8_t RuntimeFramePipeline_ProcessOneFrame(void);
void RuntimeFramePipeline_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeFramePipeline_Commit(void);

#endif /* RUNTIME_C31_FRAME_SOURCE_H */

