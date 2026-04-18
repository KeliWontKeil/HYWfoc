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
} runtime_c31_step_signal_t;

void RuntimeC31_Init(void);
void RuntimeC31_ApplySignals(const runtime_c31_step_signal_t *signal);
uint8_t RuntimeC31_ProcessOneFrame(void);
void RuntimeC31_Snapshot(runtime_snapshot_t *snapshot);
void RuntimeC31_Commit(void);

#endif /* RUNTIME_C31_FRAME_SOURCE_H */
