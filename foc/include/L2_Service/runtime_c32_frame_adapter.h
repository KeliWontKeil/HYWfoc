#ifndef RUNTIME_C32_FRAME_ADAPTER_H
#define RUNTIME_C32_FRAME_ADAPTER_H

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
} runtime_c32_step_signal_t;

void RuntimeC32_Init(void);
void RuntimeC32_ApplySignals(const runtime_c32_step_signal_t *signal);
uint8_t RuntimeC32_HandleFrame(const uint8_t *frame, uint16_t len);
void RuntimeC32_Snapshot(runtime_snapshot_t *snapshot);
void RuntimeC32_Commit(void);

#endif /* RUNTIME_C32_FRAME_ADAPTER_H */
