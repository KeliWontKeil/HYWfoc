#ifndef RUNTIME_C21_CYCLE_DRIVER_H
#define RUNTIME_C21_CYCLE_DRIVER_H

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
} runtime_c21_step_signal_t;

void RuntimeC21_Init(void);
uint8_t RuntimeC21_Step(uint8_t frame_budget, const runtime_c21_step_signal_t *signal);
void RuntimeC21_Snapshot(runtime_snapshot_t *snapshot);
void RuntimeC21_Commit(void);

#endif /* RUNTIME_C21_CYCLE_DRIVER_H */
