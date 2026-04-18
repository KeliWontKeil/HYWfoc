#ifndef RUNTIME_C41_RUNTIME_FSM_H
#define RUNTIME_C41_RUNTIME_FSM_H

#include <stdint.h>

#include "L2_Service/runtime_c42_command_router.h"

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
} runtime_c41_signal_t;

void RuntimeC41_Init(void);
void RuntimeC41_ApplySignals(const runtime_c41_signal_t *signal);
uint8_t RuntimeC41_OnCommand(const protocol_command_t *cmd);
void RuntimeC41_OnFrameError(void);
void RuntimeC41_Snapshot(runtime_snapshot_t *snapshot);
void RuntimeC41_Commit(void);

#endif /* RUNTIME_C41_RUNTIME_FSM_H */
