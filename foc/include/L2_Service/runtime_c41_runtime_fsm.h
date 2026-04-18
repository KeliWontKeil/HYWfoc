#ifndef RUNTIME_STATE_MACHINE_H
#define RUNTIME_STATE_MACHINE_H

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
} runtime_state_signal_t;

void RuntimeStateMachine_Init(void);
void RuntimeStateMachine_UpdateSignals(const runtime_state_signal_t *signal);
uint8_t RuntimeStateMachine_HandleCommand(const protocol_command_t *cmd);
void RuntimeStateMachine_ReportFrameError(void);
void RuntimeStateMachine_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeStateMachine_Commit(void);

#endif /* RUNTIME_STATE_MACHINE_H */

