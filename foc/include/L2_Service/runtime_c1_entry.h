#ifndef RUNTIME_C1_ENTRY_H
#define RUNTIME_C1_ENTRY_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"

typedef enum {
    RUNTIME_INIT_CHECK_COMM = (1U << 0),
    RUNTIME_INIT_CHECK_PROTOCOL = (1U << 1),
    RUNTIME_INIT_CHECK_COMMAND = (1U << 2),
    RUNTIME_INIT_CHECK_DEBUG = (1U << 3),
    RUNTIME_INIT_CHECK_SENSOR = (1U << 4),
    RUNTIME_INIT_CHECK_PWM = (1U << 5),
    RUNTIME_INIT_CHECK_MOTOR = (1U << 6),
    RUNTIME_INIT_CHECK_VBUS = (1U << 7)
} runtime_init_check_t;


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
} runtime_c1_step_input_t;

void RuntimeC1_Init(void);
uint8_t RuntimeC1_RunStep(uint8_t frame_budget, const runtime_c1_step_input_t *input);
void RuntimeC1_GetSnapshot(runtime_snapshot_t *snapshot);
void RuntimeC1_Commit(void);

#endif /* RUNTIME_C1_ENTRY_H */


