#ifndef RUNTIME_INTERNAL_TYPES_H
#define RUNTIME_INTERNAL_TYPES_H

#include <stdint.h>

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
} runtime_step_signal_t;

typedef enum {
    RUNTIME_FAULT_NONE = 0U,
    RUNTIME_FAULT_SENSOR_ADC_INVALID = 1U,
    RUNTIME_FAULT_SENSOR_ENCODER_INVALID = 2U,
    RUNTIME_FAULT_UNDERVOLTAGE = 3U,
    RUNTIME_FAULT_PROTOCOL_FRAME = 4U,
    RUNTIME_FAULT_PARAM_INVALID = 5U,
    RUNTIME_FAULT_INIT_FAILED = 6U
} runtime_fault_code_t;

#endif /* RUNTIME_INTERNAL_TYPES_H */
