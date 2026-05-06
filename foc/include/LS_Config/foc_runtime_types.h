#ifndef FOC_RUNTIME_TYPES_H
#define FOC_RUNTIME_TYPES_H

#include <stdint.h>

typedef struct {
    uint16_t init_checks_pass_mask;
    uint16_t init_checks_fail_mask;
    uint8_t finalize_init;

    uint8_t sensor_state_updated;
    uint8_t adc_valid;
    uint8_t encoder_valid;

    float undervoltage_vbus;

    uint8_t control_loop_skipped;
} runtime_step_signal_t;

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

typedef enum {
    RUNTIME_FAULT_NONE = 0U,
    RUNTIME_FAULT_SENSOR_ADC_INVALID = 1U,
    RUNTIME_FAULT_SENSOR_ENCODER_INVALID = 2U,
    RUNTIME_FAULT_UNDERVOLTAGE = 3U,
    RUNTIME_FAULT_PROTOCOL_FRAME = 4U,
    RUNTIME_FAULT_PARAM_INVALID = 5U,
    RUNTIME_FAULT_INIT_FAILED = 6U
} runtime_fault_code_t;

#endif /* FOC_RUNTIME_TYPES_H */
