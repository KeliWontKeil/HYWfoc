#ifndef FOC_CONTROL_TYPES_H
#define FOC_CONTROL_TYPES_H

#include <stdint.h>

typedef struct {
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_runtime_config_t;

typedef struct {
    uint8_t enabled;
    uint8_t configured_mode;
    uint8_t active_mode;
    float blend_factor;
    float auto_open_iq_a;
    float auto_closed_iq_a;
} foc_current_soft_switch_status_t;

typedef struct {
    uint8_t enabled;
    uint8_t available;
    uint8_t source;
    uint16_t point_count;
    float iq_lsb_a;
    float speed_gate_rad_s;
    float iq_limit_a;
} foc_cogging_comp_status_t;

#endif /* FOC_CONTROL_TYPES_H */
