#ifndef RUNTIME_C43_COMMAND_STORE_H
#define RUNTIME_C43_COMMAND_STORE_H

#include <stdint.h>

#include "L2_Service/runtime_c41_command_entry.h"

typedef struct {
    float target_angle_rad;
    float angle_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;
    uint16_t semantic_freq_hz;
    uint16_t osc_freq_hz;
    uint16_t osc_param_mask;
    float pid_current_kp;
    float pid_current_ki;
    float pid_current_kd;
    float pid_angle_kp;
    float pid_angle_ki;
    float pid_angle_kd;
    float pid_speed_kp;
    float pid_speed_ki;
    float pid_speed_kd;
    float cfg_min_mech_angle_accum_delta_rad;
    float cfg_angle_hold_integral_limit;
    float cfg_angle_hold_pid_deadband_rad;
    float cfg_speed_angle_transition_start_rad;
    float cfg_speed_angle_transition_end_rad;
    uint8_t control_mode;
    uint8_t current_soft_switch_mode;
    float current_soft_switch_auto_open_iq_a;
    float current_soft_switch_auto_closed_iq_a;
} command_manager_params_t;

typedef struct {
    uint8_t motor_enable;
    uint8_t semantic_enable;
    uint8_t osc_enable;
    uint8_t current_soft_switch_enable;
} command_manager_states_t;

command_manager_runtime_state_t *CommandManager_InternalRuntimeState(void);
command_manager_params_t *CommandManager_InternalParams(void);
command_manager_states_t *CommandManager_InternalStates(void);

#endif /* RUNTIME_C43_COMMAND_STORE_H */
