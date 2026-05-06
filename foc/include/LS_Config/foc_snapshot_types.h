#ifndef FOC_SNAPSHOT_TYPES_H
#define FOC_SNAPSHOT_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_cfg_feature_switches.h"

typedef struct {
    uint8_t control_mode;
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;

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

    uint8_t motor_enabled;
    uint8_t current_soft_switch_enable;
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    uint8_t cogging_comp_enable;
#endif
    uint8_t current_soft_switch_mode;
    float current_soft_switch_auto_open_iq_a;
    float current_soft_switch_auto_closed_iq_a;
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    float cogging_comp_iq_limit_a;
    float cogging_comp_speed_gate_rad_s;
    float cogging_calib_gain_k;
#endif
} control_config_snapshot_t;

typedef struct {
    uint8_t semantic_report_enabled;
    uint8_t osc_report_enabled;
    uint16_t semantic_report_freq_hz;
    uint16_t osc_report_freq_hz;
    uint16_t osc_parameter_mask;
} telemetry_policy_snapshot_t;

typedef struct {
    uint8_t system_running;
    uint8_t system_fault;
    uint8_t params_dirty;
    uint8_t last_exec_ok;
} runtime_state_snapshot_t;

typedef struct {
    control_config_snapshot_t control_cfg;
    telemetry_policy_snapshot_t telemetry;
    runtime_state_snapshot_t runtime;
} runtime_snapshot_t;

typedef enum {
    RUNTIME_C4_EXEC_OK = 0,
    RUNTIME_C4_EXEC_PARAM_ERROR,
    RUNTIME_C4_EXEC_COMMAND_ERROR
} runtime_c4_exec_result_t;

typedef struct {
    uint8_t system_state;
    uint8_t comm_state;
    uint8_t report_mode;
    uint8_t init_diag;
    uint8_t last_fault_code;
    uint16_t init_check_mask;
    uint16_t init_fail_mask;
    uint16_t sensor_invalid_consecutive;
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
    uint8_t params_dirty;
    uint8_t last_exec_ok;
} runtime_c4_runtime_view_t;

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
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    float cogging_comp_iq_limit_a;
    float cogging_comp_speed_gate_rad_s;
    float cogging_calib_gain_k;
#endif
} runtime_c4_params_view_t;

typedef struct {
    uint8_t motor_enable;
    uint8_t semantic_enable;
    uint8_t osc_enable;
    uint8_t current_soft_switch_enable;
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    uint8_t cogging_comp_enable;
#endif
} runtime_c4_states_view_t;

#endif /* FOC_SNAPSHOT_TYPES_H */
