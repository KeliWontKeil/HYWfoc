#ifndef L2_SERVICE_CONTRACT_H
#define L2_SERVICE_CONTRACT_H

#include <stdint.h>

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
    uint8_t current_soft_switch_mode;
    float current_soft_switch_auto_open_iq_a;
    float current_soft_switch_auto_closed_iq_a;
} l2_control_config_snapshot_t;

typedef struct {
    uint8_t semantic_report_enabled;
    uint8_t osc_report_enabled;
    uint16_t semantic_report_freq_hz;
    uint16_t osc_report_freq_hz;
    uint16_t osc_parameter_mask;
} l2_telemetry_policy_snapshot_t;

typedef struct {
    uint8_t system_running;
    uint8_t system_fault;
    uint8_t params_dirty;
    uint8_t last_exec_ok;
} l2_runtime_status_snapshot_t;

typedef struct {
    l2_control_config_snapshot_t control_cfg;
    l2_telemetry_policy_snapshot_t telemetry;
    l2_runtime_status_snapshot_t runtime;
} l2_service_snapshot_t;

#endif /* L2_SERVICE_CONTRACT_H */