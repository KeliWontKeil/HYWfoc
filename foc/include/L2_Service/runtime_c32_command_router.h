#ifndef RUNTIME_COMMAND_ROUTER_H
#define RUNTIME_COMMAND_ROUTER_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"
#include "L3_Algorithm/protocol_core_types.h"

typedef enum {
    RUNTIME_CMD_EXEC_OK = 0,
    RUNTIME_CMD_EXEC_PARAM_ERROR,
    RUNTIME_CMD_EXEC_COMMAND_ERROR
} runtime_command_exec_result_t;

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
} runtime_runtime_view_t;

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
} runtime_params_view_t;

typedef struct {
    uint8_t motor_enable;
    uint8_t semantic_enable;
    uint8_t osc_enable;
    uint8_t current_soft_switch_enable;
} runtime_states_view_t;

void RuntimeCommandRouter_Init(void);
runtime_runtime_view_t *RuntimeCommandRouter_Runtime(void);
runtime_params_view_t *RuntimeCommandRouter_Params(void);
runtime_states_view_t *RuntimeCommandRouter_States(void);

void RuntimeCommandRouter_UpdateReportMode(void);
runtime_command_exec_result_t RuntimeCommandRouter_Execute(const protocol_command_t *cmd);

void RuntimeCommandRouter_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeCommandRouter_ClearDirty(void);

void RuntimeCommandRouter_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeCommandRouter_OutputRuntimeSummary(void);
void RuntimeCommandRouter_OutputFaultControlSummary(void);
const char *RuntimeCommandRouter_GetFaultName(uint8_t fault_code);
void RuntimeCommandRouter_WriteText(const char *text);

void RuntimeCommandRouter_WriteStatusFrameError(void);
void RuntimeCommandRouter_WriteStatusParamInvalid(void);
void RuntimeCommandRouter_WriteStatusCmdInvalid(void);

uint8_t RuntimeCommandRouter_RecoverFaultAndReinit(void);

#endif /* RUNTIME_COMMAND_ROUTER_H */

