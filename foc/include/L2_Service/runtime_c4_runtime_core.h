#ifndef RUNTIME_C4_RUNTIME_CORE_H
#define RUNTIME_C4_RUNTIME_CORE_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"
#include "L3_Algorithm/protocol_core_types.h"

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
} runtime_c4_params_view_t;

typedef struct {
    uint8_t motor_enable;
    uint8_t semantic_enable;
    uint8_t osc_enable;
    uint8_t current_soft_switch_enable;
    uint8_t cogging_comp_enable;
} runtime_c4_states_view_t;

void RuntimeC4_Init(void);

void RuntimeC4_AccumulateInitChecks(uint16_t pass_mask, uint16_t fail_mask);
uint16_t RuntimeC4_GetInitCheckMask(void);
uint16_t RuntimeC4_GetInitFailMask(void);
void RuntimeC4_ResetSensorInvalidConsecutive(void);
void RuntimeC4_IncrementSensorInvalidConsecutive(void);
uint16_t RuntimeC4_GetSensorInvalidConsecutive(void);
void RuntimeC4_IncrementControlSkipCount(void);
void RuntimeC4_IncrementProtocolErrorCount(void);
void RuntimeC4_IncrementParamErrorCount(void);
void RuntimeC4_SetSystemState(uint8_t system_state);
uint8_t RuntimeC4_GetSystemState(void);
void RuntimeC4_SetCommState(uint8_t comm_state);
void RuntimeC4_SetInitDiag(uint8_t init_diag);
uint8_t RuntimeC4_GetInitDiag(void);
void RuntimeC4_SetLastFaultCode(uint8_t fault_code);
uint8_t RuntimeC4_GetLastExecOk(void);
void RuntimeC4_SetLastExecOk(uint8_t last_exec_ok);

void RuntimeC4_UpdateReportMode(void);
runtime_c4_exec_result_t RuntimeC4_ExecuteCommand(const protocol_command_t *cmd);

void RuntimeC4_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeC4_ClearDirty(void);

void RuntimeC4_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeC4_OutputRuntimeSummary(void);
void RuntimeC4_OutputFaultControlSummary(void);
const char *RuntimeC4_GetFaultName(uint8_t fault_code);
void RuntimeC4_WriteText(const char *text);

void RuntimeC4_WriteStatusFrameError(void);
void RuntimeC4_WriteStatusParamInvalid(void);
void RuntimeC4_WriteStatusCmdInvalid(void);

uint8_t RuntimeC4_RecoverFaultAndReinit(void);

#endif /* RUNTIME_C4_RUNTIME_CORE_H */


