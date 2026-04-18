#ifndef RUNTIME_C43_STORE_H
#define RUNTIME_C43_STORE_H

#include <stdint.h>

#include "L2_Service/runtime_c44_output.h"
#include "L2_Service/runtime_snapshot.h"

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
} runtime_c43_runtime_state_t;

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
} runtime_c43_params_t;

typedef struct {
    uint8_t motor_enable;
    uint8_t semantic_enable;
    uint8_t osc_enable;
    uint8_t current_soft_switch_enable;
} runtime_c43_states_t;

void RuntimeC43_ResetStorageDefaults(void);
runtime_c43_runtime_state_t *RuntimeC43_Runtime(void);
runtime_c43_params_t *RuntimeC43_Params(void);
runtime_c43_states_t *RuntimeC43_States(void);

uint8_t RuntimeC43_WriteParam(char subcommand, float value);
uint8_t RuntimeC43_ReadParam(char subcommand, float *value_out);
void RuntimeC43_ReportAllParams(void);
void RuntimeC43_OutputParam(char subcommand, float value);

uint8_t RuntimeC43_WriteState(char subcommand, uint8_t state);
uint8_t RuntimeC43_ReadState(char subcommand, uint8_t *state_out);
void RuntimeC43_ReportAllStates(void);
void RuntimeC43_OutputState(char subcommand, uint8_t value);

void RuntimeC43_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeC43_ClearDirty(void);

void RuntimeC43_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeC43_OutputRuntimeSummary(void);
void RuntimeC43_OutputFaultControlSummary(void);
const char *RuntimeC43_GetFaultName(uint8_t fault_code);
void RuntimeC43_WriteText(const char *text);
void RuntimeC43_WriteStatusByte(uint8_t status);

#endif /* RUNTIME_C43_STORE_H */
