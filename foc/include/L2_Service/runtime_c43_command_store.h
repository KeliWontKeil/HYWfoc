#ifndef RUNTIME_C43_COMMAND_STORE_H
#define RUNTIME_C43_COMMAND_STORE_H

#include <stdint.h>

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
} command_manager_runtime_state_t;

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

uint8_t CommandManager_WriteParam(char subcommand, float value);
uint8_t CommandManager_ReadParam(char subcommand, float *value_out);
void CommandManager_ReportAllParams(void);

uint8_t CommandManager_WriteState(char subcommand, uint8_t state);
uint8_t CommandManager_ReadState(char subcommand, uint8_t *state_out);
void CommandManager_ReportAllStates(void);

float CommandManager_GetTargetAngleRad(void);
float CommandManager_GetAngleSpeedRadS(void);
float CommandManager_GetSpeedOnlyRadS(void);
float CommandManager_GetSensorSampleOffsetPercent(void);

uint8_t CommandManager_IsSemanticReportEnabled(void);
uint8_t CommandManager_IsOscilloscopeReportEnabled(void);
uint16_t CommandManager_GetSemanticReportFrequencyHz(void);
uint16_t CommandManager_GetOscilloscopeReportFrequencyHz(void);
uint16_t CommandManager_GetOscilloscopeParameterMask(void);

void CommandManager_ClearDirtyFlag(void);

float CommandManager_GetCurrentPidKp(void);
float CommandManager_GetCurrentPidKi(void);
float CommandManager_GetCurrentPidKd(void);

float CommandManager_GetAnglePidKp(void);
float CommandManager_GetAnglePidKi(void);
float CommandManager_GetAnglePidKd(void);

float CommandManager_GetSpeedPidKp(void);
float CommandManager_GetSpeedPidKi(void);
float CommandManager_GetSpeedPidKd(void);
float CommandManager_GetControlMinMechAngleAccumDeltaRad(void);
float CommandManager_GetControlAngleHoldIntegralLimit(void);
float CommandManager_GetControlAngleHoldPidDeadbandRad(void);
float CommandManager_GetControlSpeedAngleTransitionStartRad(void);
float CommandManager_GetControlSpeedAngleTransitionEndRad(void);

uint8_t CommandManager_GetControlMode(void);
uint8_t CommandManager_IsMotorEnabled(void);
uint8_t CommandManager_IsCurrentSoftSwitchEnabled(void);
uint8_t CommandManager_GetCurrentSoftSwitchMode(void);
float CommandManager_GetCurrentSoftSwitchAutoOpenIqA(void);
float CommandManager_GetCurrentSoftSwitchAutoClosedIqA(void);

#endif /* RUNTIME_C43_COMMAND_STORE_H */
