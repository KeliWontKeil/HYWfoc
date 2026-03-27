#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include <stdint.h>

/* Command group definitions */
#define COMMAND_MANAGER_CMD_WRITE_PARAM            'W'
#define COMMAND_MANAGER_CMD_READ_PARAM             'R'
#define COMMAND_MANAGER_CMD_READ_STATE             'Q'
#define COMMAND_MANAGER_CMD_FAULT_CONTROL          'F'

#define COMMAND_MANAGER_SUBCMD_FAULT_CLEAR_REINIT  'C'

/* Parameter sub-command definitions */
#define COMMAND_MANAGER_SUBCMD_TARGET_ANGLE        'A'
#define COMMAND_MANAGER_SUBCMD_ANGLE_SPEED         'S'
#define COMMAND_MANAGER_SUBCMD_SEMANTIC_DIV        'L'
#define COMMAND_MANAGER_SUBCMD_OSC_DIV             'H'
#define COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE     'Y'
#define COMMAND_MANAGER_SUBCMD_OSC_ENABLE          'Z'
#define COMMAND_MANAGER_SUBCMD_OSC_PARAM_MASK      'O'
#define COMMAND_MANAGER_SUBCMD_READ_ALL            'X'

/* PID and control config sub-command definitions */
#define COMMAND_MANAGER_SUBCMD_PID_CURRENT_KP      'C'
#define COMMAND_MANAGER_SUBCMD_PID_CURRENT_KI      'I'
#define COMMAND_MANAGER_SUBCMD_PID_CURRENT_KD      'J'
#define COMMAND_MANAGER_SUBCMD_PID_ANGLE_KP        'G'
#define COMMAND_MANAGER_SUBCMD_PID_ANGLE_KI        'K'
#define COMMAND_MANAGER_SUBCMD_PID_ANGLE_KD        'N'
#define COMMAND_MANAGER_SUBCMD_PID_SPEED_KP        'P'
#define COMMAND_MANAGER_SUBCMD_PID_SPEED_KI        'U'
#define COMMAND_MANAGER_SUBCMD_PID_SPEED_KD        'V'

#define COMMAND_MANAGER_SUBCMD_CFG_MIN_MECH_DELTA  'M'
#define COMMAND_MANAGER_SUBCMD_CFG_HOLD_I_LIMIT    'B'
#define COMMAND_MANAGER_SUBCMD_CFG_HOLD_DEADBAND   'E'
#define COMMAND_MANAGER_SUBCMD_CFG_BLEND_START     'F'
#define COMMAND_MANAGER_SUBCMD_CFG_BLEND_END       'T'
#define COMMAND_MANAGER_SUBCMD_CONTROL_MODE        'D'
#define COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE        'Q'

#define COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE   0U
#define COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY    1U
#define COMMAND_MANAGER_ENABLED_DISABLE             0U
#define COMMAND_MANAGER_ENABLED_ENABLE              1U

/* Protocol status code definitions */
#define COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR    'I'
#define COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR  'P'

typedef enum {
    COMMAND_MANAGER_SYSTEM_INIT = 0,
    COMMAND_MANAGER_SYSTEM_RUNNING,
    COMMAND_MANAGER_SYSTEM_FAULT
} command_manager_system_state_t;

typedef enum {
    COMMAND_MANAGER_COMM_IDLE = 0,
    COMMAND_MANAGER_COMM_ACTIVE,
    COMMAND_MANAGER_COMM_ERROR
} command_manager_comm_state_t;

typedef enum {
    COMMAND_MANAGER_REPORT_OFF = 0,
    COMMAND_MANAGER_REPORT_SEMANTIC_ONLY,
    COMMAND_MANAGER_REPORT_OSC_ONLY,
    COMMAND_MANAGER_REPORT_BOTH
} command_manager_report_mode_t;

typedef enum {
    COMMAND_MANAGER_DIAG_NOT_EXECUTED = 0,
    COMMAND_MANAGER_DIAG_SUCCESS,
    COMMAND_MANAGER_DIAG_FAILED
} command_manager_diag_state_t;

typedef enum {
    COMMAND_MANAGER_FAULT_NONE = 0,
    COMMAND_MANAGER_FAULT_SENSOR_ADC_INVALID,
    COMMAND_MANAGER_FAULT_SENSOR_ENCODER_INVALID,
    COMMAND_MANAGER_FAULT_PROTOCOL_FRAME,
    COMMAND_MANAGER_FAULT_PARAM_INVALID,
    COMMAND_MANAGER_FAULT_INIT_FAILED
} command_manager_fault_code_t;

#define COMMAND_MANAGER_INIT_CHECK_COMM        (1U << 0)
#define COMMAND_MANAGER_INIT_CHECK_PROTOCOL    (1U << 1)
#define COMMAND_MANAGER_INIT_CHECK_COMMAND     (1U << 2)
#define COMMAND_MANAGER_INIT_CHECK_DEBUG       (1U << 3)
#define COMMAND_MANAGER_INIT_CHECK_SENSOR      (1U << 4)
#define COMMAND_MANAGER_INIT_CHECK_PWM         (1U << 5)
#define COMMAND_MANAGER_INIT_CHECK_MOTOR       (1U << 6)

#ifndef COMMAND_MANAGER_INIT_REQUIRED_MASK
#define COMMAND_MANAGER_INIT_REQUIRED_MASK (COMMAND_MANAGER_INIT_CHECK_COMM     | \
                                           COMMAND_MANAGER_INIT_CHECK_PROTOCOL | \
                                           COMMAND_MANAGER_INIT_CHECK_COMMAND  | \
                                           COMMAND_MANAGER_INIT_CHECK_DEBUG    | \
                                           COMMAND_MANAGER_INIT_CHECK_SENSOR   | \
                                           COMMAND_MANAGER_INIT_CHECK_PWM      | \
                                           COMMAND_MANAGER_INIT_CHECK_MOTOR)
#endif

#define COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD      (-12.566f)
#define COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD      (12.566f)
#define COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S     (0.0f)
#define COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S     (200.0f)

#define COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN        (0.0f)
#define COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX        (50.0f)
#define COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN        (0.0f)
#define COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX        (50.0f)
#define COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MIN        (0.0f)
#define COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MAX        (10.0f)

#define COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN          (0.0f)
#define COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX          (50.0f)
#define COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN          (0.0f)
#define COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX          (50.0f)
#define COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MIN          (0.0f)
#define COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MAX          (10.0f)

#define COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN          (0.0f)
#define COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX          (50.0f)
#define COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN          (0.0f)
#define COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX          (50.0f)
#define COMMAND_MANAGER_PARAM_PID_SPEED_KD_MIN          (0.0f)
#define COMMAND_MANAGER_PARAM_PID_SPEED_KD_MAX          (10.0f)

typedef struct {
    command_manager_system_state_t system_state;
    command_manager_comm_state_t comm_state;
    command_manager_report_mode_t report_mode;
    command_manager_diag_state_t init_diag;
    command_manager_fault_code_t last_fault_code;
    uint16_t init_check_mask;
    uint16_t init_fail_mask;
    uint16_t sensor_invalid_consecutive;
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
    uint8_t params_dirty;
    uint8_t last_exec_ok;
} command_manager_runtime_state_t;

void CommandManager_Init(void);
void CommandManager_Process(void);
void CommandManager_ReportInitCheck(uint16_t check_bit, uint8_t success);
void CommandManager_FinalizeInitDiagnostics(void);
void CommandManager_ReportRuntimeSensorState(uint8_t adc_valid, uint8_t encoder_valid);
void CommandManager_ReportProtocolFrameError(void);
void CommandManager_ReportControlLoopSkip(void);

const command_manager_runtime_state_t *CommandManager_GetRuntimeState(void);

uint8_t CommandManager_WriteParam(char subcommand, float value);
uint8_t CommandManager_ReadParam(char subcommand, float *value_out);
void CommandManager_ReportAllParams(void);

float CommandManager_GetTargetAngleRad(void);
float CommandManager_GetAngleSpeedRadS(void);

uint8_t CommandManager_IsSemanticReportEnabled(void);
uint8_t CommandManager_IsOscilloscopeReportEnabled(void);
uint16_t CommandManager_GetSemanticReportDivider(void);
uint16_t CommandManager_GetOscilloscopeReportDivider(void);
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

uint8_t CommandManager_GetControlMode(void);
uint8_t CommandManager_IsMotorEnabled(void);
uint8_t CommandManager_RecoverFaultAndReinit(void);

#endif /* COMMAND_MANAGER_H */
