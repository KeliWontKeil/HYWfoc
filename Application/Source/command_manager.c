#include "command_manager.h"

#include <stdio.h>

#include "foc_platform_api.h"
#include "foc_control.h"
#include "protocol_parser.h"
#include "debug_stream.h"

#define COMMAND_MANAGER_DEFAULT_TARGET_ANGLE_RAD      3.14f
#define COMMAND_MANAGER_DEFAULT_ANGLE_SPEED_RAD_S     16.0f
#define COMMAND_MANAGER_DEFAULT_SEMANTIC_DIV          5U
#define COMMAND_MANAGER_DEFAULT_OSC_DIV               1U
#define COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK        DEBUG_STREAM_OSC_DEFAULT_PARAM_MASK
#define COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED      1U
#define COMMAND_MANAGER_DEFAULT_OSC_ENABLED           0U

#define COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP         0.0f
#define COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI         0.0f
#define COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD         0.0f
#define COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP           2.0f
#define COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI           0.8f
#define COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD           0.01f
#define COMMAND_MANAGER_DEFAULT_PID_SPEED_KP           3.0f
#define COMMAND_MANAGER_DEFAULT_PID_SPEED_KI           0.5f
#define COMMAND_MANAGER_DEFAULT_PID_SPEED_KD           0.0f
#define COMMAND_MANAGER_DEFAULT_CONTROL_MODE            COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE
#define COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE            COMMAND_MANAGER_ENABLED_ENABLE

#define COMMAND_MANAGER_REPLY_BUFFER_LEN              128U
#define COMMAND_MANAGER_SENSOR_FAULT_THRESHOLD        50U

typedef struct {
    float target_angle_rad;
    float angle_speed_rad_s;
    uint16_t semantic_div;
    uint16_t osc_div;
    uint16_t osc_param_mask;
    uint8_t semantic_enable;
    uint8_t osc_enable;
    float pid_current_kp;
    float pid_current_ki;
    float pid_current_kd;
    float pid_angle_kp;
    float pid_angle_ki;
    float pid_angle_kd;
    float pid_speed_kp;
    float pid_speed_ki;
    float pid_speed_kd;
    uint8_t control_mode;
    uint8_t motor_enable;
} command_manager_params_t;

static command_manager_runtime_state_t g_runtime_state;
static command_manager_params_t g_params;

typedef enum {
    COMMAND_EXEC_RESULT_OK = 0,
    COMMAND_EXEC_RESULT_PARAM_ERROR,
    COMMAND_EXEC_RESULT_COMMAND_ERROR
} command_exec_result_t;

static void CommandManager_ReportInitDiag(void);
static command_exec_result_t CommandManager_Execute(const protocol_command_t *cmd);
static uint8_t CommandManager_ReportSingleParam(char subcommand);
static const char *CommandManager_GetParamName(char subcommand);
static uint8_t CommandManager_IsIntegerParam(char subcommand);
static uint8_t CommandManager_IsEnableParam(char subcommand);
static const char *CommandManager_GetFaultName(command_manager_fault_code_t fault_code);
static void CommandManager_OutputDiag(const char *level, const char *module, const char *detail);
static void CommandManager_OutputParam(char subcommand, float value);
static uint8_t CommandManager_IsInRange(float value, float min_value, float max_value);

void CommandManager_Init(void)
{
    g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_INIT;
    g_runtime_state.comm_state = COMMAND_MANAGER_COMM_IDLE;
    g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_SEMANTIC_ONLY;
    g_runtime_state.init_diag = COMMAND_MANAGER_DIAG_NOT_EXECUTED;
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_NONE;
    g_runtime_state.init_check_mask = 0U;
    g_runtime_state.init_fail_mask = 0U;
    g_runtime_state.sensor_invalid_consecutive = 0U;
    g_runtime_state.protocol_error_count = 0U;
    g_runtime_state.param_error_count = 0U;
    g_runtime_state.control_skip_count = 0U;
    g_runtime_state.params_dirty = 0U;
    g_runtime_state.last_exec_ok = 0U;

    g_params.target_angle_rad = COMMAND_MANAGER_DEFAULT_TARGET_ANGLE_RAD;
    g_params.angle_speed_rad_s = COMMAND_MANAGER_DEFAULT_ANGLE_SPEED_RAD_S;
    g_params.semantic_div = COMMAND_MANAGER_DEFAULT_SEMANTIC_DIV;
    g_params.osc_div = COMMAND_MANAGER_DEFAULT_OSC_DIV;
    g_params.osc_param_mask = COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK;
    g_params.semantic_enable = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
    g_params.osc_enable = COMMAND_MANAGER_DEFAULT_OSC_ENABLED;
    g_params.pid_current_kp = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP;
    g_params.pid_current_ki = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI;
    g_params.pid_current_kd = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD;
    g_params.pid_angle_kp = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP;
    g_params.pid_angle_ki = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI;
    g_params.pid_angle_kd = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD;
    g_params.pid_speed_kp = COMMAND_MANAGER_DEFAULT_PID_SPEED_KP;
    g_params.pid_speed_ki = COMMAND_MANAGER_DEFAULT_PID_SPEED_KI;
    g_params.pid_speed_kd = COMMAND_MANAGER_DEFAULT_PID_SPEED_KD;
    g_params.control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    g_params.motor_enable = COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;

    FOC_ControlConfigResetDefault();

    CommandManager_ReportInitDiag();
}

void CommandManager_Process(void)
{
    const protocol_command_t *cmd = ProtocolParser_GetLatestCommand();
    command_exec_result_t exec_result;

    if ((cmd == 0) || (cmd->updated == 0U))
    {
        return;
    }

    g_runtime_state.comm_state = COMMAND_MANAGER_COMM_ACTIVE;
    exec_result = CommandManager_Execute(cmd);
    g_runtime_state.last_exec_ok = (exec_result == COMMAND_EXEC_RESULT_OK) ? 1U : 0U;

    if (exec_result != COMMAND_EXEC_RESULT_OK)
    {
        g_runtime_state.comm_state = COMMAND_MANAGER_COMM_ERROR;

        if (exec_result == COMMAND_EXEC_RESULT_PARAM_ERROR)
        {
            g_runtime_state.param_error_count++;
            g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PARAM_INVALID;
        }
        else
        {
            g_runtime_state.protocol_error_count++;
            g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PROTOCOL_FRAME;
        }

        FOC_Platform_DebugOutput("ERR fallback: keep previous params\r\n");
    }
    else
    {
        g_runtime_state.comm_state = COMMAND_MANAGER_COMM_ACTIVE;
    }

    if ((g_params.semantic_enable != 0U) && (g_params.osc_enable != 0U))
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_BOTH;
    }
    else if (g_params.semantic_enable != 0U)
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_SEMANTIC_ONLY;
    }
    else if (g_params.osc_enable != 0U)
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_OSC_ONLY;
    }
    else
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_OFF;
    }

    ProtocolParser_ClearUpdatedFlag();
}

void CommandManager_ReportInitCheck(uint16_t check_bit, uint8_t success)
{
    g_runtime_state.init_check_mask = (uint16_t)(g_runtime_state.init_check_mask | check_bit);

    if (success == 0U)
    {
        g_runtime_state.init_fail_mask = (uint16_t)(g_runtime_state.init_fail_mask | check_bit);
    }
}

void CommandManager_FinalizeInitDiagnostics(void)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
    uint16_t missing_mask = (uint16_t)(COMMAND_MANAGER_INIT_REQUIRED_MASK & (~g_runtime_state.init_check_mask));

    if (g_runtime_state.init_check_mask == 0U)
    {
        g_runtime_state.init_diag = COMMAND_MANAGER_DIAG_NOT_EXECUTED;
        g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_FAULT;
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_INIT_FAILED;
        CommandManager_OutputDiag("ERR", "init", "no checks executed");
        return;
    }

    if ((g_runtime_state.init_fail_mask == 0U) && (missing_mask == 0U))
    {
        g_runtime_state.init_diag = COMMAND_MANAGER_DIAG_SUCCESS;
        g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_RUNNING;
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_NONE;
    }
    else
    {
        g_runtime_state.init_diag = COMMAND_MANAGER_DIAG_FAILED;
        g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_FAULT;
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_INIT_FAILED;
    }

    if (missing_mask != 0U)
    {
        snprintf(out,
                 sizeof(out),
                 "diag.init.missing=0x%04X required=0x%04X\r\n",
                 (unsigned int)missing_mask,
                 (unsigned int)COMMAND_MANAGER_INIT_REQUIRED_MASK);
        FOC_Platform_DebugOutput(out);
    }

    snprintf(out,
             sizeof(out),
             "diag.init.result=%u checks=0x%04X fails=0x%04X\r\n",
             (unsigned int)g_runtime_state.init_diag,
             (unsigned int)g_runtime_state.init_check_mask,
             (unsigned int)g_runtime_state.init_fail_mask);
    FOC_Platform_DebugOutput(out);
}

void CommandManager_ReportRuntimeSensorState(uint8_t adc_valid, uint8_t encoder_valid)
{
    if ((adc_valid != 0U) && (encoder_valid != 0U))
    {
        g_runtime_state.sensor_invalid_consecutive = 0U;
        if (g_runtime_state.system_state != COMMAND_MANAGER_SYSTEM_FAULT)
        {
            g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_NONE;
        }
        return;
    }

    g_runtime_state.control_skip_count++;
    g_runtime_state.sensor_invalid_consecutive++;

    if (adc_valid == 0U)
    {
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_SENSOR_ADC_INVALID;
    }
    else
    {
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_SENSOR_ENCODER_INVALID;
    }

    if (g_runtime_state.sensor_invalid_consecutive >= COMMAND_MANAGER_SENSOR_FAULT_THRESHOLD)
    {
        if (g_runtime_state.system_state != COMMAND_MANAGER_SYSTEM_FAULT)
        {
            char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
            g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_FAULT;

            snprintf(out,
                     sizeof(out),
                     "sensor invalid threshold reached: %u",
                     (unsigned int)g_runtime_state.sensor_invalid_consecutive);
            CommandManager_OutputDiag("ERR", "sensor", out);
        }
    }
}

void CommandManager_ReportProtocolFrameError(void)
{
    g_runtime_state.protocol_error_count++;
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PROTOCOL_FRAME;
}

void CommandManager_ReportControlLoopSkip(void)
{
    g_runtime_state.control_skip_count++;
}

uint8_t CommandManager_RecoverFaultAndReinit(void)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    g_runtime_state.sensor_invalid_consecutive = 0U;
    g_runtime_state.protocol_error_count = 0U;
    g_runtime_state.param_error_count = 0U;
    g_runtime_state.control_skip_count = 0U;
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_NONE;
    g_runtime_state.comm_state = COMMAND_MANAGER_COMM_IDLE;
    g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_RUNNING;
    g_runtime_state.params_dirty = 1U;
    g_runtime_state.last_exec_ok = 1U;

    FOC_ControlConfigResetDefault();

    snprintf(out,
             sizeof(out),
             "diag.reinit=SOFT_DIAG_RESET state=%u fault=%s\r\n",
             (unsigned int)g_runtime_state.system_state,
             CommandManager_GetFaultName(g_runtime_state.last_fault_code));
    FOC_Platform_DebugOutput(out);

    return 1U;
}

const command_manager_runtime_state_t *CommandManager_GetRuntimeState(void)
{
    return &g_runtime_state;
}

uint8_t CommandManager_WriteParam(char subcommand, float value)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_SUBCMD_TARGET_ANGLE:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD,
                                     COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD) == 0U)
        {
            return 0U;
        }
        g_params.target_angle_rad = value;
        break;

    case COMMAND_MANAGER_SUBCMD_ANGLE_SPEED:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S,
                                     COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        g_params.angle_speed_rad_s = value;
        break;

    case COMMAND_MANAGER_SUBCMD_SEMANTIC_DIV:
        if ((value < 1.0f) || (value > 1000.0f))
        {
            return 0U;
        }
        g_params.semantic_div = (uint16_t)value;
        break;

    case COMMAND_MANAGER_SUBCMD_OSC_DIV:
        if ((value < 1.0f) || (value > 1000.0f))
        {
            return 0U;
        }
        g_params.osc_div = (uint16_t)value;
        break;

    case COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE:
        g_params.semantic_enable = (value != 0.0f) ? 1U : 0U;
        break;

    case COMMAND_MANAGER_SUBCMD_OSC_ENABLE:
        g_params.osc_enable = (value != 0.0f) ? 1U : 0U;
        break;

    case COMMAND_MANAGER_SUBCMD_OSC_PARAM_MASK:
        if ((value < 0.0f) || (value > 65535.0f))
        {
            return 0U;
        }
        g_params.osc_param_mask = (uint16_t)value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_current_kp = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_current_ki = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_current_kd = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_angle_kp = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_angle_ki = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_angle_kd = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_speed_kp = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_speed_ki = value;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KD_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_speed_kd = value;
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_MIN_MECH_DELTA:
        if (value < 0.0f)
        {
            return 0U;
        }
        FOC_ControlSetMinMechAngleAccumDeltaRad(value);
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_HOLD_I_LIMIT:
        if (value < 0.0f)
        {
            return 0U;
        }
        FOC_ControlSetAngleHoldIntegralLimit(value);
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_HOLD_DEADBAND:
        if (value < 0.0f)
        {
            return 0U;
        }
        FOC_ControlSetAngleHoldPidDeadbandRad(value);
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_BLEND_START:
        if (value < 0.0f)
        {
            return 0U;
        }
        FOC_ControlSetSpeedAngleTransitionStartRad(value);
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_BLEND_END:
        if (value < 0.0f)
        {
            return 0U;
        }
        FOC_ControlSetSpeedAngleTransitionEndRad(value);
        break;

    case COMMAND_MANAGER_SUBCMD_CONTROL_MODE:
        if ((value < 0.0f) || (value > 1.0f))
        {
            return 0U;
        }
        g_params.control_mode = (uint8_t)value;
        break;

    case COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE:
        g_params.motor_enable = (value != 0.0f) ? COMMAND_MANAGER_ENABLED_ENABLE : COMMAND_MANAGER_ENABLED_DISABLE;
        break;

    default:
        return 0U;
    }

    g_runtime_state.params_dirty = 1U;
    return 1U;
}

uint8_t CommandManager_ReadParam(char subcommand, float *value_out)
{
    if (value_out == 0)
    {
        return 0U;
    }

    switch (subcommand)
    {
    case COMMAND_MANAGER_SUBCMD_TARGET_ANGLE:
        *value_out = g_params.target_angle_rad;
        break;

    case COMMAND_MANAGER_SUBCMD_ANGLE_SPEED:
        *value_out = g_params.angle_speed_rad_s;
        break;

    case COMMAND_MANAGER_SUBCMD_SEMANTIC_DIV:
        *value_out = (float)g_params.semantic_div;
        break;

    case COMMAND_MANAGER_SUBCMD_OSC_DIV:
        *value_out = (float)g_params.osc_div;
        break;

    case COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE:
        *value_out = (float)g_params.semantic_enable;
        break;

    case COMMAND_MANAGER_SUBCMD_OSC_ENABLE:
        *value_out = (float)g_params.osc_enable;
        break;

    case COMMAND_MANAGER_SUBCMD_OSC_PARAM_MASK:
        *value_out = (float)g_params.osc_param_mask;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KP:
        *value_out = g_params.pid_current_kp;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KI:
        *value_out = g_params.pid_current_ki;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KD:
        *value_out = g_params.pid_current_kd;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KP:
        *value_out = g_params.pid_angle_kp;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KI:
        *value_out = g_params.pid_angle_ki;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KD:
        *value_out = g_params.pid_angle_kd;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KP:
        *value_out = g_params.pid_speed_kp;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KI:
        *value_out = g_params.pid_speed_ki;
        break;

    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KD:
        *value_out = g_params.pid_speed_kd;
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_MIN_MECH_DELTA:
        *value_out = FOC_ControlGetRuntimeConfig()->min_mech_angle_accum_delta_rad;
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_HOLD_I_LIMIT:
        *value_out = FOC_ControlGetRuntimeConfig()->angle_hold_integral_limit;
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_HOLD_DEADBAND:
        *value_out = FOC_ControlGetRuntimeConfig()->angle_hold_pid_deadband_rad;
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_BLEND_START:
        *value_out = FOC_ControlGetRuntimeConfig()->speed_angle_transition_start_rad;
        break;

    case COMMAND_MANAGER_SUBCMD_CFG_BLEND_END:
        *value_out = FOC_ControlGetRuntimeConfig()->speed_angle_transition_end_rad;
        break;

    case COMMAND_MANAGER_SUBCMD_CONTROL_MODE:
        *value_out = (float)g_params.control_mode;
        break;

    case COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE:
        *value_out = (float)g_params.motor_enable;
        break;

    default:
        return 0U;
    }

    return 1U;
}

void CommandManager_ReportAllParams(void)
{
    float value;
    const char params[] = {
        COMMAND_MANAGER_SUBCMD_TARGET_ANGLE,
        COMMAND_MANAGER_SUBCMD_ANGLE_SPEED,
        COMMAND_MANAGER_SUBCMD_SEMANTIC_DIV,
        COMMAND_MANAGER_SUBCMD_OSC_DIV,
        COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE,
        COMMAND_MANAGER_SUBCMD_OSC_ENABLE,
        COMMAND_MANAGER_SUBCMD_OSC_PARAM_MASK,
        COMMAND_MANAGER_SUBCMD_PID_CURRENT_KP,
        COMMAND_MANAGER_SUBCMD_PID_CURRENT_KI,
        COMMAND_MANAGER_SUBCMD_PID_CURRENT_KD,
        COMMAND_MANAGER_SUBCMD_PID_ANGLE_KP,
        COMMAND_MANAGER_SUBCMD_PID_ANGLE_KI,
        COMMAND_MANAGER_SUBCMD_PID_ANGLE_KD,
        COMMAND_MANAGER_SUBCMD_PID_SPEED_KP,
        COMMAND_MANAGER_SUBCMD_PID_SPEED_KI,
        COMMAND_MANAGER_SUBCMD_PID_SPEED_KD,
        COMMAND_MANAGER_SUBCMD_CFG_MIN_MECH_DELTA,
        COMMAND_MANAGER_SUBCMD_CFG_HOLD_I_LIMIT,
        COMMAND_MANAGER_SUBCMD_CFG_HOLD_DEADBAND,
        COMMAND_MANAGER_SUBCMD_CFG_BLEND_START,
        COMMAND_MANAGER_SUBCMD_CFG_BLEND_END,
        COMMAND_MANAGER_SUBCMD_CONTROL_MODE,
        COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE
    };
    uint16_t i;

    for (i = 0U; i < (uint16_t)(sizeof(params) / sizeof(params[0])); i++)
    {
        if (CommandManager_ReadParam(params[i], &value) != 0U)
        {
            CommandManager_OutputParam(params[i], value);
        }
    }
}

float CommandManager_GetTargetAngleRad(void)
{
    return g_params.target_angle_rad;
}

float CommandManager_GetAngleSpeedRadS(void)
{
    return g_params.angle_speed_rad_s;
}

uint8_t CommandManager_IsSemanticReportEnabled(void)
{
    return g_params.semantic_enable;
}

uint8_t CommandManager_IsOscilloscopeReportEnabled(void)
{
    return g_params.osc_enable;
}

uint16_t CommandManager_GetSemanticReportDivider(void)
{
    return g_params.semantic_div;
}

uint16_t CommandManager_GetOscilloscopeReportDivider(void)
{
    return g_params.osc_div;
}

uint16_t CommandManager_GetOscilloscopeParameterMask(void)
{
    return g_params.osc_param_mask;
}

void CommandManager_ClearDirtyFlag(void)
{
    g_runtime_state.params_dirty = 0U;
}

float CommandManager_GetCurrentPidKp(void)
{
    return g_params.pid_current_kp;
}

float CommandManager_GetCurrentPidKi(void)
{
    return g_params.pid_current_ki;
}

float CommandManager_GetCurrentPidKd(void)
{
    return g_params.pid_current_kd;
}

float CommandManager_GetAnglePidKp(void)
{
    return g_params.pid_angle_kp;
}

float CommandManager_GetAnglePidKi(void)
{
    return g_params.pid_angle_ki;
}

float CommandManager_GetAnglePidKd(void)
{
    return g_params.pid_angle_kd;
}

float CommandManager_GetSpeedPidKp(void)
{
    return g_params.pid_speed_kp;
}

float CommandManager_GetSpeedPidKi(void)
{
    return g_params.pid_speed_ki;
}

float CommandManager_GetSpeedPidKd(void)
{
    return g_params.pid_speed_kd;
}

uint8_t CommandManager_GetControlMode(void)
{
    return g_params.control_mode;
}

uint8_t CommandManager_IsMotorEnabled(void)
{
    return g_params.motor_enable;
}

static void CommandManager_ReportInitDiag(void)
{
    FOC_Platform_DebugOutput("diag.command_manager=READY\r\n");
    FOC_Platform_DebugOutput("diag.protocol_exec=NOT_EXECUTED\r\n");
    FOC_Platform_DebugOutput("diag.fallback=KEEP_LAST_VALID\r\n");
}

static command_exec_result_t CommandManager_Execute(const protocol_command_t *cmd)
{
    if (cmd->frame_valid == 0U)
    {
        FOC_Platform_FeedbackOutput((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
        return COMMAND_EXEC_RESULT_COMMAND_ERROR;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_WRITE_PARAM)
    {
        if (cmd->has_param == 0U)
        {
            FOC_Platform_FeedbackOutput((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            return COMMAND_EXEC_RESULT_PARAM_ERROR;
        }

        if (CommandManager_WriteParam(cmd->subcommand, cmd->param_value) == 0U)
        {
            FOC_Platform_FeedbackOutput((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            return COMMAND_EXEC_RESULT_PARAM_ERROR;
        }

        CommandManager_OutputParam(cmd->subcommand, cmd->param_value);
        return COMMAND_EXEC_RESULT_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_READ_PARAM)
    {
        if (cmd->subcommand == COMMAND_MANAGER_SUBCMD_READ_ALL)
        {
            CommandManager_ReportAllParams();
            return COMMAND_EXEC_RESULT_OK;
        }

        if (CommandManager_ReportSingleParam(cmd->subcommand) == 0U)
        {
            FOC_Platform_FeedbackOutput((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            return COMMAND_EXEC_RESULT_PARAM_ERROR;
        }

        return COMMAND_EXEC_RESULT_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_READ_STATE)
    {
        char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
        const command_manager_runtime_state_t *state = CommandManager_GetRuntimeState();

        snprintf(out,
                 sizeof(out),
                 "STATE SYS=%u COMM=%u REPORT=%u DIRTY=%u LAST=%u INIT=%u FAULT=%s SENS_INV=%u PROTO_ERR=%lu PARAM_ERR=%lu CTRL_SKIP=%lu\r\n",
                 (unsigned int)state->system_state,
                 (unsigned int)state->comm_state,
                 (unsigned int)state->report_mode,
                 (unsigned int)state->params_dirty,
                 (unsigned int)state->last_exec_ok,
                 (unsigned int)state->init_diag,
                 CommandManager_GetFaultName(state->last_fault_code),
                 (unsigned int)state->sensor_invalid_consecutive,
                 (unsigned long)state->protocol_error_count,
                 (unsigned long)state->param_error_count,
                 (unsigned long)state->control_skip_count);
        FOC_Platform_DebugOutput(out);
        return COMMAND_EXEC_RESULT_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_FAULT_CONTROL)
    {
        if (cmd->subcommand == COMMAND_MANAGER_SUBCMD_FAULT_CLEAR_REINIT)
        {
            if (CommandManager_RecoverFaultAndReinit() != 0U)
            {
                const command_manager_runtime_state_t *state = CommandManager_GetRuntimeState();
                char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

                snprintf(out,
                         sizeof(out),
                         "FAULT_CTRL state=%u fault=%s proto_err=%lu param_err=%lu ctrl_skip=%lu\r\n",
                         (unsigned int)state->system_state,
                         CommandManager_GetFaultName(state->last_fault_code),
                         (unsigned long)state->protocol_error_count,
                         (unsigned long)state->param_error_count,
                         (unsigned long)state->control_skip_count);
                FOC_Platform_DebugOutput(out);
                return COMMAND_EXEC_RESULT_OK;
            }

            return COMMAND_EXEC_RESULT_COMMAND_ERROR;
        }

        FOC_Platform_FeedbackOutput((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        return COMMAND_EXEC_RESULT_PARAM_ERROR;
    }

    FOC_Platform_FeedbackOutput((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
    return COMMAND_EXEC_RESULT_COMMAND_ERROR;
}

static uint8_t CommandManager_IsInRange(float value, float min_value, float max_value)
{
    if ((value < min_value) || (value > max_value))
    {
        return 0U;
    }
    return 1U;
}

static uint8_t CommandManager_ReportSingleParam(char subcommand)
{
    float value;

    if (CommandManager_ReadParam(subcommand, &value) == 0U)
    {
        return 0U;
    }

    CommandManager_OutputParam(subcommand, value);
    return 1U;
}

static const char *CommandManager_GetParamName(char subcommand)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_SUBCMD_TARGET_ANGLE:
        return "target_angle_rad";
    case COMMAND_MANAGER_SUBCMD_ANGLE_SPEED:
        return "angle_position_speed_rad_s";
    case COMMAND_MANAGER_SUBCMD_SEMANTIC_DIV:
        return "semantic_report_divider";
    case COMMAND_MANAGER_SUBCMD_OSC_DIV:
        return "oscilloscope_report_divider";
    case COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE:
        return "semantic_report_enabled";
    case COMMAND_MANAGER_SUBCMD_OSC_ENABLE:
        return "oscilloscope_report_enabled";
    case COMMAND_MANAGER_SUBCMD_OSC_PARAM_MASK:
        return "oscilloscope_param_mask";
    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KP:
        return "pid_current_kp";
    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KI:
        return "pid_current_ki";
    case COMMAND_MANAGER_SUBCMD_PID_CURRENT_KD:
        return "pid_current_kd";
    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KP:
        return "pid_angle_kp";
    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KI:
        return "pid_angle_ki";
    case COMMAND_MANAGER_SUBCMD_PID_ANGLE_KD:
        return "pid_angle_kd";
    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KP:
        return "pid_speed_kp";
    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KI:
        return "pid_speed_ki";
    case COMMAND_MANAGER_SUBCMD_PID_SPEED_KD:
        return "pid_speed_kd";
    case COMMAND_MANAGER_SUBCMD_CFG_MIN_MECH_DELTA:
        return "control_min_mech_angle_accum_delta_rad";
    case COMMAND_MANAGER_SUBCMD_CFG_HOLD_I_LIMIT:
        return "control_angle_hold_integral_limit";
    case COMMAND_MANAGER_SUBCMD_CFG_HOLD_DEADBAND:
        return "control_angle_hold_pid_deadband_rad";
    case COMMAND_MANAGER_SUBCMD_CFG_BLEND_START:
        return "control_speed_angle_transition_start_rad";
    case COMMAND_MANAGER_SUBCMD_CFG_BLEND_END:
        return "control_speed_angle_transition_end_rad";
    case COMMAND_MANAGER_SUBCMD_CONTROL_MODE:
        return "control_mode";
    case COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE:
        return "motor_enable";
    default:
        return "unknown";
    }
}

static uint8_t CommandManager_IsIntegerParam(char subcommand)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_SUBCMD_SEMANTIC_DIV:
    case COMMAND_MANAGER_SUBCMD_OSC_DIV:
    case COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE:
    case COMMAND_MANAGER_SUBCMD_OSC_ENABLE:
    case COMMAND_MANAGER_SUBCMD_OSC_PARAM_MASK:
    case COMMAND_MANAGER_SUBCMD_CONTROL_MODE:
    case COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE:
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t CommandManager_IsEnableParam(char subcommand)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_SUBCMD_SEMANTIC_ENABLE:
    case COMMAND_MANAGER_SUBCMD_OSC_ENABLE:
    case COMMAND_MANAGER_SUBCMD_MOTOR_ENABLE:
        return 1U;
    default:
        return 0U;
    }
}

static const char *CommandManager_GetFaultName(command_manager_fault_code_t fault_code)
{
    switch (fault_code)
    {
    case COMMAND_MANAGER_FAULT_NONE:
        return "NONE";
    case COMMAND_MANAGER_FAULT_SENSOR_ADC_INVALID:
        return "SENSOR_ADC_INVALID";
    case COMMAND_MANAGER_FAULT_SENSOR_ENCODER_INVALID:
        return "SENSOR_ENCODER_INVALID";
    case COMMAND_MANAGER_FAULT_PROTOCOL_FRAME:
        return "PROTOCOL_FRAME";
    case COMMAND_MANAGER_FAULT_PARAM_INVALID:
        return "PARAM_INVALID";
    case COMMAND_MANAGER_FAULT_INIT_FAILED:
        return "INIT_FAILED";
    default:
        return "UNKNOWN";
    }
}

static void CommandManager_OutputDiag(const char *level, const char *module, const char *detail)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "diag.level=%s module=%s detail=%s\r\n",
             (level != 0) ? level : "INFO",
             (module != 0) ? module : "general",
             (detail != 0) ? detail : "none");
    FOC_Platform_DebugOutput(out);
}

static void CommandManager_OutputParam(char subcommand, float value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    if (CommandManager_IsEnableParam(subcommand) != 0U)
    {
        snprintf(out,
                 sizeof(out),
                 "parameter.%s=%s\r\n",
                 CommandManager_GetParamName(subcommand),
                 (value != 0.0f) ? "ENABLE" : "DISABLE");
        FOC_Platform_DebugOutput(out);
        return;
    }

    if (CommandManager_IsIntegerParam(subcommand) != 0U)
    {
        snprintf(out,
                 sizeof(out),
                 "parameter.%s=%u\r\n",
                 CommandManager_GetParamName(subcommand),
                 (unsigned int)((value < 0.0f) ? 0U : (uint16_t)value));
    }
    else
    {
        snprintf(out,
                 sizeof(out),
                 "parameter.%s=%.3f\r\n",
                 CommandManager_GetParamName(subcommand),
                 value);
    }

    FOC_Platform_DebugOutput(out);
}
