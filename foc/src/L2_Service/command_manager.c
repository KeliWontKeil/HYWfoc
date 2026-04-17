#include "L2_Service/command_manager.h"

#include <stdio.h>

#include "L42_PAL/foc_platform_api.h"
#include "L2_Service/protocol_parser.h"
#include "L2_Service/command_manager_diag.h"
#include "L2_Service/command_manager_dispatch.h"
#include "LS_Config/foc_config.h"

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

static command_manager_runtime_state_t g_runtime_state;
static command_manager_params_t g_params;
static command_manager_states_t g_states;

static uint8_t CommandManager_IsInRange(float value, float min_value, float max_value);

#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
#define CMD_DIAG_OUTPUT(text) FOC_Platform_WriteDebugText(text)
#else
#define CMD_DIAG_OUTPUT(text) ((void)0)
#endif

#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
#define CMD_DIAG_STATS_INC(field_name) do { g_runtime_state.field_name++; } while (0)
#else
#define CMD_DIAG_STATS_INC(field_name) ((void)0)
#endif

void CommandManager_Init(void)
{
    ProtocolParser_Init();

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
    g_params.speed_only_rad_s = COMMAND_MANAGER_DEFAULT_SPEED_ONLY_RAD_S;
    g_params.sensor_sample_offset_percent = FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT;
    g_params.semantic_freq_hz = COMMAND_MANAGER_DEFAULT_SEMANTIC_FREQ_HZ;
    g_params.osc_freq_hz = COMMAND_MANAGER_DEFAULT_OSC_FREQ_HZ;
    g_params.osc_param_mask = COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK;
    g_params.pid_current_kp = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP;
    g_params.pid_current_ki = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI;
    g_params.pid_current_kd = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD;
    g_params.pid_angle_kp = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP;
    g_params.pid_angle_ki = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI;
    g_params.pid_angle_kd = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD;
    g_params.pid_speed_kp = COMMAND_MANAGER_DEFAULT_PID_SPEED_KP;
    g_params.pid_speed_ki = COMMAND_MANAGER_DEFAULT_PID_SPEED_KI;
    g_params.pid_speed_kd = COMMAND_MANAGER_DEFAULT_PID_SPEED_KD;
    g_params.cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    g_params.cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    g_params.cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    g_params.cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    g_params.cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
    g_params.control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    g_params.current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    g_params.current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    g_params.current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;

    g_states.motor_enable = COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;
    g_states.semantic_enable = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
    g_states.osc_enable = COMMAND_MANAGER_DEFAULT_OSC_ENABLED;
    g_states.current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;

    if ((g_states.semantic_enable != 0U) && (g_states.osc_enable != 0U))
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_BOTH;
    }
    else if (g_states.semantic_enable != 0U)
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_SEMANTIC_ONLY;
    }
    else if (g_states.osc_enable != 0U)
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_OSC_ONLY;
    }
    else
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_OFF;
    }

    CommandManager_DispatchReportInitDiag();
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
    exec_result = CommandManager_DispatchExecute(cmd);
    g_runtime_state.last_exec_ok = (exec_result == COMMAND_EXEC_RESULT_OK) ? 1U : 0U;

    if (exec_result != COMMAND_EXEC_RESULT_OK)
    {
        g_runtime_state.comm_state = COMMAND_MANAGER_COMM_ERROR;

        if (exec_result == COMMAND_EXEC_RESULT_PARAM_ERROR)
        {
            CMD_DIAG_STATS_INC(param_error_count);
            g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PARAM_INVALID;
        }
        else
        {
            CMD_DIAG_STATS_INC(protocol_error_count);
            g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PROTOCOL_FRAME;
        }

        CMD_DIAG_OUTPUT("ERR fallback: keep previous params\r\n");
    }
    else
    {
        g_runtime_state.comm_state = COMMAND_MANAGER_COMM_ACTIVE;
    }

    if ((g_states.semantic_enable != 0U) && (g_states.osc_enable != 0U))
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_BOTH;
    }
    else if (g_states.semantic_enable != 0U)
    {
        g_runtime_state.report_mode = COMMAND_MANAGER_REPORT_SEMANTIC_ONLY;
    }
    else if (g_states.osc_enable != 0U)
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

#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    {
        char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

        if (missing_mask != 0U)
        {
            snprintf(out,
                     sizeof(out),
                     "diag.init.missing=0x%04X required=0x%04X\r\n",
                     (unsigned int)missing_mask,
                     (unsigned int)COMMAND_MANAGER_INIT_REQUIRED_MASK);
            FOC_Platform_WriteDebugText(out);
        }

        snprintf(out,
                 sizeof(out),
                 "diag.init.result=%u checks=0x%04X fails=0x%04X\r\n",
                 (unsigned int)g_runtime_state.init_diag,
                 (unsigned int)g_runtime_state.init_check_mask,
                 (unsigned int)g_runtime_state.init_fail_mask);
        FOC_Platform_WriteDebugText(out);
    }
#endif
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

    CMD_DIAG_STATS_INC(control_skip_count);
    g_runtime_state.sensor_invalid_consecutive++;

    if (adc_valid == 0U)
    {
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_SENSOR_ADC_INVALID;
    }
    else
    {
        g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_SENSOR_ENCODER_INVALID;
    }

    if (g_runtime_state.sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
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

void CommandManager_ReportUndervoltageFault(float vbus_voltage)
{
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_FAULT;
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_UNDERVOLTAGE;
    CMD_DIAG_STATS_INC(control_skip_count);

    snprintf(out,
             sizeof(out),
             "vbus undervoltage: %.3fV < %.3fV",
             vbus_voltage,
             FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT);
    CommandManager_OutputDiag("ERR", "protection", out);
#else
    (void)vbus_voltage;
#endif
}

void CommandManager_ReportProtocolFrameError(void)
{
    CMD_DIAG_STATS_INC(protocol_error_count);
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PROTOCOL_FRAME;
}

void CommandManager_ReportControlLoopSkip(void)
{
    CMD_DIAG_STATS_INC(control_skip_count);
}

uint8_t CommandManager_RecoverFaultAndReinit(void)
{
    g_runtime_state.sensor_invalid_consecutive = 0U;
    g_runtime_state.protocol_error_count = 0U;
    g_runtime_state.param_error_count = 0U;
    g_runtime_state.control_skip_count = 0U;
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_NONE;
    g_runtime_state.comm_state = COMMAND_MANAGER_COMM_IDLE;
    g_runtime_state.system_state = COMMAND_MANAGER_SYSTEM_RUNNING;
    g_runtime_state.params_dirty = 1U;
    g_runtime_state.last_exec_ok = 1U;

    g_params.cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    g_params.cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    g_params.cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    g_params.cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    g_params.cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
    g_params.current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    g_params.current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    g_params.current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    g_states.current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;

#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    {
        char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
        snprintf(out,
                 sizeof(out),
                 "diag.reinit=SOFT_DIAG_RESET state=%u fault=%s\r\n",
                 (unsigned int)g_runtime_state.system_state,
                 CommandManager_GetFaultName(g_runtime_state.last_fault_code));
        FOC_Platform_WriteDebugText(out);
    }
#endif

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
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD,
                                     COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD) == 0U)
        {
            return 0U;
        }
        g_params.target_angle_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S,
                                     COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        g_params.angle_speed_rad_s = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_SPEED_ONLY_MIN_RAD_S,
                                     COMMAND_MANAGER_PARAM_SPEED_ONLY_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        g_params.speed_only_rad_s = value;
        break;

#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_SENSOR_SAMPLE_OFFSET_MIN_PERCENT,
                                     COMMAND_MANAGER_PARAM_SENSOR_SAMPLE_OFFSET_MAX_PERCENT) == 0U)
        {
            return 0U;
        }
        g_params.sensor_sample_offset_percent = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        if ((value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ) ||
            (value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ))
        {
            return 0U;
        }
        g_params.semantic_freq_hz = (uint16_t)value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        if ((value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ) ||
            (value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ))
        {
            return 0U;
        }
        g_params.osc_freq_hz = (uint16_t)value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        if ((value < COMMAND_MANAGER_PARAM_OSC_MASK_MIN) ||
            (value > COMMAND_MANAGER_PARAM_OSC_MASK_MAX))
        {
            return 0U;
        }
        g_params.osc_param_mask = (uint16_t)value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_current_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_current_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_current_kd = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_angle_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_angle_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_angle_kd = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_speed_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_speed_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KD_MAX) == 0U)
        {
            return 0U;
        }
        g_params.pid_speed_kd = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
        if (value < 0.0f)
        {
            return 0U;
        }
        g_params.cfg_min_mech_angle_accum_delta_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
        if (value < 0.0f)
        {
            return 0U;
        }
        g_params.cfg_angle_hold_integral_limit = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
        if (value < 0.0f)
        {
            return 0U;
        }
        g_params.cfg_angle_hold_pid_deadband_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
        if (value < 0.0f)
        {
            return 0U;
        }
        g_params.cfg_speed_angle_transition_start_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
        if (value < 0.0f)
        {
            return 0U;
        }
        g_params.cfg_speed_angle_transition_end_rad = value;
        break;
    #endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CONTROL_MODE_MIN) ||
            (value > COMMAND_MANAGER_PARAM_CONTROL_MODE_MAX))
        {
            return 0U;
        }
        g_params.control_mode = (uint8_t)value;
        break;

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MIN) ||
            (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MAX))
        {
            return 0U;
        }
        g_params.current_soft_switch_mode = (uint8_t)value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MIN_A) ||
            (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MAX_A) ||
            (value > g_params.current_soft_switch_auto_closed_iq_a))
        {
            return 0U;
        }
        g_params.current_soft_switch_auto_open_iq_a = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MIN_A) ||
            (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MAX_A) ||
            (value < g_params.current_soft_switch_auto_open_iq_a))
        {
            return 0U;
        }
        g_params.current_soft_switch_auto_closed_iq_a = value;
        break;
    #endif

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
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        *value_out = g_params.target_angle_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        *value_out = g_params.angle_speed_rad_s;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        *value_out = g_params.speed_only_rad_s;
        break;

#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        *value_out = g_params.sensor_sample_offset_percent;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        *value_out = (float)g_params.semantic_freq_hz;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        *value_out = (float)g_params.osc_freq_hz;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        *value_out = (float)g_params.osc_param_mask;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
        *value_out = g_params.pid_current_kp;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        *value_out = g_params.pid_current_ki;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        *value_out = g_params.pid_current_kd;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
        *value_out = g_params.pid_angle_kp;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        *value_out = g_params.pid_angle_ki;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        *value_out = g_params.pid_angle_kd;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
        *value_out = g_params.pid_speed_kp;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        *value_out = g_params.pid_speed_ki;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        *value_out = g_params.pid_speed_kd;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
        *value_out = g_params.cfg_min_mech_angle_accum_delta_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
        *value_out = g_params.cfg_angle_hold_integral_limit;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
        *value_out = g_params.cfg_angle_hold_pid_deadband_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
        *value_out = g_params.cfg_speed_angle_transition_start_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
        *value_out = g_params.cfg_speed_angle_transition_end_rad;
        break;
#endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        *value_out = (float)g_params.control_mode;
        break;

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        *value_out = (float)g_params.current_soft_switch_mode;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        *value_out = g_params.current_soft_switch_auto_open_iq_a;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        *value_out = g_params.current_soft_switch_auto_closed_iq_a;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

void CommandManager_ReportAllParams(void)
{
    float value;
    const char params[] = {
        COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE,
        COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED,
        COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED,
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET,
#endif
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV,
        COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV,
        COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END,
#endif
        COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE,
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE,
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ,
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ
#endif
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

uint8_t CommandManager_WriteState(char subcommand, uint8_t state)
{
    uint8_t normalized_state = (state != 0U) ? COMMAND_MANAGER_ENABLED_ENABLE : COMMAND_MANAGER_ENABLED_DISABLE;

    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        g_states.motor_enable = normalized_state;
        break;

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
        g_states.semantic_enable = normalized_state;
        break;

    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
        g_states.osc_enable = normalized_state;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
        g_states.current_soft_switch_enable = normalized_state;
        g_runtime_state.params_dirty = 1U;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

uint8_t CommandManager_ReadState(char subcommand, uint8_t *state_out)
{
    if (state_out == 0)
    {
        return 0U;
    }

    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        *state_out = g_states.motor_enable;
        break;

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
        *state_out = g_states.semantic_enable;
        break;

    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
        *state_out = g_states.osc_enable;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
        *state_out = g_states.current_soft_switch_enable;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

void CommandManager_ReportAllStates(void)
{
    uint8_t state;
    const char states[] = {
        COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE,
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE,
        COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE
#endif
    };
    uint16_t i;

    for (i = 0U; i < (uint16_t)(sizeof(states) / sizeof(states[0])); i++)
    {
        if (CommandManager_ReadState(states[i], &state) != 0U)
        {
            CommandManager_OutputState(states[i], state);
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

float CommandManager_GetSpeedOnlyRadS(void)
{
    return g_params.speed_only_rad_s;
}

float CommandManager_GetSensorSampleOffsetPercent(void)
{
    return g_params.sensor_sample_offset_percent;
}

uint8_t CommandManager_IsSemanticReportEnabled(void)
{
    return g_states.semantic_enable;
}

uint8_t CommandManager_IsOscilloscopeReportEnabled(void)
{
    return g_states.osc_enable;
}

uint16_t CommandManager_GetSemanticReportFrequencyHz(void)
{
    return g_params.semantic_freq_hz;
}

uint16_t CommandManager_GetOscilloscopeReportFrequencyHz(void)
{
    return g_params.osc_freq_hz;
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

float CommandManager_GetControlMinMechAngleAccumDeltaRad(void)
{
    return g_params.cfg_min_mech_angle_accum_delta_rad;
}

float CommandManager_GetControlAngleHoldIntegralLimit(void)
{
    return g_params.cfg_angle_hold_integral_limit;
}

float CommandManager_GetControlAngleHoldPidDeadbandRad(void)
{
    return g_params.cfg_angle_hold_pid_deadband_rad;
}

float CommandManager_GetControlSpeedAngleTransitionStartRad(void)
{
    return g_params.cfg_speed_angle_transition_start_rad;
}

float CommandManager_GetControlSpeedAngleTransitionEndRad(void)
{
    return g_params.cfg_speed_angle_transition_end_rad;
}

uint8_t CommandManager_GetControlMode(void)
{
    return g_params.control_mode;
}

uint8_t CommandManager_IsMotorEnabled(void)
{
    return g_states.motor_enable;
}

uint8_t CommandManager_IsCurrentSoftSwitchEnabled(void)
{
    return g_states.current_soft_switch_enable;
}

uint8_t CommandManager_GetCurrentSoftSwitchMode(void)
{
    return g_params.current_soft_switch_mode;
}

float CommandManager_GetCurrentSoftSwitchAutoOpenIqA(void)
{
    return g_params.current_soft_switch_auto_open_iq_a;
}

float CommandManager_GetCurrentSoftSwitchAutoClosedIqA(void)
{
    return g_params.current_soft_switch_auto_closed_iq_a;
}

static uint8_t CommandManager_IsInRange(float value, float min_value, float max_value)
{
    if ((value < min_value) || (value > max_value))
    {
        return 0U;
    }
    return 1U;
}
