#include "L2_Service/runtime_c41_command_entry.h"

#include <stdio.h>
#include <string.h>

#include "L2_Service/runtime_c35_protocol_parser.h"
#include "L2_Service/runtime_c44_command_diag.h"
#include "L2_Service/runtime_c42_command_dispatch.h"
#include "L2_Service/runtime_c43_command_store.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define g_runtime_state (*CommandManager_InternalRuntimeState())
#define g_params (*CommandManager_InternalParams())
#define g_states (*CommandManager_InternalStates())

static void CommandManager_UpdateReportMode(void);
static uint8_t CommandManager_ProcessLatestCommand(void);
static command_exec_result_t CommandManager_HandleSystemCommand(const protocol_command_t *cmd);
static void CommandManager_OutputRuntimeSummary(const command_manager_runtime_state_t *state);
static void CommandManager_ReportProtocolFrameError(void);
static uint8_t CommandManager_RecoverFaultAndReinit(void);

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

static void CommandManager_UpdateReportMode(void)
{
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
}

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

    CommandManager_UpdateReportMode();

    CommandManager_DispatchReportInitDiag();
}

uint8_t CommandManager_ProcessCommStep(uint8_t max_frames)
{
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

    if (max_frames == 0U)
    {
        return 0U;
    }

    while ((ProtocolParser_IsParsePending() != 0U) && (consumed < max_frames))
    {
        ProtocolParser_Process();

        if (CommandManager_ProcessLatestCommand() != 0U)
        {
            has_comm_activity = 1U;
        }

        consumed++;
    }

    return has_comm_activity;
}

static uint8_t CommandManager_ProcessLatestCommand(void)
{
    const protocol_command_t *cmd = ProtocolParser_GetLatestCommand();
    command_exec_result_t exec_result;

    if (ProtocolParser_GetLastResult() == PROTOCOL_PARSER_RESULT_FRAME_ERROR)
    {
        CommandManager_ReportProtocolFrameError();
    }

    if ((cmd == 0) || (cmd->updated == 0U))
    {
        return 0U;
    }

    g_runtime_state.comm_state = COMMAND_MANAGER_COMM_ACTIVE;

    if (cmd->command == COMMAND_MANAGER_CMD_SYSTEM)
    {
        exec_result = CommandManager_HandleSystemCommand(cmd);
    }
    else
    {
        exec_result = CommandManager_DispatchExecute(cmd);
    }

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

    CommandManager_UpdateReportMode();

    ProtocolParser_ClearUpdatedFlag();
    return g_runtime_state.last_exec_ok;
}

static command_exec_result_t CommandManager_HandleSystemCommand(const protocol_command_t *cmd)
{
    if (cmd->has_param != 0U)
    {
        FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        return COMMAND_EXEC_RESULT_PARAM_ERROR;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY)
    {
        CommandManager_OutputRuntimeSummary(&g_runtime_state);
        return COMMAND_EXEC_RESULT_OK;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT)
    {
        if (CommandManager_RecoverFaultAndReinit() != 0U)
        {
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
            CommandManager_OutputRuntimeSummary(&g_runtime_state);
#endif
            return COMMAND_EXEC_RESULT_OK;
        }

        return COMMAND_EXEC_RESULT_COMMAND_ERROR;
    }

    FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
    return COMMAND_EXEC_RESULT_PARAM_ERROR;
}

static void CommandManager_OutputRuntimeSummary(const command_manager_runtime_state_t *state)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

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
    FOC_Platform_WriteDebugText(out);
#else
    (void)state;
#endif
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

static void CommandManager_ReportProtocolFrameError(void)
{
    CMD_DIAG_STATS_INC(protocol_error_count);
    g_runtime_state.last_fault_code = COMMAND_MANAGER_FAULT_PROTOCOL_FRAME;
}

void CommandManager_ReportControlLoopSkip(void)
{
    CMD_DIAG_STATS_INC(control_skip_count);
}

static uint8_t CommandManager_RecoverFaultAndReinit(void)
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

void CommandManager_CaptureSnapshot(runtime_snapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));

    snapshot->runtime.system_running = (g_runtime_state.system_state == COMMAND_MANAGER_SYSTEM_RUNNING) ? 1U : 0U;
    snapshot->runtime.system_fault = (g_runtime_state.system_state == COMMAND_MANAGER_SYSTEM_FAULT) ? 1U : 0U;
    snapshot->runtime.params_dirty = g_runtime_state.params_dirty;
    snapshot->runtime.last_exec_ok = g_runtime_state.last_exec_ok;

    snapshot->control_cfg.control_mode = g_params.control_mode;
    snapshot->control_cfg.target_angle_rad = g_params.target_angle_rad;
    snapshot->control_cfg.angle_position_speed_rad_s = g_params.angle_speed_rad_s;
    snapshot->control_cfg.speed_only_rad_s = g_params.speed_only_rad_s;
    snapshot->control_cfg.sensor_sample_offset_percent = g_params.sensor_sample_offset_percent;
    snapshot->control_cfg.pid_current_kp = g_params.pid_current_kp;
    snapshot->control_cfg.pid_current_ki = g_params.pid_current_ki;
    snapshot->control_cfg.pid_current_kd = g_params.pid_current_kd;
    snapshot->control_cfg.pid_angle_kp = g_params.pid_angle_kp;
    snapshot->control_cfg.pid_angle_ki = g_params.pid_angle_ki;
    snapshot->control_cfg.pid_angle_kd = g_params.pid_angle_kd;
    snapshot->control_cfg.pid_speed_kp = g_params.pid_speed_kp;
    snapshot->control_cfg.pid_speed_ki = g_params.pid_speed_ki;
    snapshot->control_cfg.pid_speed_kd = g_params.pid_speed_kd;
    snapshot->control_cfg.cfg_min_mech_angle_accum_delta_rad = g_params.cfg_min_mech_angle_accum_delta_rad;
    snapshot->control_cfg.cfg_angle_hold_integral_limit = g_params.cfg_angle_hold_integral_limit;
    snapshot->control_cfg.cfg_angle_hold_pid_deadband_rad = g_params.cfg_angle_hold_pid_deadband_rad;
    snapshot->control_cfg.cfg_speed_angle_transition_start_rad = g_params.cfg_speed_angle_transition_start_rad;
    snapshot->control_cfg.cfg_speed_angle_transition_end_rad = g_params.cfg_speed_angle_transition_end_rad;
    snapshot->control_cfg.motor_enabled = g_states.motor_enable;
    snapshot->control_cfg.current_soft_switch_enable = g_states.current_soft_switch_enable;
    snapshot->control_cfg.current_soft_switch_mode = g_params.current_soft_switch_mode;
    snapshot->control_cfg.current_soft_switch_auto_open_iq_a = g_params.current_soft_switch_auto_open_iq_a;
    snapshot->control_cfg.current_soft_switch_auto_closed_iq_a = g_params.current_soft_switch_auto_closed_iq_a;

    snapshot->telemetry.semantic_report_enabled = g_states.semantic_enable;
    snapshot->telemetry.osc_report_enabled = g_states.osc_enable;
    snapshot->telemetry.semantic_report_freq_hz = g_params.semantic_freq_hz;
    snapshot->telemetry.osc_report_freq_hz = g_params.osc_freq_hz;
    snapshot->telemetry.osc_parameter_mask = g_params.osc_param_mask;
}

