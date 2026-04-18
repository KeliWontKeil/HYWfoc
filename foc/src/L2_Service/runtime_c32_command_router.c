#include "L2_Service/runtime_c32_command_router.h"

#include "L3_Algorithm/protocol_core.h"
#include "LS_Config/foc_config.h"

void RuntimeCommandRouter_Init(void)
{
    RuntimeStore_ResetStorageDefaults();
    RuntimeStore_OutputDiag("INFO", "command_manager", "READY");
    RuntimeStore_OutputDiag("INFO", "protocol_exec", "NOT_EXECUTED");
    RuntimeStore_OutputDiag("INFO", "fallback", "KEEP_LAST_VALID");
}

runtime_runtime_view_t *RuntimeCommandRouter_Runtime(void)
{
    return RuntimeStore_Runtime();
}

runtime_params_view_t *RuntimeCommandRouter_Params(void)
{
    return RuntimeStore_Params();
}

runtime_states_view_t *RuntimeCommandRouter_States(void)
{
    return RuntimeStore_States();
}

void RuntimeCommandRouter_UpdateReportMode(void)
{
    runtime_runtime_view_t *runtime = RuntimeCommandRouter_Runtime();
    const runtime_states_view_t *states = RuntimeCommandRouter_States();

    if ((states->semantic_enable != 0U) && (states->osc_enable != 0U))
    {
        runtime->report_mode = 3U;
    }
    else if (states->semantic_enable != 0U)
    {
        runtime->report_mode = 1U;
    }
    else if (states->osc_enable != 0U)
    {
        runtime->report_mode = 2U;
    }
    else
    {
        runtime->report_mode = 0U;
    }
}

static uint8_t RuntimeCommandRouter_ReportSingleParam(char subcommand)
{
    float value;

    if (RuntimeStore_ReadParam(subcommand, &value) == 0U)
    {
        return 0U;
    }

    RuntimeStore_OutputParam(subcommand, value);
    return 1U;
}

static uint8_t RuntimeCommandRouter_ReportSingleState(char subcommand)
{
    uint8_t state;

    if (RuntimeStore_ReadState(subcommand, &state) == 0U)
    {
        return 0U;
    }

    RuntimeStore_OutputState(subcommand, state);
    return 1U;
}

runtime_command_exec_result_t RuntimeCommandRouter_Execute(const protocol_command_t *cmd)
{
    float value = 0.0f;

    if ((cmd == 0) || (cmd->frame_valid == 0U))
    {
        RuntimeCommandRouter_WriteStatusFrameError();
        return RUNTIME_CMD_EXEC_COMMAND_ERROR;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_PARAM)
    {
        if (cmd->has_param != 0U)
        {
            if (RuntimeStore_WriteParam(cmd->subcommand, cmd->param_value) == 0U)
            {
                RuntimeCommandRouter_WriteStatusParamInvalid();
                return RUNTIME_CMD_EXEC_PARAM_ERROR;
            }

            if (RuntimeStore_ReadParam(cmd->subcommand, &value) != 0U)
            {
                RuntimeStore_OutputParam(cmd->subcommand, value);
            }
            return RUNTIME_CMD_EXEC_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_PARAM_SUBCMD_READ_ALL)
        {
            RuntimeStore_ReportAllParams();
            return RUNTIME_CMD_EXEC_OK;
        }

        if (RuntimeCommandRouter_ReportSingleParam(cmd->subcommand) == 0U)
        {
            RuntimeCommandRouter_WriteStatusParamInvalid();
            return RUNTIME_CMD_EXEC_PARAM_ERROR;
        }

        return RUNTIME_CMD_EXEC_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_STATE)
    {
        uint8_t state = 0U;

        if (cmd->has_param != 0U)
        {
            if (ProtocolCore_ParseStateValue(cmd->param_value, &state) == 0U)
            {
                RuntimeCommandRouter_WriteStatusParamInvalid();
                return RUNTIME_CMD_EXEC_PARAM_ERROR;
            }

            if (RuntimeStore_WriteState(cmd->subcommand, state) == 0U)
            {
                RuntimeCommandRouter_WriteStatusParamInvalid();
                return RUNTIME_CMD_EXEC_PARAM_ERROR;
            }

            if (RuntimeStore_ReadState(cmd->subcommand, &state) == 0U)
            {
                RuntimeCommandRouter_WriteStatusParamInvalid();
                return RUNTIME_CMD_EXEC_PARAM_ERROR;
            }

            RuntimeStore_OutputState(cmd->subcommand, state);
            return RUNTIME_CMD_EXEC_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_STATE_SUBCMD_READ_ALL)
        {
            RuntimeStore_ReportAllStates();
            return RUNTIME_CMD_EXEC_OK;
        }

        if (RuntimeCommandRouter_ReportSingleState(cmd->subcommand) == 0U)
        {
            RuntimeCommandRouter_WriteStatusParamInvalid();
            return RUNTIME_CMD_EXEC_PARAM_ERROR;
        }

        return RUNTIME_CMD_EXEC_OK;
    }

    RuntimeCommandRouter_WriteStatusCmdInvalid();
    return RUNTIME_CMD_EXEC_COMMAND_ERROR;
}

void RuntimeCommandRouter_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeStore_BuildSnapshot(snapshot);
}

void RuntimeCommandRouter_ClearDirty(void)
{
    RuntimeStore_ClearDirty();
}

void RuntimeCommandRouter_OutputDiag(const char *level, const char *module, const char *detail)
{
    RuntimeStore_OutputDiag(level, module, detail);
}

void RuntimeCommandRouter_OutputRuntimeSummary(void)
{
    RuntimeStore_OutputRuntimeSummary();
}

void RuntimeCommandRouter_OutputFaultControlSummary(void)
{
    RuntimeStore_OutputFaultControlSummary();
}

const char *RuntimeCommandRouter_GetFaultName(uint8_t fault_code)
{
    return RuntimeStore_GetFaultName(fault_code);
}

void RuntimeCommandRouter_WriteText(const char *text)
{
    RuntimeStore_WriteText(text);
}

void RuntimeCommandRouter_WriteStatusFrameError(void)
{
    RuntimeStore_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
}

void RuntimeCommandRouter_WriteStatusParamInvalid(void)
{
    RuntimeStore_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
}

void RuntimeCommandRouter_WriteStatusCmdInvalid(void)
{
    RuntimeStore_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
}

uint8_t RuntimeCommandRouter_RecoverFaultAndReinit(void)
{
    runtime_runtime_view_t *runtime = RuntimeCommandRouter_Runtime();
    runtime_params_view_t *params = RuntimeCommandRouter_Params();
    runtime_states_view_t *states = RuntimeCommandRouter_States();

    runtime->sensor_invalid_consecutive = 0U;
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
    runtime->protocol_error_count = 0U;
    runtime->param_error_count = 0U;
    runtime->control_skip_count = 0U;
#endif
    runtime->last_fault_code = 0U;
    runtime->comm_state = 0U;
    runtime->system_state = 1U;
#if ((FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE) || \
     (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE) || \
     (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE))
    runtime->params_dirty = 1U;
#else
    runtime->params_dirty = 0U;
#endif
    runtime->last_exec_ok = 1U;

#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    params->cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    params->cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    params->cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    params->cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    params->cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    params->current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    params->current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    params->current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    states->current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
#endif

    return 1U;
}

