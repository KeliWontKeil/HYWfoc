#include "L2_Service/runtime_c42_command_router.h"

#include "L3_Algorithm/protocol_core.h"
#include "LS_Config/foc_config.h"

void RuntimeC42_Init(void)
{
    RuntimeC43_ResetStorageDefaults();
    RuntimeC43_OutputDiag("INFO", "command_manager", "READY");
    RuntimeC43_OutputDiag("INFO", "protocol_exec", "NOT_EXECUTED");
    RuntimeC43_OutputDiag("INFO", "fallback", "KEEP_LAST_VALID");
}

runtime_c42_runtime_view_t *RuntimeC42_Runtime(void)
{
    return RuntimeC43_Runtime();
}

runtime_c42_params_view_t *RuntimeC42_Params(void)
{
    return RuntimeC43_Params();
}

runtime_c42_states_view_t *RuntimeC42_States(void)
{
    return RuntimeC43_States();
}

void RuntimeC42_UpdateReportMode(void)
{
    runtime_c42_runtime_view_t *runtime = RuntimeC42_Runtime();
    const runtime_c42_states_view_t *states = RuntimeC42_States();

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

static uint8_t RuntimeC42_ReportSingleParam(char subcommand)
{
    float value;

    if (RuntimeC43_ReadParam(subcommand, &value) == 0U)
    {
        return 0U;
    }

    RuntimeC43_OutputParam(subcommand, value);
    return 1U;
}

static uint8_t RuntimeC42_ReportSingleState(char subcommand)
{
    uint8_t state;

    if (RuntimeC43_ReadState(subcommand, &state) == 0U)
    {
        return 0U;
    }

    RuntimeC43_OutputState(subcommand, state);
    return 1U;
}

runtime_c42_exec_result_t RuntimeC42_RouteCommand(const protocol_command_t *cmd)
{
    float value = 0.0f;

    if ((cmd == 0) || (cmd->frame_valid == 0U))
    {
        RuntimeC42_WriteStatusFrameError();
        return RUNTIME_C42_EXEC_COMMAND_ERROR;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_PARAM)
    {
        if (cmd->has_param != 0U)
        {
            if (RuntimeC43_WriteParam(cmd->subcommand, cmd->param_value) == 0U)
            {
                RuntimeC42_WriteStatusParamInvalid();
                return RUNTIME_C42_EXEC_PARAM_ERROR;
            }

            if (RuntimeC43_ReadParam(cmd->subcommand, &value) != 0U)
            {
                RuntimeC43_OutputParam(cmd->subcommand, value);
            }
            return RUNTIME_C42_EXEC_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_PARAM_SUBCMD_READ_ALL)
        {
            RuntimeC43_ReportAllParams();
            return RUNTIME_C42_EXEC_OK;
        }

        if (RuntimeC42_ReportSingleParam(cmd->subcommand) == 0U)
        {
            RuntimeC42_WriteStatusParamInvalid();
            return RUNTIME_C42_EXEC_PARAM_ERROR;
        }

        return RUNTIME_C42_EXEC_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_STATE)
    {
        uint8_t state = 0U;

        if (cmd->has_param != 0U)
        {
            if (ProtocolCore_ParseStateValue(cmd->param_value, &state) == 0U)
            {
                RuntimeC42_WriteStatusParamInvalid();
                return RUNTIME_C42_EXEC_PARAM_ERROR;
            }

            if (RuntimeC43_WriteState(cmd->subcommand, state) == 0U)
            {
                RuntimeC42_WriteStatusParamInvalid();
                return RUNTIME_C42_EXEC_PARAM_ERROR;
            }

            if (RuntimeC43_ReadState(cmd->subcommand, &state) == 0U)
            {
                RuntimeC42_WriteStatusParamInvalid();
                return RUNTIME_C42_EXEC_PARAM_ERROR;
            }

            RuntimeC43_OutputState(cmd->subcommand, state);
            return RUNTIME_C42_EXEC_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_STATE_SUBCMD_READ_ALL)
        {
            RuntimeC43_ReportAllStates();
            return RUNTIME_C42_EXEC_OK;
        }

        if (RuntimeC42_ReportSingleState(cmd->subcommand) == 0U)
        {
            RuntimeC42_WriteStatusParamInvalid();
            return RUNTIME_C42_EXEC_PARAM_ERROR;
        }

        return RUNTIME_C42_EXEC_OK;
    }

    RuntimeC42_WriteStatusCmdInvalid();
    return RUNTIME_C42_EXEC_COMMAND_ERROR;
}

void RuntimeC42_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC43_BuildSnapshot(snapshot);
}

void RuntimeC42_ClearDirty(void)
{
    RuntimeC43_ClearDirty();
}

void RuntimeC42_OutputDiag(const char *level, const char *module, const char *detail)
{
    RuntimeC43_OutputDiag(level, module, detail);
}

void RuntimeC42_OutputRuntimeSummary(void)
{
    RuntimeC43_OutputRuntimeSummary();
}

void RuntimeC42_OutputFaultControlSummary(void)
{
    RuntimeC43_OutputFaultControlSummary();
}

const char *RuntimeC42_GetFaultName(uint8_t fault_code)
{
    return RuntimeC43_GetFaultName(fault_code);
}

void RuntimeC42_WriteText(const char *text)
{
    RuntimeC43_WriteText(text);
}

void RuntimeC42_WriteStatusFrameError(void)
{
    RuntimeC43_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
}

void RuntimeC42_WriteStatusParamInvalid(void)
{
    RuntimeC43_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
}

void RuntimeC42_WriteStatusCmdInvalid(void)
{
    RuntimeC43_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
}

uint8_t RuntimeC42_RecoverFaultAndReinit(void)
{
    runtime_c42_runtime_view_t *runtime = RuntimeC42_Runtime();
    runtime_c42_params_view_t *params = RuntimeC42_Params();
    runtime_c42_states_view_t *states = RuntimeC42_States();

    runtime->sensor_invalid_consecutive = 0U;
    runtime->protocol_error_count = 0U;
    runtime->param_error_count = 0U;
    runtime->control_skip_count = 0U;
    runtime->last_fault_code = 0U;
    runtime->comm_state = 0U;
    runtime->system_state = 1U;
    runtime->params_dirty = 1U;
    runtime->last_exec_ok = 1U;

    params->cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    params->cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    params->cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    params->cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    params->cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
    params->current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    params->current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    params->current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    states->current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;

    return 1U;
}
