#include "L2_Service/command_manager_dispatch.h"

#include <stdio.h>

#include "L2_Service/command_manager.h"
#include "L2_Service/command_manager_diag.h"
#include "L3_Algorithm/protocol_core.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

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

static uint8_t CommandManager_ReportSingleState(char subcommand)
{
    uint8_t state;

    if (CommandManager_ReadState(subcommand, &state) == 0U)
    {
        return 0U;
    }

    CommandManager_OutputState(subcommand, state);
    return 1U;
}

void CommandManager_DispatchReportInitDiag(void)
{
    FOC_Platform_WriteDebugText("diag.command_manager=READY\r\n");
    FOC_Platform_WriteDebugText("diag.protocol_exec=NOT_EXECUTED\r\n");
    FOC_Platform_WriteDebugText("diag.fallback=KEEP_LAST_VALID\r\n");
}

command_exec_result_t CommandManager_DispatchExecute(const protocol_command_t *cmd)
{
    float value = 0.0f;

    if (cmd->frame_valid == 0U)
    {
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
        return COMMAND_EXEC_RESULT_COMMAND_ERROR;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_PARAM)
    {
        if (cmd->has_param != 0U)
        {
            if (CommandManager_WriteParam(cmd->subcommand, cmd->param_value) == 0U)
            {
                FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
                return COMMAND_EXEC_RESULT_PARAM_ERROR;
            }

            if (CommandManager_ReadParam(cmd->subcommand, &value) != 0U)
            {
                CommandManager_OutputParam(cmd->subcommand, value);
            }
            return COMMAND_EXEC_RESULT_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_PARAM_SUBCMD_READ_ALL)
        {
            CommandManager_ReportAllParams();
            return COMMAND_EXEC_RESULT_OK;
        }

        if (CommandManager_ReportSingleParam(cmd->subcommand) == 0U)
        {
            FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            return COMMAND_EXEC_RESULT_PARAM_ERROR;
        }

        return COMMAND_EXEC_RESULT_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_STATE)
    {
        uint8_t state = 0U;

        if (cmd->has_param != 0U)
        {
            if (ProtocolCore_ParseStateValue(cmd->param_value, &state) == 0U)
            {
                FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
                return COMMAND_EXEC_RESULT_PARAM_ERROR;
            }

            if (CommandManager_WriteState(cmd->subcommand, state) == 0U)
            {
                FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
                return COMMAND_EXEC_RESULT_PARAM_ERROR;
            }

            if (CommandManager_ReadState(cmd->subcommand, &state) == 0U)
            {
                FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
                return COMMAND_EXEC_RESULT_PARAM_ERROR;
            }

            CommandManager_OutputState(cmd->subcommand, state);
            return COMMAND_EXEC_RESULT_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_STATE_SUBCMD_READ_ALL)
        {
            CommandManager_ReportAllStates();
            return COMMAND_EXEC_RESULT_OK;
        }

        if (CommandManager_ReportSingleState(cmd->subcommand) == 0U)
        {
            FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            return COMMAND_EXEC_RESULT_PARAM_ERROR;
        }

        return COMMAND_EXEC_RESULT_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_SYSTEM)
    {
        const command_manager_runtime_state_t *state = CommandManager_GetRuntimeState();

        if (cmd->has_param != 0U)
        {
            FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            return COMMAND_EXEC_RESULT_PARAM_ERROR;
        }

        if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY)
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
            return COMMAND_EXEC_RESULT_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT)
        {
            if (CommandManager_RecoverFaultAndReinit() != 0U)
            {
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
                const command_manager_runtime_state_t *state_reinit = CommandManager_GetRuntimeState();
                char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

                snprintf(out,
                         sizeof(out),
                         "FAULT_CTRL state=%u fault=%s proto_err=%lu param_err=%lu ctrl_skip=%lu\r\n",
                         (unsigned int)state_reinit->system_state,
                         CommandManager_GetFaultName(state_reinit->last_fault_code),
                         (unsigned long)state_reinit->protocol_error_count,
                         (unsigned long)state_reinit->param_error_count,
                         (unsigned long)state_reinit->control_skip_count);
                FOC_Platform_WriteDebugText(out);
#endif
                return COMMAND_EXEC_RESULT_OK;
            }

            return COMMAND_EXEC_RESULT_COMMAND_ERROR;
        }

        FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        return COMMAND_EXEC_RESULT_PARAM_ERROR;
    }

    FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
    return COMMAND_EXEC_RESULT_COMMAND_ERROR;
}
