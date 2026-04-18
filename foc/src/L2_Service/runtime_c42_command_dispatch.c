#include "L2_Service/runtime_c42_command_dispatch.h"

#include "L2_Service/runtime_c43_command_store.h"
#include "L2_Service/runtime_c44_command_diag.h"
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

    FOC_Platform_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
    return COMMAND_EXEC_RESULT_COMMAND_ERROR;
}
