#include "L2_Service/runtime_c3_runtime_fsm.h"

#include <stdio.h>

#include "L2_Service/runtime_c4_runtime_core.h"
#include "LS_Config/foc_config.h"

#define RUNTIME_STATE_SYSTEM_INIT 0U
#define RUNTIME_STATE_SYSTEM_RUNNING 1U
#define RUNTIME_STATE_SYSTEM_FAULT 2U

#define RUNTIME_STATE_COMM_IDLE 0U
#define RUNTIME_STATE_COMM_ACTIVE 1U
#define RUNTIME_STATE_COMM_ERROR 2U

#define RUNTIME_STATE_DIAG_NOT_EXECUTED 0U
#define RUNTIME_STATE_DIAG_SUCCESS 1U
#define RUNTIME_STATE_DIAG_FAILED 2U

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
#define RUNTIME_STATE_INIT_REQUIRED_MASK ((1U << 0) | \
                                       (1U << 1) | \
                                       (1U << 2) | \
                                       (1U << 3) | \
                                       (1U << 4) | \
                                       (1U << 5) | \
                                       (1U << 6) | \
                                       (1U << 7))
#else
#define RUNTIME_STATE_INIT_REQUIRED_MASK ((1U << 0) | \
                                       (1U << 1) | \
                                       (1U << 2) | \
                                       (1U << 3) | \
                                       (1U << 4) | \
                                       (1U << 5) | \
                                       (1U << 6))
#endif


static void RuntimeC3_FinalizeInitDiagnostics(void);
static runtime_c4_exec_result_t RuntimeC3_HandleSystemCommand(const protocol_command_t *cmd);

void RuntimeC3_Init(void)
{
    RuntimeC4_Init();
    RuntimeC4_UpdateReportMode();
}

void RuntimeC3_UpdateSignals(const runtime_step_signal_t *signal)
{
    if (signal == 0)
    {
        return;
    }

    RuntimeC4_AccumulateInitChecks(signal->init_checks_pass_mask, signal->init_checks_fail_mask);

    if (signal->sensor_state_updated != 0U)
    {
        if ((signal->adc_valid != 0U) && (signal->encoder_valid != 0U))
        {
            RuntimeC4_ResetSensorInvalidConsecutive();
            if (RuntimeC4_GetSystemState() != RUNTIME_STATE_SYSTEM_FAULT)
            {
                RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_NONE);
            }
        }
        else
        {
            RuntimeC4_IncrementControlSkipCount();
            RuntimeC4_IncrementSensorInvalidConsecutive();

            if (signal->adc_valid == 0U)
            {
                RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_SENSOR_ADC_INVALID);
            }
            else
            {
                RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_SENSOR_ENCODER_INVALID);
            }

            if (RuntimeC4_GetSensorInvalidConsecutive() >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
            {
                if (RuntimeC4_GetSystemState() != RUNTIME_STATE_SYSTEM_FAULT)
                {
                    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                    RuntimeC4_SetSystemState(RUNTIME_STATE_SYSTEM_FAULT);

                    snprintf(out,
                             sizeof(out),
                             "sensor invalid threshold reached: %u",
                             (unsigned int)RuntimeC4_GetSensorInvalidConsecutive());
                    RuntimeC4_OutputDiag("ERR", "sensor", out);
                }
            }
        }
    }

    if (signal->control_loop_skipped != 0U)
    {
        RuntimeC4_IncrementControlSkipCount();
    }

    if (signal->undervoltage_fault != 0U)
    {
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
        char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
        RuntimeC4_SetSystemState(RUNTIME_STATE_SYSTEM_FAULT);
        RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_UNDERVOLTAGE);
        RuntimeC4_IncrementControlSkipCount();

        snprintf(out,
                 sizeof(out),
                 "vbus undervoltage: %.3fV < %.3fV",
                 signal->undervoltage_vbus,
                 FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT);
        RuntimeC4_OutputDiag("ERR", "protection", out);
#else
        (void)signal->undervoltage_vbus;
#endif
    }

    if (signal->finalize_init != 0U)
    {
        RuntimeC3_FinalizeInitDiagnostics();
    }
}

uint8_t RuntimeC3_HandleCommand(const protocol_command_t *cmd)
{
    runtime_c4_exec_result_t exec_result;

    if (cmd == 0)
    {
        return 0U;
    }

    RuntimeC4_SetCommState(RUNTIME_STATE_COMM_ACTIVE);

    if (cmd->command == COMMAND_MANAGER_CMD_SYSTEM)
    {
        exec_result = RuntimeC3_HandleSystemCommand(cmd);
    }
    else
    {
        exec_result = RuntimeC4_ExecuteCommand(cmd);
    }

    RuntimeC4_SetLastExecOk((exec_result == RUNTIME_C4_EXEC_OK) ? 1U : 0U);

    if (exec_result != RUNTIME_C4_EXEC_OK)
    {
        RuntimeC4_SetCommState(RUNTIME_STATE_COMM_ERROR);

        if (exec_result == RUNTIME_C4_EXEC_PARAM_ERROR)
        {
            RuntimeC4_IncrementParamErrorCount();
            RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_PARAM_INVALID);
        }
        else
        {
            RuntimeC4_IncrementProtocolErrorCount();
            RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_PROTOCOL_FRAME);
        }

        RuntimeC4_OutputDiag("ERR", "fallback", "keep previous params");
    }
    else
    {
        RuntimeC4_SetCommState(RUNTIME_STATE_COMM_ACTIVE);
    }

    RuntimeC4_UpdateReportMode();
    return RuntimeC4_GetLastExecOk();
}

void RuntimeC3_ReportFrameError(void)
{
    RuntimeC4_IncrementProtocolErrorCount();
    RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_PROTOCOL_FRAME);
    RuntimeC4_WriteStatusFrameError();
}

void RuntimeC3_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC4_BuildSnapshot(snapshot);
}

void RuntimeC3_Commit(void)
{
    RuntimeC4_ClearDirty();
}

static void RuntimeC3_FinalizeInitDiagnostics(void)
{
    uint16_t init_check_mask = RuntimeC4_GetInitCheckMask();
    uint16_t init_fail_mask = RuntimeC4_GetInitFailMask();
    uint16_t missing_mask = (uint16_t)(RUNTIME_STATE_INIT_REQUIRED_MASK & (~init_check_mask));

    if (init_check_mask == 0U)
    {
        RuntimeC4_SetInitDiag(RUNTIME_STATE_DIAG_NOT_EXECUTED);
        RuntimeC4_SetSystemState(RUNTIME_STATE_SYSTEM_FAULT);
        RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_INIT_FAILED);
        RuntimeC4_OutputDiag("ERR", "init", "no checks executed");
        return;
    }

    if ((init_fail_mask == 0U) && (missing_mask == 0U))
    {
        RuntimeC4_SetInitDiag(RUNTIME_STATE_DIAG_SUCCESS);
        RuntimeC4_SetSystemState(RUNTIME_STATE_SYSTEM_RUNNING);
        RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_NONE);
    }
    else
    {
        RuntimeC4_SetInitDiag(RUNTIME_STATE_DIAG_FAILED);
        RuntimeC4_SetSystemState(RUNTIME_STATE_SYSTEM_FAULT);
        RuntimeC4_SetLastFaultCode((uint8_t)RUNTIME_FAULT_INIT_FAILED);
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
                     (unsigned int)RUNTIME_STATE_INIT_REQUIRED_MASK);
            RuntimeC4_WriteText(out);
        }

        snprintf(out,
                 sizeof(out),
                 "diag.init.result=%u checks=0x%04X fails=0x%04X\r\n",
                 (unsigned int)RuntimeC4_GetInitDiag(),
                 (unsigned int)init_check_mask,
                 (unsigned int)init_fail_mask);
        RuntimeC4_WriteText(out);
    }
#endif
}

static runtime_c4_exec_result_t RuntimeC3_HandleSystemCommand(const protocol_command_t *cmd)
{
    if (cmd->has_param != 0U)
    {
        RuntimeC4_WriteStatusParamInvalid();
        return RUNTIME_C4_EXEC_PARAM_ERROR;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY)
    {
        RuntimeC4_OutputRuntimeSummary();
        return RUNTIME_C4_EXEC_OK;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT)
    {
        if (RuntimeC4_RecoverFaultAndReinit() != 0U)
        {
            RuntimeC4_OutputFaultControlSummary();
            return RUNTIME_C4_EXEC_OK;
        }

        return RUNTIME_C4_EXEC_COMMAND_ERROR;
    }

    RuntimeC4_WriteStatusParamInvalid();
    return RUNTIME_C4_EXEC_PARAM_ERROR;
}


