#include "L2_Service/runtime_c41_runtime_fsm.h"

#include <stdio.h>

#include "LS_Config/foc_config.h"

#define RUNTIME_C41_SYSTEM_INIT 0U
#define RUNTIME_C41_SYSTEM_RUNNING 1U
#define RUNTIME_C41_SYSTEM_FAULT 2U

#define RUNTIME_C41_COMM_IDLE 0U
#define RUNTIME_C41_COMM_ACTIVE 1U
#define RUNTIME_C41_COMM_ERROR 2U

#define RUNTIME_C41_DIAG_NOT_EXECUTED 0U
#define RUNTIME_C41_DIAG_SUCCESS 1U
#define RUNTIME_C41_DIAG_FAILED 2U

#define RUNTIME_C41_FAULT_NONE 0U
#define RUNTIME_C41_FAULT_SENSOR_ADC_INVALID 1U
#define RUNTIME_C41_FAULT_SENSOR_ENCODER_INVALID 2U
#define RUNTIME_C41_FAULT_UNDERVOLTAGE 3U
#define RUNTIME_C41_FAULT_PROTOCOL_FRAME 4U
#define RUNTIME_C41_FAULT_PARAM_INVALID 5U
#define RUNTIME_C41_FAULT_INIT_FAILED 6U

#define RUNTIME_C41_INIT_REQUIRED_MASK ((1U << 0) | \
                                       (1U << 1) | \
                                       (1U << 2) | \
                                       (1U << 3) | \
                                       (1U << 4) | \
                                       (1U << 5) | \
                                       (1U << 6))

static void RuntimeC41_FinalizeInitDiagnostics(void);
static runtime_c42_exec_result_t RuntimeC41_HandleSystemCommand(const protocol_command_t *cmd);

void RuntimeC41_Init(void)
{
    RuntimeC42_Init();
    RuntimeC42_UpdateReportMode();
}

void RuntimeC41_ApplySignals(const runtime_c41_signal_t *signal)
{
    runtime_c42_runtime_view_t *runtime = RuntimeC42_Runtime();

    if (signal == 0)
    {
        return;
    }

    runtime->init_check_mask = (uint16_t)(runtime->init_check_mask | signal->init_checks_pass_mask | signal->init_checks_fail_mask);
    runtime->init_fail_mask = (uint16_t)(runtime->init_fail_mask | signal->init_checks_fail_mask);

    if (signal->sensor_state_updated != 0U)
    {
        if ((signal->adc_valid != 0U) && (signal->encoder_valid != 0U))
        {
            runtime->sensor_invalid_consecutive = 0U;
            if (runtime->system_state != RUNTIME_C41_SYSTEM_FAULT)
            {
                runtime->last_fault_code = RUNTIME_C41_FAULT_NONE;
            }
        }
        else
        {
            runtime->control_skip_count++;
            runtime->sensor_invalid_consecutive++;

            if (signal->adc_valid == 0U)
            {
                runtime->last_fault_code = RUNTIME_C41_FAULT_SENSOR_ADC_INVALID;
            }
            else
            {
                runtime->last_fault_code = RUNTIME_C41_FAULT_SENSOR_ENCODER_INVALID;
            }

            if (runtime->sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
            {
                if (runtime->system_state != RUNTIME_C41_SYSTEM_FAULT)
                {
                    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                    runtime->system_state = RUNTIME_C41_SYSTEM_FAULT;

                    snprintf(out,
                             sizeof(out),
                             "sensor invalid threshold reached: %u",
                             (unsigned int)runtime->sensor_invalid_consecutive);
                    RuntimeC42_OutputDiag("ERR", "sensor", out);
                }
            }
        }
    }

    if (signal->control_loop_skipped != 0U)
    {
        runtime->control_skip_count++;
    }

    if (signal->undervoltage_fault != 0U)
    {
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
        char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
        runtime->system_state = RUNTIME_C41_SYSTEM_FAULT;
        runtime->last_fault_code = RUNTIME_C41_FAULT_UNDERVOLTAGE;
        runtime->control_skip_count++;

        snprintf(out,
                 sizeof(out),
                 "vbus undervoltage: %.3fV < %.3fV",
                 signal->undervoltage_vbus,
                 FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT);
        RuntimeC42_OutputDiag("ERR", "protection", out);
#else
        (void)signal->undervoltage_vbus;
#endif
    }

    if (signal->finalize_init != 0U)
    {
        RuntimeC41_FinalizeInitDiagnostics();
    }
}

uint8_t RuntimeC41_OnCommand(const protocol_command_t *cmd)
{
    runtime_c42_runtime_view_t *runtime = RuntimeC42_Runtime();
    runtime_c42_exec_result_t exec_result;

    if (cmd == 0)
    {
        return 0U;
    }

    runtime->comm_state = RUNTIME_C41_COMM_ACTIVE;

    if (cmd->command == COMMAND_MANAGER_CMD_SYSTEM)
    {
        exec_result = RuntimeC41_HandleSystemCommand(cmd);
    }
    else
    {
        exec_result = RuntimeC42_RouteCommand(cmd);
    }

    runtime->last_exec_ok = (exec_result == RUNTIME_C42_EXEC_OK) ? 1U : 0U;

    if (exec_result != RUNTIME_C42_EXEC_OK)
    {
        runtime->comm_state = RUNTIME_C41_COMM_ERROR;

        if (exec_result == RUNTIME_C42_EXEC_PARAM_ERROR)
        {
            runtime->param_error_count++;
            runtime->last_fault_code = RUNTIME_C41_FAULT_PARAM_INVALID;
        }
        else
        {
            runtime->protocol_error_count++;
            runtime->last_fault_code = RUNTIME_C41_FAULT_PROTOCOL_FRAME;
        }

        RuntimeC42_OutputDiag("ERR", "fallback", "keep previous params");
    }
    else
    {
        runtime->comm_state = RUNTIME_C41_COMM_ACTIVE;
    }

    RuntimeC42_UpdateReportMode();
    return runtime->last_exec_ok;
}

void RuntimeC41_OnFrameError(void)
{
    runtime_c42_runtime_view_t *runtime = RuntimeC42_Runtime();

    runtime->protocol_error_count++;
    runtime->last_fault_code = RUNTIME_C41_FAULT_PROTOCOL_FRAME;
    RuntimeC42_WriteStatusFrameError();
}

void RuntimeC41_Snapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC42_BuildSnapshot(snapshot);
}

void RuntimeC41_Commit(void)
{
    RuntimeC42_ClearDirty();
}

static void RuntimeC41_FinalizeInitDiagnostics(void)
{
    runtime_c42_runtime_view_t *runtime = RuntimeC42_Runtime();
    uint16_t missing_mask = (uint16_t)(RUNTIME_C41_INIT_REQUIRED_MASK & (~runtime->init_check_mask));

    if (runtime->init_check_mask == 0U)
    {
        runtime->init_diag = RUNTIME_C41_DIAG_NOT_EXECUTED;
        runtime->system_state = RUNTIME_C41_SYSTEM_FAULT;
        runtime->last_fault_code = RUNTIME_C41_FAULT_INIT_FAILED;
        RuntimeC42_OutputDiag("ERR", "init", "no checks executed");
        return;
    }

    if ((runtime->init_fail_mask == 0U) && (missing_mask == 0U))
    {
        runtime->init_diag = RUNTIME_C41_DIAG_SUCCESS;
        runtime->system_state = RUNTIME_C41_SYSTEM_RUNNING;
        runtime->last_fault_code = RUNTIME_C41_FAULT_NONE;
    }
    else
    {
        runtime->init_diag = RUNTIME_C41_DIAG_FAILED;
        runtime->system_state = RUNTIME_C41_SYSTEM_FAULT;
        runtime->last_fault_code = RUNTIME_C41_FAULT_INIT_FAILED;
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
                     (unsigned int)RUNTIME_C41_INIT_REQUIRED_MASK);
            RuntimeC42_WriteText(out);
        }

        snprintf(out,
                 sizeof(out),
                 "diag.init.result=%u checks=0x%04X fails=0x%04X\r\n",
                 (unsigned int)runtime->init_diag,
                 (unsigned int)runtime->init_check_mask,
                 (unsigned int)runtime->init_fail_mask);
        RuntimeC42_WriteText(out);
    }
#endif
}

static runtime_c42_exec_result_t RuntimeC41_HandleSystemCommand(const protocol_command_t *cmd)
{
    if (cmd->has_param != 0U)
    {
        RuntimeC42_WriteStatusParamInvalid();
        return RUNTIME_C42_EXEC_PARAM_ERROR;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY)
    {
        RuntimeC42_OutputRuntimeSummary();
        return RUNTIME_C42_EXEC_OK;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT)
    {
        if (RuntimeC42_RecoverFaultAndReinit() != 0U)
        {
            RuntimeC42_OutputFaultControlSummary();
            return RUNTIME_C42_EXEC_OK;
        }

        return RUNTIME_C42_EXEC_COMMAND_ERROR;
    }

    RuntimeC42_WriteStatusParamInvalid();
    return RUNTIME_C42_EXEC_PARAM_ERROR;
}
