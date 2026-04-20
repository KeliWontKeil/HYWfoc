#include "L2_Service/runtime_c31_runtime_fsm.h"

#include <stdio.h>

#include "L2_Service/runtime_c32_command_router.h"
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

#define RUNTIME_STATE_INIT_REQUIRED_MASK ((1U << 0) | \
                                       (1U << 1) | \
                                       (1U << 2) | \
                                       (1U << 3) | \
                                       (1U << 4) | \
                                       (1U << 5) | \
                                       (1U << 6))

static void RuntimeStateMachine_FinalizeInitDiagnostics(void);
static runtime_command_exec_result_t RuntimeStateMachine_HandleSystemCommand(const protocol_command_t *cmd);

void RuntimeStateMachine_Init(void)
{
    RuntimeCommandRouter_Init();
    RuntimeCommandRouter_UpdateReportMode();
}

void RuntimeStateMachine_UpdateSignals(const runtime_step_signal_t *signal)
{
    runtime_runtime_view_t *runtime = RuntimeCommandRouter_Runtime();

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
            if (runtime->system_state != RUNTIME_STATE_SYSTEM_FAULT)
            {
                runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_NONE;
            }
        }
        else
        {
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
            runtime->control_skip_count++;
#endif
            runtime->sensor_invalid_consecutive++;

            if (signal->adc_valid == 0U)
            {
                runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_SENSOR_ADC_INVALID;
            }
            else
            {
                runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_SENSOR_ENCODER_INVALID;
            }

            if (runtime->sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
            {
                if (runtime->system_state != RUNTIME_STATE_SYSTEM_FAULT)
                {
                    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                    runtime->system_state = RUNTIME_STATE_SYSTEM_FAULT;

                    snprintf(out,
                             sizeof(out),
                             "sensor invalid threshold reached: %u",
                             (unsigned int)runtime->sensor_invalid_consecutive);
                    RuntimeCommandRouter_OutputDiag("ERR", "sensor", out);
                }
            }
        }
    }

    if (signal->control_loop_skipped != 0U)
    {
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
        runtime->control_skip_count++;
#endif
    }

    if (signal->undervoltage_fault != 0U)
    {
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
        char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
        runtime->system_state = RUNTIME_STATE_SYSTEM_FAULT;
        runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_UNDERVOLTAGE;
    #if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
        runtime->control_skip_count++;
    #endif

        snprintf(out,
                 sizeof(out),
                 "vbus undervoltage: %.3fV < %.3fV",
                 signal->undervoltage_vbus,
                 FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT);
        RuntimeCommandRouter_OutputDiag("ERR", "protection", out);
#else
        (void)signal->undervoltage_vbus;
#endif
    }

    if (signal->finalize_init != 0U)
    {
        RuntimeStateMachine_FinalizeInitDiagnostics();
    }
}

uint8_t RuntimeStateMachine_HandleCommand(const protocol_command_t *cmd)
{
    runtime_runtime_view_t *runtime = RuntimeCommandRouter_Runtime();
    runtime_command_exec_result_t exec_result;

    if (cmd == 0)
    {
        return 0U;
    }

    runtime->comm_state = RUNTIME_STATE_COMM_ACTIVE;

    if (cmd->command == COMMAND_MANAGER_CMD_SYSTEM)
    {
        exec_result = RuntimeStateMachine_HandleSystemCommand(cmd);
    }
    else
    {
        exec_result = RuntimeCommandRouter_Execute(cmd);
    }

    runtime->last_exec_ok = (exec_result == RUNTIME_CMD_EXEC_OK) ? 1U : 0U;

    if (exec_result != RUNTIME_CMD_EXEC_OK)
    {
        runtime->comm_state = RUNTIME_STATE_COMM_ERROR;

        if (exec_result == RUNTIME_CMD_EXEC_PARAM_ERROR)
        {
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
            runtime->param_error_count++;
#endif
            runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_PARAM_INVALID;
        }
        else
        {
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
            runtime->protocol_error_count++;
#endif
            runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_PROTOCOL_FRAME;
        }

        RuntimeCommandRouter_OutputDiag("ERR", "fallback", "keep previous params");
    }
    else
    {
        runtime->comm_state = RUNTIME_STATE_COMM_ACTIVE;
    }

    RuntimeCommandRouter_UpdateReportMode();
    return runtime->last_exec_ok;
}

void RuntimeStateMachine_ReportFrameError(void)
{
    runtime_runtime_view_t *runtime = RuntimeCommandRouter_Runtime();

#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
    runtime->protocol_error_count++;
#endif
    runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_PROTOCOL_FRAME;
    RuntimeCommandRouter_WriteStatusFrameError();
}

void RuntimeStateMachine_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeCommandRouter_BuildSnapshot(snapshot);
}

void RuntimeStateMachine_Commit(void)
{
    RuntimeCommandRouter_ClearDirty();
}

static void RuntimeStateMachine_FinalizeInitDiagnostics(void)
{
    runtime_runtime_view_t *runtime = RuntimeCommandRouter_Runtime();
    uint16_t missing_mask = (uint16_t)(RUNTIME_STATE_INIT_REQUIRED_MASK & (~runtime->init_check_mask));

    if (runtime->init_check_mask == 0U)
    {
        runtime->init_diag = RUNTIME_STATE_DIAG_NOT_EXECUTED;
        runtime->system_state = RUNTIME_STATE_SYSTEM_FAULT;
        runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_INIT_FAILED;
        RuntimeCommandRouter_OutputDiag("ERR", "init", "no checks executed");
        return;
    }

    if ((runtime->init_fail_mask == 0U) && (missing_mask == 0U))
    {
        runtime->init_diag = RUNTIME_STATE_DIAG_SUCCESS;
        runtime->system_state = RUNTIME_STATE_SYSTEM_RUNNING;
        runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_NONE;
    }
    else
    {
        runtime->init_diag = RUNTIME_STATE_DIAG_FAILED;
        runtime->system_state = RUNTIME_STATE_SYSTEM_FAULT;
        runtime->last_fault_code = (uint8_t)RUNTIME_FAULT_INIT_FAILED;
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
            RuntimeCommandRouter_WriteText(out);
        }

        snprintf(out,
                 sizeof(out),
                 "diag.init.result=%u checks=0x%04X fails=0x%04X\r\n",
                 (unsigned int)runtime->init_diag,
                 (unsigned int)runtime->init_check_mask,
                 (unsigned int)runtime->init_fail_mask);
        RuntimeCommandRouter_WriteText(out);
    }
#endif
}

static runtime_command_exec_result_t RuntimeStateMachine_HandleSystemCommand(const protocol_command_t *cmd)
{
    if (cmd->has_param != 0U)
    {
        RuntimeCommandRouter_WriteStatusParamInvalid();
        return RUNTIME_CMD_EXEC_PARAM_ERROR;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY)
    {
        RuntimeCommandRouter_OutputRuntimeSummary();
        return RUNTIME_CMD_EXEC_OK;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT)
    {
        if (RuntimeCommandRouter_RecoverFaultAndReinit() != 0U)
        {
            RuntimeCommandRouter_OutputFaultControlSummary();
            return RUNTIME_CMD_EXEC_OK;
        }

        return RUNTIME_CMD_EXEC_COMMAND_ERROR;
    }

    RuntimeCommandRouter_WriteStatusParamInvalid();
    return RUNTIME_CMD_EXEC_PARAM_ERROR;
}

