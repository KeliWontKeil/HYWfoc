#include "L2_Service/runtime_c41_command_entry.h"
#include "L2_Service/runtime_c35_protocol_parser.h"
#include "L2_Service/runtime_c21_comm_chain.h"

/* Keep explicit declarations to avoid stale index diagnostics during header migration. */
uint8_t CommandManager_Process(void);
void CommandManager_CaptureSnapshot(runtime_snapshot_t *snapshot);
void CommandManager_ClearDirtyFlag(void);

static uint8_t RuntimeChain_ProcessCommStepInternal(uint8_t max_frames)
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

        if (CommandManager_Process() != 0U)
        {
            has_comm_activity = 1U;
        }

        consumed++;
    }

    return has_comm_activity;
}

static void RuntimeChain_BuildSnapshotInternal(runtime_snapshot_t *snapshot)
{
    CommandManager_CaptureSnapshot(snapshot);
}

static void RuntimeChain_CommitAppliedConfigInternal(void)
{
    CommandManager_ClearDirtyFlag();
}

void RuntimeService_Init(void)
{
    CommandManager_Init();
}

void RuntimeService_ReportInitCheck(uint16_t check_bit, uint8_t success)
{
    CommandManager_ReportInitCheck(check_bit, success);
}

void RuntimeService_FinalizeInitDiagnostics(void)
{
    CommandManager_FinalizeInitDiagnostics();
}

void RuntimeService_ReportRuntimeSensorState(uint8_t adc_valid, uint8_t encoder_valid)
{
    CommandManager_ReportRuntimeSensorState(adc_valid, encoder_valid);
}

void RuntimeService_ReportUndervoltageFault(float vbus_voltage)
{
    CommandManager_ReportUndervoltageFault(vbus_voltage);
}

void RuntimeService_ReportControlLoopSkip(void)
{
    CommandManager_ReportControlLoopSkip();
}

uint8_t RuntimeService_ProcessCommStep(uint8_t max_frames, runtime_snapshot_t *snapshot)
{
    uint8_t has_comm_activity = RuntimeChain_ProcessCommStepInternal(max_frames);
    RuntimeChain_BuildSnapshotInternal(snapshot);
    return has_comm_activity;
}

void RuntimeService_RefreshSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeChain_BuildSnapshotInternal(snapshot);
}

void RuntimeService_CommitAppliedConfig(void)
{
    RuntimeChain_CommitAppliedConfigInternal();
}
