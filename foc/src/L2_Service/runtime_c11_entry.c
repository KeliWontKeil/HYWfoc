#include "L2_Service/runtime_service.h"

#include "L2_Service/command_manager.h"
#include "L2_Service/runtime_chain_internal.h"

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
    uint8_t has_comm_activity = RuntimeChain_ProcessCommStep(max_frames);
    RuntimeChain_BuildSnapshot(snapshot);
    return has_comm_activity;
}

void RuntimeService_RefreshSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeChain_BuildSnapshot(snapshot);
}

void RuntimeService_CommitAppliedConfig(void)
{
    RuntimeChain_CommitAppliedConfig();
}
