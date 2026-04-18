#include "L2_Service/runtime_c32_frame_adapter.h"

#include "L2_Service/runtime_c41_runtime_fsm.h"
#include "L3_Algorithm/protocol_core.h"

void RuntimeC32_Init(void)
{
    RuntimeC41_Init();
}

void RuntimeC32_ApplySignals(const runtime_c32_step_signal_t *signal)
{
    runtime_c41_signal_t c41_signal;

    if (signal == 0)
    {
        RuntimeC41_ApplySignals(0);
        return;
    }

    c41_signal.init_checks_pass_mask = signal->init_checks_pass_mask;
    c41_signal.init_checks_fail_mask = signal->init_checks_fail_mask;
    c41_signal.finalize_init = signal->finalize_init;
    c41_signal.sensor_state_updated = signal->sensor_state_updated;
    c41_signal.adc_valid = signal->adc_valid;
    c41_signal.encoder_valid = signal->encoder_valid;
    c41_signal.control_loop_skipped = signal->control_loop_skipped;
    c41_signal.undervoltage_fault = signal->undervoltage_fault;
    c41_signal.undervoltage_vbus = signal->undervoltage_vbus;

    RuntimeC41_ApplySignals(&c41_signal);
}

uint8_t RuntimeC32_HandleFrame(const uint8_t *frame, uint16_t len)
{
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    protocol_core_frame_parse_result_t parse_result;
    protocol_command_t command;

    if ((frame == 0) || (len == 0U))
    {
        return 0U;
    }

    if (ProtocolCore_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        RuntimeC41_OnFrameError();
        return 0U;
    }

    parse_result = ProtocolCore_ParseFrame(payload, payload_len, &command);
    if (parse_result == PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH)
    {
        return 0U;
    }

    if (parse_result != PROTOCOL_CORE_FRAME_PARSE_OK)
    {
        RuntimeC41_OnFrameError();
        return 0U;
    }

    command.updated = 1U;
    return RuntimeC41_OnCommand(&command);
}

void RuntimeC32_Snapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC41_Snapshot(snapshot);
}

void RuntimeC32_Commit(void)
{
    RuntimeC41_Commit();
}
