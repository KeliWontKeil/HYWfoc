#include "L2_Service/runtime_c1_entry.h"

#include "L2_Service/runtime_c2_frame_source.h"

void RuntimeC1_Init(void)
{
    RuntimeC2_Init();
}

uint8_t RuntimeC1_RunStep(uint8_t frame_budget, const runtime_c1_step_input_t *input)
{
    runtime_step_signal_t pipeline_signal;
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

    if (input == 0)
    {
        RuntimeC2_UpdateSignals(0);
    }
    else
    {
        pipeline_signal.init_checks_pass_mask = input->init_checks_pass_mask;
        pipeline_signal.init_checks_fail_mask = input->init_checks_fail_mask;
        pipeline_signal.finalize_init = input->finalize_init;
        pipeline_signal.sensor_state_updated = input->sensor_state_updated;
        pipeline_signal.adc_valid = input->adc_valid;
        pipeline_signal.encoder_valid = input->encoder_valid;
        pipeline_signal.control_loop_skipped = input->control_loop_skipped;
        pipeline_signal.undervoltage_fault = input->undervoltage_fault;
        pipeline_signal.undervoltage_vbus = input->undervoltage_vbus;
        RuntimeC2_UpdateSignals(&pipeline_signal);
    }

    while (consumed < frame_budget)
    {
        if (RuntimeC2_ProcessOneFrame() != 0U)
        {
            has_comm_activity = 1U;
        }
        consumed++;
    }

    return has_comm_activity;
}

void RuntimeC1_GetSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC2_BuildSnapshot(snapshot);
}

void RuntimeC1_Commit(void)
{
    RuntimeC2_Commit();
}


