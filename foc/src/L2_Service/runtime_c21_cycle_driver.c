#include "L2_Service/runtime_c21_cycle_driver.h"

#include "L2_Service/runtime_c31_frame_source.h"

void RuntimeC21_Init(void)
{
    RuntimeC31_Init();
}

uint8_t RuntimeC21_Step(uint8_t frame_budget, const runtime_c21_step_signal_t *signal)
{
    runtime_c31_step_signal_t c31_signal;
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

    if (signal != 0)
    {
        c31_signal.init_checks_pass_mask = signal->init_checks_pass_mask;
        c31_signal.init_checks_fail_mask = signal->init_checks_fail_mask;
        c31_signal.finalize_init = signal->finalize_init;
        c31_signal.sensor_state_updated = signal->sensor_state_updated;
        c31_signal.adc_valid = signal->adc_valid;
        c31_signal.encoder_valid = signal->encoder_valid;
        c31_signal.control_loop_skipped = signal->control_loop_skipped;
        c31_signal.undervoltage_fault = signal->undervoltage_fault;
        c31_signal.undervoltage_vbus = signal->undervoltage_vbus;
        RuntimeC31_ApplySignals(&c31_signal);
    }
    else
    {
        RuntimeC31_ApplySignals(0);
    }

    while (consumed < frame_budget)
    {
        if (RuntimeC31_ProcessOneFrame() != 0U)
        {
            has_comm_activity = 1U;
        }
        consumed++;
    }

    return has_comm_activity;
}

void RuntimeC21_Snapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC31_Snapshot(snapshot);
}

void RuntimeC21_Commit(void)
{
    RuntimeC31_Commit();
}
