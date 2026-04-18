#include "L2_Service/runtime_c11_entry.h"

#include "L2_Service/runtime_c21_cycle_driver.h"

void RuntimeC11_Init(void)
{
    RuntimeC21_Init();
}

uint8_t RuntimeC11_Step(uint8_t frame_budget, const runtime_c11_step_input_t *input)
{
    runtime_c21_step_signal_t c21_signal;

    if (input == 0)
    {
        return RuntimeC21_Step(frame_budget, 0);
    }

    c21_signal.init_checks_pass_mask = input->init_checks_pass_mask;
    c21_signal.init_checks_fail_mask = input->init_checks_fail_mask;
    c21_signal.finalize_init = input->finalize_init;
    c21_signal.sensor_state_updated = input->sensor_state_updated;
    c21_signal.adc_valid = input->adc_valid;
    c21_signal.encoder_valid = input->encoder_valid;
    c21_signal.control_loop_skipped = input->control_loop_skipped;
    c21_signal.undervoltage_fault = input->undervoltage_fault;
    c21_signal.undervoltage_vbus = input->undervoltage_vbus;

    return RuntimeC21_Step(frame_budget, &c21_signal);
}

void RuntimeC11_Snapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC21_Snapshot(snapshot);
}

void RuntimeC11_Commit(void)
{
    RuntimeC21_Commit();
}
