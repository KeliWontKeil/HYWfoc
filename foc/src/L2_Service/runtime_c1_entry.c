#include "L2_Service/runtime_c1_entry.h"

#include "L2_Service/runtime_c2_frame_source.h"

void Runtime_Init(void)
{
    RuntimeC2_Init();
}

uint8_t Runtime_FrameRunStep(uint8_t frame_budget)
{
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

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

void Runtime_UpdateSignals(const runtime_step_signal_t *signals)
{
    RuntimeC2_UpdateSignals(signals);
}

void Runtime_GetSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC2_BuildSnapshot(snapshot);
}

void Runtime_Commit(void)
{
    RuntimeC2_Commit();
}


