#include "L2_Core/Runtime/foc_task_scheduler.h"

#include "L3_Hal/foc_platform_api.h"

void ControlScheduler_Init(control_scheduler_t *sched)
{
    if (sched == 0) return;
    sched->tick_counter = 0;
    sched->execution_cycles = 0;
    sched->dwt_enabled = 0;
    ControlScheduler_ClearAllCallbacks(sched);
}

void ControlScheduler_EnableDWT(control_scheduler_t *sched)
{
    if (sched == 0) return;
    FOC_Platform_EnableCycleCounter();
    sched->dwt_enabled = 1;
}

void ControlScheduler_RunTick(control_scheduler_t *sched)
{
    uint32_t start_cycles;

    if (sched == 0) return;

    if (sched->dwt_enabled == 0U)
    {
        ControlScheduler_EnableDWT(sched);
    }

    start_cycles = FOC_Platform_ReadCycleCounter();
    sched->tick_counter++;

    if ((sched->tick_counter % FOC_SCHEDULER_CONTROL_DIVIDER) == 0U &&
        sched->callbacks[FOC_TASK_RATE_FAST_CONTROL] != NULL)
    {
        sched->callbacks[FOC_TASK_RATE_FAST_CONTROL]();
    }
    if ((sched->tick_counter % FOC_SCHEDULER_SERVICE_DIVIDER) == 0U &&
        sched->callbacks[FOC_TASK_RATE_SERVICE] != NULL)
    {
        sched->callbacks[FOC_TASK_RATE_SERVICE]();
    }
    if ((sched->tick_counter % FOC_SCHEDULER_MONITOR_DIVIDER) == 0U &&
        sched->callbacks[FOC_TASK_RATE_MONITOR] != NULL)
    {
        sched->callbacks[FOC_TASK_RATE_MONITOR]();
    }
    if ((sched->tick_counter == FOC_SCHEDULER_HEARTBEAT_DIVIDER) &&
        sched->callbacks[FOC_TASK_RATE_HEARTBEAT] != NULL)
    {
        sched->callbacks[FOC_TASK_RATE_HEARTBEAT]();
    }

    if (sched->tick_counter >= FOC_SCHEDULER_HEARTBEAT_DIVIDER)
    {
        sched->tick_counter = 0U;
    }

    sched->execution_cycles = FOC_Platform_ReadCycleCounter() - start_cycles;
}

uint32_t ControlScheduler_GetExecutionCycles(const control_scheduler_t *sched)
{
    if (sched == 0) return 0U;
    return sched->execution_cycles;
}

uint16_t ControlScheduler_GetTickCounter(const control_scheduler_t *sched)
{
    if (sched == 0) return 0U;
    return sched->tick_counter;
}

void ControlScheduler_ResetTickCounter(control_scheduler_t *sched)
{
    if (sched == 0) return;
    sched->tick_counter = 0U;
}

void ControlScheduler_SetCallback(control_scheduler_t *sched, FOC_TaskRate_t rate, void (*callback)(void))
{
    if ((sched == 0) || (rate >= FOC_TASK_RATE_COUNT)) return;
    sched->callbacks[rate] = callback;
}

void ControlScheduler_ClearCallback(control_scheduler_t *sched, FOC_TaskRate_t rate)
{
    if ((sched == 0) || (rate >= FOC_TASK_RATE_COUNT)) return;
    sched->callbacks[rate] = NULL;
}

void ControlScheduler_ClearAllCallbacks(control_scheduler_t *sched)
{
    uint8_t i;

    if (sched == 0) return;
    for (i = 0U; i < (uint8_t)FOC_TASK_RATE_COUNT; i++)
    {
        sched->callbacks[i] = NULL;
    }
}