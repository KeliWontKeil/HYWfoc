#include "L1_Orchestration/control_scheduler.h"

#include "L42_PAL/foc_platform_api.h"

static volatile uint16_t g_sched_tick_counter = 0;
static uint32_t g_sched_execution_cycles = 0;

static ControlScheduler_Callback_t g_sched_callbacks[FOC_TASK_RATE_COUNT] = {NULL};

void ControlScheduler_Init(void)
{
    g_sched_tick_counter = 0;
    g_sched_execution_cycles = 0;
    ControlScheduler_ClearAllCallbacks();
}

void ControlScheduler_EnableDWT(void)
{
    FOC_Platform_EnableCycleCounter();
}

void ControlScheduler_RunTick(void)
{
    static uint8_t dwt_enabled = 0;
    uint32_t start_cycles;

    if (!dwt_enabled)
    {
        ControlScheduler_EnableDWT();
        dwt_enabled = 1;
    }

    start_cycles = FOC_Platform_ReadCycleCounter();
    g_sched_tick_counter++;

    if ((g_sched_tick_counter % FOC_SCHEDULER_CONTROL_DIVIDER) == 0U &&
        g_sched_callbacks[FOC_TASK_RATE_FAST_CONTROL] != NULL)
    {
        g_sched_callbacks[FOC_TASK_RATE_FAST_CONTROL]();
    }
    if ((g_sched_tick_counter % FOC_SCHEDULER_SERVICE_DIVIDER) == 0U &&
        g_sched_callbacks[FOC_TASK_RATE_SERVICE] != NULL)
    {
        g_sched_callbacks[FOC_TASK_RATE_SERVICE]();
    }
    if ((g_sched_tick_counter % FOC_SCHEDULER_MONITOR_DIVIDER) == 0U &&
        g_sched_callbacks[FOC_TASK_RATE_MONITOR] != NULL)
    {
        g_sched_callbacks[FOC_TASK_RATE_MONITOR]();
    }
    if ((g_sched_tick_counter == FOC_SCHEDULER_HEARTBEAT_DIVIDER) &&
        g_sched_callbacks[FOC_TASK_RATE_HEARTBEAT] != NULL)
    {
        g_sched_callbacks[FOC_TASK_RATE_HEARTBEAT]();
    }

    if (g_sched_tick_counter >= FOC_SCHEDULER_HEARTBEAT_DIVIDER)
    {
        g_sched_tick_counter = 0U;
    }

    g_sched_execution_cycles = FOC_Platform_ReadCycleCounter() - start_cycles;
}

uint32_t ControlScheduler_GetExecutionCycles(void)
{
    return g_sched_execution_cycles;
}

uint16_t ControlScheduler_GetTickCounter(void)
{
    return g_sched_tick_counter;
}

void ControlScheduler_ResetTickCounter(void)
{
    g_sched_tick_counter = 0U;
}

void ControlScheduler_SetCallback(FOC_TaskRate_t rate, ControlScheduler_Callback_t callback)
{
    if (rate < FOC_TASK_RATE_COUNT)
    {
        g_sched_callbacks[rate] = callback;
    }
}

void ControlScheduler_ClearCallback(FOC_TaskRate_t rate)
{
    if (rate < FOC_TASK_RATE_COUNT)
    {
        g_sched_callbacks[rate] = NULL;
    }
}

void ControlScheduler_ClearAllCallbacks(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)FOC_TASK_RATE_COUNT; i++)
    {
        g_sched_callbacks[i] = NULL;
    }
}
