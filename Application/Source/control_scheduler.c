#include "control_scheduler.h"

static volatile uint16_t g_sched_tick_counter = 0;
static uint32_t g_sched_execution_cycles = 0;

static FOC_SchedulerCallback_t g_sched_callbacks[FOC_SCHEDULER_RATE_COUNT] = {NULL};

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

    if (g_sched_callbacks[FOC_SCHEDULER_RATE_CONTROL_1KHZ] != NULL)
    {
        g_sched_callbacks[FOC_SCHEDULER_RATE_CONTROL_1KHZ]();
    }
    if ((g_sched_tick_counter % 10U) == 0U && g_sched_callbacks[FOC_SCHEDULER_RATE_SERVICE_100HZ] != NULL)
    {
        g_sched_callbacks[FOC_SCHEDULER_RATE_SERVICE_100HZ]();
    }
    if ((g_sched_tick_counter % 100U) == 0U && g_sched_callbacks[FOC_SCHEDULER_RATE_MONITOR_10HZ] != NULL)
    {
        g_sched_callbacks[FOC_SCHEDULER_RATE_MONITOR_10HZ]();
    }
    if (g_sched_tick_counter == 1000U && g_sched_callbacks[FOC_SCHEDULER_RATE_HEARTBEAT_1HZ] != NULL)
    {
        g_sched_callbacks[FOC_SCHEDULER_RATE_HEARTBEAT_1HZ]();
    }

    if (g_sched_tick_counter >= 1000U)
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

void ControlScheduler_SetCallback(FOC_SchedulerRate_t rate, FOC_SchedulerCallback_t callback)
{
    if (rate < FOC_SCHEDULER_RATE_COUNT)
    {
        g_sched_callbacks[rate] = callback;
    }
}

void ControlScheduler_ClearCallback(FOC_SchedulerRate_t rate)
{
    if (rate < FOC_SCHEDULER_RATE_COUNT)
    {
        g_sched_callbacks[rate] = NULL;
    }
}

void ControlScheduler_ClearAllCallbacks(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)FOC_SCHEDULER_RATE_COUNT; i++)
    {
        g_sched_callbacks[i] = NULL;
    }
}
