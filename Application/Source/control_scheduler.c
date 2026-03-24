#include "control_scheduler.h"

static volatile uint16_t g_sched_tick_counter = 0;
static uint32_t g_sched_execution_cycles = 0;

static ControlScheduler_Callback_t g_sched_callbacks[CONTROL_SCHED_RATE_COUNT] = {NULL};

void ControlScheduler_Init(void)
{
    g_sched_tick_counter = 0;
    g_sched_execution_cycles = 0;
    ControlScheduler_ClearAllCallbacks();
}

void ControlScheduler_EnableDWT(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
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

    start_cycles = DWT->CYCCNT;
    g_sched_tick_counter++;

    if (g_sched_callbacks[CONTROL_SCHED_RATE_1KHZ] != NULL)
    {
        g_sched_callbacks[CONTROL_SCHED_RATE_1KHZ]();
    }
    if ((g_sched_tick_counter % 10U) == 0U && g_sched_callbacks[CONTROL_SCHED_RATE_100HZ] != NULL)
    {
        g_sched_callbacks[CONTROL_SCHED_RATE_100HZ]();
    }
    if ((g_sched_tick_counter % 100U) == 0U && g_sched_callbacks[CONTROL_SCHED_RATE_10HZ] != NULL)
    {
        g_sched_callbacks[CONTROL_SCHED_RATE_10HZ]();
    }
    if (g_sched_tick_counter == 1000U && g_sched_callbacks[CONTROL_SCHED_RATE_1HZ] != NULL)
    {
        g_sched_callbacks[CONTROL_SCHED_RATE_1HZ]();
    }

    if (g_sched_tick_counter >= 1000U)
    {
        g_sched_tick_counter = 0U;
    }

    g_sched_execution_cycles = DWT->CYCCNT - start_cycles;
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

void ControlScheduler_SetCallback(ControlScheduler_Rate_t rate, ControlScheduler_Callback_t callback)
{
    if (rate < CONTROL_SCHED_RATE_COUNT)
    {
        g_sched_callbacks[rate] = callback;
    }
}

void ControlScheduler_ClearCallback(ControlScheduler_Rate_t rate)
{
    if (rate < CONTROL_SCHED_RATE_COUNT)
    {
        g_sched_callbacks[rate] = NULL;
    }
}

void ControlScheduler_ClearAllCallbacks(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)CONTROL_SCHED_RATE_COUNT; i++)
    {
        g_sched_callbacks[i] = NULL;
    }
}
