#include "timer1_algorithm.h"

/* Algorithm timing counters */
static volatile uint16_t timer1_counter = 0;
static uint32_t algorithm_execution_time = 0;

/* Callback function pointers */
static Timer1_Callback_t algorithm_callbacks[TIMER1_CALLBACK_COUNT] = {NULL};

void Timer1_Algorithm_Init(void)
{
    /* Timer configuration for 1kHz interrupt */
    #define TIMER1_CLOCK_HZ         120000000U  /* System clock: 120MHz */
    #define TIMER1_INTERRUPT_FREQ   1000U       /* 1kHz interrupt */
    #define TIMER1_PRESCALER        11999       /* 120MHz / 12000 = 10kHz */
    #define TIMER1_PERIOD           9           /* 10kHz / 10 = 1kHz */
    
    Timer1_Init(TIMER1_PRESCALER, TIMER1_PERIOD);
    Timer1_SetCallback(Timer1_Algorithm_Handler);
    Timer1_Start();
}

void Timer1_EnableDWT(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void Timer1_Algorithm_Handler(void)
{
    static uint8_t dwt_enabled = 0;
    
    if (!dwt_enabled)
    {
        Timer1_EnableDWT();
        dwt_enabled = 1;
    }
    
    uint32_t start_cycles = DWT->CYCCNT;
    timer1_counter++;
    
    /* Execute callbacks based on counter value */
    if ( (timer1_counter % 1) == 0 && algorithm_callbacks[TIMER1_CALLBACK_1KHZ] != NULL)
    {
        algorithm_callbacks[TIMER1_CALLBACK_1KHZ]();
    }
    if ( (timer1_counter % 10) == 0 && algorithm_callbacks[TIMER1_CALLBACK_100HZ] != NULL)
    {
        algorithm_callbacks[TIMER1_CALLBACK_100HZ]();
    }
    if ( (timer1_counter % 100) == 0 && algorithm_callbacks[TIMER1_CALLBACK_10HZ] != NULL)
    {
        algorithm_callbacks[TIMER1_CALLBACK_10HZ]();
    }
    if (timer1_counter == 1000 && algorithm_callbacks[TIMER1_CALLBACK_1HZ] != NULL)
    {
        algorithm_callbacks[TIMER1_CALLBACK_1HZ]();
    }
    
    if(timer1_counter >= 1000)
    {
        timer1_counter = 0;
    }
    
    algorithm_execution_time = DWT->CYCCNT - start_cycles;
}

uint32_t Timer1_GetExecutionTime(void)
{
    return algorithm_execution_time;
}

uint16_t Timer1_GetCounter(void)
{
    return timer1_counter;
}

void Timer1_ResetCounter(void)
{
    timer1_counter = 0;
}

/* Callback management functions */
void Timer1_SetAlgorithmCallback(Timer1_CallbackRate_t rate, Timer1_Callback_t callback)
{
    if (rate < TIMER1_CALLBACK_COUNT)
    {
        algorithm_callbacks[rate] = callback;
    }
}

void Timer1_ClearAlgorithmCallback(Timer1_CallbackRate_t rate)
{
    if (rate < TIMER1_CALLBACK_COUNT)
    {
        algorithm_callbacks[rate] = NULL;
    }
}

void Timer1_ClearAllCallbacks(void)
{
    for (int i = 0; i < TIMER1_CALLBACK_COUNT; i++)
    {
        algorithm_callbacks[i] = NULL;
    }
}
