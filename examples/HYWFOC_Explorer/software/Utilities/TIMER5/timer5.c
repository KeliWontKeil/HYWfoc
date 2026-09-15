#include "timer5.h"

/* Private variables */
static timer5_callback_t timer5_callback = 0;
static volatile uint8_t timer5_initialized = 0;

/*!
    \brief      Initialize TIMER5 as control scheduler tick
    \param[in]  prescaler: timer prescaler value
    \param[in]  period: timer period value
    \param[out] none
    \retval     none
*/
void Timer5_Init(uint32_t prescaler, uint32_t period)
{
    rcu_periph_clock_enable(TIMER5_RCU);
    timer_deinit(TIMER5_PERIPH);

    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER5_PERIPH, &timer_initpara);

    timer5_initialized = 1;
}

void Timer5_Start(void)
{
    if (timer5_initialized)
    {
        timer_enable(TIMER5_PERIPH);
    }
}

void Timer5_Stop(void)
{
    timer_disable(TIMER5_PERIPH);
}

void Timer5_SetUpdateInterruptEnabled(uint8_t enable)
{
    if (enable != 0U)
    {
        timer_interrupt_enable(TIMER5_PERIPH, TIMER_INT_UP);
        nvic_irq_enable(TIMER5_IRQn, TIMER5_PRIORITY_GROUP, TIMER5_PRIORITY_SUBGROUP);
    }
    else
    {
        timer_interrupt_disable(TIMER5_PERIPH, TIMER_INT_UP);
        nvic_irq_disable(TIMER5_IRQn);
    }
}

void Timer5_SetCallback(timer5_callback_t callback)
{
    timer5_callback = callback;
}

void Timer5_IRQHandler_Internal(void)
{
    if (timer_interrupt_flag_get(TIMER5_PERIPH, TIMER_INT_FLAG_UP) != RESET)
    {
        timer_interrupt_flag_clear(TIMER5_PERIPH, TIMER_INT_FLAG_UP);

        if (timer5_callback != 0)
        {
            timer5_callback();
        }
    }
}

