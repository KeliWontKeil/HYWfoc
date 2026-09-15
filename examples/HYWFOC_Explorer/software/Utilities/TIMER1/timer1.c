#include "timer1.h"

/* Private variables */
static timer1_callback_t timer1_callback = 0;
static volatile uint8_t timer1_initialized = 0;

/*!
    \brief      Initialize TIMER1 as sync master
    \param[in]  prescaler: timer prescaler value
    \param[in]  period: timer period value
    \param[out] none
    \retval     none
*/
void Timer1_Init(uint32_t prescaler, uint32_t period)
{
    rcu_periph_clock_enable(TIMER1_RCU);
    timer_deinit(TIMER1_PERIPH);

    /* TIMER1 as sync master: TRGO=UPDATE, sync PWM(TIMER0) and TIMER3(ADC). */
    timer_master_slave_mode_config(TIMER1_PERIPH, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_master_output_trigger_source_select(TIMER1_PERIPH, TIMER_TRI_OUT_SRC_UPDATE);

    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1_PERIPH, &timer_initpara);

    timer1_initialized = 1;
}

void Timer1_Start(void)
{
    if (timer1_initialized)
    {
        timer_enable(TIMER1_PERIPH);
    }
}

void Timer1_Stop(void)
{
    timer_disable(TIMER1_PERIPH);
}

void Timer1_SetUpdateInterruptEnabled(uint8_t enable)
{
    /* TIMER1 as sync master does not use interrupt; control tick moved to TIMER5. */
    (void)enable;
}

void Timer1_SetCallback(timer1_callback_t callback)
{
    timer1_callback = callback;
}

void Timer1_IRQHandler_Internal(void)
{
    if (timer_interrupt_flag_get(TIMER1_PERIPH, TIMER_INT_FLAG_UP) != RESET)
    {
        timer_interrupt_flag_clear(TIMER1_PERIPH, TIMER_INT_FLAG_UP);

        if (timer1_callback != 0)
        {
            timer1_callback();
        }
    }
}

