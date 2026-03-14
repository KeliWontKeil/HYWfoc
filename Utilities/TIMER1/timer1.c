#include "timer1.h"

/* Private variables */
static timer1_callback_t timer1_callback = 0;
static volatile uint8_t timer1_initialized = 0;

/*!
    \brief      Initialize TIMER1 with specified prescaler and period
    \param[in]  prescaler: timer prescaler value
    \param[in]  period: timer period value
    \param[out] none
    \retval     none
*/
void Timer1_Init(uint32_t prescaler, uint32_t period)
{
    /* Enable TIMER1 clock */
    rcu_periph_clock_enable(TIMER1_RCU);
    
    /* Deinitialize TIMER1 */
    timer_deinit(TIMER1_PERIPH);
    
    /* Configure TIMER1 with provided parameters */
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1_PERIPH, &timer_initpara);
    
    /* Enable TIMER1 update interrupt */
    timer_interrupt_enable(TIMER1_PERIPH, TIMER_INT_UP);
    
    /* Configure NVIC for TIMER1 */
    nvic_irq_enable(TIMER1_IRQn, TIMER1_PRIORITY_GROUP, TIMER1_PRIORITY_SUBGROUP);
    
    /* Mark as initialized */
    timer1_initialized = 1;
}

/*!
    \brief      Start TIMER1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Timer1_Start(void)
{
    if (timer1_initialized)
    {
        timer_enable(TIMER1_PERIPH);
    }
}

/*!
    \brief      Stop TIMER1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Timer1_Stop(void)
{
    timer_disable(TIMER1_PERIPH);
}

/*!
    \brief      Set callback function for TIMER1 interrupt
    \param[in]  callback: function to call from interrupt
    \param[out] none
    \retval     none
*/
void Timer1_SetCallback(timer1_callback_t callback)
{
    timer1_callback = callback;
}

/*!
    \brief      TIMER1 interrupt service routine implementation
    \note       This function should be called from gd32f30x_it.c
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Timer1_IRQHandler_Internal(void)
{
    if (timer_interrupt_flag_get(TIMER1_PERIPH, TIMER_INT_FLAG_UP) != RESET)
    {
        /* Clear interrupt flag */
        timer_interrupt_flag_clear(TIMER1_PERIPH, TIMER_INT_FLAG_UP);
        
        /* Call callback function if set */
        if (timer1_callback != 0)
        {
            timer1_callback();
        }
    }
}
