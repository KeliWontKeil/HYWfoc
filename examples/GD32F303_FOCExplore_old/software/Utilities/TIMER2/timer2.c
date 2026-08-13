#include "timer2.h"
#include "timer3.h"

/* Private variables */
static timer2_callback_t timer2_callback = 0;
static volatile uint8_t timer2_initialized = 0;

/*!
    \brief      Initialize TIMER2 with specified prescaler and period
    \param[in]  prescaler: timer prescaler value
    \param[in]  period: timer period value
    \param[out] none
    \retval     none
*/
void Timer2_Init(uint32_t prescaler, uint32_t period)
{
    /* Enable TIMER2 clock */
    rcu_periph_clock_enable(TIMER2_RCU);
    
    /* Deinitialize TIMER2 */
    timer_deinit(TIMER2_PERIPH);
    
    /* Configure TIMER2 with provided parameters */
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2_PERIPH, &timer_initpara);

    timer_interrupt_enable(TIMER2_PERIPH, TIMER_INT_UP);
    NVIC_CONFIG(TIMER2_IRQn, TIMER2_PRIORITY_GROUP, TIMER2_PRIORITY_SUBGROUP);
    
    /* Configure TIMER2 as master to trigger TIMER0 and ADC */
    timer_master_slave_mode_config(TIMER2_PERIPH, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_master_output_trigger_source_select(TIMER2_PERIPH, TIMER_TRI_OUT_SRC_UPDATE);
    
    /* Mark as initialized */
    timer2_initialized = 1;
}

/*!
    \brief      Start TIMER2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Timer2_Start(void)
{
    if (timer2_initialized)
    {
        timer_enable(TIMER2_PERIPH);
    }
}

/*!
    \brief      Stop TIMER2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Timer2_Stop(void)
{
    timer_disable(TIMER2_PERIPH);
}

/*!
    \brief      Set callback function for TIMER2 interrupt
    \param[in]  callback: function to call from interrupt
    \param[out] none
    \retval     none
*/
void Timer2_SetCallback(timer2_callback_t callback)
{
    timer2_callback = callback;
}

/*!
    \brief      TIMER2 interrupt service routine implementation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void Timer2_IRQHandler_Internal(void)
{
    if (timer_interrupt_flag_get(TIMER2_PERIPH, TIMER_INT_FLAG_UP) == RESET)
    {
        return;
    }

    timer_interrupt_flag_clear(TIMER2_PERIPH, TIMER_INT_FLAG_UP);

    if (timer2_callback != 0)
    {
        timer2_callback();
    }
}
