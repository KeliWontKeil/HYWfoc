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
    rcu_periph_clock_enable(FOC_ISR_VIS_TIMER1_GPIO_RCU);
    gpio_init(FOC_ISR_VIS_TIMER1_GPIO_PORT,
              GPIO_MODE_OUT_PP,
              GPIO_OSPEED_50MHZ,
              FOC_ISR_VIS_TIMER1_GPIO_PIN);
    gpio_bit_reset(FOC_ISR_VIS_TIMER1_GPIO_PORT, FOC_ISR_VIS_TIMER1_GPIO_PIN);

    /* Enable TIMER1 clock */
    rcu_periph_clock_enable(TIMER1_RCU);
    
    /* Deinitialize TIMER1 */
    timer_deinit(TIMER1_PERIPH);
    
    timer_slave_mode_select(TIMER1_PERIPH, TIMER_SLAVE_MODE_EVENT);
    timer_master_slave_mode_config(TIMER1_PERIPH, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_input_trigger_source_select(TIMER1_PERIPH, TIMER_SMCFG_TRGSEL_ITI2);  /* Trigger from TIMER2 (ITI2) */

    /* Configure TIMER1 with provided parameters */
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1_PERIPH, &timer_initpara);
    
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

void Timer1_SetUpdateInterruptEnabled(uint8_t enable)
{
    if (enable != 0U)
    {
        timer_interrupt_enable(TIMER1_PERIPH, TIMER_INT_UP);
        nvic_irq_enable(TIMER1_IRQn, TIMER1_PRIORITY_GROUP, TIMER1_PRIORITY_SUBGROUP);
    }
    else
    {
        timer_interrupt_disable(TIMER1_PERIPH, TIMER_INT_UP);
        nvic_irq_disable(TIMER1_IRQn);
    }
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

        if (gpio_output_bit_get(FOC_ISR_VIS_TIMER1_GPIO_PORT, FOC_ISR_VIS_TIMER1_GPIO_PIN) != RESET)
        {
            gpio_bit_reset(FOC_ISR_VIS_TIMER1_GPIO_PORT, FOC_ISR_VIS_TIMER1_GPIO_PIN);
        }
        else
        {

					gpio_bit_set(FOC_ISR_VIS_TIMER1_GPIO_PORT, FOC_ISR_VIS_TIMER1_GPIO_PIN);
        }
        
        /* Call callback function if set */
        if (timer1_callback != 0)
        {
            timer1_callback();
        }
    }
}
