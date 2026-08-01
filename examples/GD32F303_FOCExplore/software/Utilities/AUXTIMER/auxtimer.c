#include "auxtimer.h"

/* Private variables */
static auxtimer_callback_t auxtimer_callback = 0;
static volatile uint8_t auxtimer_initialized = 0;

/*!
    \brief      Initialize AUX timer (自由运行模式，无从触发)
    \param[in]  prescaler: timer prescaler value
    \param[in]  period: timer period value
    \param[out] none
    \retval     none
*/
void AuxTimer_Init(uint32_t prescaler, uint32_t period)
{
    rcu_periph_clock_enable(AUXTIMER_CURRENT_LOOP_RCU);
    timer_deinit(AUXTIMER_CURRENT_LOOP_PERIPH);

    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(AUXTIMER_CURRENT_LOOP_PERIPH, &timer_initpara);

    /* 自由运行：不配置从模式，不接 TIMER2 同步，实现与 PWM 频率解耦 */
    auxtimer_initialized = 1;
}

/*!
    \brief      Start AUX timer
    \param[in]  none
    \param[out] none
    \retval     none
*/
void AuxTimer_Start(void)
{
    if (auxtimer_initialized)
    {
        timer_enable(AUXTIMER_CURRENT_LOOP_PERIPH);
    }
}

/*!
    \brief      Stop AUX timer
    \param[in]  none
    \param[out] none
    \retval     none
*/
void AuxTimer_Stop(void)
{
    timer_disable(AUXTIMER_CURRENT_LOOP_PERIPH);
}

void AuxTimer_SetUpdateInterruptEnabled(uint8_t enable)
{
    if (enable != 0U)
    {
        timer_interrupt_enable(AUXTIMER_CURRENT_LOOP_PERIPH, TIMER_INT_UP);
        nvic_irq_enable(AUXTIMER_CURRENT_LOOP_IRQn,
                        AUXTIMER_CURRENT_LOOP_PRIO_GROUP,
                        AUXTIMER_CURRENT_LOOP_PRIO_SUBGROUP);
    }
    else
    {
        timer_interrupt_disable(AUXTIMER_CURRENT_LOOP_PERIPH, TIMER_INT_UP);
        nvic_irq_disable(AUXTIMER_CURRENT_LOOP_IRQn);
    }
}

void AuxTimer_SetCallback(auxtimer_callback_t callback)
{
    auxtimer_callback = callback;
}

/*!
    \brief      AUX timer interrupt service routine implementation
    \note       This function should be called from gd32f30x_it.c
    \param[in]  none
    \param[out] none
    \retval     none
*/
void AuxTimer_IRQHandler_Internal(void)
{
    if (timer_interrupt_flag_get(AUXTIMER_CURRENT_LOOP_PERIPH, TIMER_INT_FLAG_UP) == RESET)
    {
        return;
    }

    timer_interrupt_flag_clear(AUXTIMER_CURRENT_LOOP_PERIPH, TIMER_INT_FLAG_UP);

    if (auxtimer_callback != 0)
    {
        auxtimer_callback();
    }
}
