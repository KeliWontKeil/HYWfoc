#include "timer3.h"

static volatile uint8_t timer3_initialized = 0;

void Timer3_Init(uint32_t prescaler, uint32_t period)
{
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocpara;

    rcu_periph_clock_enable(TIMER3_RCU);
    timer_deinit(TIMER3_PERIPH);

    timer_initpara.prescaler = prescaler;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = period;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3_PERIPH, &timer_initpara);

    /* Restart TIMER3 on every TIMER2 update so sampling phase is locked to the PWM cycle. */
    timer_slave_mode_select(TIMER3_PERIPH, TIMER_SLAVE_MODE_RESTART);
    timer_master_slave_mode_config(TIMER3_PERIPH, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_input_trigger_source_select(TIMER3_PERIPH, TIMER3_SYNC_TRIGGER_SOURCE);

    timer_channel_output_struct_para_init(&timer_ocpara);
    timer_ocpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER3_PERIPH, TIMER_CH_3, &timer_ocpara);
    /* Use TOGGLE mode so CH3 produces a real compare edge for ADC external trigger logic. */
    timer_channel_output_mode_config(TIMER3_PERIPH, TIMER_CH_3, TIMER_OC_MODE_PWM1);
    timer_channel_output_pulse_value_config(TIMER3_PERIPH, TIMER_CH_3, period - 2);
    timer_channel_output_shadow_config(TIMER3_PERIPH, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* Ensure a known phase before first sync trigger arrives from TIMER2. */
    timer_counter_value_config(TIMER3_PERIPH, 0U);
    timer_auto_reload_shadow_enable(TIMER3_PERIPH);

    timer3_initialized = 1;
}

void Timer3_Start(void)
{
    if (timer3_initialized)
    {
        timer_enable(TIMER3_PERIPH);
    }
}

void Timer3_Stop(void)
{
    timer_disable(TIMER3_PERIPH);
}

void Timer3_SetSampleOffsetPercent(uint16_t period, float percent)
{
    uint16_t compare_value;

    compare_value = (uint16_t)((float)period * percent / 100.0f);
    timer_channel_output_pulse_value_config(TIMER3_PERIPH, TIMER_CH_3, compare_value);
}
