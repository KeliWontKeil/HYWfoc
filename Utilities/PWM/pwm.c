#include "pwm.h"

/* Private variables */
static uint8_t current_duty[PWM_CHANNEL_COUNT] = {
    PWM_DEFAULT_DUTY_CH0,
    PWM_DEFAULT_DUTY_CH1,
    PWM_DEFAULT_DUTY_CH2
};

static uint16_t pwm_period = 0;

/* Private function prototypes */
void PWM_GPIO_Config(void);
void PWM_Timer_Config(uint32_t prescaler, uint32_t period);
static uint16_t PWM_CalculateCompareValue(uint8_t duty_percent, uint32_t period);

void PWM_Init(uint8_t freq_kHz,uint8_t deadtime_percent)
{
    
    PWM_GPIO_Config();
    /* For center-aligned mode, period should be half of the desired PWM frequency */
    pwm_period = PWM_TIMER_CLOCK_HZ / 1000 / freq_kHz / 2;

    PWM_Timer_Config(0, pwm_period - 1);
    PWM_SetDeadTime(pwm_period * deadtime_percent / 100);
    

    PWM_SetDutyCycle(PWM_CHANNEL_0, PWM_DEFAULT_DUTY_CH0);
    PWM_SetDutyCycle(PWM_CHANNEL_1, PWM_DEFAULT_DUTY_CH1);
    PWM_SetDutyCycle(PWM_CHANNEL_2, PWM_DEFAULT_DUTY_CH2);
    
    PWM_EnableComplementaryOutputs();
}

/*!
    \brief      Start PWM generation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PWM_Start(void)
{
    /* Enable TIMER0 - it will wait for trigger from TIMER2 in slave mode */
    timer_enable(PWM_TIMER0_PERIPH);
}

/*!
    \brief      Stop PWM generation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PWM_Stop(void)
{
    timer_disable(PWM_TIMER0_PERIPH);
}

/*!
    \brief      Set duty cycle for specific channel
    \param[in]  channel: PWM channel (0, 1, or 2)
    \param[in]  duty_percent: duty cycle percentage (0-100)
    \param[out] none
    \retval     none
*/
void PWM_SetDutyCycle(pwm_channel_t channel, uint8_t duty_percent)
{
    uint16_t compare_value;
    uint16_t timer_channel;
    
    /* Limit duty cycle to 0-100% */
    if (duty_percent > 100)
    {
        duty_percent = 100;
    }
    
    /* Store duty cycle */
    current_duty[channel] = duty_percent;
    
    /* Calculate compare value */
    compare_value = PWM_CalculateCompareValue(duty_percent, pwm_period);
    
    /* Map channel to timer channel */
    switch (channel)
    {
        case PWM_CHANNEL_0:
            timer_channel = TIMER_CH_0;
            break;
        case PWM_CHANNEL_1:
            timer_channel = TIMER_CH_1;
            break;
        case PWM_CHANNEL_2:
            timer_channel = TIMER_CH_2;
            break;
        default:
            return;
    }
    
    /* Configure channel output pulse value */
    timer_channel_output_pulse_value_config(PWM_TIMER0_PERIPH, timer_channel, compare_value);
}

/*!
    \brief      Get current duty cycle for specific channel
    \param[in]  channel: PWM channel (0, 1, or 2)
    \param[out] none
    \retval     duty cycle percentage (0-100)
*/
uint8_t PWM_GetDutyCycle(pwm_channel_t channel)
{
    if (channel >= PWM_CHANNEL_COUNT)
    {
        return 0;
    }
    return current_duty[channel];
}

/*!
    \brief      Set dead time for complementary outputs
    \param[in]  dead_time_cycles: dead time in timer clock cycles (0-255)
    \param[out] none
    \retval     none
*/
void PWM_SetDeadTime(uint16_t dead_time_cycles)
{
    timer_break_parameter_struct timer_breakpara;
    
    /* Limit dead time to 0-255 as per GD32 specification */
    if (dead_time_cycles > 255)
    {
        dead_time_cycles = 255;
    }
    
    /* Configure break parameters including dead time */
    timer_break_struct_para_init(&timer_breakpara);
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE;
    timer_breakpara.deadtime         = dead_time_cycles;
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_0;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    
    timer_break_config(PWM_TIMER0_PERIPH, &timer_breakpara);
}

/*!
    \brief      Enable complementary outputs
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PWM_EnableComplementaryOutputs(void)
{
    timer_primary_output_config(PWM_TIMER0_PERIPH, ENABLE);
}

/*!
    \brief      Disable complementary outputs
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PWM_DisableComplementaryOutputs(void)
{
    timer_primary_output_config(PWM_TIMER0_PERIPH, DISABLE);
}

/*!
    \brief      Configure GPIO pins for PWM outputs
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void PWM_GPIO_Config(void)
{
    /* Enable GPIO and alternate function clocks */
    rcu_periph_clock_enable(PWM_TIMER0_CH0_RCU);
    rcu_periph_clock_enable(PWM_TIMER0_CH1_RCU);
    rcu_periph_clock_enable(PWM_TIMER0_CH2_RCU);
    rcu_periph_clock_enable(PWM_TIMER0_CH0N_RCU);
    rcu_periph_clock_enable(PWM_TIMER0_CH1N_RCU);
    rcu_periph_clock_enable(PWM_TIMER0_CH2N_RCU);
    rcu_periph_clock_enable(RCU_AF);
    
    /* Configure main output channels as alternate function push-pull */
    gpio_init(PWM_TIMER0_CH0_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_TIMER0_CH0_PIN);
    gpio_init(PWM_TIMER0_CH1_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_TIMER0_CH1_PIN);
    gpio_init(PWM_TIMER0_CH2_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_TIMER0_CH2_PIN);
    
    /* Configure complementary output channels as alternate function push-pull */
    gpio_init(PWM_TIMER0_CH0N_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_TIMER0_CH0N_PIN);
    gpio_init(PWM_TIMER0_CH1N_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_TIMER0_CH1N_PIN);
    gpio_init(PWM_TIMER0_CH2N_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PWM_TIMER0_CH2N_PIN);
}

static void PWM_Timer_Config(uint32_t prescaler, uint32_t period)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    
    rcu_periph_clock_enable(PWM_TIMER0_RCU);
    timer_deinit(PWM_TIMER0_PERIPH);
    
    timer_initpara.prescaler         = prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;  /* Central aligned mode for FOC */
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(PWM_TIMER0_PERIPH, &timer_initpara);
    
    /* Configure TIMER0 as slave mode - triggered by TIMER2 */
    timer_slave_mode_select(PWM_TIMER0_PERIPH, TIMER_SLAVE_MODE_RESTART);
    timer_master_slave_mode_config(PWM_TIMER0_PERIPH, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_input_trigger_source_select(PWM_TIMER0_PERIPH, TIMER_SMCFG_TRGSEL_ITI1);  /* Trigger from TIMER2 (ITI1) */
    
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    
    timer_channel_output_config(PWM_TIMER0_PERIPH, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(PWM_TIMER0_PERIPH, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(PWM_TIMER0_PERIPH, TIMER_CH_2, &timer_ocintpara);
    
    timer_channel_output_mode_config(PWM_TIMER0_PERIPH, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(PWM_TIMER0_PERIPH, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    
    timer_channel_output_mode_config(PWM_TIMER0_PERIPH, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(PWM_TIMER0_PERIPH, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    
    timer_channel_output_mode_config(PWM_TIMER0_PERIPH, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(PWM_TIMER0_PERIPH, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    
    timer_auto_reload_shadow_enable(PWM_TIMER0_PERIPH);
    
    #ifdef PWM_DEAD_TIME
    if (PWM_DEAD_TIME > 0)
    {
        PWM_SetDeadTime(PWM_DEAD_TIME);
    }
    #endif
}

static uint16_t PWM_CalculateCompareValue(uint8_t duty_percent, uint32_t period)
{
    uint32_t compare_value;
    
    compare_value = (uint32_t)duty_percent * (period + 1);
    compare_value = compare_value / 100;
    
    if (compare_value > period)
    {
        compare_value = period;
    }
    
    return (uint16_t)compare_value;
}
