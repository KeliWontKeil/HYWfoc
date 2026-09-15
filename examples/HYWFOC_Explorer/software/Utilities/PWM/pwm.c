#include "pwm.h"
#include "interrupt_priority.h"

/* Private variables */
/* 硬件通道索引 -> TIMER0 通道（索引 0/1/2 的引脚定义见 pwm.h） */
static const uint16_t s_timer_channel[PWM_CHANNEL_COUNT] = {
    TIMER_CH_0,
    TIMER_CH_1,
    TIMER_CH_2
};

/* 最近一次写入的占空比（0.0~1.0，按硬件通道索引），唯一写入点 PWM_WriteDutyChannel */
static float current_duty[PWM_CHANNEL_COUNT] = {
    0.0f,
    0.0f,
    0.0f
};

static uint16_t pwm_period = 0;
static pwm_update_callback_t pwm_update_callback = 0;

/* Private function prototypes */
static void PWM_GPIO_Config(void);
static void PWM_Timer_Config(uint32_t prescaler, uint32_t period);
static uint16_t PWM_CalculateCompareValueFloat(float duty, uint32_t period);
static void PWM_WriteDutyChannel(uint8_t channel_index, float duty);

void PWM_Init(uint8_t freq_kHz,uint8_t deadtime_percent)
{
    
    PWM_GPIO_Config();
    /* For center-aligned mode, period should be half of the desired PWM frequency */
    pwm_period = PWM_TIMER_CLOCK_HZ / 1000 / freq_kHz / 2;

    PWM_Timer_Config(0, pwm_period - 1);
    /* 死区唯一设置点：deadtime_percent 来自 LS 配置 FOC_SVPWM_DEADTIME_PERCENT_DEFAULT
     * （foc_cfg_init_values.h），经平台层 FOC_Platform_PWMInit 传入本函数 */
    PWM_SetDeadTime(pwm_period * deadtime_percent / 100);
    

    /* 默认占空比按相映射写入，保证相标签与驱动腿同源 */
    PWM_SetDutyCycleTripleFloat(PWM_DEFAULT_DUTY_PHASE_A,
                                PWM_DEFAULT_DUTY_PHASE_B,
                                PWM_DEFAULT_DUTY_PHASE_C);
    
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

void PWM_SetUpdateInterruptEnabled(uint8_t enable)
{
    if (enable != 0U)
    {
        timer_interrupt_enable(PWM_TIMER0_PERIPH, TIMER_INT_UP);
        NVIC_CONFIG(TIMER0_UP_IRQn, TIMER0_UP_PRIORITY_GROUP, TIMER0_UP_PRIORITY_SUBGROUP);
    }
    else
    {
        timer_interrupt_disable(PWM_TIMER0_PERIPH, TIMER_INT_UP);
        nvic_irq_disable(TIMER0_UP_IRQn);
    }
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

void PWM_SetUpdateCallback(pwm_update_callback_t callback)
{
    pwm_update_callback = callback;
}

void PWM_Timer0Update_IRQHandler_Internal(void)
{
    if (timer_interrupt_flag_get(PWM_TIMER0_PERIPH, TIMER_INT_FLAG_UP) == RESET)
    {
        return;
    }

    timer_interrupt_flag_clear(PWM_TIMER0_PERIPH, TIMER_INT_FLAG_UP);

    if (pwm_update_callback != 0)
    {
        pwm_update_callback();
    }
}

/* 占空比唯一写入口：硬件通道索引 -> TIMER0 通道，夹取后写比较值寄存器 */
static void PWM_WriteDutyChannel(uint8_t channel_index, float duty)
{
    uint16_t timer_channel;

    if (channel_index >= PWM_CHANNEL_COUNT)
    {
        return;
    }

    if (duty < 0.0f)
    {
        duty = 0.0f;
    }
    else if (duty > 1.0f)
    {
        duty = 1.0f;
    }

    timer_channel = s_timer_channel[channel_index];
    current_duty[channel_index] = duty;

    timer_channel_output_pulse_value_config(PWM_TIMER0_PERIPH,
                                            timer_channel,
                                            PWM_CalculateCompareValueFloat(duty, pwm_period));
}

/*!
    \brief      Set duty cycle for specific channel
    \param[in]  channel: PWM channel (0, 1, or 2; hardware channel index)
    \param[in]  duty_percent: duty cycle percentage (0-100)
    \param[out] none
    \retval     none
*/
void PWM_SetDutyCycle(pwm_channel_t channel, uint8_t duty_percent)
{
    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    PWM_WriteDutyChannel((uint8_t)channel, (float)duty_percent * 0.01f);
}

void PWM_SetDutyCycleFloat(pwm_channel_t channel, float duty)
{
    PWM_WriteDutyChannel((uint8_t)channel, duty);
}

/* 三相占空比（FOC 相坐标）：相 -> TIMER0 通道映射见 pwm.h 的 PWM_PHASE_*_CHANNEL */
void PWM_SetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c)
{
    PWM_WriteDutyChannel(PWM_PHASE_A_CHANNEL, duty_a);
    PWM_WriteDutyChannel(PWM_PHASE_B_CHANNEL, duty_b);
    PWM_WriteDutyChannel(PWM_PHASE_C_CHANNEL, duty_c);
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
        return 0U;
    }

    return (uint8_t)((current_duty[channel] * 100.0f) + 0.5f);
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
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;  /* Central aligned mode for FOC */
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(PWM_TIMER0_PERIPH, &timer_initpara);
    
    /* 配置 TIMER0 为从机：由同步主 TIMER1(ITI1) 的 TRGO 重启。 */
    timer_slave_mode_select(PWM_TIMER0_PERIPH, TIMER_SLAVE_MODE_RESTART);
    timer_master_slave_mode_config(PWM_TIMER0_PERIPH, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_input_trigger_source_select(PWM_TIMER0_PERIPH, TIMER_SMCFG_TRGSEL_ITI1);  /* Trigger from TIMER1 (ITI1) */
    
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
    
    //timer_auto_reload_shadow_enable(PWM_TIMER0_PERIPH);

}

static uint16_t PWM_CalculateCompareValueFloat(float duty, uint32_t period)
{
    float duty_limited;
    uint32_t compare_value;

    if (duty < 0.0f)
    {
        duty_limited = 0.0f;
    }
    else if (duty > 1.0f)
    {
        duty_limited = 1.0f;
    }
    else
    {
        duty_limited = duty;
    }

    compare_value = (uint32_t)(duty_limited * (float)(period + 1U));
    if (compare_value > period)
    {
        compare_value = period;
    }

    return (uint16_t)compare_value;
}
