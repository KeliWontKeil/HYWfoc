#include "foc_platform_api.h"

#include "systick.h"
#include "LED.h"
#include "usart1.h"
#include "usart2.h"
#include "comm_frame_mux.h"
#include "timer1.h"
#include "timer2.h"
#include "timer3.h"
#include "adc.h"
#include "as5600.h"
#include "pwm.h"
#include "foc_config.h"

static uint16_t g_platform_sensor_timer_period = 0U;

void FOC_Platform_RuntimeInit(void)
{
    systick_config();
}

void FOC_Platform_IndicatorInit(void)
{
    LED_Init();
}

void FOC_Platform_SetHeartbeatIndicator(uint8_t on)
{
    LED_SetState(3U, on);
}

void FOC_Platform_HighRateClockInit(uint16_t pwm_freq_khz)
{
    uint32_t period;

    period = (FOC_PLATFORM_BASE_CLOCK_KHZ / (uint32_t)pwm_freq_khz) - 1U;
    Timer2_Init(0U, period);
}

void FOC_Platform_ControlTickSourceInit(void)
{
    Timer1_Init(FOC_PLATFORM_CONTROL_TIMER_PRESCALER, FOC_PLATFORM_CONTROL_TIMER_PERIOD);
}

void FOC_Platform_BindControlTickCallback(FOC_SchedulerCallback_t callback)
{
    Timer1_SetCallback(callback);
}

void FOC_Platform_StartControlTickSource(void)
{
    Timer1_Start();
}

void FOC_Platform_RegisterHighRateCallback(FOC_Platform_HighRateCallback_t callback)
{
    Timer2_SetCallback(callback);
}

void FOC_Platform_StartHighRateClock(void)
{
    Timer2_Start();
}

void FOC_Platform_CommInit(const FOC_Platform_CommConfig_t *config)
{
    comm_frame_mux_config_t mux_config;

    USART1_Init();
    USART2_Init();

    mux_config.source_mask = (uint8_t)FOC_PLATFORM_COMM_SOURCE_ALL;
    mux_config.arbitration_policy = (uint8_t)FOC_PLATFORM_COMM_ARB_ROUND_ROBIN;

    if (config != NULL)
    {
        if ((config->source_mask & (uint8_t)FOC_PLATFORM_COMM_SOURCE_ALL) != 0U)
        {
            mux_config.source_mask = (uint8_t)(config->source_mask & (uint8_t)FOC_PLATFORM_COMM_SOURCE_ALL);
        }
        mux_config.arbitration_policy = config->arbitration_policy;
    }

    CommFrameMux_Init(&mux_config);
}

void FOC_Platform_SetCommRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback)
{
    CommFrameMux_SetRxTriggerCallback((comm_frame_mux_rx_trigger_callback_t)callback);
}

void FOC_Platform_DebugOutput(const char *str)
{
    USART1_SendString(str);
}

void FOC_Platform_FeedbackOutput(uint8_t status_code)
{
    USART2_SendByte(status_code);
}

uint8_t FOC_Platform_CommHasPendingFrame(void)
{
    return CommFrameMux_HasPendingFrame();
}

uint16_t FOC_Platform_ReceiveFrame(uint8_t *buffer, uint16_t max_len)
{
    return CommFrameMux_TryDequeueFrame(buffer, max_len);
}

void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz)
{
    g_platform_sensor_timer_period = (uint16_t)((FOC_PLATFORM_BASE_CLOCK_KHZ / (uint32_t)pwm_freq_khz) - 1U);

    AS5600_Init();
    Timer3_Init(0U, (uint32_t)g_platform_sensor_timer_period - 1U);
    Timer3_Start();
    ADC_Init();
    ADC_Start();
}

uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b)
{
    return ADC_ReadPhaseCurrentABOk(phase_current_a, phase_current_b, ADC_AVG_DEFAULT_COUNT);
}

uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad)
{
    return AS5600_ReadAngleRadOk(angle_rad);
}

void FOC_Platform_SetSensorSampleOffsetPercent(float percent)
{
    Timer3_SetSampleOffsetPercent(g_platform_sensor_timer_period, percent);
}


void FOC_Platform_WaitMs(uint32_t ms)
{
    delay_1ms(ms);
}


void FOC_Platform_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t FOC_Platform_ReadCycleCounter(void)
{
    return DWT->CYCCNT;
}


void FOC_Platform_PWMInit(uint8_t freq_khz, uint8_t deadtime_percent)
{
    PWM_Init(freq_khz, deadtime_percent);
}

void FOC_Platform_PWMStart(void)
{
    PWM_Start();
}

void FOC_Platform_PWMSetDutyCycle(foc_platform_pwm_channel_t channel, uint8_t duty_percent)
{
    PWM_SetDutyCycle((pwm_channel_t)channel, duty_percent);
}

void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c)
{
    PWM_SetDutyCycleTripleFloat(duty_a, duty_b, duty_c);
}
