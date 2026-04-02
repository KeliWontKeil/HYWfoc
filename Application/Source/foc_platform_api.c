#include "foc_platform_api.h"

#include "systick.h"
#include "LED.h"
#include "usart1.h"
#include "usart2.h"
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

void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on)
{
    LED_SetState(led_index, on);
}

void FOC_Platform_SetHeartbeatIndicator(uint8_t on)
{
    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, on);
}

void FOC_Platform_HighRateClockInit(uint16_t pwm_freq_khz)
{
    /* TIMER2 update frequency: F = base_clk_khz / ((PSC + 1) * (ARR + 1)). */
    Timer2_Init(9U, (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * (uint32_t)pwm_freq_khz)) - 1U);
}

void FOC_Platform_ControlTickSourceInit(void)
{
    /* TIMER1 control tick frequency: F = base_clk_khz / ((PSC + 1) * (ARR + 1)). */
    Timer1_Init(9U, (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * FOC_PLATFORM_CONTROL_TIMER_FREQ_KHZ)) - 1U);
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

void FOC_Platform_CommInit(void)
{
    USART1_Init();
    USART2_Init();
}

void FOC_Platform_CommSource1_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback)
{
    USART1_SetIdleCallback((usart1_idle_callback_t)callback);
}

void FOC_Platform_CommSource2_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback)
{
    USART2_SetIdleCallback((usart2_idle_callback_t)callback);
}

__attribute__((weak)) void FOC_Platform_CommSource3_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback)
{
    (void)callback;
}

__attribute__((weak)) void FOC_Platform_CommSource4_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback)
{
    (void)callback;
}

uint16_t FOC_Platform_CommSource1_ReadFrame(uint8_t *buffer, uint16_t max_len)
{
    return USART1_ReadFrame(buffer, max_len);
}

uint16_t FOC_Platform_CommSource2_ReadFrame(uint8_t *buffer, uint16_t max_len)
{
    return USART2_ReadFrame(buffer, max_len);
}

__attribute__((weak)) uint16_t FOC_Platform_CommSource3_ReadFrame(uint8_t *buffer, uint16_t max_len)
{
    (void)buffer;
    (void)max_len;
    return 0U;
}

__attribute__((weak)) uint16_t FOC_Platform_CommSource4_ReadFrame(uint8_t *buffer, uint16_t max_len)
{
    (void)buffer;
    (void)max_len;
    return 0U;
}

void FOC_Platform_DebugOutput(const char *str)
{
    USART1_SendString(str);
}

void FOC_Platform_FeedbackOutput(uint8_t status_code)
{
    USART2_SendByte(status_code);
}

void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz)
{
    /* TIMER3 sampling frequency follows PWM by the same timer equation. */
    g_platform_sensor_timer_period = (uint16_t)((FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * (uint32_t)pwm_freq_khz)) - 1U);

    AS5600_Init();
    Timer3_Init(9U, (uint32_t)g_platform_sensor_timer_period);
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

void FOC_Platform_UndervoltageProtect(float vbus_voltage)
{
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    /*
     * Hook point reserved for future hardware undervoltage protection action.
     * Current board does not support this control path yet.
     */
    (void)vbus_voltage;
#else
    /* Feature is trimmed out by configuration. */
    (void)vbus_voltage;
#endif
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

void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c)
{
    PWM_SetDutyCycleTripleFloat(duty_a, duty_b, duty_c);
}
