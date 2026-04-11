#include "interface/foc_platform_api.h"
#include "config/foc_config.h"

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

void FOC_Platform_ControlTickSourceInit(void)
{
    /* TIMER1 control tick frequency: F = base_clk_khz / ((PSC + 1) * (ARR + 1)). */
    Timer1_Init(9U,
                (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * FOC_PLATFORM_CONTROL_TIMER_FREQ_KHZ)) - 1U);
}

void FOC_Platform_SetControlTickCallback(FOC_Platform_TickCallback_t callback)
{
    Timer1_SetCallback(callback);
}

void FOC_Platform_StartControlTickSource(void)
{
    Timer1_Start();
}

void FOC_Platform_CommInit(void)
{
    USART1_Init();
    USART2_Init();
}

uint8_t FOC_Platform_CommSource1_IsFrameReady(void)
{
    return USART1_IsFrameReady();
}

uint8_t FOC_Platform_CommSource2_IsFrameReady(void)
{
    return USART2_IsFrameReady();
}

__attribute__((weak)) uint8_t FOC_Platform_CommSource3_IsFrameReady(void)
{
    return 0U;
}

__attribute__((weak)) uint8_t FOC_Platform_CommSource4_IsFrameReady(void)
{
    return 0U;
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

void FOC_Platform_WriteDebugText(const char *str)
{
    USART1_SendString(str);
}

void FOC_Platform_WriteStatusByte(uint8_t status_code)
{
    USART1_SendByte(status_code);
}

void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz)
{
    uint32_t timer_period;

    timer_period = (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * (uint32_t)pwm_freq_khz)) - 1U;

    AS5600_Init();

    /* TIMER2 is the sync master; TIMER0(PWM) and TIMER3(ADC trigger) restart from its update event. */
    Timer2_Init(9U, timer_period * 2);
    Timer2_Start();

    Timer3_Init(9U, timer_period);
    Timer3_Start();

    ADC_Init();
    ADC_Start();
}

uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b)
{
    return ADC_ReadPhaseCurrentABOk(phase_current_a,
                                    phase_current_b,
                                    (uint16_t)FOC_SENSOR_ADC_AVG_COUNT_SLOW);
}

uint8_t FOC_Platform_ReadPhaseCurrentABFast(float *phase_current_a, float *phase_current_b)
{
    return ADC_ReadPhaseCurrentABOk(phase_current_a,
                                    phase_current_b,
                                    (uint16_t)FOC_SENSOR_ADC_AVG_COUNT_FAST);
}

uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad)
{
    return AS5600_ReadAngleRadOk(angle_rad);
}

void FOC_Platform_SetSensorSampleOffsetPercent(float percent)
{
    Timer3_SetSampleOffsetPercent(percent);
}


void FOC_Platform_WaitMs(uint32_t ms)
{
    delay_1ms(ms);
}

void FOC_Platform_UndervoltageProtect(float vbus_voltage)
{
    (void)vbus_voltage;
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

void FOC_Platform_SetPwmUpdateCallback(FOC_Platform_PwmIsrCallback_t callback)
{
    PWM_SetUpdateCallback((pwm_update_callback_t)callback);
}