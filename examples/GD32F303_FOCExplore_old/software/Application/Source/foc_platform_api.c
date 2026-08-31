#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#include "systick.h"
#include "LED.h"
#include "usart1.h"
#include "usart2.h"
#include "timer1.h"
#include "timer2.h"
#include "timer3.h"
#include "auxtimer.h"
#include "adc.h"
#include "as5600.h"
#include "pwm.h"

/*****************************************************************************
 * Runtime & Clock（运行时/时钟）
 *****************************************************************************/

void FOC_Platform_RuntimeInit(void)
{
    systick_config();
}

void FOC_Platform_SetPwmUpdateCallback(FOC_Platform_IsrCallback_t callback)
{
    PWM_SetUpdateCallback((pwm_update_callback_t)callback);
}

void FOC_Platform_ControlTickSourceInit(void)
{
    /* TIMER1 control tick frequency: F = base_clk_khz / ((PSC + 1) * (ARR + 1)). */
    Timer1_Init(9U,
                (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * FOC_SCHEDULER_TICK_HZ / 1000U)) - 1U);
}

void FOC_Platform_SetControlTickCallback(FOC_Platform_IsrCallback_t callback)
{
    Timer1_SetCallback(callback);
}

void FOC_Platform_StartControlTickSource(void)
{
    Timer1_Start();
}

void FOC_Platform_SetControlInterruptsEnabled(uint8_t enable)
{
    Timer1_SetUpdateInterruptEnabled(enable);
    PWM_SetUpdateInterruptEnabled(enable);
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    AuxTimer_SetUpdateInterruptEnabled(enable);
#endif
}

void FOC_Platform_AuxTimerInit(FOC_Platform_AuxTimerId_t id,
                               uint32_t freq_hz,
                               FOC_Platform_IsrCallback_t callback)
{
    uint32_t timer_period;

    if ((id != FOC_AUX_TIMER_CURRENT_LOOP) || (freq_hz == 0U))
    {
        return;
    }

    timer_period = (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * (freq_hz / 1000U))) - 1U;
    AuxTimer_Init(9U, timer_period);
    AuxTimer_SetCallback((auxtimer_callback_t)callback);
    AuxTimer_SetUpdateInterruptEnabled(0U);
}

void FOC_Platform_AuxTimerStart(FOC_Platform_AuxTimerId_t id)
{
    if (id != FOC_AUX_TIMER_CURRENT_LOOP)
    {
        return;
    }
    AuxTimer_Start();
}

void FOC_Platform_AuxTimerStop(FOC_Platform_AuxTimerId_t id)
{
    if (id != FOC_AUX_TIMER_CURRENT_LOOP)
    {
        return;
    }
    AuxTimer_Stop();
}

void FOC_Platform_SetAuxTimerCallback(FOC_Platform_AuxTimerId_t id,
                                      FOC_Platform_IsrCallback_t callback)
{
    if (id != FOC_AUX_TIMER_CURRENT_LOOP)
    {
        return;
    }
    AuxTimer_SetCallback((auxtimer_callback_t)callback);
}

void FOC_Platform_WaitMs(uint32_t ms)
{
    delay_1ms(ms);
}

void FOC_Platform_MemoryBarrier(void)
{
    __DMB();
}

/*****************************************************************************
 * Indicator（指示灯）
 *****************************************************************************/

void FOC_Platform_IndicatorInit(void)
{
    LED_Init();
}

void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on)
{
    LED_SetState(led_index, on);
}

/*****************************************************************************
 * Communication（通信）
 *****************************************************************************/

void FOC_Platform_CommInit(void)
{
    USART1_Init();
    USART2_Init();
}

uint16_t FOC_Platform_CommSource_ReadFrame(FOC_Platform_CommSourceId_t id,
                                           uint8_t *buffer,
                                           uint16_t max_len)
{
    switch (id)
    {
    case FOC_COMM_SOURCE_0:
        return USART1_ReadFrame(buffer, max_len);
    case FOC_COMM_SOURCE_1:
        return USART2_ReadFrame(buffer, max_len);
    default:
        break;
    }
    (void)buffer;
    (void)max_len;
    return 0U;
}

void FOC_Platform_WriteDebugText(const char *str)
{
    USART1_SlowWriter_SendData((const uint8_t *)str, (uint16_t)strlen(str));
}

void FOC_Platform_WriteDebugFast(const char *str)
{
    USART1_FastWriter_PutString(str);
}

void FOC_Platform_WriteStatusByte(uint8_t status_code)
{
    USART1_FastWriter_PutByte(status_code);
}

/*****************************************************************************
 * Sensor / Acquisition（传感器/采集）
 *****************************************************************************/

void FOC_Platform_SensorInputInit(void)
{
    uint32_t timer_period;

    timer_period = (FOC_PLATFORM_BASE_CLOCK_KHZ / (10U * (uint32_t)FOC_SENSOR_SAMPLE_FREQ_KHZ)) - 1U;

    AS5600_Init();

    /* TIMER2 is the sync master; TIMER0(PWM) and TIMER3(ADC trigger) restart from its update event. */
    Timer2_Init(9U, timer_period);
    Timer2_Start();

    Timer3_Init(9U, timer_period);
    Timer3_Start();

    ADC_Init();
    ADC_Start();
}

void FOC_Platform_SetSensorSampleOffsetPercent(float percent)
{
    Timer3_SetSampleOffsetPercent(percent);
}

uint8_t FOC_Platform_ReadPhaseCurrent(float *phase_current_a, float *phase_current_b, float *phase_current_c)
{
    return ADC_ReadPhaseCurrentABOk(phase_current_a,
                                    phase_current_b,
                                    1U);
}

uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad)
{
    return AS5600_ReadAngleRadOk(angle_rad);
}

uint8_t FOC_Platform_ReadVbusVoltage(float *vbus_v)
{
    return ADC2_ReadVbus(vbus_v);
}

/*****************************************************************************
 * PWM / Actuation（PWM/驱动输出）
 *****************************************************************************/

void FOC_Platform_PWMInit(void)
{
    PWM_Init(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
}

void FOC_Platform_PWMStart(void)
{
    PWM_Start();
}

void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c)
{
    PWM_SetDutyCycleTripleFloat(duty_a, duty_b, duty_c);
}

/*****************************************************************************
 * Diagnostics / Profiler（诊断/性能分析）
 *****************************************************************************/

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
