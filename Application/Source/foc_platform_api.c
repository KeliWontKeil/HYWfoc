#include "foc_platform_api.h"

#define CONTROL_LOOP_TICK_HZ                   1000U
#define CONTROL_LOOP_TIMER_PRESCALER           11999U
#define CONTROL_LOOP_TIMER_PERIOD              9U
#define PLATFORM_BASE_CLOCK_KHZ                120000U

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
    if (on != 0U)
    {
        Set_LED(3);
    }
    else
    {
        Reset_LED(3);
    }
}

void FOC_Platform_HighRateClockInit(uint16_t pwm_freq_khz)
{
    uint32_t period;

    if (pwm_freq_khz == 0U)
    {
        pwm_freq_khz = 24U;
    }

    period = (PLATFORM_BASE_CLOCK_KHZ / (uint32_t)pwm_freq_khz) - 1U;
    Timer2_Init(0U, period);
}

void FOC_Platform_ControlTickSourceInit(void)
{
    (void)CONTROL_LOOP_TICK_HZ;
    Timer1_Init(CONTROL_LOOP_TIMER_PRESCALER, CONTROL_LOOP_TIMER_PERIOD);
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

void FOC_Platform_TelemetryInit(void)
{
    USART1_Init();
    USART2_Init();
}

void FOC_Platform_TelemetryWrite(const char *str)
{
    USART1_SendString(str);
}

void FOC_Platform_TelemetryWriteByte(uint8_t byte)
{
    USART1_SendByte(byte);
}

void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz)
{
    if (pwm_freq_khz == 0U)
    {
        pwm_freq_khz = 24U;
    }

    g_platform_sensor_timer_period = (uint16_t)((PLATFORM_BASE_CLOCK_KHZ / (uint32_t)pwm_freq_khz) - 1U);

    AS5600_Init();
    Timer3_Init(0U, (uint32_t)g_platform_sensor_timer_period - 1U);
    Timer3_Start();
    ADC_Init();
    ADC_Start();
}

uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b)
{
    float sample[2];

    if ((phase_current_a == 0) || (phase_current_b == 0))
    {
        return 0U;
    }

    if (ADC_GetAverageSample(sample, CURRENT, ADC_AVG_DEFAULT_COUNT) != ADC_STATUS_OK)
    {
        return 0U;
    }

    *phase_current_a = sample[0];
    *phase_current_b = sample[1];

    return 1U;
}

uint8_t FOC_Platform_ReadEncoderRawAngle(uint16_t *angle_raw)
{
    if (angle_raw == 0)
    {
        return 0U;
    }

    if (AS5600_ReadAngle(angle_raw) != I2C_OK)
    {
        return 0U;
    }

    return 1U;
}

void FOC_Platform_SetSensorSampleOffsetPercent(float percent)
{
    uint16_t compare_value;

    compare_value = (uint16_t)((float)g_platform_sensor_timer_period * percent / 100.0f);
    timer_channel_output_pulse_value_config(TIMER3_PERIPH, TIMER_CH_3, compare_value);
}

uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad)
{
    uint16_t angle_raw;

    if (angle_rad == NULL)
    {
        return 0U;
    }

    if (FOC_Platform_ReadEncoderRawAngle(&angle_raw) == 0U)
    {
        return 0U;
    }

    *angle_rad = (float)angle_raw * AS5600_ANGLE_TO_RAD;
    return 1U;
}

void FOC_Platform_WaitMs(uint32_t ms)
{
    if (ms > 0xFFFFU)
    {
        ms = 0xFFFFU;
    }

    delay_1ms((uint16_t)ms);
}

void FOC_Platform_EnableCycleCounter(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t FOC_Platform_ReadCycleCounter(void)
{
    return DWT->CYCCNT;
}
