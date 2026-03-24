#include "foc_platform_api.h"

#include <stddef.h>

#include "systick.h"
#include "LED.h"
#include "usart1.h"
#include "timer1.h"
#include "timer2.h"
#include "sensor.h"
#include "svpwm.h"
#include "as5600.h"
#include "uart_debug.h"

#define CONTROL_SCHED_TICK_HZ 1000U
#define TIMER1_PRESCALER_FOR_1KHZ 11999U
#define TIMER1_PERIOD_FOR_1KHZ 9U

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

    period = (120000U / (uint32_t)pwm_freq_khz) - 1U;
    Timer2_Init(0U, period);
}

void FOC_Platform_ControlSchedulerInit(void)
{
    (void)CONTROL_SCHED_TICK_HZ;
    Timer1_Init(TIMER1_PRESCALER_FOR_1KHZ, TIMER1_PERIOD_FOR_1KHZ);
    ControlScheduler_Init();
    Timer1_SetCallback(ControlScheduler_RunTick);
}

void FOC_Platform_RegisterControlSchedulerCallback(ControlScheduler_Rate_t rate, ControlScheduler_Callback_t callback)
{
    ControlScheduler_SetCallback(rate, callback);
}

void FOC_Platform_StartControlScheduler(void)
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
}

void FOC_Platform_TelemetryWrite(const char *str)
{
    USART1_SendString(str);
}

void FOC_Platform_FeedbackPipelineInit(uint8_t pwm_freq_khz)
{
    Sensor_Init(pwm_freq_khz);
}

void FOC_Platform_RefreshFeedbackSample(void)
{
    Sensor_ReadAll();
}

uint8_t FOC_Platform_ReadControlFeedback(foc_control_feedback_t *feedback)
{
    sensor_data_t *sensor;

    if (feedback == NULL)
    {
        return 0U;
    }

    sensor = Sensor_GetData();
    if (sensor == NULL)
    {
        return 0U;
    }

    feedback->phase_current_a = sensor->current_a.output_value;
    feedback->phase_current_b = sensor->current_b.output_value;
    feedback->phase_current_c = sensor->current_c.output_value;
    feedback->mech_angle_rad = sensor->mech_angle_rad.output_value;
    feedback->adc_valid = sensor->adc_valid;
    feedback->encoder_valid = sensor->encoder_valid;

    return 1U;
}

uint8_t FOC_Platform_ReadDebugFeedback(foc_debug_feedback_t *feedback)
{
    sensor_data_t *sensor;

    if (feedback == NULL)
    {
        return 0U;
    }

    sensor = Sensor_GetData();
    if (sensor == NULL)
    {
        return 0U;
    }

    feedback->phase_current_a = sensor->current_a.output_value;
    feedback->phase_current_b = sensor->current_b.output_value;
    feedback->phase_current_c = sensor->current_c.output_value;
    feedback->mech_angle_raw_rad = sensor->mech_angle_rad.raw_value;
    feedback->mech_angle_filtered_rad = sensor->mech_angle_rad.output_value;
    feedback->adc_valid = sensor->adc_valid;
    feedback->encoder_valid = sensor->encoder_valid;

    return 1U;
}

uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad)
{
    uint16_t angle_raw;

    if (angle_rad == NULL)
    {
        return 0U;
    }

    if (AS5600_ReadAngle(&angle_raw) != I2C_OK)
    {
        return 0U;
    }

    *angle_rad = (float)angle_raw * AS5600_ANGLE_TO_RAD;
    return 1U;
}

void FOC_Platform_ModulationInit(uint16_t freq_khz, uint8_t deadtime_percent)
{
    SVPWM_Init(freq_khz, deadtime_percent);
}

void FOC_Platform_PublishDebugWave(float iq)
{
    UART_Debug_OutputOscilloscope(iq);
}

void FOC_Platform_WaitMs(uint32_t ms)
{
    if (ms > 0xFFFFU)
    {
        ms = 0xFFFFU;
    }

    delay_1ms((uint16_t)ms);
}
