#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>
#include "foc_shared_types.h"

#define FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(name) void (*name)(void)
typedef FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(FOC_Platform_HighRateCallback_t);

typedef enum {
	FOC_PLATFORM_PWM_CHANNEL_0 = 0,
	FOC_PLATFORM_PWM_CHANNEL_1,
	FOC_PLATFORM_PWM_CHANNEL_2
} foc_platform_pwm_channel_t;

void FOC_Platform_RuntimeInit(void);
void FOC_Platform_IndicatorInit(void);
void FOC_Platform_SetHeartbeatIndicator(uint8_t on);

void FOC_Platform_HighRateClockInit(uint16_t pwm_freq_khz);
void FOC_Platform_ControlTickSourceInit(void);
void FOC_Platform_BindControlTickCallback(FOC_SchedulerCallback_t callback);
void FOC_Platform_StartControlTickSource(void);
void FOC_Platform_RegisterHighRateCallback(FOC_Platform_HighRateCallback_t callback);
void FOC_Platform_StartHighRateClock(void);

void FOC_Platform_TelemetryInit(void);
void FOC_Platform_TelemetryWrite(const char *str);
void FOC_Platform_TelemetryWriteByte(uint8_t byte);

void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz);
uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b);
uint8_t FOC_Platform_ReadEncoderRawAngle(uint16_t *angle_raw);
void FOC_Platform_SetSensorSampleOffsetPercent(float percent);
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad);
void FOC_Platform_WaitMs(uint32_t ms);

void FOC_Platform_EnableCycleCounter(void);
uint32_t FOC_Platform_ReadCycleCounter(void);

void FOC_Platform_PWMInit(uint8_t freq_khz, uint8_t deadtime_percent);
void FOC_Platform_PWMStart(void);
void FOC_Platform_PWMSetDutyCycle(foc_platform_pwm_channel_t channel, uint8_t duty_percent);
void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);

#endif /* FOC_PLATFORM_API_H */
