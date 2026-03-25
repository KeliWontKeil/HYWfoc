#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>
#include "foc_shared_types.h"

#include "systick.h"
#include "LED.h"
#include "usart1.h"
#include "usart2.h"
#include "timer1.h"
#include "timer2.h"
#include "timer3.h"
#include "adc.h"
#include "as5600.h"

#define FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(name) void (*name)(void)
typedef FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(FOC_Platform_HighRateCallback_t);

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

#endif /* FOC_PLATFORM_API_H */
