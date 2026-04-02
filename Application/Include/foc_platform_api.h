#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>
#include "foc_shared_types.h"

#define FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(name) void (*name)(void)
typedef FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(FOC_Platform_HighRateCallback_t);

typedef void (*FOC_Platform_CommRxTriggerCallback_t)(void);

void FOC_Platform_RuntimeInit(void);
void FOC_Platform_IndicatorInit(void);
void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on);
void FOC_Platform_SetHeartbeatIndicator(uint8_t on);

void FOC_Platform_HighRateClockInit(uint16_t pwm_freq_khz);
void FOC_Platform_ControlTickSourceInit(void);
void FOC_Platform_BindControlTickCallback(FOC_SchedulerCallback_t callback);
void FOC_Platform_StartControlTickSource(void);
void FOC_Platform_RegisterHighRateCallback(FOC_Platform_HighRateCallback_t callback);
void FOC_Platform_StartHighRateClock(void);

void FOC_Platform_CommInit(void);
void FOC_Platform_CommSource1_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);
void FOC_Platform_CommSource2_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);
void FOC_Platform_CommSource3_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);
void FOC_Platform_CommSource4_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);
uint16_t FOC_Platform_CommSource1_ReadFrame(uint8_t *buffer, uint16_t max_len);
uint16_t FOC_Platform_CommSource2_ReadFrame(uint8_t *buffer, uint16_t max_len);
uint16_t FOC_Platform_CommSource3_ReadFrame(uint8_t *buffer, uint16_t max_len);
uint16_t FOC_Platform_CommSource4_ReadFrame(uint8_t *buffer, uint16_t max_len);
void FOC_Platform_DebugOutput(const char *str);
void FOC_Platform_FeedbackOutput(uint8_t status_code);

void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz);
uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b);
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad);
void FOC_Platform_SetSensorSampleOffsetPercent(float percent);
void FOC_Platform_WaitMs(uint32_t ms);

/*
 * Placeholder API for undervoltage protection hook.
 * vbus_voltage: current DC bus voltage in volts.
 * Current hardware does not provide a controllable undervoltage protection actuator,
 * so this function is intentionally implemented as a no-op in platform layer.
 */
void FOC_Platform_UndervoltageProtect(float vbus_voltage);

void FOC_Platform_EnableCycleCounter(void);
uint32_t FOC_Platform_ReadCycleCounter(void);

void FOC_Platform_PWMInit(uint8_t freq_khz, uint8_t deadtime_percent);
void FOC_Platform_PWMStart(void);
void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);

#endif /* FOC_PLATFORM_API_H */
