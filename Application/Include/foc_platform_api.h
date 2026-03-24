#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>
#include "control_scheduler.h"

#define FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(name) void (*name)(void)
typedef FOC_PLATFORM_HIGH_RATE_CALLBACK_TYPE(FOC_Platform_HighRateCallback_t);

typedef struct {
	float phase_current_a;
	float phase_current_b;
	float phase_current_c;
	float mech_angle_rad;
	uint8_t adc_valid;
	uint8_t encoder_valid;
} foc_control_feedback_t;

typedef struct {
	float phase_current_a;
	float phase_current_b;
	float phase_current_c;
	float mech_angle_raw_rad;
	float mech_angle_filtered_rad;
	uint8_t adc_valid;
	uint8_t encoder_valid;
} foc_debug_feedback_t;

void FOC_Platform_RuntimeInit(void);
void FOC_Platform_IndicatorInit(void);
void FOC_Platform_SetHeartbeatIndicator(uint8_t on);

void FOC_Platform_HighRateClockInit(uint16_t pwm_freq_khz);
void FOC_Platform_ControlSchedulerInit(void);
void FOC_Platform_RegisterControlSchedulerCallback(ControlScheduler_Rate_t rate, ControlScheduler_Callback_t callback);
void FOC_Platform_StartControlScheduler(void);
void FOC_Platform_RegisterHighRateCallback(FOC_Platform_HighRateCallback_t callback);
void FOC_Platform_StartHighRateClock(void);

void FOC_Platform_TelemetryInit(void);
void FOC_Platform_TelemetryWrite(const char *str);

void FOC_Platform_FeedbackPipelineInit(uint8_t pwm_freq_khz);
void FOC_Platform_RefreshFeedbackSample(void);
uint8_t FOC_Platform_ReadControlFeedback(foc_control_feedback_t *feedback);
uint8_t FOC_Platform_ReadDebugFeedback(foc_debug_feedback_t *feedback);
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad);

void FOC_Platform_ModulationInit(uint16_t freq_khz, uint8_t deadtime_percent);

void FOC_Platform_PublishDebugWave(float iq);
void FOC_Platform_WaitMs(uint32_t ms);

#endif /* FOC_PLATFORM_API_H */
