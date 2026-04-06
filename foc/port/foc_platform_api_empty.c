#include "../include/foc_platform_api.h"

/*
 * Empty platform API template for new ports.
 * Keep all hooks as no-op or zero-return placeholders, then replace with target implementation.
 */

void FOC_Platform_RuntimeInit(void) {}
void FOC_Platform_HighRateClockInit(uint16_t high_rate_khz) { (void)high_rate_khz; }
void FOC_Platform_SetHighRateCallback(FOC_Platform_HighRateCallback_t callback) { (void)callback; }
void FOC_Platform_StartHighRateClock(void) {}
void FOC_Platform_ControlTickSourceInit(void) {}
void FOC_Platform_SetControlTickCallback(FOC_Platform_TickCallback_t callback) { (void)callback; }
void FOC_Platform_StartControlTickSource(void) {}
void FOC_Platform_IndicatorInit(void) {}
void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on) { (void)led_index; (void)on; }
void FOC_Platform_SetHeartbeatIndicator(uint8_t on) { (void)on; }
void FOC_Platform_CommInit(void) {}
void FOC_Platform_CommSource1_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback) { (void)callback; }
void FOC_Platform_CommSource2_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback) { (void)callback; }
void FOC_Platform_CommSource3_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback) { (void)callback; }
void FOC_Platform_CommSource4_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback) { (void)callback; }
uint16_t FOC_Platform_CommSource1_ReadFrame(uint8_t *buffer, uint16_t max_len) { (void)buffer; (void)max_len; return 0U; }
uint16_t FOC_Platform_CommSource2_ReadFrame(uint8_t *buffer, uint16_t max_len) { (void)buffer; (void)max_len; return 0U; }
uint16_t FOC_Platform_CommSource3_ReadFrame(uint8_t *buffer, uint16_t max_len) { (void)buffer; (void)max_len; return 0U; }
uint16_t FOC_Platform_CommSource4_ReadFrame(uint8_t *buffer, uint16_t max_len) { (void)buffer; (void)max_len; return 0U; }
void FOC_Platform_WriteDebugText(const char *str) { (void)str; }
void FOC_Platform_WriteStatusByte(uint8_t status_code) { (void)status_code; }
void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz) { (void)pwm_freq_khz; }
void FOC_Platform_SetSensorSampleOffsetPercent(float percent) { (void)percent; }
uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b) { (void)phase_current_a; (void)phase_current_b; return 0U; }
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad) { (void)angle_rad; return 0U; }
void FOC_Platform_WaitMs(uint32_t ms) { (void)ms; }
void FOC_Platform_PWMInit(uint8_t freq_khz, uint8_t deadtime_percent) { (void)freq_khz; (void)deadtime_percent; }
void FOC_Platform_PWMStart(void) {}
void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c) { (void)duty_a; (void)duty_b; (void)duty_c; }
void FOC_Platform_EnableCycleCounter(void) {}
uint32_t FOC_Platform_ReadCycleCounter(void) { return 0U; }
void FOC_Platform_UndervoltageProtect(float vbus_voltage) { (void)vbus_voltage; }
