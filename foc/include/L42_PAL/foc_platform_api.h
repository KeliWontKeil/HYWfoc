#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>

/** @brief Control tick callback type for the platform timer source. */
typedef void (*FOC_Platform_TickCallback_t)(void);

/** @brief PWM update ISR callback type for high-rate modulation update path. */
typedef void (*FOC_Platform_PwmIsrCallback_t)(void);

/* ===== Runtime / Clock ===== */

/** @brief Initialize runtime base services for the platform. */
void FOC_Platform_RuntimeInit(void);

/** @brief Set callback used by PWM update ISR path. */
void FOC_Platform_SetPwmUpdateCallback(FOC_Platform_PwmIsrCallback_t callback);

/** @brief Initialize the control-tick source. */
void FOC_Platform_ControlTickSourceInit(void);

/** @brief Set the callback used by the control-tick source. */
void FOC_Platform_SetControlTickCallback(FOC_Platform_TickCallback_t callback);

/** @brief Start the control-tick source. */
void FOC_Platform_StartControlTickSource(void);

/** @brief Enable or disable runtime control interrupts. */
void FOC_Platform_SetControlRuntimeInterrupts(uint8_t enable);

/* ===== Indicator ===== */

/** @brief Initialize board indicator peripherals. */
void FOC_Platform_IndicatorInit(void);

/** @brief Set indicator on/off state by logical indicator index. */
void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on);

/** @brief Set heartbeat indicator state. */
void FOC_Platform_SetHeartbeatIndicator(uint8_t on);

/* ===== Communication ===== */

/** @brief Initialize communication peripherals used by platform transport. */
void FOC_Platform_CommInit(void);

/** @brief Return non-zero when communication source 1 has one complete frame ready. */
uint8_t FOC_Platform_CommSource1_IsFrameReady(void);

/** @brief Return non-zero when communication source 2 has one complete frame ready. */
uint8_t FOC_Platform_CommSource2_IsFrameReady(void);

/** @brief Return non-zero when communication source 3 has one complete frame ready. */
uint8_t FOC_Platform_CommSource3_IsFrameReady(void);

/** @brief Return non-zero when communication source 4 has one complete frame ready. */
uint8_t FOC_Platform_CommSource4_IsFrameReady(void);

/** @brief Read one received frame from communication source 1. */
uint16_t FOC_Platform_CommSource1_ReadFrame(uint8_t *buffer, uint16_t max_len);

/** @brief Read one received frame from communication source 2. */
uint16_t FOC_Platform_CommSource2_ReadFrame(uint8_t *buffer, uint16_t max_len);

/** @brief Read one received frame from communication source 3. */
uint16_t FOC_Platform_CommSource3_ReadFrame(uint8_t *buffer, uint16_t max_len);

/** @brief Read one received frame from communication source 4. */
uint16_t FOC_Platform_CommSource4_ReadFrame(uint8_t *buffer, uint16_t max_len);

/** @brief Write human-readable debug text to host output channel. */
void FOC_Platform_WriteDebugText(const char *str);

/** @brief Write one compact status byte to host output channel. */
void FOC_Platform_WriteStatusByte(uint8_t status_code);

/* ===== Sensor / Acquisition ===== */

/** @brief Initialize sensor input pipeline for current and angle acquisition. */
void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz);

/** @brief Set sensor sampling trigger offset percent in the control period. */
void FOC_Platform_SetSensorSampleOffsetPercent(float percent);

/** @brief Read phase-A and phase-B currents. */
uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b);

/** @brief Read phase-A and phase-B currents for fast current-loop path. */
uint8_t FOC_Platform_ReadPhaseCurrentABFast(float *phase_current_a, float *phase_current_b);

/** @brief Read mechanical angle in radians. */
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad);

/** @brief Perform a blocking delay in milliseconds. */
void FOC_Platform_WaitMs(uint32_t ms);

/* ===== PWM / Actuation ===== */

/** @brief Initialize PWM peripheral for motor actuation. */
void FOC_Platform_PWMInit(uint8_t freq_khz, uint8_t deadtime_percent);

/** @brief Start PWM output path. */
void FOC_Platform_PWMStart(void);

/** @brief Write three-phase duty cycles in normalized range. */
void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);

/* ===== Diagnostics / Profiler ===== */

/** @brief Enable cycle counter for execution time profiling. */
void FOC_Platform_EnableCycleCounter(void);

/** @brief Read current cycle counter value. */
uint32_t FOC_Platform_ReadCycleCounter(void);

/* ===== VBUS Voltage Sampling ===== */

/** @brief Read VBUS voltage via ADC2 (software trigger, EOC polling). */
uint8_t FOC_Platform_ReadVbusVoltage(float *vbus_v);

/* ===== Protection Hook ===== */

/** @brief Execute optional undervoltage protection hook. */
void FOC_Platform_UndervoltageProtect(float vbus_voltage);

#endif /* FOC_PLATFORM_API_H */


