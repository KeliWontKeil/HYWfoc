#ifndef FOC_PLATFORM_API_H
#define FOC_PLATFORM_API_H

#include <stdint.h>

/* Control-tick callback type used by platform timer source. */
typedef void (*FOC_Platform_TickCallback_t)(void);

/* High-rate callback type used by modulation/ISR tick source. */
typedef void (*FOC_Platform_HighRateCallback_t)(void);

/* Communication RX trigger callback type (e.g., DMA IDLE event hook). */
typedef void (*FOC_Platform_CommRxTriggerCallback_t)(void);

/*
 * Lifecycle contract summary:
 * 1) Init-and-activate immediately: Runtime, Communication, SensorInput.
 * 2) Init plus explicit Start required: ControlTickSource, HighRateClock, PWM.
 * 3) For callback-driven modules, callback must be set before Start.
 */

/* ===== Runtime / Clock ===== */

/*
 * Initialize runtime base services (current GD32 mapping: SysTick).
 * Call timing: once at startup before other platform APIs.
 * Parameters: none.
 * Return: none.
 * Stub guidance: may be empty for targets without runtime base service.
 */
void FOC_Platform_RuntimeInit(void);

/*
 * Initialize high-rate hardware clock source in kHz.
 * Call timing: startup phase, before FOC_Platform_SetHighRateCallback and Start.
 * Parameters:
 * - high_rate_khz: desired high-rate source frequency in kHz, must be > 0.
 * Return: none.
 * Lifecycle: init-only, output does not start until FOC_Platform_StartHighRateClock.
 * Stub guidance: if unsupported, keep empty and ensure Start is also harmless.
 */
void FOC_Platform_HighRateClockInit(uint16_t high_rate_khz);

/*
 * Set high-rate callback function.
 * Call timing: after HighRateClockInit and before StartHighRateClock.
 * Parameters:
 * - callback: callback function pointer, NULL allowed to disable callback.
 * Return: none.
 * IRQ notes: callback must be IRQ-safe and bounded in execution time.
 */
void FOC_Platform_SetHighRateCallback(FOC_Platform_HighRateCallback_t callback);

/*
 * Start high-rate hardware clock source.
 * Call timing: after HighRateClockInit and SetHighRateCallback.
 * Parameters: none.
 * Return: none.
 * Lifecycle: explicit start required.
 */
void FOC_Platform_StartHighRateClock(void);

/*
 * Initialize control tick source (current GD32 mapping: TIMER1 update source).
 * Call timing: startup phase before callback bind/start.
 * Parameters: none.
 * Return: none.
 * Lifecycle: init-only, source does not run until StartControlTickSource.
 */
void FOC_Platform_ControlTickSourceInit(void);

/*
 * Set control tick callback function.
 * Call timing: after ControlTickSourceInit and before StartControlTickSource.
 * Parameters:
 * - callback: callback function pointer, NULL allowed to disable callback.
 * Return: none.
 * IRQ notes: callback runs in timer IRQ context and must be non-blocking.
 */
void FOC_Platform_SetControlTickCallback(FOC_Platform_TickCallback_t callback);

/*
 * Start control tick source.
 * Call timing: after ControlTickSourceInit and SetControlTickCallback.
 * Parameters: none.
 * Return: none.
 * Lifecycle: explicit start required.
 */
void FOC_Platform_StartControlTickSource(void);

/* ===== Indicator ===== */

/*
 * Initialize board indicator peripherals (e.g., LEDs).
 * Call timing: startup phase.
 * Parameters: none.
 * Return: none.
 * Lifecycle: init-and-activate immediately.
 */
void FOC_Platform_IndicatorInit(void);

/*
 * Set indicator on/off state by logical index.
 * Call timing: runtime safe.
 * Parameters:
 * - led_index: logical indicator index defined by upper-layer config.
 * - on: 0 for off, non-zero for on.
 * Return: none.
 * Stub guidance: unsupported index may be ignored safely.
 */
void FOC_Platform_SetIndicator(uint8_t led_index, uint8_t on);

/*
 * Set heartbeat indicator state.
 * Call timing: runtime safe.
 * Parameters:
 * - on: 0 for off, non-zero for on.
 * Return: none.
 */
void FOC_Platform_SetHeartbeatIndicator(uint8_t on);

/* ===== Communication ===== */

/*
 * Initialize communication peripherals used by platform transport.
 * Call timing: startup phase.
 * Parameters: none.
 * Return: none.
 * Lifecycle: init-and-activate immediately.
 */
void FOC_Platform_CommInit(void);

/*
 * Set RX trigger callback for communication source 1.
 * Parameters:
 * - callback: trigger callback function, NULL allowed to disable trigger.
 * Return: none.
 */
void FOC_Platform_CommSource1_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);

/*
 * Set RX trigger callback for communication source 2.
 * Parameters:
 * - callback: trigger callback function, NULL allowed to disable trigger.
 * Return: none.
 */
void FOC_Platform_CommSource2_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);

/*
 * Set RX trigger callback for communication source 3.
 * Parameters:
 * - callback: trigger callback function, NULL allowed.
 * Return: none.
 * Stub guidance: source 3 may be weak-empty on platforms without this source.
 */
void FOC_Platform_CommSource3_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);

/*
 * Set RX trigger callback for communication source 4.
 * Parameters:
 * - callback: trigger callback function, NULL allowed.
 * Return: none.
 * Stub guidance: source 4 may be weak-empty on platforms without this source.
 */
void FOC_Platform_CommSource4_SetRxTriggerCallback(FOC_Platform_CommRxTriggerCallback_t callback);

/*
 * Read one received frame from communication source 1.
 * Parameters:
 * - buffer: destination byte buffer, must not be NULL.
 * - max_len: buffer capacity in bytes.
 * Return:
 * - >0: frame length in bytes.
 * - 0: no frame available or unsupported.
 */
uint16_t FOC_Platform_CommSource1_ReadFrame(uint8_t *buffer, uint16_t max_len);

/*
 * Read one received frame from communication source 2.
 * Parameters:
 * - buffer: destination byte buffer, must not be NULL.
 * - max_len: buffer capacity in bytes.
 * Return:
 * - >0: frame length in bytes.
 * - 0: no frame available or unsupported.
 */
uint16_t FOC_Platform_CommSource2_ReadFrame(uint8_t *buffer, uint16_t max_len);

/*
 * Read one received frame from communication source 3.
 * Parameters:
 * - buffer: destination byte buffer, must not be NULL.
 * - max_len: buffer capacity in bytes.
 * Return:
 * - >0: frame length in bytes.
 * - 0: no frame available or unsupported.
 * Stub guidance: weak implementation should return 0 without side effects.
 */
uint16_t FOC_Platform_CommSource3_ReadFrame(uint8_t *buffer, uint16_t max_len);

/*
 * Read one received frame from communication source 4.
 * Parameters:
 * - buffer: destination byte buffer, must not be NULL.
 * - max_len: buffer capacity in bytes.
 * Return:
 * - >0: frame length in bytes.
 * - 0: no frame available or unsupported.
 * Stub guidance: weak implementation should return 0 without side effects.
 */
uint16_t FOC_Platform_CommSource4_ReadFrame(uint8_t *buffer, uint16_t max_len);

/*
 * Write human-readable debug text to host output channel.
 * Call timing: runtime safe; avoid heavy/continuous output in tight real-time paths.
 * Parameters:
 * - str: null-terminated text pointer, must not be NULL.
 * Return: none.
 * Stub guidance: empty implementation is allowed on no-console targets.
 */
void FOC_Platform_WriteDebugText(const char *str);

/*
 * Write one compact status byte to host output channel.
 * Call timing: runtime safe, suitable for protocol feedback.
 * Parameters:
 * - status_code: protocol status byte.
 * Return: none.
 * Stub guidance: empty implementation is allowed on no-transport targets.
 */
void FOC_Platform_WriteStatusByte(uint8_t status_code);

/* ===== Sensor / Acquisition ===== */

/*
 * Initialize sensor input pipeline (encoder + trigger timer + ADC path).
 * Call timing: startup phase before sensor reads.
 * Parameters:
 * - pwm_freq_khz: control PWM-related frequency in kHz, must be > 0.
 * Return: none.
 * Lifecycle: init-and-activate immediately in current implementation.
 */
void FOC_Platform_SensorInputInit(uint8_t pwm_freq_khz);

/*
 * Set sensor sampling trigger offset as percent within control period.
 * Call timing: runtime safe.
 * Parameters:
 * - percent: recommended range [0, 100].
 * Return: none.
 */
void FOC_Platform_SetSensorSampleOffsetPercent(float percent);

/*
 * Read phase-A and phase-B currents.
 * Parameters:
 * - phase_current_a: output pointer for phase-A current, must not be NULL.
 * - phase_current_b: output pointer for phase-B current, must not be NULL.
 * Return:
 * - non-zero: read success.
 * - 0: read failed or data unavailable.
 */
uint8_t FOC_Platform_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b);

/*
 * Read mechanical angle in radians.
 * Parameters:
 * - angle_rad: output pointer for angle value, must not be NULL.
 * Return:
 * - non-zero: read success.
 * - 0: read failed or data unavailable.
 */
uint8_t FOC_Platform_ReadMechanicalAngleRad(float *angle_rad);

/*
 * Blocking delay in milliseconds.
 * Call timing: avoid in hard real-time control ISR paths.
 * Parameters:
 * - ms: delay duration in milliseconds.
 * Return: none.
 * Stub guidance: may be mapped to no-op in unit-test target if timing is mocked.
 */
void FOC_Platform_WaitMs(uint32_t ms);

/* ===== PWM / Actuation ===== */

/*
 * Initialize PWM peripheral for motor actuation.
 * Call timing: startup phase before PWMStart.
 * Parameters:
 * - freq_khz: PWM carrier frequency in kHz, must be > 0.
 * - deadtime_percent: deadtime percentage configuration.
 * Return: none.
 * Lifecycle: init-only, output does not run until FOC_Platform_PWMStart.
 */
void FOC_Platform_PWMInit(uint8_t freq_khz, uint8_t deadtime_percent);

/*
 * Start PWM output path.
 * Call timing: after PWMInit and initial duty setup.
 * Parameters: none.
 * Return: none.
 * Lifecycle: explicit start required.
 */
void FOC_Platform_PWMStart(void);

/*
 * Write three-phase duty cycles.
 * Call timing: runtime safe, often called in high-rate callback path.
 * Parameters:
 * - duty_a: phase-A duty in normalized range [0, 1].
 * - duty_b: phase-B duty in normalized range [0, 1].
 * - duty_c: phase-C duty in normalized range [0, 1].
 * Return: none.
 */
void FOC_Platform_PWMSetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);

/* ===== Diagnostics / Profiler ===== */

/*
 * Enable cycle counter for execution time profiling.
 * Call timing: runtime startup before cycle reads.
 * Parameters: none.
 * Return: none.
 * Stub guidance: on targets without cycle counter, keep empty and return 0 in read API.
 */
void FOC_Platform_EnableCycleCounter(void);

/*
 * Read current cycle counter value.
 * Parameters: none.
 * Return: cycle counter value; 0 is allowed for unsupported stub targets.
 */
uint32_t FOC_Platform_ReadCycleCounter(void);

/* ===== Protection Hook ===== */

/*
 * Optional undervoltage protection hook.
 * Call timing: runtime control loop path.
 * Parameters:
 * - vbus_voltage: measured bus voltage in volts.
 * Return: none.
 * Stub guidance: no-op implementation is valid when platform lacks actuator support.
 */
void FOC_Platform_UndervoltageProtect(float vbus_voltage);

#endif /* FOC_PLATFORM_API_H */
