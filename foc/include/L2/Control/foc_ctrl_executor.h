#ifndef FOC_CTRL_EXECUTOR_H
#define FOC_CTRL_EXECUTOR_H

#include <stdint.h>

#include "L2/foc_ctrl_types.h"

/*
 * Control executor: runs the ISR fast-current path and the control-cycle
 * outer-loop pipeline.
 *
 * ISR path (FOC_ControlExecutor_RunISR):
 *   Sensor_ReadCurrentFast → Sensor_AccumulateEcycle → FOC_CurrentControlStep → SVPWM
 *
 * Control cycle (FOC_ControlExecutor_RunCycle):
 *   Sensor_ReadAll → FOC_ControlExecutor_RunOuterLoop
 *
 * RunCycle return codes (L2 → L1):
 */
#define FOC_CYCLE_OK            0U  /* control executed normally */
#define FOC_CYCLE_SKIPPED       1U  /* skipped (motor disabled, sensor temp invalid) */
#define FOC_CYCLE_FAULT_SENSOR  2U  /* sensor invalid threshold crossed */
#define FOC_CYCLE_FAULT_UVLO    3U  /* undervoltage lockout */

/* Initialise the executor state (after motor init). */
void FOC_ControlExecutor_Init(foc_motor_t *motor);

/* Stop fast current loop and zero output. */
void FOC_ControlExecutor_Stop(foc_motor_t *motor);

/* Apply safe (zero-voltage, zero-duty) output immediately.
 * Used by ISR on system_fault; also available for L1 to call synchronously. */
void FOC_ControlExecutor_SafeOutput(foc_motor_t *motor);

/* Run one control cycle: sensor → control → outer-loop.
 * Returns FOC_CYCLE_* code.  Does NOT set system_fault or system_running —
 * L1 uses the return code to update system state. */
uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor, float dt_sec);

/* Run outer-loop control: dispatch speed/angle mode + cogging compensation.
 * Called from RunCycle; also kept as separate API for testability. */
void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor,
                                      const sensor_data_t *sensor,
                                      float dt_sec);

/* Run one fast current-loop step inside PWM ISR.
 * Pipeline: sample → e-cycle → control → SVPWM. */
void FOC_ControlExecutor_RunISR(foc_motor_t *motor);

#endif /* FOC_CTRL_EXECUTOR_H */
																			