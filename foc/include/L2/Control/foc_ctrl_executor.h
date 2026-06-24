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
 *   Sensor_ReadAll → fault-check → FOC_ControlExecutor_RunOuterLoop
 */

/* Initialise the executor state (after motor init). */
void FOC_ControlExecutor_Init(foc_motor_t *motor);

/* Stop fast current loop and zero output. */
void FOC_ControlExecutor_Stop(foc_motor_t *motor);

/* Run one control cycle: sensor → fault → outer-loop.
 * Returns 0 if control was executed, 1 if skipped (fault / disabled / sensor invalid). */
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