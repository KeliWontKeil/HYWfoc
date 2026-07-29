#ifndef FOC_CTRL_EXECUTOR_H
#define FOC_CTRL_EXECUTOR_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

/** RunCycle return codes */
#define FOC_CYCLE_OK                0U
#define FOC_CYCLE_SKIPPED           1U
#define FOC_CYCLE_FAULT_SENSOR      2U
#define FOC_CYCLE_FAULT_UVLO        3U

/** @brief Initialise per-motor control executor state. */
void FOC_ControlExecutor_Init(foc_motor_t *motor);

/** @brief PWM ISR entry: fast current-loop sampling → control → SVPWM.
 *         Skips when control_phase != NORMAL. */
void FOC_ControlExecutor_RunISR(foc_motor_t *motor);

/**
 * @brief Normal control cycle: sensor validity checked by L1.
 *        Called from L1 only when control_phase == NORMAL.
 */
uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor, float dt_sec);

/** @brief Full stop: reset all control state and zero PWM output.
 *         Converges fault / disable / phase-switch stop paths. */
void FOC_ControlExecutor_FullStop(foc_motor_t *motor);

/** @brief Safe output: zero voltage, open-loop stop. */
void FOC_ControlExecutor_SafeOutput(foc_motor_t *motor);

/** @brief Stop motor output (open-loop zero). */
void FOC_ControlExecutor_Stop(foc_motor_t *motor);

/** @brief Outer-loop unified entry (speed or speed-angle). */
void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor, float dt_sec);

#endif /* FOC_CTRL_EXECUTOR_H */
