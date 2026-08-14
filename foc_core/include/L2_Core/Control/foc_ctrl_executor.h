#ifndef FOC_CTRL_EXECUTOR_H
#define FOC_CTRL_EXECUTOR_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Control/foc_ctrl_source_mgr.h"
#include "LS_Config/foc_config.h"

/** RunCycle return codes */
#define FOC_CYCLE_OK                0U
#define FOC_CYCLE_SKIPPED           1U
#define FOC_CYCLE_FAULT_SENSOR      2U
#define FOC_CYCLE_FAULT_UVLO        3U

/** @brief Initialise per-motor control executor state. */
void FOC_ControlExecutor_Init(foc_motor_t *motor);

/** @brief PWM ISR entry（双 ISR 模式）: fast current-loop sampling → control → SVPWM.
 *         Skips when control_phase != NORMAL. */
void FOC_ControlExecutor_RunISR(foc_motor_t *motor);

#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
/** @brief PWM ISR entry（三 ISR 模式）: 仅插值 + 守卫检查，不运行电流环。 */
void FOC_ControlExecutor_RunISR_PwmOnly(foc_motor_t *motor);

/** @brief 电流环 ISR entry（三 ISR 模式）: 独立定时器驱动，与 PWM 频率解耦。 */
void FOC_ControlExecutor_RunISR_CurrentLoop(foc_motor_t *motor);
#endif

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

/** @brief 控制参考单点发布（控制 ISR 过程末尾调用，供电流环 ISR 原子获取）。 */
void FOC_ControlExecutor_PublishControlRef(foc_motor_t *motor);

/** @brief 构建 Source Manager 上下文视图（L1 初始化与 ISR 快线共用，消除重复填充）。 */
void FOC_ControlExecutor_BuildSourceMgrCtx(foc_motor_t *motor,
                                           const foc_control_ref_t *ref,
                                           foc_source_mgr_ctx_t *ctx);

#endif /* FOC_CTRL_EXECUTOR_H */
