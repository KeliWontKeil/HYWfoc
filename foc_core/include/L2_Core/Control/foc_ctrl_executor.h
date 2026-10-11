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

#if (FOC_SPECIAL_PHASE_ABORT_ENABLE == FOC_CFG_ENABLE)
/** @brief 特殊控制相位中止（协议 Y:A 与 L1 自动退出守卫共用的单一收口）：
 *         各相位回滚安全态 → fast 通告 → 归 NORMAL → 全停 → 重建控制基准。
 *         ISR 与主循环均可调用，只用 fast 通道与字面量文本。 */
void FOC_ControlExecutor_AbortSpecialPhase(foc_motor_t *motor);
#endif

/** @brief 错误复位（协议 Y:C）收口：清错误计数 + 重建控制基准 + 按参数就绪情况回 STARTUP/NORMAL。
 *         须在 system_fault 仍为 1（ISR 走 FullStop）时调用，无竞态。 */
void FOC_ControlExecutor_RecoverFault(foc_motor_t *motor);

/** @brief 处理一个控制周期结果（L1 采样/阈值判定产出码后转调）：
 *         OK→维持运行；FAULT_*→锁存故障 + 全停 + 突发短码。
 *         运行状态迁移与安全动作统一收口于 L2。 */
void FOC_ControlExecutor_OnCycleResult(foc_motor_t *motor, uint8_t cycle_result);

/** @brief Outer-loop unified entry (speed or speed-angle). */
void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor, float dt_sec);

/** @brief 控制参考单点发布（控制 ISR 过程末尾调用，供电流环 ISR 原子获取）。 */
void FOC_ControlExecutor_PublishControlRef(foc_motor_t *motor);

/** @brief 构建 Source Manager 上下文视图（L1 初始化与 ISR 快线共用，消除重复填充）。 */
void FOC_ControlExecutor_BuildSourceMgrCtx(foc_motor_t *motor,
                                           const foc_control_ref_t *ref,
                                           foc_source_mgr_ctx_t *ctx);

#endif /* FOC_CTRL_EXECUTOR_H */
