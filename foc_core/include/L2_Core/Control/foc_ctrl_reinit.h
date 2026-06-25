#ifndef FOC_CTRL_REINIT_H
#define FOC_CTRL_REINIT_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "LS_Config/foc_config.h"

/*
 * =====================================================================
 * 非阻塞重初始化模块（L2/Control）
 *
 * 职责：将运行时重初始化改写为非阻塞状态机，由 L1 控制任务
 * 在每个控制周期步进，避免阻塞主循环。
 *
 * 上电初始化保持阻塞（FOC_MotorInit → FOC_CalibrateElectricalAngleAndDirection），
 * 不受本模块影响。
 *
 * 调用关系：
 *   L1 ControlTrigger → motor->state.control_phase == REINIT → FOC_ReInit_RunStep()
 *   协议 Y:R 命令 → motor->state.control_phase = REINIT → 下一个控制周期启动模块
 *
 * 编译期裁剪：由 FOC_REINIT_ENABLE 宏控制。
 *    - 当 FOC_REINIT_ENABLE == FOC_CFG_DISABLE 时，所有 API 展开为空内联。
 * =====================================================================
 */

#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)

/* 请求重初始化（由协议 Y:R 命令调用） */
static inline void FOC_ReInit_Request(foc_motor_t *motor)
{
    if (motor != 0)
    {
        motor->state.control_phase = FOC_CONTROL_PHASE_REINIT;
    }
}

/* 返回非零表示重初始化正在进行中 */
static inline uint8_t FOC_ReInit_IsBusy(const foc_motor_t *motor)
{
    if (motor == 0) return 0U;
    return (motor->state.control_phase == FOC_CONTROL_PHASE_REINIT) ? 1U : 0U;
}

/*
 * 重初始化状态机步进，由 L1 控制任务在每个控制周期调用。
 * 返回 1 表示仍在进行中，0 表示已完成（且 control_phase 已切回 NORMAL）。
 */
uint8_t FOC_ReInit_RunStep(foc_motor_t *motor, float dt_sec);

#else /* FOC_REINIT_ENABLE == FOC_CFG_DISABLE */

static inline void FOC_ReInit_Request(foc_motor_t *motor) { (void)motor; }
static inline uint8_t FOC_ReInit_IsBusy(const foc_motor_t *motor) { (void)motor; return 0U; }
static inline uint8_t FOC_ReInit_RunStep(foc_motor_t *motor, float dt_sec) { (void)motor; (void)dt_sec; return 0U; }

#endif /* FOC_REINIT_ENABLE */

#endif /* FOC_CTRL_REINIT_H */
