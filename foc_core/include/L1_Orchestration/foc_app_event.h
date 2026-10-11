#ifndef FOC_APP_EVENT_H
#define FOC_APP_EVENT_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L2_Core/foc_motor_aggregate.h"
#include "L1_Orchestration/foc_system_types.h"

/*
 * L1 应用级事件与通告（主循环上下文）
 *
 * 收敛"由系统状态跃迁驱动的对外通告"：
 *   - fault 0→1 跃迁当轮补发完整可读详情（慢路径可靠文本）
 *   - 对齐/重新对齐完成后补发启动信息（ISR 内不做长文本）
 *   - 就绪沿声学播报（上电长鸣 / 其后进入正常态两短鸣）
 *
 * 事件状态由 L1 持有（`foc_system_t.events`，跨 reinit 保持），不散落为模块级静态变量。
 */

/* 主循环事件段（FOC_App_Loop 首段调用） */
void FOC_AppEvent_Run(foc_system_t *sys, foc_motor_t *motor);

#endif /* FOC_APP_EVENT_H */
