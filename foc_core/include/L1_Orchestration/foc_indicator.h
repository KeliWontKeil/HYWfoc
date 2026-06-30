#ifndef FOC_INDICATOR_H
#define FOC_INDICATOR_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L1_Orchestration/foc_system_types.h"

/*
 * L1 指示器管理
 *
 * 根据电机状态和运行时状态更新 LED 指示器。
 * 与 foc_app.c 的主循环解耦，可在 ISR 上下文中调用。
 */
void FOC_Indicator_Update(foc_motor_t *motor, foc_runtime_ctx_t *runtime);

#endif /* FOC_INDICATOR_H */