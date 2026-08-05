#ifndef FOC_CONTROL_C13_CFG_STATE_H
#define FOC_CONTROL_C13_CFG_STATE_H

#include <stdint.h>

#include "L3_Hal/foc_math_types.h"
#include "L2_Core/foc_ctrl_types.h"

void FOC_ControlConfigResetDefault(foc_motor_t *motor);
void FOC_Control_ApplyConfig(foc_motor_t *motor);

/* 软切换状态只读窗（协议/调试层查询） */
const foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatus(const foc_motor_t *motor);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

#endif /* FOC_CONTROL_C13_CFG_STATE_H */
