#ifndef FOC_CONTROL_C13_CFG_STATE_H
#define FOC_CONTROL_C13_CFG_STATE_H

#include <stdint.h>

#include "L3_Hal/foc_math_types.h"
#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"

void FOC_ControlConfigResetDefault(foc_control_cfg_t *cfg,
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
                                   foc_current_soft_switch_status_t *soft_switch,
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
                                   foc_cogging_comp_status_t *comp_status,
                                   int16_t *comp_table_q15,
#endif
                                   uint16_t comp_table_point_count);
void FOC_Control_ApplyConfig(foc_control_runtime_t *ctrl,
                             foc_pid_t *torque_pid,
                             foc_pid_t *speed_pid,
                             foc_pid_t *angle_pid,
                             foc_control_cfg_t *cfg,
                             const foc_motor_params_t *params);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

/* 复位 PID 动态状态（积分与误差历史），保留配置 */
void FOC_PIDReset(foc_pid_t *pid);

#endif /* FOC_CONTROL_C13_CFG_STATE_H */
