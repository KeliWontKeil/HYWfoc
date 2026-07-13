#ifndef FOC_INIT_H
#define FOC_INIT_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L1_Orchestration/foc_system_types.h"
#include "L3_Hal/foc_platform_api.h"

/*
 * L1 系统初始化管理
 *
 * 整合系统初始化序列和完整性校验。
 * 所有函数在 FOC_App_Init 中按序调用。
 */

/* 初始化 runtime 子系统（调度器/队列/通信/协议/调试流/硬件） */
void FOC_Init_Runtime(foc_system_t *sys, foc_motor_t *motor,
                      FOC_Platform_TickCallback_t tick_cb,
                      FOC_Platform_TickCallback_t service_cb,
                      FOC_Platform_TickCallback_t control_cb,
                      FOC_Platform_TickCallback_t monitor_cb,
                      FOC_Platform_PwmIsrCallback_t pwm_cb);

/* 初始化电机参数并应用配置 */
void FOC_Init_MotorAndCalib(foc_motor_t *motor);

/* 初始化完整性校验 */
void FOC_Init_Verify(foc_motor_t *motor, const sensor_data_t *sensor);

#endif /* FOC_INIT_H */