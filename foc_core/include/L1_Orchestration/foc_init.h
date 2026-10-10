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

/* 初始化 runtime 子系统（调度器/任务标志/通信/输出/监控/协议/硬件） */
void FOC_Init_Runtime(foc_system_t *sys, foc_motor_t *motor,
                      FOC_Platform_IsrCallback_t tick_cb,
                      FOC_Platform_IsrCallback_t service_cb,
                      FOC_Platform_IsrCallback_t control_cb,
                      FOC_Platform_IsrCallback_t monitor_cb,
                      FOC_Platform_IsrCallback_t pwm_cb,
                      FOC_Platform_IsrCallback_t current_loop_cb);

/* 初始化电机参数并应用配置（就绪前：只写配置初值，不做任何功率动作） */
void FOC_Init_Motor(foc_motor_t *motor);

/* 单一母线电压安全判定（上电门与运行期 trip 共用）：1=有效且不低于欠压阈值。
 * 保护功能关闭时恒返回 1（行为退化为不检查）。 */
uint8_t FOC_Init_IsVbusSafe(const sensor_data_t *sensor);

/* 就绪前母线电压门：多次采样后按阈值判定（唯一电压安全判定入口）。
 * 返回 1=可上电运行（电压安全），0=欠压/采样无效。 */
uint8_t FOC_Init_VbusGate(sensor_data_t *sensor);

/* 就绪前静态校验：通信/协议/命令/调试/PWM/传感器/母线电压（不含电机参数就绪位），
 * 决定 system_running / system_fault。vbus_ok 来自 FOC_Init_VbusGate。 */
void FOC_Init_Verify_Static(foc_motor_t *motor, uint8_t vbus_ok);

/* 就绪后电机参数就绪判定（STARTUP 对齐完成时调用，ISR 上下文安全：仅状态字段 + fast 短码）。
 * 返回 1=就绪并进入可运行状态，0=参数未定义（置初始化失败）。 */
uint8_t FOC_Init_Verify_Motor(foc_motor_t *motor);

#endif /* FOC_INIT_H */
