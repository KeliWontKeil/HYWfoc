#ifndef FOC_CTRL_ENTRY_H
#define FOC_CTRL_ENTRY_H

#include <stdint.h>

#include "LS_Config/foc_motor_types.h"

/* 运行一帧控制外环（速度/角度模式）。
 * 内部读取 motor->cfg.control_mode、motor->cfg.* 等参数，
 * 使用 motor->torque_current_pid / speed_pid / angle_pid。
 */
void FOC_Control_Run(foc_motor_t *motor, const sensor_data_t *sensor, float dt_sec);

/* 电流环步进（在PWM ISR中调用）。
 * 内部使用 motor->torque_current_pid。
 */
void FOC_Control_CurrentLoop(foc_motor_t *motor, const sensor_data_t *sensor,
                             float electrical_angle, float dt_sec);

/* 应用配置：从 motor->cfg 读取 PID 参数和 fine-tuning 设置，重算 PID 限幅。
 * 由 L1 在检测到 cfg_dirty 后调用。
 */
void FOC_Control_ApplyConfig(foc_motor_t *motor);

/* 初始化控制算法状态 */
void FOC_Control_Init(foc_motor_t *motor);

/* 开环步进 */
void FOC_Control_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed);

/* 查询电流环是否需要采样 */
uint8_t FOC_Control_CurrentLoopRequiresSample(void);

/* 齿槽补偿标定接口 */
uint8_t FOC_Control_CoggingCalibIsBusy(const foc_motor_t *motor);
uint8_t FOC_Control_CoggingCalibSampleStep(foc_motor_t *motor,
                                           const sensor_data_t *sensor,
                                           float dt_sec);
void FOC_Control_CoggingCalibRequestStart(foc_motor_t *motor);
void FOC_Control_CoggingCalibDumpTable(const foc_motor_t *motor);
void FOC_Control_CoggingCalibExportTable(const foc_motor_t *motor);

#endif /* FOC_CTRL_ENTRY_H */