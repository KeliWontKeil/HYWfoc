#ifndef FOC_CONTROL_C12_INIT_H
#define FOC_CONTROL_C12_INIT_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float max_phase_voltage,
                   float phase_resistance,
                   float stator_inductance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction);

/* Initialise sensor, SVPWM, and fast control-executor hardware.
 * L1 calls this once during boot (or after reinit) instead of calling
 * Sensor_*, SVPWM_*, FOC_ControlExecutor_Init directly. */
void FOC_ControlPlatform_InitHardware(foc_motor_t *motor);

/* 重建"运行期控制基准"：从停止态恢复（使能/错误复位/abort/重新对齐）时统一调用。
 * 只重建运行期状态，不触碰用户配置/电机参数/源配置。须在电机停止态调用（无 ISR 竞态）。 */
void FOC_Control_RebuildControlBasis(foc_motor_t *motor);

/* 电机参数是否已标定（方向/零点/极对数均非未定义）。
 * 供就绪判定（FOC_Control_VerifyMotor）、启动阶段选择与错误恢复（Y:C）共用同一判据。 */
uint8_t FOC_Control_IsMotorParamCalibrated(const foc_motor_params_t *params);

/* 电机参数就绪判定（STARTUP 对齐完成、或被中止时调用）：
 * 1 = 就绪并进入可运行状态，0 = 参数未定义（置初始化失败）。
 * ISR 上下文安全：仅状态字段 + fast 短码，不做长文本。 */
uint8_t FOC_Control_VerifyMotor(foc_motor_t *motor);

/* 启动阶段选择策略（唯一入口）：自检通过且电机参数待标定（有编码器且允许对齐）时返回 1。
 * 供上电初始化与错误恢复（Y:C）共用，避免同一判据在多处各写一份。 */
uint8_t FOC_Control_ShouldStartupAlign(const foc_motor_t *motor);

#endif /* FOC_CONTROL_C12_INIT_H */
