#ifndef FOC_CONTROL_C21_OUTER_LOOP_H
#define FOC_CONTROL_C21_OUTER_LOOP_H

#include <stdint.h>

#include "L3_Hal/foc_math_types.h"
#include "L2_Core/foc_ctrl_types.h"

void FOC_ControlRebaseMechanicalAngleAccum(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlResetSpeedLoopState(foc_motor_t *motor);

/* 加速器：对 speed_ref 做斜率限制 + 区域上限钳位 */
float FOC_Accel_ApplySpeedLimit(foc_motor_t *motor, float target_mech_speed_rad_s,
                                float ramp_rate_rad_s2, float speed_limit_rad_s, float dt_sec);
void FOC_Accel_ResetState(foc_motor_t *motor);

void FOC_SpeedOuterLoopStep(foc_motor_t *motor,
                            foc_pid_t *speed_pid,
                            float speed_ref_rad_s,
                            float dt_sec);

void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec);

#endif /* FOC_CONTROL_C21_OUTER_LOOP_H */
