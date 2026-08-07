#ifndef FOC_CONTROL_C21_OUTER_LOOP_H
#define FOC_CONTROL_C21_OUTER_LOOP_H

#include <stdint.h>

#include "L3_Hal/foc_math_types.h"
#include "L2_Core/foc_ctrl_types.h"

/* ========== Outer-loop runtime state (private to outer_loop) ========== */
typedef struct {
    float  speed_err_accum_rad;
    float  prev_mech_signed_rad;
    uint8_t speed_state_valid;
    float  accum_rad;
    float  prev_rad;
    uint8_t prev_valid;

    float  ramped_speed_rad_s;      /* 加速器当前限幅后速度 */
} foc_outer_loop_private_t;

/* 加速器：对 speed_ref 做斜率限制 + 区域上限钳位（纯域计算，不持 motor 聚合） */
float FOC_Accel_ApplySpeedLimit(foc_outer_loop_private_t *state,
                                float target_mech_speed_rad_s,
                                float ramp_rate_rad_s2,
                                float speed_limit_rad_s,
                                float dt_sec);

void FOC_SpeedOuterLoopStep(foc_outer_loop_private_t *state,
                            foc_pid_t *speed_pid,
                            foc_control_runtime_t *ctrl,
                            const foc_active_source_state_t *active,
                            const foc_control_cfg_t *cfg,
                            const foc_motor_params_t *params,
                            uint8_t control_region,
                            float speed_ref_rad_s,
                            float dt_sec);

void FOC_SpeedAngleOuterLoopStep(foc_outer_loop_private_t *state,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 foc_control_runtime_t *ctrl,
                                 const foc_active_source_state_t *active,
                                 const foc_control_cfg_t *cfg,
                                 const foc_motor_params_t *params,
                                 float angle_ref_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec);

#endif /* FOC_CONTROL_C21_OUTER_LOOP_H */