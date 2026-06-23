#ifndef FOC_CONTROL_C21_OUTER_LOOP_H
#define FOC_CONTROL_C21_OUTER_LOOP_H

#include <stdint.h>

#include "LS_Config/foc_math_types.h"
#include "LS_Config/foc_motor_types.h"

void FOC_ControlRebaseMechanicalAngleAccum(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlResetSpeedLoopState(void);

void FOC_SpeedOuterLoopStep(foc_motor_t *motor,
                            foc_pid_t *speed_pid,
                            float speed_ref_rad_s,
                            const sensor_data_t *sensor,
                            float dt_sec);

void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad,
                                 float angle_position_speed_rad_s,
                                 const sensor_data_t *sensor,
                                 float dt_sec);

#endif /* FOC_CONTROL_C21_OUTER_LOOP_H */
