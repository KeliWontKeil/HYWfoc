#ifndef FOC_CONTROL_C01_ENTRY_H
#define FOC_CONTROL_C01_ENTRY_H

#include <stdint.h>

#include "LS_Config/foc_shared_types.h"

uint8_t FOC_ControlOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *current_pid,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 const sensor_data_t *sensor,
                                 uint8_t control_mode,
                                 float speed_only_rad_s,
                                 float target_angle_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec);

void FOC_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed);

#endif /* FOC_CONTROL_C01_ENTRY_H */
