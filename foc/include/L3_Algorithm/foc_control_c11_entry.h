#ifndef FOC_CONTROL_C11_ENTRY_H
#define FOC_CONTROL_C11_ENTRY_H

#include <stdint.h>

#include "LS_Config/foc_math_types.h"
#include "LS_Config/foc_motor_types.h"

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

uint8_t FOC_ControlCurrentLoopRequiresSample(void);
void FOC_ControlCurrentLoopStep(foc_motor_t *motor,
                                foc_pid_t *current_pid,
                                const sensor_data_t *sensor,
                                float electrical_angle,
                                float dt_sec);
void FOC_ControlCompensationStep(foc_motor_t *motor, const sensor_data_t *sensor);
void FOC_ControlOpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed);

#endif /* FOC_CONTROL_C11_ENTRY_H */
