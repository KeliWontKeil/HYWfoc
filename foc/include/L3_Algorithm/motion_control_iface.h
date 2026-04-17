#ifndef _MOTION_CONTROL_IFACE_H_
#define _MOTION_CONTROL_IFACE_H_

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
uint8_t FOC_ControlRequiresCurrentSample(void);
void FOC_CurrentControlStep(foc_motor_t *motor,
                            foc_pid_t *current_pid,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec);
void FOC_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed);

#endif /* _MOTION_CONTROL_IFACE_H_ */
