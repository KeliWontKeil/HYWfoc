#ifndef FOC_CTRL_CURRENT_LOOP_H

#define FOC_CTRL_CURRENT_LOOP_H


#include <stdint.h>

#include "LS_Config/foc_math_types.h"
#include "LS_Config/foc_motor_types.h"

void FOC_CurrentControlStep(foc_motor_t *motor,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec);

void FOC_CurrentControlOpenLoopStep(foc_motor_t *motor,
                                    float voltage,
                                    float turn_speed,
                                    float dt_sec);

void FOC_CurrentControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle);

uint8_t FOC_ControlRequiresCurrentSample(void);

#endif /* FOC_CTRL_CURRENT_LOOP_H */
