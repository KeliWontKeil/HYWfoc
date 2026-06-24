#ifndef FOC_CTRL_PARAM_LEARN_H

#define FOC_CTRL_PARAM_LEARN_H


#include <stdint.h>

#include "L2/foc_ctrl_types.h"

uint8_t FOC_SampleLockedMechanicalAngle(foc_motor_t *motor,
                                        float electrical_angle,
                                        uint16_t settle_ms,
                                        uint16_t sample_count,
                                        float *mech_angle_rad);

uint8_t FOC_EstimateDirectionAndPolePairs(foc_motor_t *motor,
                                          int8_t *direction_est,
                                          uint8_t *pole_pairs_est);


#endif /* FOC_CONTROL_C23_MOTOR_PARAM_LEARN_H */
