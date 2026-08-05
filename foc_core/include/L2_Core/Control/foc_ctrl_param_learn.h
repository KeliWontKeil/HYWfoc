#ifndef FOC_CTRL_PARAM_LEARN_H

#define FOC_CTRL_PARAM_LEARN_H


#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

/* 前置声明：避免依赖 L3 头间接声明 */
typedef struct foc_motor_t foc_motor_t;

uint8_t FOC_EstimateDirectionAndPolePairs(foc_motor_t *motor,
                                          int8_t *direction_est,
                                          uint8_t *pole_pairs_est);


#endif /* FOC_CONTROL_C23_MOTOR_PARAM_LEARN_H */
