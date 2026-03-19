#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include "math_transforms.h"

typedef struct {
    float electrical_angle;
    float ud;
    float uq;
    float set_voltage;
} foc_open_loop_input_t;

typedef struct {
    alpha_beta_frame_t alpha_beta_voltage;
    abc_frame_t phase_voltage;
} foc_open_loop_output_t;

void FOC_OpenLoopInit(void);
void FOC_OpenLoopUpdate(const foc_open_loop_input_t *input, foc_open_loop_output_t *output);

#endif /* _FOC_CONTROL_H_ */
