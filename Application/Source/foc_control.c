#include "foc_control.h"
#include <math.h>

void FOC_OpenLoopInit(void)
{
}

void FOC_OpenLoopUpdate(const foc_open_loop_input_t *input, foc_open_loop_output_t *output)
{
    dq_frame_t dq;
    float sin_theta;
    float cos_theta;

    if ((input == 0) || (output == 0))
    {
        return;
    }

    dq.d = input->ud;
    dq.q = input->uq;

    sin_theta = sinf(input->electrical_angle);
    cos_theta = cosf(input->electrical_angle);

    Math_InverseParkTransform(&dq, sin_theta, cos_theta, &output->alpha_beta_voltage);
    Math_InverseClarkeTransform(&output->alpha_beta_voltage, &output->phase_voltage);
}
