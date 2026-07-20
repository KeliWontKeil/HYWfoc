#include "L2_Core/Control/foc_ctrl_estim.h"

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)

void FOC_EstimHFI_Init(foc_motor_t *motor)
{
    if (motor == 0) return;
}

void FOC_EstimHFI_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec)
{
    (void)motor;
    (void)out;
    (void)dt_sec;
}

#endif