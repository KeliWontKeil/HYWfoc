#include "L2_Core/Control/foc_ctrl_estim.h"

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)

void FOC_EstimSMO_Init(foc_motor_t *motor)
{
    if (motor == 0) return;
}

void FOC_EstimSMO_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec)
{
    (void)motor;
    (void)out;
    (void)dt_sec;
}

#endif