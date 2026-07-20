#include "L2_Core/Control/foc_ctrl_estim.h"

void FOC_Estimator_Select(foc_motor_t *motor, uint8_t estimator_type)
{
    if (motor == 0) return;

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if (estimator_type == FOC_ESTIMATOR_TYPE_ENCODER)
    {
        motor->estimator_step_fn = FOC_EstimEncoder_Step;
        motor->est_state.source = FOC_ESTIMATOR_TYPE_ENCODER;
        return;
    }
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (estimator_type == FOC_ESTIMATOR_TYPE_SMO)
    {
        motor->estimator_step_fn = FOC_EstimSMO_Step;
        motor->est_state.source = FOC_ESTIMATOR_TYPE_SMO;
        return;
    }
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    if (estimator_type == FOC_ESTIMATOR_TYPE_HFI)
    {
        motor->estimator_step_fn = FOC_EstimHFI_Step;
        motor->est_state.source = FOC_ESTIMATOR_TYPE_HFI;
        return;
    }
#endif
    motor->estimator_step_fn = 0;
    motor->est_state.source = FOC_ESTIMATOR_TYPE_NONE;
}