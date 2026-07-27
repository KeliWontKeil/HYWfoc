#include "L2_Core/Control/foc_ctrl_estim.h"

#include "LS_Config/foc_config.h"

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)

void FOC_EstimHFI_Init(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->estim_hfi_state.hf_sin_demod = 0.0f;
    motor->estim_hfi_state.hf_cos_demod = 0.0f;
}

void FOC_EstimHFI_Step(foc_motor_t *motor, float dt_sec)
{
    (void)dt_sec;

    if (motor == 0) return;

    motor->source_hfi_snapshot.source = FOC_SOURCE_TYPE_HFI;
    motor->source_hfi_snapshot.state = FOC_SOURCE_STATE_INIT;
    motor->source_hfi_snapshot.valid = 0U;
    motor->source_hfi_snapshot.confidence = 0.0f;
    motor->source_hfi_snapshot.elec_angle_rad = 0.0f;
    motor->source_hfi_snapshot.elec_speed_rad_s = 0.0f;
    motor->source_hfi_snapshot.mech_angle_rad = 0.0f;
    motor->source_hfi_snapshot.mech_angle_accum_rad = 0.0f;
    motor->source_hfi_snapshot.mech_speed_rad_s = 0.0f;
}

#endif /* FOC_ESTIMATOR_HFI_ENABLE */
