#include "L2_Core/Control/foc_ctrl_estim.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_math_types.h"

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)

void FOC_EstimEncoder_Init(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->estim_encoder_state.lpf_valid = 0U;
    motor->estim_encoder_state.lpf_state = 0.0f;

    motor->estim_encoder_state.mech_angle_kalman.filtered_value = 0.0f;
    motor->estim_encoder_state.mech_angle_kalman.raw_value = 0.0f;
    motor->estim_encoder_state.mech_angle_kalman.output_value = 0.0f;

    motor->estimator_step_fn_alt = 0;
}

void FOC_EstimEncoder_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec)
{
    float angle_rad;
    (void)dt_sec;

    if ((motor == 0) || (out == 0)) return;

    angle_rad = motor->sensor.mech_angle_rad.output_value;

    out->mech_angle_rad = angle_rad;
    out->elec_angle_rad  = angle_rad * (float)motor->pole_pairs;
    out->valid = 1U;
    out->state = FOC_ESTIMATOR_STATE_LOCKED;
    out->confidence = 1.0f;
}

#endif /* FOC_ESTIMATOR_ENCODER_ENABLE */