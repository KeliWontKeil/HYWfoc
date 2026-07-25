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
}

#endif /* FOC_ESTIMATOR_ENCODER_ENABLE */
