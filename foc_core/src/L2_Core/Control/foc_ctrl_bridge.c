#include "L2_Core/Control/foc_ctrl_bridge.h"
#include "L2_Core/Control/foc_ctrl_estim.h"

void FOC_Bridge_CopyInput(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->ctrl_input.source = motor->est_state.source;

    if (motor->est_state.valid == 0U)
    {
        motor->ctrl_input.valid = 0U;
        return;
    }

    motor->ctrl_input.valid = 1U;

    if (motor->est_state.source == FOC_ESTIMATOR_TYPE_BLEND)
    {
#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)
        float w = motor->transition_state.blend_factor;
        motor->ctrl_input.mech_angle_rad = (1.0f - w) * motor->est_state.mech_angle_rad
                                         + w * motor->est_state_alt.mech_angle_rad;
#else
        motor->ctrl_input.mech_angle_rad = motor->est_state.mech_angle_rad;
#endif
    }
    else
    {
        motor->ctrl_input.mech_angle_rad = motor->est_state.mech_angle_rad;
    }

    motor->ctrl_input.current_a = motor->sensor.current_a.output_value;
    motor->ctrl_input.current_b = motor->sensor.current_b.output_value;
    motor->ctrl_input.current_c = motor->sensor.current_c.output_value;
}