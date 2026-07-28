#include "L2_Core/Control/foc_ctrl_transition.h"

#include <math.h>

#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_math_types.h"
#include "LS_Config/foc_config.h"

static void Transition_ResetPID(foc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void FOC_Transition_OnModeSwitch(foc_motor_t *motor, uint8_t new_mode, uint8_t old_mode)
{
    if (motor == 0) return;

    motor->outer_loop.accum_rad = motor->active_source_state.mech_angle_rad;
    motor->outer_loop.prev_rad = motor->active_source_state.mech_angle_rad;
    motor->outer_loop.prev_valid = 1U;

    motor->outer_loop.ramped_speed_rad_s = 0.0f;

    motor->outer_loop.speed_state_valid = 0U;
    motor->outer_loop.speed_err_accum_rad = 0.0f;

    Transition_ResetPID(&motor->speed_pid);
    Transition_ResetPID(&motor->angle_pid);

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->current_soft_switch_status.enabled = 0U;
    motor->current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    motor->current_soft_switch_status.blend_initialized = 0U;
#endif

    (void)old_mode;
}
