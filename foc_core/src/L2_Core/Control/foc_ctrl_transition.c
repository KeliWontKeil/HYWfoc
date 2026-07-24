#include "L2_Core/Control/foc_ctrl_transition.h"

#include <math.h>

#include "L2_Core/Control/foc_ctrl_estim.h"
#include "LS_Config/foc_config.h"

#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)

void FOC_Transition_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source)
{
    if (motor == 0) return;

    motor->transition_state.active        = 1U;
    motor->transition_state.low_source    = low_source;
    motor->transition_state.high_source   = high_source;
    motor->transition_state.current_source = low_source;
    motor->transition_state.speed_threshold_high_rad_s = FOC_TRANSITION_SPEED_THRESH_HIGH_DEFAULT;
    motor->transition_state.speed_threshold_low_rad_s  = FOC_TRANSITION_SPEED_THRESH_LOW_DEFAULT;
    motor->transition_state.settle_counter = 0U;
    motor->transition_state.settle_target  = 0U;
}

void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec)
{
    float mech_speed;
    uint8_t high_ready;
    uint8_t high_lost;
    uint8_t low_to_high;
    uint8_t high_to_low;

    if (motor == 0) return;
    (void)dt_sec;
    if (motor->transition_state.active == 0U) return;

    mech_speed = fabsf(motor->est_state.mech_speed_rad_s);
    high_ready = (motor->est_state.state >= FOC_ESTIMATOR_STATE_CONVERGING) ? 1U : 0U;
    high_lost  = (motor->est_state.state == FOC_ESTIMATOR_STATE_DIVERGED) ? 1U : 0U;

    low_to_high = (uint8_t)(high_ready && (mech_speed > motor->transition_state.speed_threshold_high_rad_s));
    high_to_low = (uint8_t)(high_lost || (mech_speed < motor->transition_state.speed_threshold_low_rad_s));

    if (low_to_high && (motor->transition_state.current_source != motor->transition_state.high_source))
    {
        motor->transition_state.settle_target = 1U;
    }
    else if (high_to_low && (motor->transition_state.current_source != motor->transition_state.low_source))
    {
        motor->transition_state.settle_target = 2U;
    }
    else
    {
        motor->transition_state.settle_counter = 0U;
        motor->transition_state.settle_target  = 0U;
    }

    if (motor->transition_state.settle_target == 1U)
    {
        motor->transition_state.settle_counter++;
        if (motor->transition_state.settle_counter >= FOC_TRANSITION_SETTLE_CYCLES)
        {
            /* 角色互换：切换为副估计器（后台预收敛的 high_source） */
            foc_estimator_step_t tmp_fn = motor->estimator_step_fn;
            motor->estimator_step_fn = motor->estimator_step_fn_alt;
            motor->estimator_step_fn_alt = tmp_fn;

            foc_est_state_t tmp_st = motor->est_state;
            motor->est_state = motor->est_state_alt;
            motor->est_state_alt = tmp_st;

            motor->est_state.source = motor->transition_state.high_source;
            motor->transition_state.current_source  = motor->transition_state.high_source;
            motor->transition_state.settle_counter = 0U;
            motor->transition_state.settle_target  = 0U;
        }
    }

    if (motor->transition_state.settle_target == 2U)
    {
        motor->transition_state.settle_counter++;
        if (motor->transition_state.settle_counter >= FOC_TRANSITION_SETTLE_CYCLES)
        {
            /* 角色互换：切换为副估计器（后台预收敛的 low_source） */
            foc_estimator_step_t tmp_fn = motor->estimator_step_fn;
            motor->estimator_step_fn = motor->estimator_step_fn_alt;
            motor->estimator_step_fn_alt = tmp_fn;

            foc_est_state_t tmp_st = motor->est_state;
            motor->est_state = motor->est_state_alt;
            motor->est_state_alt = tmp_st;

            motor->est_state.source = motor->transition_state.low_source;
            motor->transition_state.current_source  = motor->transition_state.low_source;
            motor->transition_state.settle_counter = 0U;
            motor->transition_state.settle_target  = 0U;
        }
    }
}

#else

void FOC_Transition_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source)
{
    (void)motor;
    (void)low_source;
    (void)high_source;
}

void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec)
{
    (void)motor;
    (void)dt_sec;
}

#endif
