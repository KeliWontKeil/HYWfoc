#include "L2_Core/Control/foc_ctrl_openloop.h"

#include <math.h>

#include "L3_Hal/foc_math_types.h"
#include "LS_Config/foc_config.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)

static float OpenLoop_Wrap2Pi(float angle_rad)
{
    while (angle_rad > FOC_MATH_TWO_PI) angle_rad -= FOC_MATH_TWO_PI;
    while (angle_rad < 0.0f) angle_rad += FOC_MATH_TWO_PI;
    return angle_rad;
}

void FOC_OpenLoopLowSpeedPolicy_Init(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->openloop_low_speed_policy_state.phase = FOC_OPENLOOP_STATE_RUNNING;
    motor->openloop_low_speed_policy_state.current_ref_a = FOC_OPENLOOP_CURRENT_A;

    motor->openloop_angle_source_state.phase = FOC_OPENLOOP_STATE_RUNNING;
    motor->openloop_angle_source_state.virtual_angle_rad = 0.0f;
    motor->openloop_angle_source_state.virtual_speed_rad_s = 0.0f;
    motor->openloop_angle_source_state.ramp_rate_rad_s2 = FOC_OPENLOOP_RAMP_RATE_RAD_S2;
    motor->openloop_angle_source_state.target_speed_rad_s = FOC_OPENLOOP_TARGET_SPEED_RAD_S;
}

void FOC_OpenLoopLowSpeedPolicy_RunStep(foc_motor_t *motor, float dt_sec)
{
    float virtual_speed;
    float virtual_angle;

    if (motor == 0) return;
    if (motor->openloop_low_speed_policy_state.phase == FOC_OPENLOOP_STATE_FAILED) return;
    if ((motor->pole_pairs == 0U) || (dt_sec <= 0.0f)) return;

    if (motor->source_mgr_state.active_source == FOC_SOURCE_TYPE_OPENLOOP)
    {
        motor->openloop_low_speed_policy_state.phase = FOC_OPENLOOP_STATE_RUNNING;
        motor->iq_target = motor->openloop_low_speed_policy_state.current_ref_a;

        /* 更新虚拟角度 ramp */
        motor->openloop_angle_source_state.virtual_speed_rad_s +=
            motor->openloop_angle_source_state.ramp_rate_rad_s2 * dt_sec;
        virtual_speed = motor->openloop_angle_source_state.virtual_speed_rad_s;
        if (virtual_speed > motor->openloop_angle_source_state.target_speed_rad_s)
        {
            virtual_speed = motor->openloop_angle_source_state.target_speed_rad_s;
        }
        motor->openloop_angle_source_state.virtual_speed_rad_s = virtual_speed;

        virtual_angle = motor->openloop_angle_source_state.virtual_angle_rad + virtual_speed * dt_sec;
        virtual_angle = OpenLoop_Wrap2Pi(virtual_angle);
        motor->openloop_angle_source_state.virtual_angle_rad = virtual_angle;

        /* 写入 source_openloop_snapshot */
        motor->source_openloop_snapshot.source = FOC_SOURCE_TYPE_OPENLOOP;
        motor->source_openloop_snapshot.state = FOC_SOURCE_STATE_LOCKED;
        motor->source_openloop_snapshot.valid = 1U;
        motor->source_openloop_snapshot.confidence = 0.5f;
        motor->source_openloop_snapshot.elec_angle_rad = virtual_angle;
        motor->source_openloop_snapshot.elec_speed_rad_s = virtual_speed;
        motor->source_openloop_snapshot.mech_angle_rad = virtual_angle / (float)motor->pole_pairs;
        motor->source_openloop_snapshot.mech_angle_accum_rad = motor->source_openloop_snapshot.mech_angle_rad;
        motor->source_openloop_snapshot.mech_speed_rad_s = virtual_speed / (float)motor->pole_pairs;
    }
    else if (motor->openloop_low_speed_policy_state.phase == FOC_OPENLOOP_STATE_RUNNING)
    {
        motor->openloop_low_speed_policy_state.phase = FOC_OPENLOOP_STATE_DONE;
        motor->openloop_angle_source_state.phase = FOC_OPENLOOP_STATE_DONE;
    }
}

uint8_t FOC_OpenLoopLowSpeedPolicy_IsComplete(const foc_motor_t *motor)
{
    if (motor == 0) return 0U;
    return (motor->openloop_low_speed_policy_state.phase == FOC_OPENLOOP_STATE_DONE) ? 1U : 0U;
}

void FOC_OpenLoopLowSpeedPolicy_Abort(foc_motor_t *motor)
{
    if (motor == 0) return;
    motor->openloop_low_speed_policy_state.phase = FOC_OPENLOOP_STATE_FAILED;
    motor->openloop_angle_source_state.phase = FOC_OPENLOOP_STATE_FAILED;
}

#endif
