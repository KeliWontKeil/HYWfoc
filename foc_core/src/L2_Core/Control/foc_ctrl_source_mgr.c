#include "L2_Core/Control/foc_ctrl_source_mgr.h"

#include <math.h>

#include "LS_Config/foc_config.h"

static void SourceMgr_UpdateEncoderServices(foc_motor_t *motor)
{
    uint8_t encoder_active;

    if (motor == 0) return;

    encoder_active = (motor->source_mgr_state.active_source == FOC_SOURCE_TYPE_ENCODER) ? 1U : 0U;
    motor->encoder_services.comp_available = 0U;
    motor->encoder_services.comp_active = 0U;
    motor->encoder_services.calib_available = encoder_active;
    motor->encoder_services.reinit_available = encoder_active;

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    if (encoder_active != 0U)
    {
        motor->encoder_services.comp_available = motor->cogging_comp_status.available;
        motor->encoder_services.comp_active = (uint8_t)((motor->cogging_comp_status.available != 0U) &&
                                                         (motor->cogging_comp_status.enabled != 0U));
    }
#endif
}

void FOC_SourceMgr_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source)
{
    if (motor == 0) return;

    motor->source_mgr_state.active_source = low_source;
    motor->source_mgr_state.standby_source = high_source;
    motor->source_mgr_state.switch_in_progress = 0U;
    motor->source_mgr_state.switch_counter = 0U;
    motor->source_mgr_state.control_region =
        ((high_source == FOC_SOURCE_TYPE_NONE) || (high_source == low_source)) ?
        FOC_CONTROL_REGION_FULL : FOC_CONTROL_REGION_LOW;

    motor->source_switch_state.active = 1U;
    motor->source_switch_state.low_source = low_source;
    motor->source_switch_state.high_source = high_source;
    motor->source_switch_state.current_source = low_source;
    motor->source_switch_state.speed_threshold_high_rad_s = FOC_SOURCE_SWITCH_SPEED_THRESH_HIGH_DEFAULT;
    motor->source_switch_state.speed_threshold_low_rad_s = FOC_SOURCE_SWITCH_SPEED_THRESH_LOW_DEFAULT;
    motor->source_switch_state.settle_counter = 0U;
    motor->source_switch_state.settle_target = 0U;

    motor->active_source_state.source = low_source;
    motor->active_source_state.state = FOC_SOURCE_STATE_INIT;
    motor->active_source_state.valid = 0U;
    motor->active_source_state.confidence = 0.0f;
    motor->active_source_state.elec_angle_rad = 0.0f;
    motor->active_source_state.elec_speed_rad_s = 0.0f;
    motor->active_source_state.mech_angle_rad = 0.0f;
    motor->active_source_state.mech_angle_accum_rad = 0.0f;
    motor->active_source_state.mech_speed_rad_s = 0.0f;

    SourceMgr_UpdateEncoderServices(motor);
}

/* ================================================================
 * Select：纯决策函数。直接从 motor 内部状态取值做切换判断。
 *   不发布 active_source_state，不写 electrical_phase_angle。
 * ================================================================ */
void FOC_SourceMgr_Select(foc_motor_t *motor)
{
    uint8_t low;
    uint8_t high;
    uint8_t active;
    float speed_abs;

    if (motor == 0) return;

    low = motor->source_switch_state.low_source;
    high = motor->source_switch_state.high_source;
    active = motor->source_mgr_state.active_source;

    if ((high == FOC_SOURCE_TYPE_NONE) || (high == low))
    {
        motor->source_mgr_state.control_region = FOC_CONTROL_REGION_FULL;
        motor->source_mgr_state.standby_source = high;
        motor->source_mgr_state.switch_in_progress = 0U;
        motor->source_mgr_state.switch_counter = 0U;
        return;
    }

    /* 从 active source 的内部状态取值 */
    speed_abs = 0.0f;
    switch (active)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        speed_abs = fabsf(motor->sensor.mech_angle_rad.output_value);
        break;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        if (motor->pole_pairs > 0U)
            speed_abs = fabsf(motor->estim_smo_state.pll_speed_rad_s / (float)motor->pole_pairs);
        break;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        if (motor->pole_pairs > 0U)
            speed_abs = fabsf(motor->openloop_angle_source_state.virtual_speed_rad_s / (float)motor->pole_pairs);
        break;
#endif
    default:
        break;
    }

    /* 收敛状态检查 */
    {
        uint8_t high_state = FOC_SOURCE_STATE_INIT;
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
        if (high == FOC_SOURCE_TYPE_SMO)
        {
            if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_LOCK_CONSECUTIVE)
                high_state = FOC_SOURCE_STATE_LOCKED;
            else if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
                high_state = FOC_SOURCE_STATE_CONVERGING;
            else if (motor->estim_smo_state.lock_counter > FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
                high_state = FOC_SOURCE_STATE_DIVERGED;
        }
#endif
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
        if (high == FOC_SOURCE_TYPE_ENCODER)
        {
            high_state = FOC_SOURCE_STATE_LOCKED;
        }
#endif

        /* 三分支切换判断 */
        {
            uint8_t div = FOC_SOURCE_TYPE_NONE;

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
            if ((active == FOC_SOURCE_TYPE_OPENLOOP) &&
                (high_state >= FOC_SOURCE_STATE_CONVERGING) &&
                (fabsf(motor->estim_smo_state.pll_speed_rad_s
                       - motor->openloop_angle_source_state.virtual_speed_rad_s)
                 < FOC_OPENLOOP_SWITCH_SPEED_THRESHOLD_RAD_S))
            {
                div = high;
            }
            else
#endif
            if ((active != high) &&
                (high_state >= FOC_SOURCE_STATE_CONVERGING) &&
                (speed_abs > motor->source_switch_state.speed_threshold_high_rad_s))
            {
                div = high;
            }
            else if ((active == high) &&
                     ((high_state == FOC_SOURCE_STATE_DIVERGED) ||
                      (speed_abs < motor->source_switch_state.speed_threshold_low_rad_s)))
            {
                div = low;
            }

            if (div == FOC_SOURCE_TYPE_NONE)
            {
                motor->source_mgr_state.switch_in_progress = 0U;
                motor->source_mgr_state.switch_counter = 0U;
                motor->source_switch_state.settle_counter = 0U;
                motor->source_switch_state.settle_target = 0U;
            }
            else
            {
                motor->source_mgr_state.switch_in_progress = 1U;
                motor->source_mgr_state.switch_counter++;
                motor->source_switch_state.settle_counter = motor->source_mgr_state.switch_counter;
                motor->source_switch_state.settle_target = (div == high) ? 1U : 2U;

                if (motor->source_mgr_state.switch_counter >= FOC_SOURCE_SWITCH_SETTLE_CYCLES)
                {
                    motor->source_mgr_state.active_source = div;
                    motor->source_mgr_state.standby_source = (div == high) ? low : high;
                    motor->source_mgr_state.control_region =
                        (div == high) ? FOC_CONTROL_REGION_HIGH : FOC_CONTROL_REGION_LOW;
                    motor->source_mgr_state.switch_in_progress = 0U;
                    motor->source_mgr_state.switch_counter = 0U;
                    motor->source_switch_state.current_source = div;
                    motor->source_switch_state.settle_counter = 0U;
                    motor->source_switch_state.settle_target = 0U;
                }
            }
        }
    }
}

/* ================================================================
 * Publish：纯发布函数。直接从 motor 内部状态读取数据，
 *   填入 active_source_state + 派生写 electrical_phase_angle。
 * ================================================================ */
void FOC_SourceMgr_Publish(foc_motor_t *motor)
{
    uint8_t src;

    if (motor == 0) return;

    src = motor->source_mgr_state.active_source;
    motor->active_source_state.source = src;

    switch (src)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
    {
        motor->active_source_state.state = FOC_SOURCE_STATE_LOCKED;
        motor->active_source_state.valid = 1U;
        motor->active_source_state.confidence = 1.0f;
        motor->active_source_state.mech_angle_rad = motor->sensor.mech_angle_rad.output_value;
        motor->active_source_state.elec_angle_rad = motor->sensor.mech_angle_rad.output_value * (float)motor->pole_pairs;
        motor->active_source_state.mech_speed_rad_s = 0.0f;
        motor->active_source_state.elec_speed_rad_s = 0.0f;
        break;
    }
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
    {
        motor->active_source_state.elec_angle_rad = motor->estim_smo_state.pll_angle_rad;
        motor->active_source_state.elec_speed_rad_s = motor->estim_smo_state.pll_speed_rad_s;
        if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_LOCK_CONSECUTIVE)
            motor->active_source_state.state = FOC_SOURCE_STATE_LOCKED;
        else if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
            motor->active_source_state.state = FOC_SOURCE_STATE_CONVERGING;
        else if (motor->estim_smo_state.lock_counter > FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
            motor->active_source_state.state = FOC_SOURCE_STATE_DIVERGED;
        else
            motor->active_source_state.state = FOC_SOURCE_STATE_INIT;
        motor->active_source_state.valid =
            (motor->active_source_state.state >= FOC_SOURCE_STATE_CONVERGING) ? 1U : 0U;
        motor->active_source_state.confidence = (motor->active_source_state.state == FOC_SOURCE_STATE_LOCKED) ? 0.9f : 0.0f;
        if (motor->pole_pairs > 0U)
        {
            motor->active_source_state.mech_angle_rad =
                motor->estim_smo_state.pll_angle_rad / (float)motor->pole_pairs;
            motor->active_source_state.mech_speed_rad_s =
                motor->estim_smo_state.pll_speed_rad_s / (float)motor->pole_pairs;
        }
        else
        {
            motor->active_source_state.mech_angle_rad = 0.0f;
            motor->active_source_state.mech_speed_rad_s = 0.0f;
        }
        break;
    }
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
    {
        motor->active_source_state.state = FOC_SOURCE_STATE_LOCKED;
        motor->active_source_state.valid = 1U;
        motor->active_source_state.confidence = 0.5f;
        motor->active_source_state.elec_angle_rad = motor->openloop_angle_source_state.virtual_angle_rad;
        motor->active_source_state.elec_speed_rad_s = motor->openloop_angle_source_state.virtual_speed_rad_s;
        if (motor->pole_pairs > 0U)
        {
            motor->active_source_state.mech_angle_rad =
                motor->openloop_angle_source_state.virtual_angle_rad / (float)motor->pole_pairs;
            motor->active_source_state.mech_speed_rad_s =
                motor->openloop_angle_source_state.virtual_speed_rad_s / (float)motor->pole_pairs;
        }
        break;
    }
#endif
    default:
        motor->active_source_state.state = FOC_SOURCE_STATE_INIT;
        motor->active_source_state.valid = 0U;
        motor->active_source_state.confidence = 0.0f;
        motor->active_source_state.elec_angle_rad = 0.0f;
        motor->active_source_state.elec_speed_rad_s = 0.0f;
        motor->active_source_state.mech_angle_rad = 0.0f;
        motor->active_source_state.mech_angle_accum_rad = 0.0f;
        motor->active_source_state.mech_speed_rad_s = 0.0f;
        break;
    }

    if (motor->active_source_state.valid != 0U)
    {
        motor->electrical_phase_angle = motor->active_source_state.elec_angle_rad;
    }

    SourceMgr_UpdateEncoderServices(motor);
}

const foc_active_source_state_t *FOC_SourceMgr_GetActive(const foc_motor_t *motor)
{
    return (motor == 0) ? 0 : &motor->active_source_state;
}
