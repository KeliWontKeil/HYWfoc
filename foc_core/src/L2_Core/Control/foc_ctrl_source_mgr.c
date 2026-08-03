#include "L2_Core/Control/foc_ctrl_source_mgr.h"

#include <math.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L3_Hal/foc_math_transforms.h"
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

static uint8_t SourceMgr_GetSourceState(const foc_motor_t *motor, uint8_t source)
{
    if (motor == 0) return FOC_SOURCE_STATE_INIT;

    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        return FOC_SOURCE_STATE_LOCKED;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        if (motor->estim_smo_state.lock_counter > FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
            return FOC_SOURCE_STATE_DIVERGED;
        if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_LOCK_CONSECUTIVE)
            return FOC_SOURCE_STATE_LOCKED;
        if (motor->estim_smo_state.converge_counter > FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
            return FOC_SOURCE_STATE_CONVERGING;
        return FOC_SOURCE_STATE_INIT;
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_HFI:
        return FOC_SOURCE_STATE_LOCKED;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        return FOC_SOURCE_STATE_LOCKED;
#endif
    default:
        return FOC_SOURCE_STATE_INIT;
    }
}

static uint8_t SourceMgr_SourceCanLow(uint8_t source)
{
    return ((source == FOC_SOURCE_TYPE_ENCODER) ||
            (source == FOC_SOURCE_TYPE_HFI) ||
            (source == FOC_SOURCE_TYPE_OPENLOOP)) ? 1U : 0U;
}

static uint8_t SourceMgr_SourceCanHigh(uint8_t source)
{
    return ((source == FOC_SOURCE_TYPE_ENCODER) ||
            (source == FOC_SOURCE_TYPE_SMO) ||
            (source == FOC_SOURCE_TYPE_FLUX)) ? 1U : 0U;
}

static uint8_t SourceMgr_StateAcquireReady(uint8_t state)
{
    return ((state == FOC_SOURCE_STATE_CONVERGING) ||
            (state == FOC_SOURCE_STATE_LOCKED)) ? 1U : 0U;
}

static uint8_t SourceMgr_StateEntryReady(uint8_t state)
{
    return (state == FOC_SOURCE_STATE_LOCKED) ? 1U : 0U;
}

static uint8_t SourceMgr_StateHoldValid(uint8_t state)
{
    return ((state == FOC_SOURCE_STATE_CONVERGING) ||
            (state == FOC_SOURCE_STATE_LOCKED)) ? 1U : 0U;
}

static uint8_t SourceMgr_SourceValid(const foc_motor_t *motor, uint8_t source)
{
    if (motor == 0) return 0U;

#if ((FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE) || (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE))
    uint8_t state;
    state = SourceMgr_GetSourceState(motor, source);
#endif

    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        return motor->sensor.encoder_valid;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        return SourceMgr_StateAcquireReady(state);
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_HFI:
        return SourceMgr_StateEntryReady(state);
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        return (motor->openloop_state.phase != FOC_OPENLOOP_STATE_FAILED) ? 1U : 0U;
#endif
    default:
        return 0U;
    }
}

uint8_t FOC_SourceMgr_ReadSourceSpeed(const foc_motor_t *motor, uint8_t source, float *speed_out)
{
    if ((motor == 0) || (speed_out == 0)) return 0U;

    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        if (motor->sensor.encoder_valid == 0U) return 0U;
        *speed_out = motor->sensor.mech_speed_rad_s;
        return 1U;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        if (SourceMgr_StateAcquireReady(SourceMgr_GetSourceState(motor, FOC_SOURCE_TYPE_SMO)) == 0U) return 0U;
        *speed_out = motor->estim_smo_state.mech_speed_rad_s;
        return 1U;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        if (motor->openloop_state.phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
        *speed_out = motor->openloop_state.mech_speed_rad_s;
        return 1U;
#endif
    default:
        return 0U;
    }
}

static uint8_t SourceMgr_GetSwitchSpeedAbs(const foc_motor_t *motor, float *speed_abs)
{
    float speed;

    if ((motor == 0) || (speed_abs == 0)) return 0U;

    if (FOC_SourceMgr_ReadSourceSpeed(motor, motor->source_mgr_state.active_source, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }

    if (FOC_SourceMgr_ReadSourceSpeed(motor, motor->source_switch_state.high_source, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if (FOC_SourceMgr_ReadSourceSpeed(motor, FOC_SOURCE_TYPE_ENCODER, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }
#endif

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (FOC_SourceMgr_ReadSourceSpeed(motor, FOC_SOURCE_TYPE_SMO, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }
#endif

    *speed_abs = 0.0f;
    return 0U;
}

static uint8_t SourceMgr_LowMotionAbove(const foc_motor_t *motor, uint8_t low_source,
                                        float threshold_rad_s)
{
    float speed;

    if (motor == 0) return 0U;

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    if (low_source == FOC_SOURCE_TYPE_OPENLOOP)
    {
        if (motor->openloop_state.phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
        return (fabsf(motor->openloop_state.mech_speed_rad_s) > threshold_rad_s) ? 1U : 0U;
    }
#endif

    if (FOC_SourceMgr_ReadSourceSpeed(motor, low_source, &speed) != 0U)
    {
        return (fabsf(speed) > threshold_rad_s) ? 1U : 0U;
    }

    return (fabsf(motor->outer_loop.ramped_speed_rad_s) > threshold_rad_s) ? 1U : 0U;
}

static uint8_t SourceMgr_CandidateSpeedAbove(uint8_t speed_valid, float speed_abs,
                                             float threshold_rad_s)
{
    return ((speed_valid != 0U) && (speed_abs > threshold_rad_s * FOC_SOURCE_SWITCH_SPEED_SCALE)) ? 1U : 0U;
}

uint8_t FOC_SourceMgr_ReadSourceAngle(const foc_motor_t *motor, uint8_t source,
                                      float *mech_out, float *elec_out)
{
    float mech_local = 0.0f;
    float elec_local = 0.0f;

    if ((motor == 0) || ((mech_out == 0) && (elec_out == 0))) return 0U;

    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        if (motor->sensor.encoder_valid == 0U) return 0U;
        mech_local = motor->sensor.mech_angle_rad.output_value;
        elec_local = FOC_ControlMechanicalToElectricalAngle(motor, mech_local);
        break;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        if (SourceMgr_StateAcquireReady(SourceMgr_GetSourceState(motor, FOC_SOURCE_TYPE_SMO)) == 0U) return 0U;
        elec_local = motor->estim_smo_state.pll_angle_rad;
        if (motor->params.pole_pairs > 0U) mech_local = elec_local / (float)motor->params.pole_pairs;
        break;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        if (motor->openloop_state.phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
        elec_local = motor->openloop_state.virtual_angle_rad;
        if (motor->params.pole_pairs > 0U) mech_local = elec_local / (float)motor->params.pole_pairs;
        break;
#endif
    default:
        return 0U;
    }

    if (mech_out != 0) *mech_out = mech_local;
    if (elec_out != 0) *elec_out = elec_local;
    return 1U;
}

static uint8_t SourceMgr_AngleCompatible(const foc_motor_t *motor, uint8_t from_source, uint8_t to_source)
{
    float from_elec;
    float to_elec;

    if (FOC_SourceMgr_ReadSourceAngle(motor, from_source, 0, &from_elec) == 0U) return 0U;
    if (FOC_SourceMgr_ReadSourceAngle(motor, to_source, 0, &to_elec) == 0U) return 0U;

    return (fabsf(Math_WrapRadDelta(to_elec - from_elec)) < FOC_MATH_PI_BY_3) ? 1U : 0U;
}

static void SourceMgr_RebaseSource(foc_motor_t *motor, uint8_t new_source, uint8_t old_source)
{
    float old_elec;

    if (motor == 0) return;
    if (FOC_SourceMgr_ReadSourceAngle(motor, old_source, 0, &old_elec) == 0U) return;

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    float old_speed;

    if (new_source == FOC_SOURCE_TYPE_OPENLOOP)
    {
        motor->openloop_state.virtual_angle_rad =
            Math_WrapNearest(motor->openloop_state.virtual_angle_rad, old_elec);
        if ((motor->params.pole_pairs > 0U) &&
            (FOC_SourceMgr_ReadSourceSpeed(motor, old_source, &old_speed) != 0U))
        {
            motor->openloop_state.virtual_speed_rad_s = old_speed * (float)motor->params.pole_pairs;
            motor->openloop_state.mech_speed_rad_s = old_speed;
        }
        else
        {
            float fallback = fabsf(motor->outer_loop.ramped_speed_rad_s);
            motor->openloop_state.virtual_speed_rad_s = fallback * (float)motor->params.pole_pairs;
            motor->openloop_state.mech_speed_rad_s = fallback;
        }
    }
#endif

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (new_source == FOC_SOURCE_TYPE_SMO)
    {
        motor->estim_smo_state.pll_angle_rad =
            Math_WrapNearest(old_elec, motor->estim_smo_state.pll_angle_rad);
    }
#endif
}

static void SourceMgr_PrimePidOutput(foc_pid_t *pid, float target_output, float error)
{
    float integral;

    if (pid == 0) return;

    pid->prev_error = error;

    if (fabsf(pid->ki) <= 1e-6f)
    {
        pid->integral = 0.0f;
        return;
    }

    target_output = Math_ClampFloat(target_output, pid->out_min, pid->out_max);
    integral = (target_output - pid->kp * error) / pid->ki;

    if (pid->ki > 0.0f)
    {
        pid->integral = Math_ClampFloat(integral,
                                        pid->out_min / pid->ki,
                                        pid->out_max / pid->ki);
    }
    else
    {
        pid->integral = Math_ClampFloat(integral,
                                        pid->out_max / pid->ki,
                                        pid->out_min / pid->ki);
    }
}

static void SourceMgr_SyncOuterLoopOnSwitch(foc_motor_t *motor, uint8_t new_source,
                                            uint8_t old_source)
{
    float mech_angle = 0.0f;
    float elec_angle = 0.0f;

    if (motor == 0) return;

    if (FOC_SourceMgr_ReadSourceAngle(motor, new_source, &mech_angle, &elec_angle) == 0U)
    {
        if (FOC_SourceMgr_ReadSourceAngle(motor, old_source, &mech_angle, &elec_angle) == 0U)
        {
            mech_angle = motor->active_source_state.mech_angle_rad;
        }
    }

    motor->outer_loop.accum_rad = mech_angle;
    motor->outer_loop.prev_rad = mech_angle;
    motor->outer_loop.prev_valid = 1U;
    motor->outer_loop.prev_mech_signed_rad = (float)motor->params.direction * mech_angle;
    motor->outer_loop.speed_err_accum_rad = 0.0f;
    motor->outer_loop.speed_state_valid = 1U;

    SourceMgr_PrimePidOutput(&motor->speed_pid, motor->ctrl.iq_target, 0.0f);
}

static void SourceMgr_SyncCurrentLoopOnSwitch(foc_motor_t *motor)
{
    float iq_error;

    if (motor == 0) return;

    iq_error = motor->ctrl.iq_target - motor->ctrl.iq_measured;
    SourceMgr_PrimePidOutput(&motor->torque_current_pid, motor->ctrl.uq, iq_error);

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    if (motor->current_soft_switch_status.blend_factor > 0.5f)
    {
        motor->current_soft_switch_status.blend_factor = 0.5f;
    }
    motor->current_soft_switch_status.blend_initialized = 1U;
#endif
}

static void SourceMgr_CommitSwitch(foc_motor_t *motor, uint8_t new_source, uint8_t new_region)
{
    uint8_t old_source;

    if (motor == 0) return;

    old_source = motor->source_mgr_state.active_source;
    SourceMgr_RebaseSource(motor, new_source, old_source);

    motor->source_mgr_state.active_source = new_source;
    motor->source_mgr_state.standby_source =
        (new_source == motor->source_switch_state.high_source) ?
        motor->source_switch_state.low_source : motor->source_switch_state.high_source;
    motor->source_mgr_state.control_region = new_region;

    SourceMgr_SyncOuterLoopOnSwitch(motor, new_source, old_source);
    SourceMgr_SyncCurrentLoopOnSwitch(motor);

    motor->source_mgr_state.switch_in_progress = 0U;
    motor->source_mgr_state.switch_counter = 0U;
    motor->source_mgr_state.degrade_hold_counter = 0U;
}

static void SourceMgr_SetFixedSource(foc_motor_t *motor, uint8_t source)
{
    if (motor == 0) return;

    motor->source_mgr_state.active_source = source;
    motor->source_mgr_state.standby_source = FOC_SOURCE_TYPE_NONE;
    motor->source_mgr_state.control_region = FOC_CONTROL_REGION_FULL;
    motor->source_mgr_state.region_state = FOC_REGION_STATE_FULL_ACTIVE;
    motor->source_mgr_state.switch_in_progress = 0U;
    motor->source_mgr_state.switch_counter = 0U;
    motor->source_mgr_state.degrade_hold_counter = 0U;
}

/* 速域切换仅服务速度控制模式（角度模式依赖编码器可靠源，不作源切换） */
static uint8_t SourceMgr_SpeedModeAllows(const foc_motor_t *motor)
{
    return (motor->state.control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY) ? 1U : 0U;
}

/* 目标速度位于高速域（> 高速切换门限）才允许驻留/恢复高速域 */
static uint8_t SourceMgr_TargetInHighRegion(const foc_motor_t *motor)
{
    return ((SourceMgr_SpeedModeAllows(motor) != 0U) &&
            (fabsf(motor->cfg.speed_only_rad_s) > motor->source_switch_state.speed_threshold_high_rad_s)) ? 1U : 0U;
}

void FOC_SourceMgr_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source)
{
    uint8_t switchable;

    if (motor == 0) return;

    switchable = ((high_source != FOC_SOURCE_TYPE_NONE) &&
                  (high_source != low_source) &&
                  (SourceMgr_SourceCanLow(low_source) != 0U) &&
                  (SourceMgr_SourceCanHigh(high_source) != 0U)) ? 1U : 0U;

    motor->source_mgr_state.active_source = low_source;
    motor->source_mgr_state.standby_source = switchable ? high_source : FOC_SOURCE_TYPE_NONE;
    motor->source_mgr_state.switch_in_progress = 0U;
    motor->source_mgr_state.switch_counter = 0U;
    motor->source_mgr_state.config_valid = switchable;
    motor->source_mgr_state.control_region =
        (switchable != 0U) ? FOC_CONTROL_REGION_LOW : FOC_CONTROL_REGION_FULL;
    motor->source_mgr_state.region_state =
        (switchable != 0U) ? FOC_REGION_STATE_LOW_ACTIVE : FOC_REGION_STATE_FULL_ACTIVE;

    motor->source_switch_state.low_source = low_source;
    motor->source_switch_state.high_source = high_source;
    motor->source_switch_state.speed_threshold_high_rad_s = FOC_SOURCE_SWITCH_SPEED_THRESH_HIGH_DEFAULT;
    motor->source_switch_state.speed_threshold_low_rad_s = FOC_SOURCE_SWITCH_SPEED_THRESH_LOW_DEFAULT;

    motor->active_source_state.source = low_source;
    motor->active_source_state.state = FOC_SOURCE_STATE_INIT;
    motor->active_source_state.valid = 0U;
    motor->active_source_state.confidence = 0.0f;
    motor->active_source_state.elec_angle_rad = 0.0f;
    motor->active_source_state.mech_angle_rad = 0.0f;

    motor->source_mgr_state.degrade_hold_counter = 0U;

    SourceMgr_UpdateEncoderServices(motor);
}

void FOC_SourceMgr_Select(foc_motor_t *motor)
{
    uint8_t low;
    uint8_t high;
    uint8_t high_state;
    float speed_abs;
    uint8_t speed_valid;

    if (motor == 0) return;

    low = motor->source_switch_state.low_source;
    high = motor->source_switch_state.high_source;

    if (motor->source_mgr_state.config_valid == 0U)
    {
        SourceMgr_SetFixedSource(motor, low);
        return;
    }

    high_state = SourceMgr_GetSourceState(motor, high);
    speed_valid = SourceMgr_GetSwitchSpeedAbs(motor, &speed_abs);

    /* 非速度控制模式：锁定低速源，不做速域切换（角度模式依赖编码器可靠源） */
    if (SourceMgr_SpeedModeAllows(motor) == 0U)
    {
        motor->source_mgr_state.active_source = low;
        motor->source_mgr_state.standby_source = FOC_SOURCE_TYPE_NONE;
        motor->source_mgr_state.control_region = FOC_CONTROL_REGION_LOW;
        motor->source_mgr_state.region_state = FOC_REGION_STATE_LOW_ACTIVE;
        motor->source_mgr_state.switch_in_progress = 0U;
        motor->source_mgr_state.switch_counter = 0U;
        motor->source_mgr_state.degrade_hold_counter = 0U;
        return;
    }

    switch (motor->source_mgr_state.region_state)
    {
    case FOC_REGION_STATE_LOW_ACTIVE:
        motor->source_mgr_state.active_source = low;
        motor->source_mgr_state.control_region = FOC_CONTROL_REGION_LOW;
        if ((SourceMgr_TargetInHighRegion(motor) != 0U) &&
            (SourceMgr_LowMotionAbove(motor, low,
                motor->source_switch_state.speed_threshold_high_rad_s) != 0U) &&
            (SourceMgr_CandidateSpeedAbove(speed_valid, speed_abs,
                motor->source_switch_state.speed_threshold_high_rad_s) != 0U) &&
            (SourceMgr_StateAcquireReady(high_state) != 0U) &&
            (SourceMgr_SourceValid(motor, high) != 0U))
        {
            motor->source_mgr_state.region_state = FOC_REGION_STATE_HIGH_ACQUIRE;
            motor->source_mgr_state.switch_in_progress = 1U;
            motor->source_mgr_state.switch_counter = 1U;
        }
        else
        {
            motor->source_mgr_state.switch_counter = 0U;
        }
        break;

    case FOC_REGION_STATE_HIGH_ACQUIRE:
        if ((SourceMgr_TargetInHighRegion(motor) == 0U) ||
            (SourceMgr_LowMotionAbove(motor, low,
                motor->source_switch_state.speed_threshold_low_rad_s) == 0U) ||
            (SourceMgr_CandidateSpeedAbove(speed_valid, speed_abs,
                motor->source_switch_state.speed_threshold_low_rad_s) == 0U) ||
            (SourceMgr_StateAcquireReady(high_state) == 0U) ||
            (SourceMgr_SourceValid(motor, high) == 0U))
        {
            motor->source_mgr_state.region_state = FOC_REGION_STATE_LOW_ACTIVE;
            motor->source_mgr_state.switch_in_progress = 0U;
            motor->source_mgr_state.switch_counter = 0U;
            break;
        }

        motor->source_mgr_state.switch_counter++;
        if (motor->source_mgr_state.switch_counter >= FOC_SOURCE_SWITCH_SETTLE_CYCLES)
        {
            if ((SourceMgr_StateEntryReady(high_state) != 0U) &&
                ((motor->source_mgr_state.active_source == FOC_SOURCE_TYPE_OPENLOOP) ||
                 (SourceMgr_AngleCompatible(motor, motor->source_mgr_state.active_source, high) != 0U)))
            {
                SourceMgr_CommitSwitch(motor, high, FOC_CONTROL_REGION_HIGH);
                motor->source_mgr_state.region_state = FOC_REGION_STATE_HIGH_ACTIVE;
            }
        }
        break;

    case FOC_REGION_STATE_HIGH_ACTIVE:
        motor->source_mgr_state.active_source = high;
        motor->source_mgr_state.control_region = FOC_CONTROL_REGION_HIGH;
        /* 目标不在高速域、SMO 非收敛/发散/无效、或实测速度低于门限 → 触发降级 */
        if ((SourceMgr_TargetInHighRegion(motor) == 0U) ||
            (high_state == FOC_SOURCE_STATE_DIVERGED) ||
            (SourceMgr_StateHoldValid(high_state) == 0U) ||
            (SourceMgr_SourceValid(motor, high) == 0U) ||
            ((speed_valid != 0U) &&
             (speed_abs < motor->source_switch_state.speed_threshold_low_rad_s)))
        {
            motor->source_mgr_state.region_state = FOC_REGION_STATE_HIGH_SUSPECT;
            motor->source_mgr_state.switch_in_progress = 1U;
            motor->source_mgr_state.switch_counter = 1U;
            motor->source_mgr_state.degrade_hold_counter = 0U;
        }
        else
        {
            motor->source_mgr_state.switch_in_progress = 0U;
            motor->source_mgr_state.switch_counter = 0U;
        }
        break;

    case FOC_REGION_STATE_HIGH_SUSPECT:
        /*
         * 降级恢复要求：目标在高速域 且 实测速度 ≥ high 门限 连续 DEGRADE_CONFIRM_CYCLES 拍，
         * 单拍尖峰或速度读取失败不能打断敏捷降级。
         */
        if ((SourceMgr_TargetInHighRegion(motor) != 0U) &&
            (high_state != FOC_SOURCE_STATE_DIVERGED) &&
            (SourceMgr_StateHoldValid(high_state) != 0U) &&
            (SourceMgr_SourceValid(motor, high) != 0U) &&
            (speed_valid != 0U) &&
            (speed_abs >= motor->source_switch_state.speed_threshold_high_rad_s))
        {
            motor->source_mgr_state.degrade_hold_counter++;
            if (motor->source_mgr_state.degrade_hold_counter >= FOC_SOURCE_SWITCH_DEGRADE_CONFIRM_CYCLES)
            {
                motor->source_mgr_state.region_state = FOC_REGION_STATE_HIGH_ACTIVE;
                motor->source_mgr_state.switch_in_progress = 0U;
                motor->source_mgr_state.switch_counter = 0U;
                motor->source_mgr_state.degrade_hold_counter = 0U;
            }
            break;
        }

        motor->source_mgr_state.degrade_hold_counter = 0U;
        motor->source_mgr_state.switch_counter++;
        if ((high_state == FOC_SOURCE_STATE_DIVERGED) ||
            (motor->source_mgr_state.switch_counter >= FOC_SOURCE_SWITCH_DEGRADE_CONFIRM_CYCLES))
        {
            motor->source_mgr_state.region_state = FOC_REGION_STATE_LOW_RECOVERY;
            motor->source_mgr_state.switch_in_progress = 0U;
        }
        break;

    case FOC_REGION_STATE_LOW_RECOVERY:
        if ((SourceMgr_SourceValid(motor, low) != 0U) || (low == FOC_SOURCE_TYPE_OPENLOOP))
        {
            SourceMgr_CommitSwitch(motor, low, FOC_CONTROL_REGION_LOW);
            motor->source_mgr_state.region_state = FOC_REGION_STATE_LOW_ACTIVE;
        }
        else
        {
            motor->source_mgr_state.switch_counter++;
        }
        break;

    case FOC_REGION_STATE_FULL_ACTIVE:
    default:
        SourceMgr_SetFixedSource(motor, low);
        break;
    }
}

void FOC_SourceMgr_Publish(foc_motor_t *motor)
{
    uint8_t src;
    uint8_t valid;
    float mech_angle;
    float elec_angle;

    if (motor == 0) return;

    src = motor->source_mgr_state.active_source;
    mech_angle = 0.0f;
    elec_angle = 0.0f;
    valid = FOC_SourceMgr_ReadSourceAngle(motor, src, &mech_angle, &elec_angle);

    motor->active_source_state.source = src;
    motor->active_source_state.state = SourceMgr_GetSourceState(motor, src);
    motor->active_source_state.valid = valid;
    motor->active_source_state.confidence =
        (src == FOC_SOURCE_TYPE_ENCODER) ? 1.0f :
        ((src == FOC_SOURCE_TYPE_OPENLOOP) ? 0.5f :
         ((motor->active_source_state.state == FOC_SOURCE_STATE_LOCKED) ? 0.9f : 0.4f));
    motor->active_source_state.mech_angle_rad = mech_angle;
    motor->active_source_state.elec_angle_rad = elec_angle;

    if (valid != 0U)
    {
        motor->ctrl.electrical_angle_rad = elec_angle;
    }

    SourceMgr_UpdateEncoderServices(motor);
}

const foc_active_source_state_t *FOC_SourceMgr_GetActive(const foc_motor_t *motor)
{
    return (motor == 0) ? 0 : &motor->active_source_state;
}
