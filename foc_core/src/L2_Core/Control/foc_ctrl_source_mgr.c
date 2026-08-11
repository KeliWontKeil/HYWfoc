#include "L2_Core/Control/foc_ctrl_source_mgr.h"

#include <math.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

static void SourceMgr_UpdateEncoderServices(foc_source_mgr_ctx_t *ctx)
{
    uint8_t encoder_active;

    encoder_active = (ctx->state->active_source == FOC_SOURCE_TYPE_ENCODER) ? 1U : 0U;
    ctx->encoder_services->comp_available = 0U;
    ctx->encoder_services->comp_active = 0U;
    ctx->encoder_services->calib_available = encoder_active;
    ctx->encoder_services->reinit_available = encoder_active;

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    if (encoder_active != 0U)
    {
        ctx->encoder_services->comp_available = ctx->cogging_status->available;
        ctx->encoder_services->comp_active = (uint8_t)((ctx->cogging_status->available != 0U) &&
                                                         (ctx->cogging_status->enabled != 0U));
    }
#endif
}

static uint8_t SourceMgr_GetSourceState(const foc_source_read_ctx_t *rctx, uint8_t source)
{
    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        return FOC_SOURCE_STATE_LOCKED;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        if (rctx->smo_state->lock_counter > FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
            return FOC_SOURCE_STATE_DIVERGED;
        if (rctx->smo_state->converge_counter > FOC_ESTIM_SMO_LOCK_CONSECUTIVE)
            return FOC_SOURCE_STATE_LOCKED;
        if (rctx->smo_state->converge_counter > FOC_ESTIM_SMO_CONVERGE_CONSECUTIVE)
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

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)

/* 编码器采样位置决定其读取源：
 * - ANGLE_FAST == DISABLE：编码器由控制 ISR 采样，经 ctrl_ref 原子发布 → 读 ref。
 * - ANGLE_FAST == ENABLE ：编码器由电流环 ISR 采样，自持于 sensor → 读 sensor。 */
static uint8_t SourceMgr_EncoderValid(const foc_source_read_ctx_t *ctx)
{
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    return ctx->ref->encoder_valid;
#else
    return ctx->sensor->encoder_valid;
#endif
}

static float SourceMgr_EncoderAngleRad(const foc_source_read_ctx_t *ctx)
{
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    return ctx->ref->mech_angle_rad;
#else
    return ctx->sensor->mech_angle_rad.output_value;
#endif
}

static float SourceMgr_EncoderSpeed(const foc_source_read_ctx_t *ctx)
{
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    return ctx->ref->mech_speed_rad_s;
#else
    return ctx->sensor->mech_speed_rad_s;
#endif
}

#endif

static uint8_t SourceMgr_SourceValid(const foc_source_read_ctx_t *rctx, uint8_t source)
{
    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        return SourceMgr_EncoderValid(rctx);
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        return SourceMgr_StateAcquireReady(SourceMgr_GetSourceState(rctx, source));
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_HFI:
        return SourceMgr_StateEntryReady(SourceMgr_GetSourceState(rctx, source));
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        return (rctx->ref->openloop_phase != FOC_OPENLOOP_STATE_FAILED) ? 1U : 0U;
#endif
    default:
        return 0U;
    }
}

uint8_t FOC_SourceMgr_ReadSourceSpeed(const foc_source_read_ctx_t *ctx, uint8_t source, float *speed_out)
{
    if ((ctx == 0) || (speed_out == 0)) return 0U;

    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        if (SourceMgr_EncoderValid(ctx) == 0U) return 0U;
        *speed_out = SourceMgr_EncoderSpeed(ctx);   /* 物理坐标系 */
        return 1U;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_SMO:
        if (SourceMgr_StateAcquireReady(SourceMgr_GetSourceState(ctx, FOC_SOURCE_TYPE_SMO)) == 0U) return 0U;
        *speed_out = ctx->smo_state->mech_speed_rad_s * (float)ctx->params->direction;   /* 物理坐标系（与 ENCODER/OPENLOOP 一致） */
        return 1U;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        if (ctx->ref->openloop_phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
        *speed_out = ctx->ref->openloop_mech_speed_rad_s * (float)ctx->params->direction;   /* 控制→物理 */
        return 1U;
#endif
    default:
        return 0U;
    }
}

static foc_source_read_ctx_t SourceMgr_ReadCtx(const foc_source_mgr_ctx_t *ctx)
{
    foc_source_read_ctx_t rctx;

    rctx.sensor = ctx->sensor;
    rctx.params = ctx->params;
    rctx.ctrl = ctx->ctrl;
    rctx.ref = ctx->ref;
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    rctx.smo_state = ctx->smo_state;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    rctx.openloop_state = ctx->openloop_state;
#endif
    return rctx;
}

static uint8_t SourceMgr_GetSwitchSpeedAbs(const foc_source_mgr_ctx_t *ctx, float *speed_abs)
{
    float speed;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    if (speed_abs == 0) return 0U;

    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, ctx->state->active_source, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }

    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, ctx->switch_cfg->high_source, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, FOC_SOURCE_TYPE_ENCODER, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }
#endif

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, FOC_SOURCE_TYPE_SMO, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }
#endif

    *speed_abs = 0.0f;
    return 0U;
}

/* 真实机械速度证据：Encoder 实测优先，无编码器时回退 SMO 估计；显式排除 OpenLoop 虚拟速度 */
static uint8_t SourceMgr_GetRealSpeedAbs(const foc_source_mgr_ctx_t *ctx, float *speed_abs)
{
    float speed;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    if (speed_abs == 0) return 0U;

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, FOC_SOURCE_TYPE_ENCODER, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, FOC_SOURCE_TYPE_SMO, &speed) != 0U)
    {
        *speed_abs = fabsf(speed);
        return 1U;
    }
#endif

    *speed_abs = 0.0f;
    return 0U;
}

static uint8_t SourceMgr_LowMotionAbove(const foc_source_mgr_ctx_t *ctx, uint8_t low_source,
                                        float threshold_rad_s)
{
    float speed;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    if (low_source == FOC_SOURCE_TYPE_OPENLOOP)
    {
        if (ctx->ref->openloop_phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
        return (fabsf(ctx->ref->openloop_mech_speed_rad_s) > threshold_rad_s) ? 1U : 0U;
    }
#endif

    if (FOC_SourceMgr_ReadSourceSpeed(&rctx, low_source, &speed) != 0U)
    {
        return (fabsf(speed) > threshold_rad_s) ? 1U : 0U;
    }

    return (fabsf(ctx->ref->ramped_speed_rad_s) > threshold_rad_s) ? 1U : 0U;
}

static uint8_t SourceMgr_CandidateSpeedAbove(uint8_t speed_valid, float speed_abs,
                                             float threshold_rad_s)
{
    return ((speed_valid != 0U) && (speed_abs > threshold_rad_s * FOC_SOURCE_SWITCH_SPEED_SCALE)) ? 1U : 0U;
}

static uint8_t SourceMgr_SpeedAbove(uint8_t speed_valid, float speed_abs,
                                    float threshold_rad_s)
{
    return ((speed_valid != 0U) && (speed_abs > threshold_rad_s)) ? 1U : 0U;
}

uint8_t FOC_SourceMgr_ReadSourceAngle(const foc_source_read_ctx_t *ctx, uint8_t source,
                                      float *mech_out, float *elec_out)
{
    float mech_local = 0.0f;
    float elec_local = 0.0f;

    if ((ctx == 0) || ((mech_out == 0) && (elec_out == 0))) return 0U;

    switch (source)
    {
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_ENCODER:
        if (SourceMgr_EncoderValid(ctx) == 0U) return 0U;
        mech_local = SourceMgr_EncoderAngleRad(ctx);
        elec_local = FOC_ControlMechanicalToElectricalAngle(ctx->params,
                                                            ctx->ctrl->electrical_angle_rad,
                                                            mech_local);
        break;
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
     case FOC_SOURCE_TYPE_SMO:
        if (SourceMgr_StateAcquireReady(SourceMgr_GetSourceState(ctx, FOC_SOURCE_TYPE_SMO)) == 0U) return 0U;
        elec_local = ctx->smo_state->pll_angle_rad;// * (float)ctx->params->direction;
        if (ctx->params->pole_pairs > 0U)
        {
            mech_local = ctx->smo_state->pll_angle_rad / (float)ctx->params->pole_pairs;   /* 物理直通 */
        }
        break;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    case FOC_SOURCE_TYPE_OPENLOOP:
        if (ctx->ref->openloop_phase == FOC_OPENLOOP_STATE_FAILED) return 0U;
        elec_local = ctx->ref->openloop_virtual_angle_rad;
        if (ctx->params->pole_pairs > 0U)
        {
            mech_local = (elec_local / (float)ctx->params->pole_pairs) * (float)ctx->params->direction;
        }
        break;
#endif
    default:
        return 0U;
    }

    if (mech_out != 0) *mech_out = mech_local;
    if (elec_out != 0) *elec_out = elec_local;
    return 1U;
}

static uint8_t SourceMgr_AngleCompatible(const foc_source_mgr_ctx_t *ctx, uint8_t from_source, uint8_t to_source)
{
    float from_elec;
    float to_elec;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    if (FOC_SourceMgr_ReadSourceAngle(&rctx, from_source, 0, &from_elec) == 0U) return 0U;
    if (FOC_SourceMgr_ReadSourceAngle(&rctx, to_source, 0, &to_elec) == 0U) return 0U;

    return (fabsf(Math_WrapRadDelta(to_elec - from_elec)) < FOC_MATH_PI_BY_3) ? 1U : 0U;
}

static void SourceMgr_RebaseSource(foc_source_mgr_ctx_t *ctx, uint8_t new_source, uint8_t old_source)
{
    float old_elec;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    if (FOC_SourceMgr_ReadSourceAngle(&rctx, old_source, 0, &old_elec) == 0U) return;

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    float old_speed;

    if (new_source == FOC_SOURCE_TYPE_OPENLOOP)
    {
        ctx->openloop_state->virtual_angle_rad =
            Math_WrapNearest(ctx->openloop_state->virtual_angle_rad, old_elec);
        if ((ctx->params->pole_pairs > 0U) &&
            (FOC_SourceMgr_ReadSourceSpeed(&rctx, old_source, &old_speed) != 0U))
        {
            /* 只读窗返回物理速度，OpenLoop 虚拟状态需控制坐标系 */
            float old_speed_ctl = old_speed * (float)ctx->params->direction;
            ctx->openloop_state->virtual_speed_rad_s = old_speed_ctl * (float)ctx->params->pole_pairs;
            ctx->openloop_state->mech_speed_rad_s = old_speed_ctl;
        }
        else
        {
            /* ramped_speed 为控制坐标系带符号，直接使用（保留符号） */
            float fallback = ctx->ref->ramped_speed_rad_s;
            ctx->openloop_state->virtual_speed_rad_s = fallback * (float)ctx->params->pole_pairs;
            ctx->openloop_state->mech_speed_rad_s = fallback;
        }
    }
#endif

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (new_source == FOC_SOURCE_TYPE_SMO)
    {
        /* pll_angle 物理系，old_elec 控制系 → 转物理后对齐 */
        ctx->smo_state->pll_angle_rad =
            Math_WrapNearest((float)ctx->params->direction * old_elec, ctx->smo_state->pll_angle_rad);
    }
#endif
}

static void SourceMgr_PrimePidOutput(foc_pid_t *pid, float target_output, float error)
{
    float integral;

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

static void SourceMgr_SyncOuterLoopOnSwitch(foc_source_mgr_ctx_t *ctx, uint8_t new_source,
                                            uint8_t old_source)
{
    float mech_angle = 0.0f;
    float elec_angle = 0.0f;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    if (FOC_SourceMgr_ReadSourceAngle(&rctx, new_source, &mech_angle, &elec_angle) == 0U)
    {
        if (FOC_SourceMgr_ReadSourceAngle(&rctx, old_source, &mech_angle, &elec_angle) == 0U)
        {
            mech_angle = ctx->active->mech_angle_rad;
        }
    }

    ctx->outer_loop->accum_rad = mech_angle;
    ctx->outer_loop->prev_rad = mech_angle;
    ctx->outer_loop->prev_valid = 1U;

    /* 速度环无扰接管：以新源滤波速度作为反馈，按当前速度误差预置速度 PID */
    {
        float new_speed = 0.0f;
        float speed_error = 0.0f;
        if (FOC_SourceMgr_ReadSourceSpeed(&rctx, new_source, &new_speed) != 0U)
        {
            speed_error = ctx->outer_loop->ramped_speed_rad_s
                        - (float)ctx->params->direction * new_speed;
        }
        SourceMgr_PrimePidOutput(ctx->speed_pid, ctx->ctrl->iq_target, speed_error);
    }
}

static void SourceMgr_SyncCurrentLoopOnSwitch(foc_source_mgr_ctx_t *ctx)
{
    float iq_error;

    iq_error = ctx->ctrl->iq_target - ctx->ctrl->iq_measured;
    SourceMgr_PrimePidOutput(ctx->torque_current_pid, ctx->ctrl->uq, iq_error);

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    if (ctx->soft_switch->blend_factor > 0.5f)
    {
        ctx->soft_switch->blend_factor = 0.5f;
    }
    ctx->soft_switch->blend_initialized = 1U;
#endif
}

static void SourceMgr_CommitSwitch(foc_source_mgr_ctx_t *ctx, uint8_t new_source, uint8_t new_region)
{
    uint8_t old_source;

    old_source = ctx->state->active_source;
    SourceMgr_RebaseSource(ctx, new_source, old_source);

    ctx->state->active_source = new_source;
    ctx->state->standby_source =
        (new_source == ctx->switch_cfg->high_source) ?
        ctx->switch_cfg->low_source : ctx->switch_cfg->high_source;
    ctx->state->control_region = new_region;

    SourceMgr_SyncOuterLoopOnSwitch(ctx, new_source, old_source);
    SourceMgr_SyncCurrentLoopOnSwitch(ctx);

    ctx->state->switch_in_progress = 0U;
    ctx->state->switch_counter = 0U;
    ctx->state->degrade_hold_counter = 0U;
}

static void SourceMgr_SetFixedSource(foc_source_mgr_ctx_t *ctx, uint8_t source)
{
    ctx->state->active_source = source;
    ctx->state->standby_source = FOC_SOURCE_TYPE_NONE;
    ctx->state->control_region = FOC_CONTROL_REGION_FULL;
    ctx->state->region_state = FOC_REGION_STATE_FULL_ACTIVE;
    ctx->state->switch_in_progress = 0U;
    ctx->state->switch_counter = 0U;
    ctx->state->degrade_hold_counter = 0U;
}

/* 速域切换仅服务速度控制模式（角度模式依赖编码器可靠源，不作源切换） */
static uint8_t SourceMgr_SpeedModeAllows(const foc_source_mgr_ctx_t *ctx)
{
    return (ctx->control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY) ? 1U : 0U;
}

/* 目标速度位于高速域（> 高速切换门限）才允许驻留/恢复高速域 */
static uint8_t SourceMgr_TargetInHighRegion(const foc_source_mgr_ctx_t *ctx)
{
    return ((SourceMgr_SpeedModeAllows(ctx) != 0U) &&
            (fabsf(ctx->cfg->speed_only_rad_s) > ctx->switch_cfg->speed_threshold_high_rad_s)) ? 1U : 0U;
}

void FOC_SourceMgr_Init(foc_source_mgr_ctx_t *ctx, uint8_t low_source, uint8_t high_source)
{
    uint8_t switchable;

    switchable = ((high_source != FOC_SOURCE_TYPE_NONE) &&
                  (high_source != low_source) &&
                  (SourceMgr_SourceCanLow(low_source) != 0U) &&
                  (SourceMgr_SourceCanHigh(high_source) != 0U)) ? 1U : 0U;

    /* 滞回约束：升域进入门槛(high_th*SCALE)必须高于降级门槛(low_th)，非法配置禁用切换 */
    if (switchable != 0U)
    {
        if ((FOC_SOURCE_SWITCH_SPEED_THRESH_HIGH_DEFAULT <= FOC_SOURCE_SWITCH_SPEED_THRESH_LOW_DEFAULT) ||
            ((FOC_SOURCE_SWITCH_SPEED_THRESH_HIGH_DEFAULT * FOC_SOURCE_SWITCH_SPEED_SCALE) <=
             FOC_SOURCE_SWITCH_SPEED_THRESH_LOW_DEFAULT))
        {
            switchable = 0U;
        }
    }

    ctx->state->active_source = low_source;
    ctx->state->standby_source = switchable ? high_source : FOC_SOURCE_TYPE_NONE;
    ctx->state->switch_in_progress = 0U;
    ctx->state->switch_counter = 0U;
    ctx->state->config_valid = switchable;
    ctx->state->control_region =
        (switchable != 0U) ? FOC_CONTROL_REGION_LOW : FOC_CONTROL_REGION_FULL;
    ctx->state->region_state =
        (switchable != 0U) ? FOC_REGION_STATE_LOW_ACTIVE : FOC_REGION_STATE_FULL_ACTIVE;

    ctx->switch_cfg->low_source = low_source;
    ctx->switch_cfg->high_source = high_source;
    ctx->switch_cfg->speed_threshold_high_rad_s = FOC_SOURCE_SWITCH_SPEED_THRESH_HIGH_DEFAULT;
    ctx->switch_cfg->speed_threshold_low_rad_s = FOC_SOURCE_SWITCH_SPEED_THRESH_LOW_DEFAULT;

    ctx->active->source = low_source;
    ctx->active->state = FOC_SOURCE_STATE_INIT;
    ctx->active->valid = 0U;
    ctx->active->confidence = 0.0f;
    ctx->active->elec_angle_rad = 0.0f;
    ctx->active->mech_angle_rad = 0.0f;

    ctx->state->degrade_hold_counter = 0U;

    SourceMgr_UpdateEncoderServices(ctx);
}

void FOC_SourceMgr_Select(foc_source_mgr_ctx_t *ctx)
{
    uint8_t low;
    uint8_t high;
    uint8_t high_state;
    float speed_abs;
    uint8_t speed_valid;
    float real_speed_abs;
    uint8_t real_speed_valid;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    low = ctx->switch_cfg->low_source;
    high = ctx->switch_cfg->high_source;

    if (ctx->state->config_valid == 0U)
    {
        SourceMgr_SetFixedSource(ctx, low);
        return;
    }

    high_state = SourceMgr_GetSourceState(&rctx, high);
    speed_valid = SourceMgr_GetSwitchSpeedAbs(ctx, &speed_abs);
    real_speed_valid = SourceMgr_GetRealSpeedAbs(ctx, &real_speed_abs);

    /* 非速度控制模式：锁定低速源，不做速域切换（角度模式依赖编码器可靠源） */
    if (SourceMgr_SpeedModeAllows(ctx) == 0U)
    {
        ctx->state->active_source = low;
        ctx->state->standby_source = FOC_SOURCE_TYPE_NONE;
        ctx->state->control_region = FOC_CONTROL_REGION_LOW;
        ctx->state->region_state = FOC_REGION_STATE_LOW_ACTIVE;
        ctx->state->switch_in_progress = 0U;
        ctx->state->switch_counter = 0U;
        ctx->state->degrade_hold_counter = 0U;
        return;
    }

    switch (ctx->state->region_state)
    {
    case FOC_REGION_STATE_LOW_ACTIVE:
        ctx->state->active_source = low;
        ctx->state->control_region = FOC_CONTROL_REGION_LOW;
        if ((SourceMgr_TargetInHighRegion(ctx) != 0U) &&
            (SourceMgr_LowMotionAbove(ctx, low,
                ctx->switch_cfg->speed_threshold_high_rad_s) != 0U) &&
            (SourceMgr_CandidateSpeedAbove(real_speed_valid, real_speed_abs,
                ctx->switch_cfg->speed_threshold_high_rad_s) != 0U) &&
            (SourceMgr_StateAcquireReady(high_state) != 0U) &&
            (SourceMgr_SourceValid(&rctx, high) != 0U))
        {
            ctx->state->region_state = FOC_REGION_STATE_HIGH_ACQUIRE;
            ctx->state->switch_in_progress = 1U;
            ctx->state->switch_counter = 1U;
        }
        else
        {
            ctx->state->switch_counter = 0U;
        }
        break;

    case FOC_REGION_STATE_HIGH_ACQUIRE:
        if ((SourceMgr_TargetInHighRegion(ctx) == 0U) ||
            (SourceMgr_LowMotionAbove(ctx, low,
                ctx->switch_cfg->speed_threshold_low_rad_s) == 0U) ||
            (SourceMgr_SpeedAbove(real_speed_valid, real_speed_abs,
                ctx->switch_cfg->speed_threshold_low_rad_s) == 0U) ||
            (SourceMgr_StateAcquireReady(high_state) == 0U) ||
            (SourceMgr_SourceValid(&rctx, high) == 0U))
        {
            ctx->state->region_state = FOC_REGION_STATE_LOW_ACTIVE;
            ctx->state->switch_in_progress = 0U;
            ctx->state->switch_counter = 0U;
            break;
        }

        ctx->state->switch_counter++;
        if (ctx->state->switch_counter >= FOC_SOURCE_SWITCH_SETTLE_CYCLES)
        {
            if ((SourceMgr_StateEntryReady(high_state) != 0U) &&
                ((ctx->state->active_source == FOC_SOURCE_TYPE_OPENLOOP) ||
                 (SourceMgr_AngleCompatible(ctx, ctx->state->active_source, high) != 0U)))
            {
                SourceMgr_CommitSwitch(ctx, high, FOC_CONTROL_REGION_HIGH);
                ctx->state->region_state = FOC_REGION_STATE_HIGH_ACTIVE;
            }
        }
        break;

    case FOC_REGION_STATE_HIGH_ACTIVE:
        ctx->state->active_source = high;
        ctx->state->control_region = FOC_CONTROL_REGION_HIGH;
        /* 目标不在高速域、SMO 非收敛/发散/无效、或实测速度低于门限 → 触发降级 */
        if ((SourceMgr_TargetInHighRegion(ctx) == 0U) ||
            (high_state == FOC_SOURCE_STATE_DIVERGED) ||
            (SourceMgr_StateHoldValid(high_state) == 0U) ||
            (SourceMgr_SourceValid(&rctx, high) == 0U) ||
            ((speed_valid != 0U) &&
             (speed_abs < ctx->switch_cfg->speed_threshold_low_rad_s)))
        {
            ctx->state->region_state = FOC_REGION_STATE_HIGH_SUSPECT;
            ctx->state->switch_in_progress = 1U;
            ctx->state->switch_counter = 1U;
            ctx->state->degrade_hold_counter = 0U;
        }
        else
        {
            ctx->state->switch_in_progress = 0U;
            ctx->state->switch_counter = 0U;
        }
        break;

    case FOC_REGION_STATE_HIGH_SUSPECT:
        /*
         * 降级恢复要求：目标在高速域 且 实测速度 ≥ high 门限 连续 DEGRADE_CONFIRM_CYCLES 拍，
         * 单拍尖峰或速度读取失败不能打断敏捷降级。
         */
        if ((SourceMgr_TargetInHighRegion(ctx) != 0U) &&
            (high_state != FOC_SOURCE_STATE_DIVERGED) &&
            (SourceMgr_StateHoldValid(high_state) != 0U) &&
            (SourceMgr_SourceValid(&rctx, high) != 0U) &&
            (speed_valid != 0U) &&
            (speed_abs >= ctx->switch_cfg->speed_threshold_high_rad_s))
        {
            ctx->state->degrade_hold_counter++;
            if (ctx->state->degrade_hold_counter >= FOC_SOURCE_SWITCH_DEGRADE_CONFIRM_CYCLES)
            {
                ctx->state->region_state = FOC_REGION_STATE_HIGH_ACTIVE;
                ctx->state->switch_in_progress = 0U;
                ctx->state->switch_counter = 0U;
                ctx->state->degrade_hold_counter = 0U;
            }
            break;
        }

        ctx->state->degrade_hold_counter = 0U;
        ctx->state->switch_counter++;
        if ((high_state == FOC_SOURCE_STATE_DIVERGED) ||
            (ctx->state->switch_counter >= FOC_SOURCE_SWITCH_DEGRADE_CONFIRM_CYCLES))
        {
            ctx->state->region_state = FOC_REGION_STATE_LOW_RECOVERY;
            ctx->state->switch_in_progress = 0U;
        }
        break;

    case FOC_REGION_STATE_LOW_RECOVERY:
        if ((SourceMgr_SourceValid(&rctx, low) != 0U) || (low == FOC_SOURCE_TYPE_OPENLOOP))
        {
            SourceMgr_CommitSwitch(ctx, low, FOC_CONTROL_REGION_LOW);
            ctx->state->region_state = FOC_REGION_STATE_LOW_ACTIVE;
        }
        break;

    case FOC_REGION_STATE_FULL_ACTIVE:
    default:
        SourceMgr_SetFixedSource(ctx, low);
        break;
    }
}

void FOC_SourceMgr_Publish(foc_source_mgr_ctx_t *ctx)
{
    uint8_t src;
    uint8_t valid;
    float mech_angle;
    float elec_angle;
    foc_source_read_ctx_t rctx = SourceMgr_ReadCtx(ctx);

    src = ctx->state->active_source;
    mech_angle = 0.0f;
    elec_angle = 0.0f;
    valid = FOC_SourceMgr_ReadSourceAngle(&rctx, src, &mech_angle, &elec_angle);

    ctx->active->source = src;
    ctx->active->state = SourceMgr_GetSourceState(&rctx, src);
    ctx->active->valid = valid;
    ctx->active->confidence =
        (src == FOC_SOURCE_TYPE_ENCODER) ? 1.0f :
        ((src == FOC_SOURCE_TYPE_OPENLOOP) ? 0.5f :
         ((ctx->active->state == FOC_SOURCE_STATE_LOCKED) ? 0.9f : 0.4f));
    ctx->active->mech_angle_rad = mech_angle;
    ctx->active->elec_angle_rad = elec_angle;

    /* 发布 active source 滤波后机械速度（物理系，供速度环消费） */
    {
        float speed = 0.0f;
        (void)FOC_SourceMgr_ReadSourceSpeed(&rctx, src, &speed);
        ctx->active->mech_speed_rad_s = speed;
    }

    if (valid != 0U)
    {
        ctx->ctrl->electrical_angle_rad = elec_angle;
    }

    SourceMgr_UpdateEncoderServices(ctx);
}
