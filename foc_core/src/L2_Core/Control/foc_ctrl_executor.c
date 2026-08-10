#include "L2_Core/foc_motor_aggregate.h"
#include "L2_Core/Control/foc_ctrl_executor.h"

#include <math.h>
#include <string.h>

#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L2_Core/Control/foc_ctrl_openloop.h"
#include "L2_Core/Control/foc_ctrl_source_mgr.h"
#include "L2_Core/Control/foc_ctrl_estim.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_math_types.h"
#include "LS_Config/foc_config.h"

/* 电流环 ISR 的控制参考快照：控制 ISR 单点发布 ctrl_ref 后，此处原子取走；
 * 提交标志未置位（控制仍在写块/未发布）时复用上次快照，保证本 ISR 读到一致参考。 */
static foc_control_ref_t s_ctrl_ref_snap;

void FOC_ControlExecutor_FullStop(foc_motor_t *motor)
{
    /* 清零控制输出 */
    motor->ctrl.ud = 0.0f;
    motor->ctrl.uq = 0.0f;
    motor->ctrl.iq_target = 0.0f;

    /* 清零 PID */
    FOC_PIDReset(&motor->torque_current_pid);
    FOC_PIDReset(&motor->speed_pid);
    FOC_PIDReset(&motor->angle_pid);

    /* 清零外环累积状态 */
    motor->outer_loop.accum_rad = 0.0f;
    motor->outer_loop.prev_rad = 0.0f;
    motor->outer_loop.prev_valid = 0U;
    motor->outer_loop.ramped_speed_rad_s = 0.0f;

    /* 复位模式切换状态，使下次 RunCycle 重新初始化 */
    motor->mode_transition.prev_control_mode_valid = 0U;

    /* 阻断 ISR 控制链 */
    motor->state.current_loop_ready = 0U;

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->current_soft_switch_status.enabled = 0U;
    motor->current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    motor->current_soft_switch_status.blend_initialized = 0U;
#endif

    /* 归零 PWM */
    FOC_ControlRecordPhaseOutputZero(&motor->phase_output_state, &motor->ctrl,
                                     &motor->outer_loop, motor->state.control_phase, 0U);
    FOC_ControlApplyPhaseOutputRuntime(&motor->ctrl, &motor->svpwm,
                                       &motor->applied_output, &motor->alpha_beta,
                                       &motor->phase_output_state, &motor->params);
}

void FOC_ControlExecutor_SafeOutput(foc_motor_t *motor)
{
    FOC_ControlExecutor_FullStop(motor);
}

void FOC_ControlExecutor_Stop(foc_motor_t *motor)
{
    FOC_ControlExecutor_FullStop(motor);
}

void FOC_ControlExecutor_Init(foc_motor_t *motor)
{
    motor->isr_timing.fast_current_div_counter = 0U;
}

/* ================================================================
 * 电流环核心阶段：采样 → Estimator → Select → Publish → 电流环 → SVPWM。
 * 双 ISR 模式由 PWM ISR 调用；三 ISR 模式由独立电流环 ISR 调用。
 * ================================================================ */
static void FOC_ControlExecutor_RunISR_CurrentLoopCore(foc_motor_t *motor, float current_loop_dt_sec)
{
    uint32_t isr_start;

    /* 获取控制参考：控制 ISR 单点发布，此处原子取走；未提交时复用上次快照 */
    if (motor->ctrl_ref_ready != 0U)
    {
        s_ctrl_ref_snap = motor->ctrl_ref;
        motor->ctrl_ref_ready = 0U;
    }
    motor->ctrl.iq_target = s_ctrl_ref_snap.iq_target;

    isr_start = FOC_Platform_ReadCycleCounter();

    {
    /* 阶段1：硬件采样 */
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_ENABLE)
    Sensor_ReadEncoder(&motor->sensor, current_loop_dt_sec);
#endif
#endif
    if (FOC_ControlRequiresCurrentSample() != 0U)
    {
        Sensor_ReadCurrent(&motor->sensor);
        if (motor->sensor.adc_valid == 0U) return;
    }

    /* 阶段2：各 Estimator 数值迭代（读 sensor_fast，只写内部状态） */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimSMO_Step(&motor->estim_smo_state,
                      &motor->params,
                      &motor->sensor,
                      &motor->applied_output,
                      &motor->ctrl,
                      motor->source_mgr_state.active_source,
                      motor->source_mgr_state.control_region,
                      motor->cfg.speed_only_rad_s,
                      current_loop_dt_sec);
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimHFI_Step(&motor->estim_hfi_state, current_loop_dt_sec);
#endif

    /* 阶段3：数据选择与发布 */
    {
        foc_source_mgr_ctx_t sm_ctx;

        sm_ctx.sensor = &motor->sensor;
        sm_ctx.params = &motor->params;
        sm_ctx.cfg = &motor->cfg;
        sm_ctx.control_mode = motor->state.control_mode;
        sm_ctx.switch_cfg = &motor->source_switch_state;
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
        sm_ctx.smo_state = &motor->estim_smo_state;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
        sm_ctx.openloop_state = &motor->openloop_state;
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        sm_ctx.cogging_status = &motor->cogging_comp_status;
#endif
        sm_ctx.state = &motor->source_mgr_state;
        sm_ctx.active = &motor->active_source_state;
        sm_ctx.ctrl = &motor->ctrl;
        sm_ctx.ref = &s_ctrl_ref_snap;
        sm_ctx.outer_loop = &motor->outer_loop;
        sm_ctx.speed_pid = &motor->speed_pid;
        sm_ctx.torque_current_pid = &motor->torque_current_pid;
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        sm_ctx.soft_switch = &motor->current_soft_switch_status;
#endif
        sm_ctx.encoder_services = &motor->encoder_services;
        FOC_SourceMgr_Select(&sm_ctx);
        FOC_SourceMgr_Publish(&sm_ctx);
    }
    
    /* 阶段4：电流环 */
    FOC_CurrentControlStep(&motor->ctrl,
                           &motor->torque_current_pid,
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
                           &motor->current_soft_switch_status,
#else
                           0,
#endif
                           &motor->sensor,
                           &motor->params,
                           current_loop_dt_sec);

    /* 阶段5：SVPWM */
    FOC_ControlApplyElectricalAngleRuntime(&motor->ctrl, &motor->svpwm,
                                           &motor->applied_output, &motor->alpha_beta,
                                           &motor->params,
                                           motor->ctrl.electrical_angle_rad);

    motor->isr_timing.current_loop_cycles = FOC_Platform_ReadCycleCounter() - isr_start;
    }
}

/* ================================================================
 * 双 ISR 模式 PWM ISR 入口：插值 → 守卫检查 → 电流环核心。
 * ================================================================ */
void FOC_ControlExecutor_RunISR(foc_motor_t *motor)
{
    uint8_t divider;

#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    SVPWM_InterpolationISR(&motor->svpwm);
#endif

    if (motor->state.motor_enabled == 0U)
    {
        FOC_ControlExecutor_FullStop(motor);
        return;
    }

    if ((motor->state.control_phase == FOC_CONTROL_PHASE_COGGING_CALIB) ||
        (motor->state.control_phase == FOC_CONTROL_PHASE_REINIT))
    {
        if (motor->phase_output_state.valid == 0U) return;
        FOC_ControlApplyPhaseOutputRuntime(&motor->ctrl, &motor->svpwm,
                                           &motor->applied_output, &motor->alpha_beta,
                                           &motor->phase_output_state, &motor->params);
        return;
    }

    if (motor->state.control_phase != FOC_CONTROL_PHASE_NORMAL) return;
    if (motor->state.current_loop_ready == 0U) return;

    divider = (FOC_CURRENT_LOOP_ISR_DIVIDER == 0U) ? 1U : (uint8_t)FOC_CURRENT_LOOP_ISR_DIVIDER;
    motor->isr_timing.fast_current_div_counter++;
    if (motor->isr_timing.fast_current_div_counter < divider) return;
    motor->isr_timing.fast_current_div_counter = 0U;

    FOC_ControlExecutor_RunISR_CurrentLoopCore(motor, FOC_CURRENT_LOOP_DT_SEC);
}

#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
/* ================================================================
 * 三 ISR 模式 - PWM ISR 入口：仅插值 + 守卫检查（无电流环）。
 * ================================================================ */
void FOC_ControlExecutor_RunISR_PwmOnly(foc_motor_t *motor)
{
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    SVPWM_InterpolationISR(&motor->svpwm);
#endif

    if (motor->state.motor_enabled == 0U)
    {
        FOC_ControlExecutor_FullStop(motor);
        return;
    }

    if ((motor->state.control_phase == FOC_CONTROL_PHASE_COGGING_CALIB) ||
        (motor->state.control_phase == FOC_CONTROL_PHASE_REINIT))
    {
        if (motor->phase_output_state.valid == 0U) return;
        FOC_ControlApplyPhaseOutputRuntime(&motor->ctrl, &motor->svpwm,
                                           &motor->applied_output, &motor->alpha_beta,
                                           &motor->phase_output_state, &motor->params);
        return;
    }
}

/* ================================================================
 * 三 ISR 模式 - 电流环 ISR 入口：独立定时器驱动，与 PWM 频率解耦。
 * ================================================================ */
void FOC_ControlExecutor_RunISR_CurrentLoop(foc_motor_t *motor)
{

    if (motor->state.motor_enabled == 0U)
    {
        FOC_ControlExecutor_FullStop(motor);
        return;
    }

    if (motor->state.control_phase != FOC_CONTROL_PHASE_NORMAL) return;
    if (motor->state.current_loop_ready == 0U) return;

    FOC_ControlExecutor_RunISR_CurrentLoopCore(motor, FOC_CURRENT_LOOP_DT_SEC);
}
#endif

/* ================================================================
 * 控制周期（主循环调度）
 * ================================================================ */

uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor, float dt_sec)
{
    if (motor->state.motor_enabled == 0U)
    {
        return FOC_CYCLE_SKIPPED;
    }

    if (motor->state.control_mode != motor->mode_transition.prev_control_mode_check)
    {
        motor->mode_transition.prev_control_mode_check = motor->state.control_mode;
    }

    if (motor->active_source_state.valid == 0U)
    {
        motor->state.current_loop_ready = 1U;
        return FOC_CYCLE_SKIPPED;
    }

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    if (motor->source_mgr_state.active_source == FOC_SOURCE_TYPE_OPENLOOP)
    {
        float openloop_mech_speed_rad_s = 0.0f;
        if (FOC_OpenLoop_RunStep(&motor->openloop_state,
                                 &motor->ctrl,
                                 &motor->params,
                                 &motor->cfg,
                                 dt_sec,
                                 &openloop_mech_speed_rad_s) != 0U)
        {
            /* 开环段保持速度外环参考轨迹连续，使切换到 HIGH 时加速器起点正确 */
            motor->outer_loop.ramped_speed_rad_s = openloop_mech_speed_rad_s;
        }
    }
    else
#endif
    {
        FOC_ControlExecutor_RunOuterLoop(motor, dt_sec);
    }

    motor->state.current_loop_ready = 1U;

    return FOC_CYCLE_OK;
}

static void Executor_OnModeSwitch(foc_motor_t *motor, uint8_t new_mode, uint8_t old_mode)
{
    motor->outer_loop.accum_rad = motor->active_source_state.mech_angle_rad;
    motor->outer_loop.prev_rad = motor->active_source_state.mech_angle_rad;
    motor->outer_loop.prev_valid = 1U;

    motor->outer_loop.ramped_speed_rad_s = 0.0f;

    FOC_PIDReset(&motor->speed_pid);
    FOC_PIDReset(&motor->angle_pid);

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->current_soft_switch_status.enabled = 0U;
    motor->current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    motor->current_soft_switch_status.blend_initialized = 0U;
#endif

    (void)old_mode;
}

void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor, float dt_sec)
{
    uint8_t cur_mode;

    cur_mode = motor->state.control_mode;

    if (motor->mode_transition.prev_control_mode_valid == 0U)
    {
        motor->mode_transition.prev_control_mode = cur_mode;
        motor->mode_transition.prev_control_mode_valid = 1U;
    }

    if (cur_mode != motor->mode_transition.prev_control_mode)
    {
        Executor_OnModeSwitch(motor, cur_mode, motor->mode_transition.prev_control_mode);
        motor->mode_transition.prev_control_mode = cur_mode;
    }

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)

    FOC_SpeedOuterLoopStep(&motor->outer_loop, &motor->speed_pid, &motor->ctrl, &motor->active_source_state, &motor->cfg, &motor->params, motor->source_mgr_state.control_region,
                           motor->cfg.speed_only_rad_s,
                           dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)

    FOC_SpeedAngleOuterLoopStep(&motor->outer_loop, &motor->speed_pid, &motor->angle_pid, &motor->ctrl, &motor->active_source_state, &motor->cfg, &motor->params,
                                motor->cfg.target_angle_rad,
                                motor->cfg.angle_position_speed_rad_s,
                                dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)

    if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(&motor->outer_loop, &motor->speed_pid, &motor->ctrl, &motor->active_source_state, &motor->cfg, &motor->params, motor->source_mgr_state.control_region,
                               motor->cfg.speed_only_rad_s,
                               dt_sec);
    }
    else if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
    {
        FOC_SpeedAngleOuterLoopStep(&motor->outer_loop, &motor->speed_pid, &motor->angle_pid, &motor->ctrl, &motor->active_source_state, &motor->cfg, &motor->params,
                                    motor->cfg.target_angle_rad,
                                    motor->cfg.angle_position_speed_rad_s,
                                    dt_sec);
    }
    else
    {
        return;
    }

#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    /* 标定进行中跳过补偿；CALIB 裁剪时恒执行补偿 */
    if (FOC_CoggingCalibIsBusy(&motor->cogging_calib_state) == 0U)
#else
    if (1)
#endif
    {
        FOC_ControlApplyCoggingCompensation(&motor->cogging_comp_status,
                                            &motor->ctrl,
                                            motor->cogging_comp_table_q15,
                                            motor->active_source_state.mech_angle_rad,
                                            motor->cogging_comp_status.speed_ref_rad_s);
    }
#endif
}

/* ================================================================
 * 控制参考单点发布（控制 ISR 过程末尾调用）。
 * 写块 → MemoryBarrier → 置提交标志；电流环 ISR 在过程开头原子取走。
 * 仅作"边界 A（控制→电流环）"的原子化，不改变任何算法计算内容。
 * ================================================================ */
void FOC_ControlExecutor_PublishControlRef(foc_motor_t *motor)
{
    if (motor == 0) return;

    motor->ctrl_ref.iq_target = motor->ctrl.iq_target;
    motor->ctrl_ref.ramped_speed_rad_s = motor->outer_loop.ramped_speed_rad_s;
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    motor->ctrl_ref.mech_angle_rad = motor->sensor.mech_angle_rad.output_value;
    motor->ctrl_ref.mech_speed_rad_s = motor->sensor.mech_speed_rad_s;
    motor->ctrl_ref.encoder_valid = motor->sensor.encoder_valid;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    motor->ctrl_ref.openloop_phase = motor->openloop_state.phase;
    motor->ctrl_ref.openloop_virtual_angle_rad = motor->openloop_state.virtual_angle_rad;
    motor->ctrl_ref.openloop_virtual_speed_rad_s = motor->openloop_state.virtual_speed_rad_s;
    motor->ctrl_ref.openloop_mech_speed_rad_s = motor->openloop_state.mech_speed_rad_s;
#endif
    FOC_Platform_MemoryBarrier();
    motor->ctrl_ref_ready = 1U;
}
