#include "L2_Core/Control/foc_ctrl_executor.h"

#include <math.h>
#include <string.h>

#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L2_Core/Control/foc_ctrl_openloop.h"
#include "L2_Core/Control/foc_ctrl_source_mgr.h"
#include "L2_Core/Control/foc_ctrl_estim.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_math_types.h"
#include "LS_Config/foc_config.h"

static void ResetPID(foc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void FOC_ControlExecutor_FullStop(foc_motor_t *motor)
{
    if (motor == 0) return;

    /* 清零控制输出 */
    motor->ctrl.ud = 0.0f;
    motor->ctrl.uq = 0.0f;
    motor->ctrl.iq_target = 0.0f;

    /* 清零 PID */
    ResetPID(&motor->torque_current_pid);
    ResetPID(&motor->speed_pid);
    ResetPID(&motor->angle_pid);

    /* 清零外环累积状态 */
    motor->outer_loop.accum_rad = 0.0f;
    motor->outer_loop.prev_rad = 0.0f;
    motor->outer_loop.prev_valid = 0U;
    motor->outer_loop.ramped_speed_rad_s = 0.0f;
    motor->outer_loop.speed_state_valid = 0U;
    motor->outer_loop.speed_err_accum_rad = 0.0f;

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
    FOC_ControlRecordPhaseOutputZero(motor, motor->state.control_phase, 0U);
    FOC_ControlApplyPhaseOutputRuntime(motor);
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
    if (motor == 0) return;
    motor->isr_timing.fast_current_div_counter = 0U;
}

/* ================================================================
 * PWM ISR：插值 → 守卫检查 → 硬件采样 → Estimator → Select → Publish → 电流环 → SVPWM。
 * ================================================================ */
void FOC_ControlExecutor_RunISR(foc_motor_t *motor)
{
    uint8_t divider;
    float current_loop_dt_sec;

    uint32_t isr_start;
    isr_start = FOC_Platform_ReadCycleCounter();

    if (motor == 0) return;

    SVPWM_InterpolationISR(motor);

    if (motor->state.motor_enabled == 0U)
    {
        FOC_ControlExecutor_FullStop(motor);
        return;
    }

    if ((motor->state.control_phase == FOC_CONTROL_PHASE_COGGING_CALIB) ||
        (motor->state.control_phase == FOC_CONTROL_PHASE_REINIT))
    {
        if (motor->phase_output_state.valid == 0U) return;
        FOC_ControlApplyPhaseOutputRuntime(motor);
        return;
    }

    if (motor->state.control_phase != FOC_CONTROL_PHASE_NORMAL) return;

    if (motor->state.current_loop_ready == 0U) return;

    divider = (FOC_CURRENT_LOOP_ISR_DIVIDER == 0U) ? 1U : (uint8_t)FOC_CURRENT_LOOP_ISR_DIVIDER;
    motor->isr_timing.fast_current_div_counter++;
    if (motor->isr_timing.fast_current_div_counter < divider) return;
    motor->isr_timing.fast_current_div_counter = 0U;

    current_loop_dt_sec = (FOC_PWM_FREQ_KHZ == 0U) ? FOC_CONTROL_DT_SEC
                          : ((float)divider / ((float)FOC_PWM_FREQ_KHZ * 1000.0f));

    {
    /* 阶段1：硬件采样 */
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_ENABLE)
    Sensor_ReadEncoder(motor, &motor->sensor, current_loop_dt_sec);
    Sensor_AccumulateEcycle(motor, &motor->sensor);
#endif
#endif
    if (FOC_ControlRequiresCurrentSample() != 0U)
    {
        Sensor_ReadCurrent(motor);
        if (motor->sensor.adc_valid == 0U) return;
    }

    /* 阶段2：各 Estimator 数值迭代（读 sensor_fast，只写内部状态） */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimSMO_Step(motor, current_loop_dt_sec);
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimHFI_Step(motor, current_loop_dt_sec);
#endif

    /* 阶段3：数据选择与发布 */
    FOC_SourceMgr_Select(motor);
    FOC_SourceMgr_Publish(motor);
    
    /* 阶段4：电流环 */
    FOC_CurrentControlStep(motor, &motor->sensor,
                           motor->ctrl.electrical_angle_rad,
                           current_loop_dt_sec);

    /* 阶段5：SVPWM */
    FOC_ControlApplyElectricalAngleRuntime(motor, motor->ctrl.electrical_angle_rad);

    motor->isr_timing.current_loop_cycles = FOC_Platform_ReadCycleCounter() - isr_start;
    
    }
}

/* ================================================================
 * 控制周期（主循环调度）
 * ================================================================ */

uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor, float dt_sec)
{
    if (motor == 0) return FOC_CYCLE_SKIPPED;

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
        FOC_OpenLoop_RunStep(motor, dt_sec);
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
    if (motor == 0) return;

    motor->outer_loop.accum_rad = motor->active_source_state.mech_angle_rad;
    motor->outer_loop.prev_rad = motor->active_source_state.mech_angle_rad;
    motor->outer_loop.prev_valid = 1U;

    motor->outer_loop.ramped_speed_rad_s = 0.0f;

    motor->outer_loop.speed_state_valid = 0U;
    motor->outer_loop.speed_err_accum_rad = 0.0f;

    ResetPID(&motor->speed_pid);
    ResetPID(&motor->angle_pid);

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

    FOC_SpeedOuterLoopStep(motor,
                           &motor->speed_pid,
                           motor->cfg.speed_only_rad_s,
                           dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)

    FOC_SpeedAngleOuterLoopStep(motor,
                                &motor->speed_pid,
                                &motor->angle_pid,
                                motor->cfg.target_angle_rad,
                                motor->cfg.angle_position_speed_rad_s,
                                dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)

    if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(motor,
                               &motor->speed_pid,
                               motor->cfg.speed_only_rad_s,
                               dt_sec);
    }
    else if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
    {
        FOC_SpeedAngleOuterLoopStep(motor,
                                    &motor->speed_pid,
                                    &motor->angle_pid,
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
    if (FOC_CoggingCalibIsBusy(motor) == 0U)
    {
        FOC_ControlApplyCoggingCompensation(motor,
                                            motor->active_source_state.mech_angle_rad,
                                            motor->cogging_comp_status.speed_ref_rad_s);
    }
#endif
}
