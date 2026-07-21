#include "L2_Core/Control/foc_ctrl_executor.h"

#include <math.h>
#include <string.h>

#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L2_Core/Control/foc_ctrl_estim.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* ========== 内部工具 ========== */

static void ResetPIDState(foc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void Executor_SafeOutput(foc_motor_t *motor, uint8_t report_skip)
{
    FOC_CurrentControlOpenLoopStep(motor, 0.0f, 0.0f,
                                   FOC_CONTROL_DT_SEC);
    SVPWM_ApplyDirectDuty(motor, 0U, 0.0f, 0.0f, 0.0f);

    if (report_skip != 0U)
    {
        motor->state.control_skip_count++;
    }
}

/* ================================================================
 * PWM ISR：4阶段
 *   1. 插值与采样 → 2. 电流环 → 3. 角度同步+估计器 → 4. SVPWM
 * ================================================================ */

void FOC_ControlExecutor_Init(foc_motor_t *motor)
{
    if (motor == 0) return;
    motor->fast_current_div_counter = 0U;
}

void FOC_ControlExecutor_Stop(foc_motor_t *motor)
{
    if (motor == 0) return;
    FOC_CurrentControlOpenLoopStep(motor, 0.0f, 0.0f,
                                   FOC_CONTROL_DT_SEC);
    SVPWM_ApplyDirectDuty(motor, 0U, 0.0f, 0.0f, 0.0f);
}

void FOC_ControlExecutor_RunISR(foc_motor_t *motor)
{
    uint8_t divider;
    float current_loop_dt_sec;
    uint32_t isr_start;

    if (motor == 0) return;

    /* 阶段1：插值与采样 */
    SVPWM_InterpolationISR(motor);

    if (motor->state.control_phase != FOC_CONTROL_PHASE_NORMAL) return;
    if (motor->state.system_running == 0U) return;
    if (motor->state.motor_enabled == 0U) return;
    if (motor->state.current_loop_ready == 0U) return;
    if (motor->state.system_fault != 0U)
    {
        Executor_SafeOutput(motor, 0U);
        return;
    }

    divider = (FOC_CURRENT_LOOP_ISR_DIVIDER == 0U) ? 1U : (uint8_t)FOC_CURRENT_LOOP_ISR_DIVIDER;
    motor->fast_current_div_counter++;
    if (motor->fast_current_div_counter < divider) return;
    motor->fast_current_div_counter = 0U;

    current_loop_dt_sec = (FOC_PWM_FREQ_KHZ == 0U) ? FOC_CONTROL_DT_SEC
                          : ((float)divider / ((float)FOC_PWM_FREQ_KHZ * 1000.0f));

    isr_start = FOC_Platform_ReadCycleCounter();

    if (FOC_ControlRequiresCurrentSample() != 0U)
    {
        Sensor_ReadCurrent(motor);
        if (motor->sensor_fast.adc_valid == 0U) return;

#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_ENABLE)
        Sensor_ReadEncoder(motor, &motor->sensor_fast);
#endif

        Sensor_AccumulateEcycle(motor, &motor->sensor_fast);

        /* 阶段2：电流环 */
        FOC_CurrentControlStep(motor, &motor->sensor_fast,
                               motor->electrical_phase_angle,
                               current_loop_dt_sec);

        /* 阶段3：角度同步 + 估计器 */
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_ENABLE)
        motor->sensor.mech_angle_rad.output_value = motor->sensor_fast.mech_angle_rad.output_value;
        motor->sensor.encoder_valid = motor->sensor_fast.encoder_valid;
#endif

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_SMO_ENABLE   == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_HFI_ENABLE   == FOC_CFG_ENABLE)
        if (motor->estimator_step_fn != 0)
            motor->estimator_step_fn(motor, &motor->est_state, current_loop_dt_sec);
        if (motor->estimator_step_fn_alt != 0)
            motor->estimator_step_fn_alt(motor, &motor->est_state_alt, current_loop_dt_sec);
#endif

        /* 阶段4：SVPWM */
        FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
    }
    else
    {
        FOC_CurrentControlStep(motor, 0,
                               motor->electrical_phase_angle,
                               current_loop_dt_sec);

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_SMO_ENABLE   == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_HFI_ENABLE   == FOC_CFG_ENABLE)
        if (motor->estimator_step_fn != 0)
            motor->estimator_step_fn(motor, &motor->est_state, current_loop_dt_sec);
        if (motor->estimator_step_fn_alt != 0)
            motor->estimator_step_fn_alt(motor, &motor->est_state_alt, current_loop_dt_sec);
#endif

        FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
    }

    motor->current_loop_cycles = FOC_Platform_ReadCycleCounter() - isr_start;
}

void FOC_ControlExecutor_SafeOutput(foc_motor_t *motor)
{
    Executor_SafeOutput(motor, 0U);
}

/* ================================================================
 * Control ISR 控制周期
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

    FOC_ControlExecutor_RunOuterLoop(motor, dt_sec);

    motor->state.current_loop_ready = 1U;

    return FOC_CYCLE_OK;
}

/* ========== 外环统一入口 ========== */

void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor, float dt_sec)
{
    uint8_t cur_mode;

    if (motor == 0) return;

    cur_mode = motor->state.control_mode;

    if (motor->mode_transition.prev_control_mode_valid == 0U)
    {
        motor->mode_transition.prev_control_mode = cur_mode;
        motor->mode_transition.prev_control_mode_valid = 1U;
    }

    if (cur_mode != motor->mode_transition.prev_control_mode)
    {
        if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
        {
            FOC_ControlRebaseMechanicalAngleAccum(motor, motor->ctrl_input.mech_angle_rad);
        }

        ResetPIDState(&motor->torque_current_pid);
        ResetPIDState(&motor->speed_pid);
        ResetPIDState(&motor->angle_pid);
        FOC_ControlResetSpeedLoopState(motor);

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        motor->current_soft_switch_status.enabled = 0U;
        motor->current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        motor->current_soft_switch_blend_initialized = 0U;
#endif

        motor->mode_transition.prev_control_mode = cur_mode;
    }

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)

    FOC_SpeedOuterLoopStep(motor,
                           &motor->speed_pid,
                           motor->speed_only_rad_s,
                           dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)

    FOC_SpeedAngleOuterLoopStep(motor,
                                &motor->speed_pid,
                                &motor->angle_pid,
                                motor->target_angle_rad,
                                motor->angle_position_speed_rad_s,
                                dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)

    if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(motor,
                               &motor->speed_pid,
                               motor->speed_only_rad_s,
                               dt_sec);
    }
    else if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
    {
        FOC_SpeedAngleOuterLoopStep(motor,
                                    &motor->speed_pid,
                                    &motor->angle_pid,
                                    motor->target_angle_rad,
                                    motor->angle_position_speed_rad_s,
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
                                            motor->ctrl_input.mech_angle_rad,
                                            motor->cogging_speed_ref_rad_s);
    }
#endif
}