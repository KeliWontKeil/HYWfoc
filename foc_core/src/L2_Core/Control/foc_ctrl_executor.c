#include "L2_Core/Control/foc_ctrl_executor.h"

#include <math.h>

#include "L1_Orchestration/foc_output_mgr.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_actuation.h"
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

/* Safe-output helper (defined before RunISR to avoid forward-decl). */
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
 * PWM ISR：采样 → e-cycle累积 → 电流环控制 → SVPWM输出
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

    if (motor == 0) return;

    /* SVPWM 插值 ISR：任何阶段都运行，确保开环/标定/重初始化输出生效 */
    SVPWM_InterpolationISR(motor);

    /* 非 NORMAL 阶段：不运行ISR电流环（标定/重初始化使用开环驱动） */
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

    /* 阶段1: 采样 — 电流采样唯一入口 */
    if (FOC_ControlRequiresCurrentSample() != 0U)
    {
        Sensor_ReadCurrent(motor);
        if (motor->sensor_fast.adc_valid == 0U) return;

#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_ENABLE)
        /* 快速编码器：在ISR中同步读取角度 */
        Sensor_ReadEncoder(motor, &motor->sensor_fast);
#endif

        /* 阶段1.5: e-cycle 漂移抑制 */
        Sensor_AccumulateEcycle(motor, &motor->sensor_fast);

        /* 阶段2: 控制算法 */
        FOC_CurrentControlStep(motor, &motor->sensor_fast,
                               motor->electrical_phase_angle,
                               current_loop_dt_sec);

        /* 阶段3: SVPWM 执行 */
        FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
    }
    else
    {
        /* No current sensor: run open-loop control without sensor feedback. */
        FOC_CurrentControlStep(motor, 0,
                               motor->electrical_phase_angle,
                               current_loop_dt_sec);
        FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
    }
}

/* Public safe-output entry — used by L1 for synchronous zeroing. */
void FOC_ControlExecutor_SafeOutput(foc_motor_t *motor)
{
    Executor_SafeOutput(motor, 0U);
}

uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor,
                                     const sensor_data_t *sensor,
                                     float dt_sec)
{
    if ((motor == 0) || (sensor == 0)) return FOC_CYCLE_SKIPPED;

    /* 1. 传感器有效性检查 */
    if ((sensor->adc_valid == 0U) || (sensor->encoder_valid == 0U))
    {
        motor->state.sensor_invalid_consecutive++;
        motor->state.control_skip_count++;

        if (sensor->adc_valid == 0U)
            motor->state.last_fault_code = (uint8_t)FOC_FAULT_SENSOR_ADC_INVALID;
        else
            motor->state.last_fault_code = (uint8_t)FOC_FAULT_SENSOR_ENCODER_INVALID;

        if (motor->state.sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
        {
            return (uint8_t)FOC_CYCLE_FAULT_SENSOR;
        }
        return FOC_CYCLE_SKIPPED;
    }
    motor->state.sensor_invalid_consecutive = 0U;
    motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;

    /* 3. 欠压保护 */
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (sensor->vbus_voltage_filtered < FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_UNDERVOLTAGE;
        return (uint8_t)FOC_CYCLE_FAULT_UVLO;
    }
#endif

    /* 4. 电机是否使能 */
    if (motor->state.motor_enabled == 0U)
    {
        return FOC_CYCLE_SKIPPED;
    }

    /* 5. 检查控制模式变化（用于 PID 重置） */
    if (motor->state.control_mode != motor->mode_transition.prev_control_mode_check)
    {
        motor->mode_transition.prev_control_mode_check = motor->state.control_mode;
    }

    /* 6. 正常控制外环 */
    FOC_ControlExecutor_RunOuterLoop(motor, sensor, dt_sec);

    /* after first cycle: allow ISR current loop execution */
    motor->state.current_loop_ready = 1U;

    return FOC_CYCLE_OK;
}

/* ========== 外环统一入口 ========== */

void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor,
                                      const sensor_data_t *sensor,
                                      float dt_sec)
{
    uint8_t cur_mode;

    if ((motor == 0) || (sensor == 0)) return;

    cur_mode = motor->state.control_mode;

    /* 控制模式切换时重置 PID 状态 */
    if (motor->mode_transition.prev_control_mode_valid == 0U)
    {
        motor->mode_transition.prev_control_mode = cur_mode;
        motor->mode_transition.prev_control_mode_valid = 1U;
    }

    if (cur_mode != motor->mode_transition.prev_control_mode)
    {
        if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
        {
            FOC_ControlRebaseMechanicalAngleAccum(motor, sensor->mech_angle_rad.output_value);
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
                           sensor,
                           dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)

    FOC_SpeedAngleOuterLoopStep(motor,
                                &motor->speed_pid,
                                &motor->angle_pid,
                                motor->target_angle_rad,
                                motor->angle_position_speed_rad_s,
                                sensor,
                                dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)

    if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(motor,
                               &motor->speed_pid,
                               motor->speed_only_rad_s,
                               sensor,
                               dt_sec);
    }
    else if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
    {
        FOC_SpeedAngleOuterLoopStep(motor,
                                    &motor->speed_pid,
                                    &motor->angle_pid,
                                    motor->target_angle_rad,
                                    motor->angle_position_speed_rad_s,
                                    sensor,
                                    dt_sec);
    }
    else
    {
        return;
    }

#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif

    /* 齿槽补偿（运行时补偿） */
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    if (FOC_CoggingCalibIsBusy(motor) == 0U)
    {
        FOC_ControlApplyCoggingCompensation(motor,
                                            sensor->mech_angle_rad.output_value,
                                            motor->cogging_speed_ref_rad_s);
    }
#endif
}
