#include "L2/Control/foc_ctrl_entry.h"

#include <math.h>

#include "L2/Control/foc_ctrl_init.h"
#include "L2/Control/foc_ctrl_entry.h"
#include "L2/Control/foc_ctrl_cfg.h"
#include "L2/Control/foc_ctrl_outer_loop.h"
#include "L2/Control/foc_ctrl_current_loop.h"
#include "L2/Control/foc_ctrl_compensation.h"
#include "LS_Config/foc_config.h"

/* ========== 内部工具 ========== */

static void ResetPIDState(foc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/* 控制模式切换跟踪（per-file static，单电机优化；多电机扩展时需移入 motor） */
static uint8_t g_prev_control_mode = 0U;
static uint8_t g_prev_control_mode_valid = 0U;

/* ========== FOC_Control_Init ========== */

void FOC_Control_Init(foc_motor_t *motor)
{
    if (motor == 0) return;

    ResetPIDState(&motor->torque_current_pid);
    ResetPIDState(&motor->speed_pid);
    ResetPIDState(&motor->angle_pid);

    motor->current_soft_switch_status.enabled = 0U;
    motor->current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    motor->current_soft_switch_status.blend_factor = 0.0f;
    motor->current_soft_switch_blend_initialized = 0U;

    g_prev_control_mode_valid = 0U;
}

/* ========== FOC_Control_ApplyConfig ========== */

void FOC_Control_ApplyConfig(foc_motor_t *motor)
{
    float phase_res;
    float i_max;

    if (motor == 0) return;

    /* 计算限幅 */
    phase_res = (fabsf(motor->phase_resistance) > 1e-6f) ? fabsf(motor->phase_resistance) : 1e-6f;
    i_max = motor->set_voltage / phase_res;
    if (i_max < 0.0f) i_max = 0.0f;

    /* 初始化 PID */
    FOC_PIDInit(&motor->torque_current_pid,
                motor->cfg.pid_current_kp,
                motor->cfg.pid_current_ki,
                motor->cfg.pid_current_kd,
                -motor->set_voltage, motor->set_voltage);

    FOC_PIDInit(&motor->angle_pid,
                motor->cfg.pid_angle_kp,
                motor->cfg.pid_angle_ki,
                motor->cfg.pid_angle_kd,
                -i_max, i_max);

    FOC_PIDInit(&motor->speed_pid,
                motor->cfg.pid_speed_kp,
                motor->cfg.pid_speed_ki,
                motor->cfg.pid_speed_kd,
                -i_max, i_max);

    /* 应用 fine-tuning 参数 */
    motor->control_runtime_cfg.min_mech_angle_accum_delta_rad = motor->cfg.min_mech_angle_accum_delta_rad;
    motor->control_runtime_cfg.angle_hold_integral_limit = motor->cfg.angle_hold_integral_limit;
    motor->control_runtime_cfg.angle_hold_pid_deadband_rad = motor->cfg.angle_hold_pid_deadband_rad;
    motor->control_runtime_cfg.speed_angle_transition_start_rad = motor->cfg.speed_angle_transition_start_rad;
    motor->control_runtime_cfg.speed_angle_transition_end_rad = motor->cfg.speed_angle_transition_end_rad;

    /* 应用电流软切换 */
    motor->current_soft_switch_status.enabled = motor->cfg.current_soft_switch_enable;
    motor->current_soft_switch_status.configured_mode = motor->cfg.current_soft_switch_mode;
    motor->current_soft_switch_status.auto_open_iq_a = motor->cfg.current_soft_switch_auto_open_iq_a;
    motor->current_soft_switch_status.auto_closed_iq_a = motor->cfg.current_soft_switch_auto_closed_iq_a;

    /* 应用齿槽补偿 */
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    motor->cogging_comp_status.enabled = motor->cfg.cogging_comp_enable;
    motor->cogging_comp_status.iq_limit_a = motor->cfg.cogging_comp_iq_limit_a;
    motor->cogging_comp_status.speed_gate_rad_s = motor->cfg.cogging_comp_speed_gate_rad_s;
    motor->cogging_comp_status.calib_gain_k = motor->cfg.cogging_calib_gain_k;
#endif
}

/* ========== FOC_Control_CurrentLoopRequiresSample ========== */

uint8_t FOC_Control_CurrentLoopRequiresSample(void)
{
    return FOC_ControlRequiresCurrentSample();
}

/* ========== FOC_Control_CurrentLoop ========== */

void FOC_Control_CurrentLoop(foc_motor_t *motor,
                             const sensor_data_t *sensor,
                             float electrical_angle,
                             float dt_sec)
{
    if ((motor == 0) || (sensor == 0)) return;

    FOC_CurrentControlStep(motor,
                           &motor->torque_current_pid,
                           sensor,
                           electrical_angle,
                           dt_sec);
}

/* ========== FOC_Control_Run（外环统一入口） ========== */

void FOC_Control_Run(foc_motor_t *motor,
                     const sensor_data_t *sensor,
                     float dt_sec)
{
    uint8_t cur_mode;

    if ((motor == 0) || (sensor == 0)) return;

    cur_mode = motor->cfg.control_mode;

    /* 控制模式切换时重置 PID 状态 */
    if (g_prev_control_mode_valid == 0U)
    {
        g_prev_control_mode = cur_mode;
        g_prev_control_mode_valid = 1U;
    }

    if (cur_mode != g_prev_control_mode)
    {
        if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
        {
            FOC_ControlRebaseMechanicalAngleAccum(motor, sensor->mech_angle_rad.output_value);
        }

        ResetPIDState(&motor->torque_current_pid);
        ResetPIDState(&motor->speed_pid);
        ResetPIDState(&motor->angle_pid);
        FOC_ControlResetSpeedLoopState();

        motor->current_soft_switch_status.enabled = 0U;
        motor->current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        motor->current_soft_switch_blend_initialized = 0U;

        g_prev_control_mode = cur_mode;
    }

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)

    FOC_SpeedOuterLoopStep(motor,
                           &motor->speed_pid,
                           motor->cfg.speed_only_rad_s,
                           sensor,
                           dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)

    FOC_SpeedAngleOuterLoopStep(motor,
                                &motor->speed_pid,
                                &motor->angle_pid,
                                motor->cfg.target_angle_rad,
                                motor->cfg.angle_position_speed_rad_s,
                                sensor,
                                dt_sec);

#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)

    if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(motor,
                               &motor->speed_pid,
                               motor->cfg.speed_only_rad_s,
                               sensor,
                               dt_sec);
    }
    else if (cur_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
    {
        FOC_SpeedAngleOuterLoopStep(motor,
                                    &motor->speed_pid,
                                    &motor->angle_pid,
                                    motor->cfg.target_angle_rad,
                                    motor->cfg.angle_position_speed_rad_s,
                                    sensor,
                                    dt_sec);
    }
    else
    {
        return; /* 未识别的控制模式 */
    }

#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif

    /* 齿槽补偿（运行时补偿，不是标定） */
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    if (FOC_CoggingCalibIsBusy(motor) == 0U)
    {
        FOC_ControlApplyCoggingCompensation(motor,
                                            sensor->mech_angle_rad.output_value,
                                            motor->cogging_speed_ref_rad_s);
    }
#endif
}

/* ========== FOC_Control_OpenLoopStep ========== */

void FOC_Control_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed)
{
    FOC_CurrentControlOpenLoopStep(motor, voltage, turn_speed, FOC_CONTROL_DT_SEC);
}

/* ========== 齿槽补偿标定接口 ========== */

uint8_t FOC_Control_CoggingCalibIsBusy(const foc_motor_t *motor)
{
    return FOC_CoggingCalibIsBusy(motor);
}

uint8_t FOC_Control_CoggingCalibSampleStep(foc_motor_t *motor,
                                           const sensor_data_t *sensor,
                                           float dt_sec)
{
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    return FOC_CoggingCalibSampleStep(motor, sensor, dt_sec);
#else
    (void)motor;
    (void)sensor;
    (void)dt_sec;
    return 0U;
#endif
}

void FOC_Control_CoggingCalibRequestStart(foc_motor_t *motor)
{
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE) && (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    FOC_CoggingCalibRequestStart(motor);
#else
    (void)motor;
#endif
}

void FOC_Control_CoggingCalibDumpTable(const foc_motor_t *motor)
{
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE) && (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    FOC_CoggingCalibDumpTable(motor);
#else
    (void)motor;
#endif
}

void FOC_Control_CoggingCalibExportTable(const foc_motor_t *motor)
{
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE) && (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    FOC_CoggingCalibExportTable(motor);
#else
    (void)motor;
#endif
}