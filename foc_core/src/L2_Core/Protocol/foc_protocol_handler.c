#include "L2_Core/Protocol/foc_protocol_handler.h"

#include <stdio.h>
#include <string.h>

#include "L2_Core/Protocol/foc_protocol_output.h"
#include "L2_Core/Protocol/foc_protocol_parser.h"
#include "L2_Core/Control/foc_ctrl_sens_cogging_calib.h"
#include "L2_Core/Control/foc_ctrl_sens_reinit.h"
#include "L2_Core/Runtime/foc_queue.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* ========== 内部状态（指向 L1 遥测策略） ========== */
static telemetry_policy_snapshot_t *g_telemetry_ptr = 0;

/* ========== 内部工具函数 ========== */

static uint8_t IsInRange(float value, float min_value, float max_value)
{
    return (value >= min_value && value <= max_value) ? 1U : 0U;
}

/* ========== P 命令组参数读写（运行参数） ========== */

static uint8_t WriteParam(foc_motor_t *motor, char subcommand, float value)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        if (IsInRange(value, COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD, COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD) == 0U) return 0U;
        motor->target_angle_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        if (IsInRange(value, COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S, COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S) == 0U) return 0U;
        motor->angle_position_speed_rad_s = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        if (IsInRange(value, COMMAND_MANAGER_PARAM_SPEED_ONLY_MIN_RAD_S, COMMAND_MANAGER_PARAM_SPEED_ONLY_MAX_RAD_S) == 0U) return 0U;
        motor->speed_only_rad_s = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        return 0U;

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        if (value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ || value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ) return 0U;
        { telemetry_policy_snapshot_t *t = (telemetry_policy_snapshot_t *)g_telemetry_ptr; if (t != 0) t->semantic_report_freq_hz = (uint16_t)value; }
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        if (value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ || value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ) return 0U;
        { telemetry_policy_snapshot_t *t = (telemetry_policy_snapshot_t *)g_telemetry_ptr; if (t != 0) t->osc_report_freq_hz = (uint16_t)value; }
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        if (value < COMMAND_MANAGER_PARAM_OSC_MASK_MIN || value > COMMAND_MANAGER_PARAM_OSC_MASK_MAX) return 0U;
        { telemetry_policy_snapshot_t *t = (telemetry_policy_snapshot_t *)g_telemetry_ptr; if (t != 0) t->osc_parameter_mask = (uint16_t)value; }
        break;
#endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CONTROL_MODE_MIN) || (value > COMMAND_MANAGER_PARAM_CONTROL_MODE_MAX)) return 0U;
        motor->state.control_mode = (uint8_t)value;
        break;

    default:
        return 0U;
    }
    motor->state.cfg_dirty = 1U;
    return 1U;
}

static uint8_t ReadParam(const foc_motor_t *motor, char subcommand, float *value_out)
{
    if (value_out == 0) return 0U;

    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        *value_out = motor->target_angle_rad; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        *value_out = motor->angle_position_speed_rad_s; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        *value_out = motor->speed_only_rad_s; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        return 0U;
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        { const telemetry_policy_snapshot_t *t = g_telemetry_ptr; *value_out = (t != 0) ? (float)t->semantic_report_freq_hz : 0.0f; } break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        { const telemetry_policy_snapshot_t *t = g_telemetry_ptr; *value_out = (t != 0) ? (float)t->osc_report_freq_hz : 0.0f; } break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        { const telemetry_policy_snapshot_t *t = g_telemetry_ptr; *value_out = (t != 0) ? (float)t->osc_parameter_mask : 0.0f; } break;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        *value_out = (float)motor->state.control_mode; break;
    default:
        return 0U;
    }
    return 1U;
}

/* ========== C 命令组参数读写（调优/配置参数） ========== */

static uint8_t WriteConfigParam(foc_motor_t *motor, char subcommand, float value)
{
    switch (subcommand)
    {
    /* PID 电流环 */
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KP:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN, COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U) return 0U;
        motor->torque_current_pid.kp = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KI:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN, COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U) return 0U;
        motor->torque_current_pid.ki = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KD:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MIN, COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MAX) == 0U) return 0U;
        motor->torque_current_pid.kd = value;
        break;
#else
        return 0U;
#endif

    /* PID 角度环 */
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KP:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN, COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U) return 0U;
        motor->angle_pid.kp = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KI:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN, COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U) return 0U;
        motor->angle_pid.ki = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KD:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MIN, COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MAX) == 0U) return 0U;
        motor->angle_pid.kd = value;
        break;
#else
        return 0U;
#endif

    /* PID 速度环 */
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KP:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN, COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U) return 0U;
        motor->speed_pid.kp = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KI:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN, COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U) return 0U;
        motor->speed_pid.ki = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KD:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_SPEED_KD_MIN, COMMAND_MANAGER_PARAM_PID_SPEED_KD_MAX) == 0U) return 0U;
        motor->speed_pid.kd = value;
        break;
#else
        return 0U;
#endif

    /* 控制 fine-tuning */
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_MIN_MECH_DELTA:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->min_mech_angle_accum_delta_rad = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_I_LIMIT:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->angle_hold_integral_limit = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_DEADBAND:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->angle_hold_pid_deadband_rad = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_START:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->speed_angle_transition_start_rad = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_END:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->speed_angle_transition_end_rad = value;
        break;
#else
        return 0U;
#endif

    /* 齿槽补偿 */
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_IQ_LIMIT:
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_COGGING_COMP_IQ_LIMIT_MIN_A, COMMAND_MANAGER_PARAM_COGGING_COMP_IQ_LIMIT_MAX_A) == 0U) return 0U;
        motor->cogging_comp_status.iq_limit_a = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_SPEED_GATE:
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_COGGING_COMP_SPEED_GATE_MIN_RAD_S, COMMAND_MANAGER_PARAM_COGGING_COMP_SPEED_GATE_MAX_RAD_S) == 0U) return 0U;
        motor->cogging_comp_status.speed_gate_rad_s = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_CALIB_GAIN:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->cogging_comp_status.calib_gain_k = value;
        break;
#else
        return 0U;
#endif

    /* 电流软切换 */
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MIN) || (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MAX)) return 0U;
        motor->current_soft_switch_status.configured_mode = (uint8_t)value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MIN_A) || (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MAX_A) || (value > motor->current_soft_switch_status.auto_closed_iq_a)) return 0U;
        motor->current_soft_switch_status.auto_open_iq_a = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MIN_A) || (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MAX_A) || (value < motor->current_soft_switch_status.auto_open_iq_a)) return 0U;
        motor->current_soft_switch_status.auto_closed_iq_a = value;
        break;
#else
        return 0U;
#endif

    default:
        return 0U;
    }
    motor->state.cfg_dirty = 1U;
    return 1U;
}

static uint8_t ReadConfigParam(const foc_motor_t *motor, char subcommand, float *value_out)
{
    if (value_out == 0) return 0U;

    switch (subcommand)
    {
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KP:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->torque_current_pid.kp; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KI:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->torque_current_pid.ki; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KD:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->torque_current_pid.kd; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KP:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_pid.kp; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KI:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_pid.ki; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KD:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_pid.kd; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KP:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_pid.kp; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KI:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_pid.ki; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KD:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_pid.kd; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_MIN_MECH_DELTA:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->min_mech_angle_accum_delta_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_I_LIMIT:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_hold_integral_limit; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_DEADBAND:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_hold_pid_deadband_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_START:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_angle_transition_start_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_END:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_angle_transition_end_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_IQ_LIMIT:
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        *value_out = motor->cogging_comp_status.iq_limit_a; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_SPEED_GATE:
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        *value_out = motor->cogging_comp_status.speed_gate_rad_s; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_CALIB_GAIN:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        *value_out = motor->cogging_comp_status.calib_gain_k; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        *value_out = (float)motor->current_soft_switch_status.configured_mode; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        *value_out = motor->current_soft_switch_status.auto_open_iq_a; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        *value_out = motor->current_soft_switch_status.auto_closed_iq_a; break;
#else
        return 0U;
#endif
    default:
        return 0U;
    }
    return 1U;
}

/* ========== 状态读写 ========== */

static uint8_t WriteState(foc_motor_t *motor, char subcommand, uint8_t state)
{
    uint8_t normalized = (state != 0U) ? 1U : 0U;
    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        motor->state.motor_enabled = normalized; break;
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        { telemetry_policy_snapshot_t *t = (telemetry_policy_snapshot_t *)g_telemetry_ptr; if (t != 0) t->semantic_report_enabled = normalized; } break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        { telemetry_policy_snapshot_t *t = (telemetry_policy_snapshot_t *)g_telemetry_ptr; if (t != 0) t->osc_report_enabled = normalized; } break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        motor->cogging_comp_status.enabled = normalized;
        motor->state.cfg_dirty = 1U;
        break;
#else
        return 0U;
#endif
    default:
        return 0U;
    }
    return 1U;
}

static uint8_t ReadState(const foc_motor_t *motor, char subcommand, uint8_t *state_out)
{
    if (state_out == 0) return 0U;
    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        *state_out = motor->state.motor_enabled; break;
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        { const telemetry_policy_snapshot_t *t = g_telemetry_ptr; *state_out = (t != 0) ? t->semantic_report_enabled : 0; } break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        { const telemetry_policy_snapshot_t *t = g_telemetry_ptr; *state_out = (t != 0) ? t->osc_report_enabled : 0; } break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        *state_out = motor->cogging_comp_status.enabled; break;
#else
        return 0U;
#endif
    default:
        return 0U;
    }
    return 1U;
}


static uint8_t ReportSingleParam(const foc_motor_t *motor, char subcommand)
{
    float value;
    if (ReadParam(motor, subcommand, &value) == 0U) return 0U;
    FOC_Protocol_OutputParam(subcommand, value);
    return 1U;
}

static uint8_t ReportSingleConfig(const foc_motor_t *motor, char subcommand)
{
    float value;
    if (ReadConfigParam(motor, subcommand, &value) == 0U) return 0U;
    FOC_Protocol_OutputConfigParam(subcommand, value);
    return 1U;
}

static uint8_t ReportSingleState(const foc_motor_t *motor, char subcommand)
{
    uint8_t state_val;
    if (ReadState(motor, subcommand, &state_val) == 0U) return 0U;
    FOC_Protocol_OutputState(subcommand, state_val);
    return 1U;
}

/* ========== P 命令执行 ========== */

static foc_protocol_frame_result_t ExecutePCommand(foc_motor_t *motor, const protocol_command_t *cmd)
{
    foc_protocol_frame_result_t res = {0, 0, 0, 0};
    float value = 0.0f;

    if (cmd->has_param != 0U)
    {
        if (WriteParam(motor, cmd->subcommand, cmd->param_value) == 0U)
        {
            FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            res.needs_status = 1U;
            return res;
        }
        if (ReadParam(motor, cmd->subcommand, &value) != 0U)
            FOC_Protocol_OutputParam(cmd->subcommand, value);
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        res.param_changed = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_PARAM_SUBCMD_READ_ALL)
    {
        res.needs_param_dump = 1U;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (ReportSingleParam(motor, cmd->subcommand) == 0U)
    {
        FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        res.needs_status = 1U;
        return res;
    }

    FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
    res.comm_active  = 1U;
    res.needs_status = 1U;
    return res;
}

/* ========== C 命令执行 ========== */

static foc_protocol_frame_result_t ExecuteCCommand(foc_motor_t *motor, const protocol_command_t *cmd)
{
    foc_protocol_frame_result_t res = {0, 0, 0, 0};
    float value = 0.0f;

    if (cmd->has_param != 0U)
    {
        if (WriteConfigParam(motor, cmd->subcommand, cmd->param_value) == 0U)
        {
            FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            res.needs_status = 1U;
            return res;
        }
        if (ReadConfigParam(motor, cmd->subcommand, &value) != 0U)
            FOC_Protocol_OutputConfigParam(cmd->subcommand, value);
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        res.param_changed = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_CONFIG_SUBCMD_READ_ALL)
    {
        res.needs_config_dump = 1U;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (ReportSingleConfig(motor, cmd->subcommand) == 0U)
    {
        FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        res.needs_status = 1U;
        return res;
    }

    FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
    res.comm_active  = 1U;
    res.needs_status = 1U;
    return res;
}

/* ========== S 命令执行 ========== */

static foc_protocol_frame_result_t ExecuteSCommand(foc_motor_t *motor, const protocol_command_t *cmd)
{
    foc_protocol_frame_result_t res = {0, 0, 0, 0};
    uint8_t state_val = 0U;

    if (cmd->has_param != 0U)
    {
        if (ProtocolCore_ParseStateValue(cmd->param_value, &state_val) == 0U)
        {
            FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            res.needs_status = 1U;
            return res;
        }
        if (WriteState(motor, cmd->subcommand, state_val) == 0U)
        {
            FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            res.needs_status = 1U;
            return res;
        }
        if (ReadState(motor, cmd->subcommand, &state_val) == 0U)
        {
            FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
            res.needs_status = 1U;
            return res;
        }
        FOC_Protocol_OutputState(cmd->subcommand, state_val);
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active   = 1U;
        res.needs_status  = 1U;
        res.param_changed = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_STATE_SUBCMD_READ_ALL)
    {
        res.needs_state_dump = 1U;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (ReportSingleState(motor, cmd->subcommand) == 0U)
    {
        FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        res.needs_status = 1U;
        return res;
    }

    FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
    res.comm_active  = 1U;
    res.needs_status = 1U;
    return res;
}

/* ========== Y 命令执行 ========== */

static foc_protocol_frame_result_t HandleSystemCommand(foc_motor_t *motor, const protocol_command_t *cmd)
{
    foc_protocol_frame_result_t res = {0, 0, 0, 0};

    if (cmd->has_param != 0U)
    {
        FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
        res.needs_status = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY)
    {
        /* L1 根据 needs_summary 标志生成摘要文本并入 TX 队列 */
        res.comm_active   = 1U;
        res.needs_summary = 1U;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.needs_status  = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT)
    {
        motor->state.sensor_invalid_consecutive = 0U;
        motor->state.protocol_error_count = 0U;
        motor->state.param_error_count = 0U;
        motor->state.control_skip_count = 0U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
        motor->state.system_fault = 0U;
        motor->state.cfg_dirty = 1U;
        motor->state.motor_enabled = (uint8_t)COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;
        motor->state.current_loop_ready = 0U;
        motor->state.control_phase = FOC_CONTROL_PHASE_NORMAL;

        FOC_Protocol_OutputDiag("INFO", "fault_recovery", "system reset completed");
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_REINIT)
    {
#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
        FOC_ReInit_Request(motor);
#endif
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_CALIB)
    {
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        FOC_CoggingCalib_RequestStart(motor);
#endif
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_DUMP)
    {
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        FOC_CoggingCalib_RequestDump(motor);
#endif
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_EXPORT)
    {
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        FOC_CoggingCalib_RequestExport(motor);
#endif
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_INFO)
    {
        res.needs_system_info = 1U;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
        res.comm_active  = 1U;
        res.needs_status = 1U;
        return res;
    }

    FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
    res.needs_status = 1U;
    return res;
}

/* ========== 单帧解析与分发 ========== */

static foc_protocol_frame_result_t ParseAndDispatchFrame(foc_motor_t *motor, const uint8_t *frame, uint16_t len)
{
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    protocol_core_frame_parse_result_t parse_result;
    protocol_command_t command;
    foc_protocol_frame_result_t res = {0, 0, 0, 0};

    if (len == 0U) return res;
    if (ProtocolCore_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        motor->state.protocol_error_count++;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_PROTOCOL_FRAME;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_FRAME_ERROR_CHAR);
        res.needs_status = 1U;
        return res;
    }

    parse_result = ProtocolCore_ParseFrame(payload, payload_len, &command);
    if (parse_result == PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH) return res;
    if (parse_result != PROTOCOL_CORE_FRAME_PARSE_OK)
    {
        motor->state.protocol_error_count++;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_PROTOCOL_FRAME;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_FRAME_ERROR_CHAR);
        res.needs_status = 1U;
        return res;
    }

    if (command.command == COMMAND_MANAGER_CMD_SYSTEM) return HandleSystemCommand(motor, &command);
    if (command.command == COMMAND_MANAGER_CMD_PARAM)  return ExecutePCommand(motor, &command);
    if (command.command == COMMAND_MANAGER_CMD_CONFIG) return ExecuteCCommand(motor, &command);
    if (command.command == COMMAND_MANAGER_CMD_STATE)   return ExecuteSCommand(motor, &command);

    FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
    res.needs_status = 1U;
    return res;
}

/* =================================================================
 * 公开 API
 * ================================================================= */

void FOC_Protocol_Init(telemetry_policy_snapshot_t *telemetry)
{
    g_telemetry_ptr = telemetry;
    if (telemetry != 0)
    {
        telemetry->semantic_report_enabled = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
        telemetry->osc_report_enabled = COMMAND_MANAGER_DEFAULT_OSC_ENABLED;
        telemetry->semantic_report_freq_hz = COMMAND_MANAGER_DEFAULT_SEMANTIC_FREQ_HZ;
        telemetry->osc_report_freq_hz = COMMAND_MANAGER_DEFAULT_OSC_FREQ_HZ;
        telemetry->osc_parameter_mask = COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK;
    }
    FOC_Protocol_OutputDiag("INFO", "protocol", "READY");
}

foc_protocol_frame_result_t FOC_Protocol_ProcessSingle(
    foc_motor_t *motor,
    const uint8_t *frame,
    uint16_t len)
{
    foc_protocol_frame_result_t res = {0, 0, 0, 0};

    if ((motor == 0) || (frame == 0) || (len == 0U)) return res;

    res = ParseAndDispatchFrame(motor, frame, len);
    return res;
}

void FOC_Protocol_Commit(foc_motor_t *motor)
{
    motor->state.cfg_dirty = 0U;
}

const telemetry_policy_snapshot_t *FOC_Protocol_GetTelemetry(void)
{
    return g_telemetry_ptr;
}

/* ========== 批量队列输出（供 X 指令使用） ========== */

void FOC_Protocol_QueueParams(const foc_motor_t *motor, fifo_queue_t *tx_fifo)
{
    float value;
    const char params[] = {
        COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE,
        COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED,
        COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED,
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV,
        COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV,
        COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK,
#endif
        COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE,
    };
    uint16_t i;
    for (i = 0U; i < (uint16_t)(sizeof(params) / sizeof(params[0])); i++)
    {
        if (ReadParam(motor, params[i], &value) != 0U)
        {
            char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
            ProtocolText_FormatParamLine(line, sizeof(line), params[i], value);
            (void)FIFO_Enqueue(tx_fifo, (uint8_t *)line);
        }
    }
}

void FOC_Protocol_QueueConfigs(const foc_motor_t *motor, fifo_queue_t *tx_fifo)
{
    float value;
    const char configs[] = {
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KP,
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KI,
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KP,
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KI,
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KP,
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KI,
        COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_CFG_MIN_MECH_DELTA,
        COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_I_LIMIT,
        COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_DEADBAND,
        COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_START,
        COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_END,
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_IQ_LIMIT,
        COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_SPEED_GATE,
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_CALIB_GAIN,
#endif
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
        COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_MODE,
        COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ,
        COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ,
#endif
    };
    uint16_t i;
    for (i = 0U; i < (uint16_t)(sizeof(configs) / sizeof(configs[0])); i++)
    {
        if (ReadConfigParam(motor, configs[i], &value) != 0U)
        {
            char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
            ProtocolText_FormatConfigLine(line, sizeof(line), configs[i], value);
            (void)FIFO_Enqueue(tx_fifo, (uint8_t *)line);
        }
    }
}

void FOC_Protocol_QueueStates(const foc_motor_t *motor, fifo_queue_t *tx_fifo)
{
    uint8_t state_val;
    const char states[] = {
        COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE,
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE,
        COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE,
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE,
#endif
    };
    uint16_t i;
    for (i = 0U; i < (uint16_t)(sizeof(states) / sizeof(states[0])); i++)
    {
        if (ReadState(motor, states[i], &state_val) != 0U)
        {
            char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
            ProtocolText_FormatStateLine(line, sizeof(line), states[i], state_val);
            (void)FIFO_Enqueue(tx_fifo, (uint8_t *)line);
        }
    }
}

void FOC_Protocol_QueueSystemInfo(const foc_motor_t *motor, fifo_queue_t *tx_fifo)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    if (motor == 0) return;

    snprintf(out, sizeof(out), "system.phase_resistance=%.3f\r\n", motor->phase_resistance);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.pole_pairs=%u\r\n", (unsigned int)motor->pole_pairs);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.source.active=%u\r\n",
             (unsigned int)motor->source_mgr_state.active_source);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.source.standby=%u\r\n",
             (unsigned int)motor->source_mgr_state.standby_source);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.source.region=%u\r\n",
             (unsigned int)motor->source_mgr_state.control_region);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.source.switching=%u\r\n",
             (unsigned int)motor->source_mgr_state.switch_in_progress);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.active_source.valid=%u\r\n",
             (unsigned int)motor->active_source_state.valid);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    snprintf(out, sizeof(out), "system.openloop.source_state=%u\r\n",
             (unsigned int)motor->openloop_angle_source_state.phase);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.openloop.policy_state=%u\r\n",
             (unsigned int)motor->openloop_low_speed_policy_state.phase);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);
#endif

    snprintf(out, sizeof(out), "system.mech_zero_rad=%.3f\r\n", motor->mech_angle_at_elec_zero_rad);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    {
        const char *dir_str = "UNDEFINED";
        if (motor->direction == FOC_DIR_NORMAL)
            dir_str = "NORMAL";
        else if (motor->direction == FOC_DIR_REVERSED)
            dir_str = "REVERSED";
        snprintf(out, sizeof(out), "system.direction=%s\r\n", dir_str);
        (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);
    }

    snprintf(out, sizeof(out), "system.vbus_voltage_v=%.3f\r\n", motor->vbus_voltage);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.max_phase_voltage_v=%.3f\r\n", motor->max_phase_voltage);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.zero_offset_a=%.3f\r\n", motor->sensor_zero_offset_a);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

    snprintf(out, sizeof(out), "system.zero_offset_b=%.3f\r\n", motor->sensor_zero_offset_b);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    {
        const char *src_str = "DISABLED";
        if (motor->cogging_comp_status.source == FOC_COGGING_COMP_SOURCE_NONE)
            src_str = "NONE";
        else if (motor->cogging_comp_status.source == FOC_COGGING_COMP_SOURCE_STATIC)
            src_str = "STATIC";
        else if (motor->cogging_comp_status.source == FOC_COGGING_COMP_SOURCE_CALIB)
            src_str = "CALIB";
        snprintf(out, sizeof(out), "system.cogging=%s\r\n", src_str);
        (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);
    }
    snprintf(out, sizeof(out), "system.cogging_point_count=%u\r\n",
             (unsigned int)motor->cogging_comp_status.point_count);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);
    snprintf(out, sizeof(out), "system.cogging_iq_lsb_a=%.5f\r\n", motor->cogging_comp_status.iq_lsb_a);
    (void)FIFO_Enqueue(tx_fifo, (uint8_t *)out);
#endif
}
