#include "L2/Protocol/foc_protocol_handler.h"

#include <stdio.h>
#include <string.h>

#include "L2/Protocol/foc_protocol_output.h"
#include "L2/Protocol/foc_protocol_parser.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"
#include "L2/Protocol/foc_snapshot_types.h"

/* ========== 内部常量 ========== */
#define RUNTIME_COMM_SOURCE_1 1U
#define RUNTIME_COMM_SOURCE_2 2U
#define RUNTIME_COMM_SOURCE_3 3U
#define RUNTIME_COMM_SOURCE_4 4U

/* ========== 内部状态（指向 L1 系统配置中的遥测策略） ========== */
static telemetry_policy_snapshot_t *g_telemetry_ptr = 0;

/* ========== 内部工具函数 ========== */

static uint8_t IsInRange(float value, float min_value, float max_value)
{
    return (value >= min_value && value <= max_value) ? 1U : 0U;
}

/* ========== 帧读取（原 FrameSource_*） ========== */

static uint8_t FrameSource_IsReady(uint8_t source)
{
    if (source == RUNTIME_COMM_SOURCE_1) return FOC_Platform_CommSource1_IsFrameReady();
    if (source == RUNTIME_COMM_SOURCE_2) return FOC_Platform_CommSource2_IsFrameReady();
    if (source == RUNTIME_COMM_SOURCE_3) return FOC_Platform_CommSource3_IsFrameReady();
    if (source == RUNTIME_COMM_SOURCE_4) return FOC_Platform_CommSource4_IsFrameReady();
    return 0U;
}

static uint16_t FrameSource_Read(uint8_t source, uint8_t *buffer, uint16_t max_len)
{
    if (source == RUNTIME_COMM_SOURCE_1) return FOC_Platform_CommSource1_ReadFrame(buffer, max_len);
    if (source == RUNTIME_COMM_SOURCE_2) return FOC_Platform_CommSource2_ReadFrame(buffer, max_len);
    if (source == RUNTIME_COMM_SOURCE_3) return FOC_Platform_CommSource3_ReadFrame(buffer, max_len);
    if (source == RUNTIME_COMM_SOURCE_4) return FOC_Platform_CommSource4_ReadFrame(buffer, max_len);
    return 0U;
}

static uint16_t FrameSource_TryReadReady(uint8_t *buffer, uint16_t max_len)
{
    uint8_t source;
    for (source = RUNTIME_COMM_SOURCE_1; source <= RUNTIME_COMM_SOURCE_4; source++)
    {
        if (FrameSource_IsReady(source) == 0U) continue;
        uint16_t len = FrameSource_Read(source, buffer, max_len);
        if (len > 0U) return len;
    }
    return 0U;
}

/* ========== 参数读写（直接写 motor 顶层字段） ========== */
/* cfg 已消除，所有字段直接写入目标位置 */

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
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_SENSOR_SAMPLE_OFFSET_MIN_PERCENT, COMMAND_MANAGER_PARAM_SENSOR_SAMPLE_OFFSET_MAX_PERCENT) == 0U) return 0U;
        motor->sensor_sample_offset_percent = value;
        break;
#else
        return 0U;
#endif

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        if (value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ || value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ) return 0U;
        if (g_telemetry_ptr != 0) g_telemetry_ptr->semantic_report_freq_hz = (uint16_t)value;
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        if (value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ || value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ) return 0U;
        if (g_telemetry_ptr != 0) g_telemetry_ptr->osc_report_freq_hz = (uint16_t)value;
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        if (value < COMMAND_MANAGER_PARAM_OSC_MASK_MIN || value > COMMAND_MANAGER_PARAM_OSC_MASK_MAX) return 0U;
        if (g_telemetry_ptr != 0) g_telemetry_ptr->osc_parameter_mask = (uint16_t)value;
        break;
#endif

    /* PID 参数直接写入 PID 对象 */
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN, COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U) return 0U;
        motor->torque_current_pid.kp = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN, COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U) return 0U;
        motor->torque_current_pid.ki = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MIN, COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MAX) == 0U) return 0U;
        motor->torque_current_pid.kd = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN, COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U) return 0U;
        motor->angle_pid.kp = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN, COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U) return 0U;
        motor->angle_pid.ki = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MIN, COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MAX) == 0U) return 0U;
        motor->angle_pid.kd = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN, COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U) return 0U;
        motor->speed_pid.kp = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN, COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U) return 0U;
        motor->speed_pid.ki = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        if (IsInRange(value, COMMAND_MANAGER_PARAM_PID_SPEED_KD_MIN, COMMAND_MANAGER_PARAM_PID_SPEED_KD_MAX) == 0U) return 0U;
        motor->speed_pid.kd = value;
        break;
#else
        return 0U;
#endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->min_mech_angle_accum_delta_rad = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->angle_hold_integral_limit = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->angle_hold_pid_deadband_rad = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->speed_angle_transition_start_rad = value;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        if (value < 0.0f) return 0U;
        motor->speed_angle_transition_end_rad = value;
        break;
#else
        return 0U;
#endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CONTROL_MODE_MIN) || (value > COMMAND_MANAGER_PARAM_CONTROL_MODE_MAX)) return 0U;
        motor->state.control_mode = (uint8_t)value;
        break;

    /* 齿槽补偿参数 */
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_IQ_LIMIT:
        if (IsInRange(value, COMMAND_MANAGER_PARAM_COGGING_COMP_IQ_LIMIT_MIN_A, COMMAND_MANAGER_PARAM_COGGING_COMP_IQ_LIMIT_MAX_A) == 0U) return 0U;
        motor->cogging_comp_status.iq_limit_a = value;
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_SPEED_GATE:
        if (IsInRange(value, COMMAND_MANAGER_PARAM_COGGING_COMP_SPEED_GATE_MIN_RAD_S, COMMAND_MANAGER_PARAM_COGGING_COMP_SPEED_GATE_MAX_RAD_S) == 0U) return 0U;
        motor->cogging_comp_status.speed_gate_rad_s = value;
        break;
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_CALIB_GAIN:
        if (value < 0.0f) return 0U;
        motor->cogging_comp_status.calib_gain_k = value;
        break;
#endif

    /* 电流软切换参数 */
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MIN) || (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MAX)) return 0U;
        motor->current_soft_switch_status.configured_mode = (uint8_t)value;
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MIN_A) || (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MAX_A) || (value > motor->current_soft_switch_status.auto_closed_iq_a)) return 0U;
        motor->current_soft_switch_status.auto_open_iq_a = value;
        break;
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MIN_A) || (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MAX_A) || (value < motor->current_soft_switch_status.auto_open_iq_a)) return 0U;
        motor->current_soft_switch_status.auto_closed_iq_a = value;
        break;
#endif

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
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
        *value_out = motor->sensor_sample_offset_percent; break;
#else
        return 0U;
#endif
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        *value_out = (g_telemetry_ptr != 0) ? (float)g_telemetry_ptr->semantic_report_freq_hz : 0.0f; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        *value_out = (g_telemetry_ptr != 0) ? (float)g_telemetry_ptr->osc_report_freq_hz : 0.0f; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        *value_out = (g_telemetry_ptr != 0) ? (float)g_telemetry_ptr->osc_parameter_mask : 0.0f; break;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->torque_current_pid.kp; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->torque_current_pid.ki; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->torque_current_pid.kd; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_pid.kp; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_pid.ki; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_pid.kd; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_pid.kp; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_pid.ki; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_pid.kd; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->min_mech_angle_accum_delta_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_hold_integral_limit; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->angle_hold_pid_deadband_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_angle_transition_start_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        *value_out = motor->speed_angle_transition_end_rad; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        *value_out = (float)motor->state.control_mode; break;
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_IQ_LIMIT:
        *value_out = motor->cogging_comp_status.iq_limit_a; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_SPEED_GATE:
        *value_out = motor->cogging_comp_status.speed_gate_rad_s; break;
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_CALIB_GAIN:
        *value_out = motor->cogging_comp_status.calib_gain_k; break;
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        *value_out = (float)motor->current_soft_switch_status.configured_mode; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        *value_out = motor->current_soft_switch_status.auto_open_iq_a; break;
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        *value_out = motor->current_soft_switch_status.auto_closed_iq_a; break;
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
        if (g_telemetry_ptr != 0) g_telemetry_ptr->semantic_report_enabled = normalized; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        if (g_telemetry_ptr != 0) g_telemetry_ptr->osc_report_enabled = normalized; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        motor->current_soft_switch_status.enabled = normalized;
        motor->state.cfg_dirty = 1U;
        break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
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
        *state_out = (g_telemetry_ptr != 0) ? g_telemetry_ptr->semantic_report_enabled : 0; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        *state_out = (g_telemetry_ptr != 0) ? g_telemetry_ptr->osc_report_enabled : 0; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        *state_out = motor->current_soft_switch_status.enabled; break;
#else
        return 0U;
#endif
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
        *state_out = motor->cogging_comp_status.enabled; break;
#else
        return 0U;
#endif
    default:
        return 0U;
    }
    return 1U;
}

static void ReportAllParams(const foc_motor_t *motor) { /* ... unchanged ... */ float value; const char params[] = {
    COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE, COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED, COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED,
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET,
#endif
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV, COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV, COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP, COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI, COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP, COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI, COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP, COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI, COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA, COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT,
    COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND, COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START, COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END,
#endif
    COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE,
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_IQ_LIMIT, COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_SPEED_GATE,
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_COGGING_CALIB_GAIN,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE, COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ, COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ,
#endif
};
    uint16_t i;
    for (i = 0U; i < (uint16_t)(sizeof(params) / sizeof(params[0])); i++)
        if (ReadParam(motor, params[i], &value) != 0U) FOC_Protocol_OutputParam(params[i], value);
}

static void ReportAllStates(const foc_motor_t *motor) { uint8_t state_val; const char states[] = {
    COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE,
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE, COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE,
#endif
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE,
#endif
};
    uint16_t i;
    for (i = 0U; i < (uint16_t)(sizeof(states) / sizeof(states[0])); i++)
        if (ReadState(motor, states[i], &state_val) != 0U) FOC_Protocol_OutputState(states[i], state_val);
}

static uint8_t ReportSingleParam(const foc_motor_t *motor, char subcommand) {
    float value; if (ReadParam(motor, subcommand, &value) == 0U) return 0U; FOC_Protocol_OutputParam(subcommand, value); return 1U;
}

static uint8_t ReportSingleState(const foc_motor_t *motor, char subcommand) {
    uint8_t state_val; if (ReadState(motor, subcommand, &state_val) == 0U) return 0U; FOC_Protocol_OutputState(subcommand, state_val); return 1U;
}

/* ========== 命令执行 ========== */
static uint8_t ExecutePCommand(foc_motor_t *motor, const protocol_command_t *cmd) { /* ... unchanged ... */ float value = 0.0f; if (cmd->has_param != 0U) { if (WriteParam(motor, cmd->subcommand, cmd->param_value) == 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } if (ReadParam(motor, cmd->subcommand, &value) != 0U) FOC_Protocol_OutputParam(cmd->subcommand, value); FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_PARAM_SUBCMD_READ_ALL) { ReportAllParams(motor); FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (ReportSingleParam(motor, cmd->subcommand) == 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; }

static uint8_t ExecuteSCommand(foc_motor_t *motor, const protocol_command_t *cmd) { uint8_t state_val = 0U; if (cmd->has_param != 0U) { if (ProtocolCore_ParseStateValue(cmd->param_value, &state_val) == 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } if (WriteState(motor, cmd->subcommand, state_val) == 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } if (ReadState(motor, cmd->subcommand, &state_val) == 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } FOC_Protocol_OutputState(cmd->subcommand, state_val); FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_STATE_SUBCMD_READ_ALL) { ReportAllStates(motor); FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (ReportSingleState(motor, cmd->subcommand) == 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; }

static uint8_t HandleSystemCommand(foc_motor_t *motor, const protocol_command_t *cmd) { /* ... unchanged ... */ if (cmd->has_param != 0U) { FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR); return 0U; } if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_RUNTIME_SUMMARY) { char out[COMMAND_MANAGER_REPLY_BUFFER_LEN]; snprintf(out, sizeof(out), "STATE RUN=%u FLT=%u INIT=0x%04X/0x%04X SENS_INV=%u PROTO_ERR=%lu PARAM_ERR=%lu CTRL_SKIP=%lu\r\n", (unsigned int)motor->state.system_running, (unsigned int)motor->state.system_fault, (unsigned int)motor->state.init_check_mask, (unsigned int)motor->state.init_fail_mask, (unsigned int)motor->state.sensor_invalid_consecutive, (unsigned long)motor->state.protocol_error_count, (unsigned long)motor->state.param_error_count, (unsigned long)motor->state.control_skip_count); FOC_Protocol_WriteText(out); FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_FAULT_CLEAR_REINIT) { motor->state.sensor_invalid_consecutive = 0U; motor->state.protocol_error_count = 0U; motor->state.param_error_count = 0U; motor->state.control_skip_count = 0U; motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE; motor->state.system_running = 0U; motor->state.system_fault = 0U; motor->state.init_check_mask = 0U; motor->state.init_fail_mask = 0U; motor->state.pending_system_action = FOC_SYSACTION_NONE; motor->state.cfg_dirty = 1U; motor->state.motor_enabled = (uint8_t)COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE; FOC_Protocol_OutputDiag("INFO", "fault_recovery", "system reset completed"); FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_REINIT) { motor->state.reinit_pending = 1U; FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_CALIB) { motor->state.pending_system_action = FOC_SYSACTION_COGGING_START; FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_DUMP) { motor->state.pending_system_action = FOC_SYSACTION_COGGING_DUMP; FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } if (cmd->subcommand == COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_EXPORT) { motor->state.pending_system_action = FOC_SYSACTION_COGGING_EXPORT; FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR); return 1U; } FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR); return 0U; }

static uint8_t ParseAndDispatchFrame(foc_motor_t *motor, const uint8_t *frame, uint16_t len)
{
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    protocol_core_frame_parse_result_t parse_result;
    protocol_command_t command;

    if (len == 0U) return 0U;
    if (ProtocolCore_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        motor->state.protocol_error_count++;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_PROTOCOL_FRAME;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_FRAME_ERROR_CHAR);
        return 0U;
    }
    parse_result = ProtocolCore_ParseFrame(payload, payload_len, &command);
    if (parse_result == PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH) return 0U;
    if (parse_result != PROTOCOL_CORE_FRAME_PARSE_OK)
    {
        motor->state.protocol_error_count++;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_PROTOCOL_FRAME;
        FOC_Protocol_WriteStatus((uint8_t)FOC_PROTOCOL_STATUS_FRAME_ERROR_CHAR);
        return 0U;
    }
    if (command.command == COMMAND_MANAGER_CMD_SYSTEM) return HandleSystemCommand(motor, &command);
    if (command.command == COMMAND_MANAGER_CMD_PARAM) return ExecutePCommand(motor, &command);
    if (command.command == COMMAND_MANAGER_CMD_STATE) return ExecuteSCommand(motor, &command);
    FOC_Protocol_WriteStatus((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
    return 0U;
}

void FOC_Protocol_Init(telemetry_policy_snapshot_t *telemetry_cfg)
{
    g_telemetry_ptr = telemetry_cfg;
    if (g_telemetry_ptr != 0)
    {
        g_telemetry_ptr->semantic_report_enabled = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
        g_telemetry_ptr->osc_report_enabled = COMMAND_MANAGER_DEFAULT_OSC_ENABLED;
        g_telemetry_ptr->semantic_report_freq_hz = COMMAND_MANAGER_DEFAULT_SEMANTIC_FREQ_HZ;
        g_telemetry_ptr->osc_report_freq_hz = COMMAND_MANAGER_DEFAULT_OSC_FREQ_HZ;
        g_telemetry_ptr->osc_parameter_mask = COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK;
    }
    FOC_Protocol_OutputDiag("INFO", "protocol", "READY");
}

uint8_t FOC_Protocol_Process(foc_motor_t *motor, uint8_t frame_budget)
{
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;
    while (consumed < frame_budget)
    {
        uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
        uint16_t len = FrameSource_TryReadReady(frame, (uint16_t)sizeof(frame));
        if (len == 0U) break;
        if (ParseAndDispatchFrame(motor, frame, len) != 0U) has_comm_activity = 1U;
        consumed++;
    }
    return has_comm_activity;
}

void FOC_Protocol_Commit(foc_motor_t *motor)
{
    motor->state.cfg_dirty = 0U;
}

const telemetry_policy_snapshot_t *FOC_Protocol_GetTelemetry(void)
{
    return g_telemetry_ptr;
}