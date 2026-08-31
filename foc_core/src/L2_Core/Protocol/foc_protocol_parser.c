#include "L2_Core/foc_motor_aggregate.h"
#include "L2_Core/Protocol/foc_protocol_parser.h"

#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_codec.h"

protocol_core_frame_parse_result_t ProtocolCore_ParseFrame(const uint8_t *frame,
                                                           uint16_t len,
                                                           protocol_command_t *out_cmd)
{
    uint8_t driver_id;
    uint8_t cmd;
    uint8_t sub;
    uint8_t has_param;
    foc_codec_frame_parse_result_t codec_res;

    if (out_cmd == 0)
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    codec_res = Codec_ParseCommandFrame(frame, len,
                                        &driver_id, &cmd, &sub,
                                        out_cmd->param_text, sizeof(out_cmd->param_text),
                                        &has_param);
    if (codec_res == FOC_CODEC_FRAME_PARSE_ADDRESS_MISMATCH)
    {
        return PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH;
    }
    if (codec_res != FOC_CODEC_FRAME_PARSE_OK)
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    out_cmd->driver_id = driver_id;
    out_cmd->command = (char)cmd;
    out_cmd->subcommand = (char)sub;
    out_cmd->has_param = has_param;
    out_cmd->frame_valid = 1U;

    if (has_param != 0U)
    {
        if (Codec_ParseSignedFloat(out_cmd->param_text, &out_cmd->param_value) == 0U)
        {
            return PROTOCOL_CORE_FRAME_PARSE_INVALID;
        }
    }
    else
    {
        out_cmd->param_value = 0.0f;
    }

    return PROTOCOL_CORE_FRAME_PARSE_OK;
}

/* ========== P 组参数名映射 ========== */

const char *ProtocolText_GetParamName(char subcommand)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        return "target_angle_rad";
    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        return "angle_position_speed_rad_s";
    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        return "speed_only_speed_rad_s";
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        return "semantic_report_frequency_hz";
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        return "oscilloscope_report_frequency_hz";
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        return "oscilloscope_param_mask";
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        return "control_mode";
    default:
        return "unknown";
    }
}

/* ========== C 组参数名映射 ========== */

const char *ProtocolText_GetConfigName(char subcommand)
{
    switch (subcommand)
    {
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KP:
        return "pid_current_kp";
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KI:
        return "pid_current_ki";
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_CURRENT_KD:
        return "pid_current_kd";
#endif
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KP:
        return "pid_angle_kp";
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KI:
        return "pid_angle_ki";
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_ANGLE_KD:
        return "pid_angle_kd";
#endif
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KP:
        return "pid_speed_kp";
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KI:
        return "pid_speed_ki";
    case COMMAND_MANAGER_CONFIG_SUBCMD_PID_SPEED_KD:
        return "pid_speed_kd";
#endif
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_MIN_MECH_DELTA:
        return "control_min_mech_angle_accum_delta_rad";
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_I_LIMIT:
        return "control_angle_hold_integral_limit";
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_HOLD_DEADBAND:
        return "control_angle_hold_pid_deadband_rad";
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_START:
        return "control_speed_angle_transition_start_rad";
    case COMMAND_MANAGER_CONFIG_SUBCMD_CFG_BLEND_END:
        return "control_speed_angle_transition_end_rad";
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_CALIB_GAIN:
        return "cogging_calib_gain_k";
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_IQ_LIMIT:
        return "cogging_comp_iq_limit_a";
    case COMMAND_MANAGER_CONFIG_SUBCMD_COGGING_COMP_SPEED_GATE:
        return "cogging_comp_speed_gate_rad_s";
#endif
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        return "current_soft_switch_mode";
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        return "current_soft_switch_auto_open_iq_a";
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        return "current_soft_switch_auto_closed_iq_a";
#endif
    default:
        return "unknown";
    }
}

/* ========== S 组状态名映射 ========== */

const char *ProtocolText_GetStateName(char subcommand)
{
    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        return "motor_enable";
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
        return "semantic_report_enabled";
    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
        return "oscilloscope_report_enabled";
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
        return "cogging_comp_enabled";
#endif
    default:
        return "unknown";
    }
}

/* ========== P 组整数参数检测 ========== */

uint8_t ProtocolText_IsIntegerParam(char subcommand)
{
    switch (subcommand)
    {
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        return 1U;
    default:
        return 0U;
    }
}

/* ========== C 组整数参数检测 ========== */

uint8_t ProtocolText_IsIntegerConfigParam(char subcommand)
{
    switch (subcommand)
    {
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_CONFIG_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
#endif
        return 1U;
    default:
        return 0U;
    }
}

/* ========== P 组格式化输出 ========== */

void ProtocolText_FormatParamLine(char *out,
                                  uint16_t out_len,
                                  char subcommand,
                                  float value)
{
    if ((out == 0) || (out_len == 0U))
    {
        return;
    }

    (void)Codec_FormatValueLine(out, out_len, "parameter",
                                ProtocolText_GetParamName(subcommand),
                                value, ProtocolText_IsIntegerParam(subcommand));
}

/* ========== C 组格式化输出 ========== */

void ProtocolText_FormatConfigLine(char *out,
                                   uint16_t out_len,
                                   char subcommand,
                                   float value)
{
    if ((out == 0) || (out_len == 0U))
    {
        return;
    }

    (void)Codec_FormatValueLine(out, out_len, "config",
                                ProtocolText_GetConfigName(subcommand),
                                value, ProtocolText_IsIntegerConfigParam(subcommand));
}

/* ========== S 组格式化输出 ========== */

void ProtocolText_FormatStateLine(char *out,
                                  uint16_t out_len,
                                  char subcommand,
                                  uint8_t value)
{
    if ((out == 0) || (out_len == 0U))
    {
        return;
    }

    (void)Codec_FormatStateLine(out, out_len,
                                ProtocolText_GetStateName(subcommand), value);
}
