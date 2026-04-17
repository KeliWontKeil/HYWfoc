#include "L3_Algorithm/protocol_text_codec.h"

#include <stdio.h>

#include "LS_Config/foc_config.h"

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
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        return "sensor_sample_offset_percent";
#endif
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        return "semantic_report_frequency_hz";
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        return "oscilloscope_report_frequency_hz";
    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        return "oscilloscope_param_mask";
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
        return "pid_current_kp";
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        return "pid_current_ki";
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        return "pid_current_kd";
#endif
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
        return "pid_angle_kp";
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        return "pid_angle_ki";
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        return "pid_angle_kd";
#endif
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
        return "pid_speed_kp";
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        return "pid_speed_ki";
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        return "pid_speed_kd";
#endif
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
        return "control_min_mech_angle_accum_delta_rad";
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
        return "control_angle_hold_integral_limit";
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
        return "control_angle_hold_pid_deadband_rad";
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
        return "control_speed_angle_transition_start_rad";
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
        return "control_speed_angle_transition_end_rad";
#endif
    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        return "control_mode";
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        return "current_soft_switch_mode";
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        return "current_soft_switch_auto_open_iq_a";
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        return "current_soft_switch_auto_closed_iq_a";
#endif
    default:
        return "unknown";
    }
}

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
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
        return "current_soft_switch_enabled";
#endif
    default:
        return "unknown";
    }
}

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
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
#endif
        return 1U;
    default:
        return 0U;
    }
}

void ProtocolText_FormatParamLine(char *out,
                                  uint16_t out_len,
                                  char subcommand,
                                  float value)
{
    if ((out == 0) || (out_len == 0U))
    {
        return;
    }

    if (ProtocolText_IsIntegerParam(subcommand) != 0U)
    {
        snprintf(out,
                 out_len,
                 "parameter.%s=%u\r\n",
                 ProtocolText_GetParamName(subcommand),
                 (unsigned int)((value < 0.0f) ? 0U : (uint16_t)value));
    }
    else
    {
        snprintf(out,
                 out_len,
                 "parameter.%s=%.3f\r\n",
                 ProtocolText_GetParamName(subcommand),
                 value);
    }
}

void ProtocolText_FormatStateLine(char *out,
                                  uint16_t out_len,
                                  char subcommand,
                                  uint8_t value)
{
    if ((out == 0) || (out_len == 0U))
    {
        return;
    }

    snprintf(out,
             out_len,
             "state.%s=%s\r\n",
             ProtocolText_GetStateName(subcommand),
             (value != 0U) ? "ENABLE" : "DISABLE");
}
