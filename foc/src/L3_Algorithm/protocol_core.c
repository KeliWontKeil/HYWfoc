#include "L3_Algorithm/protocol_core.h"

#include <stdio.h>
#include <string.h>

#include "LS_Config/foc_config.h"

protocol_core_frame_parse_result_t ProtocolCore_ParseFrame(const uint8_t *frame,
                                                           uint16_t len,
                                                           protocol_command_t *out_cmd)
{
    uint16_t param_len;
    uint8_t driver_id;

    if ((frame == 0) || (out_cmd == 0) || (len < PROTOCOL_PARSER_MIN_FRAME_LEN))
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    if ((frame[0] != (uint8_t)PROTOCOL_PARSER_FRAME_HEAD_DEFAULT) ||
        (frame[len - 1U] != (uint8_t)PROTOCOL_PARSER_FRAME_TAIL_DEFAULT))
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    driver_id = frame[1];
    if (ProtocolCore_IsDriverIdFormatValid(driver_id) == 0U)
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    if (ProtocolCore_IsDriverAddressedToLocal(driver_id) == 0U)
    {
        return PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH;
    }

    if ((frame[2] < 'A') || (frame[2] > 'Z') || (frame[3] < 'A') || (frame[3] > 'Z'))
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    out_cmd->driver_id = driver_id;
    out_cmd->command = (char)frame[2];
    out_cmd->subcommand = (char)frame[3];
    out_cmd->param_value = 0.0f;
    out_cmd->has_param = 0U;
    out_cmd->frame_valid = 1U;

    param_len = (uint16_t)(len - PROTOCOL_PARSER_MIN_FRAME_LEN);
    if (param_len >= sizeof(out_cmd->param_text))
    {
        return PROTOCOL_CORE_FRAME_PARSE_INVALID;
    }

    if (param_len > 0U)
    {
        memcpy(out_cmd->param_text, &frame[4], param_len);
        out_cmd->param_text[param_len] = '\0';

        if (ProtocolCore_ParseSignedFloat(out_cmd->param_text, &out_cmd->param_value) == 0U)
        {
            return PROTOCOL_CORE_FRAME_PARSE_INVALID;
        }

        out_cmd->has_param = 1U;
    }
    else
    {
        out_cmd->param_text[0] = '\0';
    }

    return PROTOCOL_CORE_FRAME_PARSE_OK;
}

uint8_t ProtocolCore_IsDriverIdFormatValid(uint8_t driver_id)
{
    if (driver_id == FOC_PROTOCOL_DRIVER_ID_BROADCAST)
    {
        return 1U;
    }

    if ((driver_id >= FOC_PROTOCOL_DRIVER_ID_MIN) &&
        (driver_id <= FOC_PROTOCOL_DRIVER_ID_MAX))
    {
        return 1U;
    }

    return 0U;
}

uint8_t ProtocolCore_IsDriverAddressedToLocal(uint8_t driver_id)
{
    if (driver_id == FOC_PROTOCOL_DRIVER_ID_BROADCAST)
    {
        return 1U;
    }

    if (driver_id == FOC_PROTOCOL_LOCAL_DRIVER_ID_DEFAULT)
    {
        return 1U;
    }

    return 0U;
}

uint8_t ProtocolCore_ExtractFrame(const uint8_t *rx_data,
                                  uint16_t rx_len,
                                  const uint8_t **frame_start,
                                  uint16_t *frame_len)
{
    uint16_t i;
    uint16_t j;

    if ((rx_data == 0) || (frame_start == 0) || (frame_len == 0) || (rx_len < PROTOCOL_PARSER_MIN_FRAME_LEN))
    {
        return 0U;
    }

    for (i = 0U; i < rx_len; i++)
    {
        if (rx_data[i] != (uint8_t)PROTOCOL_PARSER_FRAME_HEAD_DEFAULT)
        {
            continue;
        }

        for (j = (uint16_t)(i + PROTOCOL_PARSER_MIN_FRAME_LEN - 1U); j < rx_len; j++)
        {
            if (rx_data[j] == (uint8_t)PROTOCOL_PARSER_FRAME_TAIL_DEFAULT)
            {
                *frame_start = &rx_data[i];
                *frame_len = (uint16_t)(j - i + 1U);
                return 1U;
            }
        }
        return 0U;
    }

    return 0U;
}

uint8_t ProtocolCore_ParseSignedFloat(const char *text, float *value_out)
{
    uint8_t i = 0U;
    uint8_t has_digit = 0U;
    uint8_t has_dot = 0U;
    float value = 0.0f;
    float frac_scale = 0.1f;
    int8_t sign = 1;

    if ((text == 0) || (value_out == 0) || (text[0] == '\0'))
    {
        return 0U;
    }

    if (text[i] == '+')
    {
        i++;
    }
    else if (text[i] == '-')
    {
        sign = -1;
        i++;
    }

    for (; text[i] != '\0'; i++)
    {
        char ch = text[i];

        if ((ch >= '0') && (ch <= '9'))
        {
            has_digit = 1U;
            if (has_dot == 0U)
            {
                value = value * 10.0f + (float)(ch - '0');
            }
            else
            {
                value += (float)(ch - '0') * frac_scale;
                frac_scale *= 0.1f;
            }
        }
        else if ((ch == '.') && (has_dot == 0U))
        {
            has_dot = 1U;
        }
        else
        {
            return 0U;
        }
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    *value_out = (float)sign * value;
    return 1U;
}

uint8_t ProtocolCore_ParseStateValue(float value, uint8_t *state_out)
{
    if (state_out == 0)
    {
        return 0U;
    }

    if (value == 0.0f)
    {
        *state_out = COMMAND_MANAGER_ENABLED_DISABLE;
        return 1U;
    }

    if (value == 1.0f)
    {
        *state_out = COMMAND_MANAGER_ENABLED_ENABLE;
        return 1U;
    }

    return 0U;
}

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