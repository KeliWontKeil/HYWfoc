#include "L3_Algorithm/protocol_core_parser.h"

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
