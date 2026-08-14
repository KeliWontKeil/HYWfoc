#include "L3_Hal/foc_codec.h"

#include <stdio.h>
#include <string.h>

#include "L3_Hal/foc_math_transforms.h"

foc_codec_frame_parse_result_t Codec_ParseCommandFrame(
    const uint8_t *rx, uint16_t rx_len,
    uint8_t *out_driver, uint8_t *out_cmd, uint8_t *out_sub,
    char *param_out, uint16_t param_max, uint8_t *has_param)
{
    const uint8_t *frame;
    uint16_t frame_len;
    uint16_t param_len;
    uint8_t driver_id;

    if ((rx == 0) || (out_driver == 0) || (out_cmd == 0) ||
        (out_sub == 0) || (has_param == 0))
    {
        return FOC_CODEC_FRAME_PARSE_INVALID;
    }

    if (Codec_ExtractFrame(rx, rx_len, &frame, &frame_len) == 0U)
    {
        return FOC_CODEC_FRAME_PARSE_INVALID;
    }

    driver_id = frame[1];
    if (Codec_IsDriverIdFormatValid(driver_id) == 0U)
    {
        return FOC_CODEC_FRAME_PARSE_INVALID;
    }

    if (Codec_IsDriverAddressedToLocal(driver_id) == 0U)
    {
        return FOC_CODEC_FRAME_PARSE_ADDRESS_MISMATCH;
    }

    if ((frame[2] < 'A') || (frame[2] > 'Z') ||
        (frame[3] < 'A') || (frame[3] > 'Z'))
    {
        return FOC_CODEC_FRAME_PARSE_INVALID;
    }

    *out_driver = driver_id;
    *out_cmd = (uint8_t)frame[2];
    *out_sub = (uint8_t)frame[3];

    param_len = (uint16_t)(frame_len - PROTOCOL_PARSER_MIN_FRAME_LEN);
    if (param_len >= param_max)
    {
        return FOC_CODEC_FRAME_PARSE_INVALID;
    }

    if (param_len > 0U)
    {
        if (param_out == 0)
        {
            return FOC_CODEC_FRAME_PARSE_INVALID;
        }
        memcpy(param_out, &frame[4], param_len);
        param_out[param_len] = '\0';
        *has_param = 1U;
    }
    else
    {
        if (param_out != 0)
        {
            param_out[0] = '\0';
        }
        *has_param = 0U;
    }

    return FOC_CODEC_FRAME_PARSE_OK;
}

uint8_t Codec_IsDriverIdFormatValid(uint8_t driver_id)
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

uint8_t Codec_IsDriverAddressedToLocal(uint8_t driver_id)
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

uint8_t Codec_ExtractFrame(const uint8_t *rx_data, uint16_t rx_len,
                           const uint8_t **frame_start, uint16_t *frame_len)
{
    uint16_t i;
    uint16_t j;

    if ((rx_data == 0) || (frame_start == 0) || (frame_len == 0) ||
        (rx_len < PROTOCOL_PARSER_MIN_FRAME_LEN))
    {
        return 0U;
    }

    for (i = 0U; i < rx_len; i++)
    {
        if (rx_data[i] != (uint8_t)FOC_PROTOCOL_FRAME_HEAD_CHAR)
        {
            continue;
        }

        for (j = (uint16_t)(i + PROTOCOL_PARSER_MIN_FRAME_LEN - 1U); j < rx_len; j++)
        {
            if (rx_data[j] == (uint8_t)FOC_PROTOCOL_FRAME_TAIL_CHAR)
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

uint8_t Codec_ParseSignedFloat(const char *text, float *value_out)
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

uint8_t Codec_ParseStateValue(float value, uint8_t *state_out)
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

uint16_t Codec_OscEncodeFrame(const uint8_t *bit_index, const float *value,
                              uint16_t count, char *out, uint16_t max_len)
{
    uint16_t off = 0U;
    uint16_t i;
    int written;
    int32_t ip;
    int32_t fp;

    (void)bit_index;

    if ((out == 0) || (max_len == 0U)) return 0U;
    if (count > FOC_OSC_CHANNEL_COUNT) count = FOC_OSC_CHANNEL_COUNT;

    written = snprintf(out, max_len, "%c", (char)DEBUG_STREAM_OSC_HEAD_BYTE);
    if ((written < 0) || ((uint16_t)written >= max_len)) return 0U;
    off = (uint16_t)written;

    for (i = 0U; i < count; i++)
    {
        if ((off + 12U) >= max_len) break;
        Math_FloatToFixed(value[i], 3, &ip, &fp);
        written = snprintf(out + off, max_len - off, " %d.%03d", (int)ip, (int)fp);
        if ((written < 0) || ((uint16_t)written >= (max_len - off))) break;
        off += (uint16_t)written;
    }

    if ((off + 6U) < max_len)
    {
        written = snprintf(out + off, max_len - off, " %c ", (char)DEBUG_STREAM_OSC_TAIL_BYTE);
        if (written > 0) off += (uint16_t)written;
    }
    if (off < max_len) out[off] = '\0';
    return off;
}

uint8_t Codec_FormatValueLine(char *out, uint16_t max,
                              const char *prefix, const char *name,
                              float value, uint8_t is_int)
{
    int32_t ip;
    int32_t fp;

    if ((out == 0) || (max == 0U)) return 0U;

    if (is_int != 0U)
    {
        snprintf(out, max, "%s.%s=%u\r\n", prefix, name,
                 (unsigned int)((value < 0.0f) ? 0U : (uint16_t)value));
    }
    else
    {
        Math_FloatToFixed(value, 3, &ip, &fp);
        snprintf(out, max, "%s.%s=%d.%03d\r\n", prefix, name, (int)ip, (int)fp);
    }
    return 1U;
}

uint8_t Codec_FormatStateLine(char *out, uint16_t max,
                              const char *name, uint8_t value)
{
    if ((out == 0) || (max == 0U)) return 0U;
    snprintf(out, max, "state.%s=%s\r\n", name, (value != 0U) ? "ENABLE" : "DISABLE");
    return 1U;
}
