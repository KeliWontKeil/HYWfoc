#include "protocol_parser.h"

#include <string.h>

#include "foc_platform_api.h"

#define PROTOCOL_PARSER_RX_MAX_LEN 128U
#define PROTOCOL_PARSER_MIN_FRAME_LEN 4U

static protocol_command_t g_latest_command;
static protocol_parser_result_t g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
static volatile uint8_t g_parse_pending = 0U;

static uint8_t ProtocolParser_ParseSignedFloat(const char *text, float *value_out);
static uint8_t ProtocolParser_ExtractFrame(const uint8_t *rx_data,
                                           uint16_t rx_len,
                                           const uint8_t **frame_start,
                                           uint16_t *frame_len);
static uint8_t ProtocolParser_ParseFrame(const uint8_t *frame, uint16_t len, protocol_command_t *out_cmd);
static void ProtocolParser_OnCommRxTrigger(void);

void ProtocolParser_Init(void)
{
    memset(&g_latest_command, 0, sizeof(g_latest_command));
    g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
    g_parse_pending = 0U;
    FOC_Platform_SetCommRxTriggerCallback(ProtocolParser_OnCommRxTrigger);
}

void ProtocolParser_TriggerParse(void)
{
    g_parse_pending = 1U;
}

uint8_t ProtocolParser_IsParsePending(void)
{
    return g_parse_pending;
}

void ProtocolParser_Process(void)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    uint16_t len;

    if (g_parse_pending == 0U)
    {
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
        return;
    }

    len = FOC_Platform_ReceiveFrame(frame, PROTOCOL_PARSER_RX_MAX_LEN);
    if (len == 0U)
    {
        g_parse_pending = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
        return;
    }

    if ((ProtocolParser_ExtractFrame(frame, len, &payload, &payload_len) != 0U) &&
        (ProtocolParser_ParseFrame(payload, payload_len, &g_latest_command) != 0U))
    {
        g_latest_command.updated = 1U;
        g_last_result = PROTOCOL_PARSER_RESULT_OK;
        FOC_Platform_FeedbackOutput((uint8_t)PROTOCOL_PARSER_STATUS_OK_CHAR);
    }
    else
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_FRAME_ERROR;
        FOC_Platform_FeedbackOutput((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
    }

    g_parse_pending = FOC_Platform_CommHasPendingFrame();
}

const protocol_command_t *ProtocolParser_GetLatestCommand(void)
{
    return &g_latest_command;
}

void ProtocolParser_ClearUpdatedFlag(void)
{
    g_latest_command.updated = 0U;
}

protocol_parser_result_t ProtocolParser_GetLastResult(void)
{
    return g_last_result;
}

static uint8_t ProtocolParser_ParseFrame(const uint8_t *frame, uint16_t len, protocol_command_t *out_cmd)
{
    uint16_t param_len;

    if ((frame == 0) || (out_cmd == 0) || (len < PROTOCOL_PARSER_MIN_FRAME_LEN))
    {
        return 0U;
    }

    if ((frame[0] != (uint8_t)PROTOCOL_PARSER_FRAME_HEAD_DEFAULT) ||
        (frame[len - 1U] != (uint8_t)PROTOCOL_PARSER_FRAME_TAIL_DEFAULT))
    {
        return 0U;
    }

    if ((frame[1] < 'A') || (frame[1] > 'Z') || (frame[2] < 'A') || (frame[2] > 'Z'))
    {
        return 0U;
    }

    out_cmd->command = (char)frame[1];
    out_cmd->subcommand = (char)frame[2];
    out_cmd->param_value = 0.0f;
    out_cmd->has_param = 0U;
    out_cmd->frame_valid = 1U;

    param_len = (uint16_t)(len - PROTOCOL_PARSER_MIN_FRAME_LEN);
    if (param_len >= sizeof(out_cmd->param_text))
    {
        return 0U;
    }

    if (param_len > 0U)
    {
        memcpy(out_cmd->param_text, &frame[3], param_len);
        out_cmd->param_text[param_len] = '\0';

        if (ProtocolParser_ParseSignedFloat(out_cmd->param_text, &out_cmd->param_value) == 0U)
        {
            return 0U;
        }

        out_cmd->has_param = 1U;
    }
    else
    {
        out_cmd->param_text[0] = '\0';
    }

    return 1U;
}

static uint8_t ProtocolParser_ExtractFrame(const uint8_t *rx_data,
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

static uint8_t ProtocolParser_ParseSignedFloat(const char *text, float *value_out)
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

static void ProtocolParser_OnCommRxTrigger(void)
{
    ProtocolParser_TriggerParse();
}
