#include "interface/protocol_parser.h"

#include <string.h>

#include "interface/foc_platform_api.h"
#include "config/foc_config.h"

#define PROTOCOL_PARSER_COMM_SOURCE_1 1U
#define PROTOCOL_PARSER_COMM_SOURCE_2 2U
#define PROTOCOL_PARSER_COMM_SOURCE_3 3U
#define PROTOCOL_PARSER_COMM_SOURCE_4 4U

typedef enum {
    PROTOCOL_PARSER_FRAME_PARSE_INVALID = 0,
    PROTOCOL_PARSER_FRAME_PARSE_OK,
    PROTOCOL_PARSER_FRAME_PARSE_ADDRESS_MISMATCH
} protocol_parser_frame_parse_result_t;

static protocol_command_t g_latest_command;
static protocol_parser_result_t g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
static uint8_t g_preferred_comm_source = 0U;

static uint8_t ProtocolParser_ParseSignedFloat(const char *text, float *value_out);
static uint8_t ProtocolParser_ExtractFrame(const uint8_t *rx_data,
                                           uint16_t rx_len,
                                           const uint8_t **frame_start,
                                           uint16_t *frame_len);
static protocol_parser_frame_parse_result_t ProtocolParser_ParseFrame(const uint8_t *frame,
                                                                      uint16_t len,
                                                                      protocol_command_t *out_cmd);
static uint8_t ProtocolParser_IsDriverIdFormatValid(uint8_t driver_id);
static uint8_t ProtocolParser_IsDriverAddressedToLocal(uint8_t driver_id);
static uint8_t ProtocolParser_IsFrameReadyFromSource(uint8_t source);
static uint16_t ProtocolParser_ReadFrameFromSource(uint8_t source, uint8_t *buffer, uint16_t max_len);
static uint16_t ProtocolParser_TryReadReadySources(uint8_t *buffer, uint16_t max_len, uint8_t *source_used);
static uint16_t ProtocolParser_TryReadAnySource(uint8_t *buffer, uint16_t max_len, uint8_t *source_used);

void ProtocolParser_Init(void)
{
    memset(&g_latest_command, 0, sizeof(g_latest_command));
    g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
    g_preferred_comm_source = 0U;
}

uint8_t ProtocolParser_IsParsePending(void)
{
    uint8_t source;

    if ((g_preferred_comm_source != 0U) &&
        (ProtocolParser_IsFrameReadyFromSource(g_preferred_comm_source) != 0U))
    {
        return 1U;
    }

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        if (source == g_preferred_comm_source)
        {
            continue;
        }

        if (ProtocolParser_IsFrameReadyFromSource(source) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

void ProtocolParser_Process(void)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    protocol_parser_frame_parse_result_t parse_result = PROTOCOL_PARSER_FRAME_PARSE_INVALID;
    uint16_t len;
    uint8_t source_used = 0U;

    len = ProtocolParser_TryReadReadySources(frame,
                                             PROTOCOL_PARSER_RX_MAX_LEN,
                                             &source_used);
    if (len == 0U)
    {
        len = ProtocolParser_TryReadAnySource(frame,
                                              PROTOCOL_PARSER_RX_MAX_LEN,
                                              &source_used);
    }

    if (len == 0U)
    {
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
        return;
    }

    if (source_used != 0U)
    {
        g_preferred_comm_source = source_used;
    }

    if (ProtocolParser_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_FRAME_ERROR;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
        return;
    }

    parse_result = ProtocolParser_ParseFrame(payload, payload_len, &g_latest_command);
    if (parse_result == PROTOCOL_PARSER_FRAME_PARSE_OK)
    {
        g_latest_command.updated = 1U;
        g_last_result = PROTOCOL_PARSER_RESULT_OK;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_OK_CHAR);
    }
    else if (parse_result == PROTOCOL_PARSER_FRAME_PARSE_ADDRESS_MISMATCH)
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
    }
    else
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_FRAME_ERROR;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
    }
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

static protocol_parser_frame_parse_result_t ProtocolParser_ParseFrame(const uint8_t *frame,
                                                                      uint16_t len,
                                                                      protocol_command_t *out_cmd)
{
    uint16_t param_len;
    uint8_t driver_id;

    if ((frame == 0) || (out_cmd == 0) || (len < PROTOCOL_PARSER_MIN_FRAME_LEN))
    {
        return PROTOCOL_PARSER_FRAME_PARSE_INVALID;
    }

    if ((frame[0] != (uint8_t)PROTOCOL_PARSER_FRAME_HEAD_DEFAULT) ||
        (frame[len - 1U] != (uint8_t)PROTOCOL_PARSER_FRAME_TAIL_DEFAULT))
    {
        return PROTOCOL_PARSER_FRAME_PARSE_INVALID;
    }

    driver_id = frame[1];
    if (ProtocolParser_IsDriverIdFormatValid(driver_id) == 0U)
    {
        return PROTOCOL_PARSER_FRAME_PARSE_INVALID;
    }

    if (ProtocolParser_IsDriverAddressedToLocal(driver_id) == 0U)
    {
        return PROTOCOL_PARSER_FRAME_PARSE_ADDRESS_MISMATCH;
    }

    if ((frame[2] < 'A') || (frame[2] > 'Z') || (frame[3] < 'A') || (frame[3] > 'Z'))
    {
        return PROTOCOL_PARSER_FRAME_PARSE_INVALID;
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
        return PROTOCOL_PARSER_FRAME_PARSE_INVALID;
    }

    if (param_len > 0U)
    {
        memcpy(out_cmd->param_text, &frame[4], param_len);
        out_cmd->param_text[param_len] = '\0';

        if (ProtocolParser_ParseSignedFloat(out_cmd->param_text, &out_cmd->param_value) == 0U)
        {
            return PROTOCOL_PARSER_FRAME_PARSE_INVALID;
        }

        out_cmd->has_param = 1U;
    }
    else
    {
        out_cmd->param_text[0] = '\0';
    }

    return PROTOCOL_PARSER_FRAME_PARSE_OK;
}

static uint8_t ProtocolParser_IsDriverIdFormatValid(uint8_t driver_id)
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

static uint8_t ProtocolParser_IsDriverAddressedToLocal(uint8_t driver_id)
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

static uint8_t ProtocolParser_IsFrameReadyFromSource(uint8_t source)
{
    if (source == PROTOCOL_PARSER_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_IsFrameReady();
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_IsFrameReady();
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_IsFrameReady();
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_IsFrameReady();
    }

    return 0U;
}

static uint16_t ProtocolParser_ReadFrameFromSource(uint8_t source, uint8_t *buffer, uint16_t max_len)
{
    if (source == PROTOCOL_PARSER_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_ReadFrame(buffer, max_len);
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_ReadFrame(buffer, max_len);
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_ReadFrame(buffer, max_len);
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_ReadFrame(buffer, max_len);
    }

    return 0U;
}

static uint16_t ProtocolParser_TryReadReadySources(uint8_t *buffer, uint16_t max_len, uint8_t *source_used)
{
    uint16_t len;
    uint8_t source;

    if ((buffer == 0) || (source_used == 0))
    {
        return 0U;
    }

    *source_used = 0U;

    if ((g_preferred_comm_source != 0U) &&
        (ProtocolParser_IsFrameReadyFromSource(g_preferred_comm_source) != 0U))
    {
        len = ProtocolParser_ReadFrameFromSource(g_preferred_comm_source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = g_preferred_comm_source;
            return len;
        }
    }

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        if (source == g_preferred_comm_source)
        {
            continue;
        }

        if (ProtocolParser_IsFrameReadyFromSource(source) == 0U)
        {
            continue;
        }

        len = ProtocolParser_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = source;
            return len;
        }
    }

    return 0U;
}

static uint16_t ProtocolParser_TryReadAnySource(uint8_t *buffer, uint16_t max_len, uint8_t *source_used)
{
    uint16_t len;
    uint8_t source;

    if ((buffer == 0) || (source_used == 0))
    {
        return 0U;
    }

    *source_used = 0U;

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        len = ProtocolParser_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = source;
            return len;
        }
    }

    return 0U;
}
