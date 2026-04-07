#include "interface/protocol_parser.h"

#include <string.h>

#include "interface/foc_platform_api.h"
#include "config/foc_config.h"

#define PROTOCOL_PARSER_COMM_SOURCE_1 1U
#define PROTOCOL_PARSER_COMM_SOURCE_2 2U
#define PROTOCOL_PARSER_COMM_SOURCE_3 3U
#define PROTOCOL_PARSER_COMM_SOURCE_4 4U

#define PROTOCOL_PARSER_COMM_SRC1_BIT (1U << 0)
#define PROTOCOL_PARSER_COMM_SRC2_BIT (1U << 1)
#define PROTOCOL_PARSER_COMM_SRC3_BIT (1U << 2)
#define PROTOCOL_PARSER_COMM_SRC4_BIT (1U << 3)

typedef enum {
    PROTOCOL_PARSER_FRAME_PARSE_INVALID = 0,
    PROTOCOL_PARSER_FRAME_PARSE_OK,
    PROTOCOL_PARSER_FRAME_PARSE_ADDRESS_MISMATCH
} protocol_parser_frame_parse_result_t;

static protocol_command_t g_latest_command;
static protocol_parser_result_t g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
static volatile uint8_t g_parse_pending = 0U;
static volatile uint8_t g_comm_trigger_mask = 0U;
static volatile uint8_t g_preferred_comm_source = 0U;

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
static uint8_t ProtocolParser_SourceToBit(uint8_t source);
static uint16_t ProtocolParser_ReadFrameFromSource(uint8_t source, uint8_t *buffer, uint16_t max_len);
static uint16_t ProtocolParser_TryReadTriggeredSources(uint8_t *buffer, uint16_t max_len, uint8_t *source_used);
static uint16_t ProtocolParser_TryReadAnySource(uint8_t *buffer, uint16_t max_len, uint8_t *source_used);
static void ProtocolParser_OnCommSource1RxTrigger(void);
static void ProtocolParser_OnCommSource2RxTrigger(void);
static void ProtocolParser_OnCommSource3RxTrigger(void);
static void ProtocolParser_OnCommSource4RxTrigger(void);

void ProtocolParser_Init(void)
{
    memset(&g_latest_command, 0, sizeof(g_latest_command));
    g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
    g_parse_pending = 0U;
    g_comm_trigger_mask = 0U;
    g_preferred_comm_source = 0U;

    FOC_Platform_CommSource1_SetRxTriggerCallback(ProtocolParser_OnCommSource1RxTrigger);
    FOC_Platform_CommSource2_SetRxTriggerCallback(ProtocolParser_OnCommSource2RxTrigger);
    FOC_Platform_CommSource3_SetRxTriggerCallback(ProtocolParser_OnCommSource3RxTrigger);
    FOC_Platform_CommSource4_SetRxTriggerCallback(ProtocolParser_OnCommSource4RxTrigger);
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
    protocol_parser_frame_parse_result_t parse_result = PROTOCOL_PARSER_FRAME_PARSE_INVALID;
    uint16_t len;
    uint8_t source_used = 0U;

    if (g_parse_pending == 0U)
    {
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
        return;
    }

    len = ProtocolParser_TryReadTriggeredSources(frame,
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
        g_parse_pending = (g_comm_trigger_mask != 0U) ? 1U : 0U;
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
        g_parse_pending = 1U;
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

    g_parse_pending = 1U;
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

static uint8_t ProtocolParser_SourceToBit(uint8_t source)
{
    if (source == PROTOCOL_PARSER_COMM_SOURCE_1)
    {
        return PROTOCOL_PARSER_COMM_SRC1_BIT;
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_2)
    {
        return PROTOCOL_PARSER_COMM_SRC2_BIT;
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_3)
    {
        return PROTOCOL_PARSER_COMM_SRC3_BIT;
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_4)
    {
        return PROTOCOL_PARSER_COMM_SRC4_BIT;
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

static uint16_t ProtocolParser_TryReadTriggeredSources(uint8_t *buffer, uint16_t max_len, uint8_t *source_used)
{
    uint16_t len;
    uint8_t source;
    uint8_t source_bit;

    if ((buffer == 0) || (source_used == 0))
    {
        return 0U;
    }

    *source_used = 0U;

    if (g_preferred_comm_source != 0U)
    {
        source_bit = ProtocolParser_SourceToBit(g_preferred_comm_source);
        if ((source_bit != 0U) && ((g_comm_trigger_mask & source_bit) != 0U))
        {
            len = ProtocolParser_ReadFrameFromSource(g_preferred_comm_source, buffer, max_len);
            if (len > 0U)
            {
                *source_used = g_preferred_comm_source;
                return len;
            }
            g_comm_trigger_mask = (uint8_t)(g_comm_trigger_mask & (uint8_t)(~source_bit));
        }
    }

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        if (source == g_preferred_comm_source)
        {
            continue;
        }

        source_bit = ProtocolParser_SourceToBit(source);
        if ((g_comm_trigger_mask & source_bit) == 0U)
        {
            continue;
        }

        len = ProtocolParser_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = source;
            return len;
        }

        g_comm_trigger_mask = (uint8_t)(g_comm_trigger_mask & (uint8_t)(~source_bit));
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

static void ProtocolParser_OnCommSource1RxTrigger(void)
{
    g_comm_trigger_mask = (uint8_t)(g_comm_trigger_mask | PROTOCOL_PARSER_COMM_SRC1_BIT);
    g_preferred_comm_source = PROTOCOL_PARSER_COMM_SOURCE_1;
    ProtocolParser_TriggerParse();
}

static void ProtocolParser_OnCommSource2RxTrigger(void)
{
    g_comm_trigger_mask = (uint8_t)(g_comm_trigger_mask | PROTOCOL_PARSER_COMM_SRC2_BIT);
    g_preferred_comm_source = PROTOCOL_PARSER_COMM_SOURCE_2;
    ProtocolParser_TriggerParse();
}

static void ProtocolParser_OnCommSource3RxTrigger(void)
{
    g_comm_trigger_mask = (uint8_t)(g_comm_trigger_mask | PROTOCOL_PARSER_COMM_SRC3_BIT);
    g_preferred_comm_source = PROTOCOL_PARSER_COMM_SOURCE_3;
    ProtocolParser_TriggerParse();
}

static void ProtocolParser_OnCommSource4RxTrigger(void)
{
    g_comm_trigger_mask = (uint8_t)(g_comm_trigger_mask | PROTOCOL_PARSER_COMM_SRC4_BIT);
    g_preferred_comm_source = PROTOCOL_PARSER_COMM_SOURCE_4;
    ProtocolParser_TriggerParse();
}
