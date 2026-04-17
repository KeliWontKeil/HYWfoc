#include "L2_Service/protocol_parser.h"

#include <string.h>

#include "L3_Algorithm/protocol_core.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define PROTOCOL_PARSER_COMM_SOURCE_1 1U
#define PROTOCOL_PARSER_COMM_SOURCE_2 2U
#define PROTOCOL_PARSER_COMM_SOURCE_3 3U
#define PROTOCOL_PARSER_COMM_SOURCE_4 4U

static protocol_command_t g_latest_command;
static protocol_parser_result_t g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
static uint8_t g_preferred_comm_source = 0U;

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
    protocol_core_frame_parse_result_t parse_result = PROTOCOL_CORE_FRAME_PARSE_INVALID;
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

    if (ProtocolCore_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_FRAME_ERROR;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
        return;
    }

    parse_result = ProtocolCore_ParseFrame(payload, payload_len, &g_latest_command);
    if (parse_result == PROTOCOL_CORE_FRAME_PARSE_OK)
    {
        g_latest_command.updated = 1U;
        g_last_result = PROTOCOL_PARSER_RESULT_OK;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_OK_CHAR);
    }
    else if (parse_result == PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH)
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
