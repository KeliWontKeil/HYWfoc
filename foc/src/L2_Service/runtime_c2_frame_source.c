#include "L2_Service/runtime_c2_frame_source.h"

#include "L2_Service/runtime_c3_runtime_fsm.h"
#include "L3_Algorithm/protocol_core.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define RUNTIME_COMM_SOURCE_1 1U
#define RUNTIME_COMM_SOURCE_2 2U
#define RUNTIME_COMM_SOURCE_3 3U
#define RUNTIME_COMM_SOURCE_4 4U

static uint8_t FrameSource_IsReady(uint8_t source)
{
    if (source == RUNTIME_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_IsFrameReady();
    }

    if (source == RUNTIME_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_IsFrameReady();
    }

    if (source == RUNTIME_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_IsFrameReady();
    }

    if (source == RUNTIME_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_IsFrameReady();
    }

    return 0U;
}

static uint16_t FrameSource_Read(uint8_t source, uint8_t *buffer, uint16_t max_len)
{
    if (source == RUNTIME_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_ReadFrame(buffer, max_len);
    }

    if (source == RUNTIME_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_ReadFrame(buffer, max_len);
    }

    if (source == RUNTIME_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_ReadFrame(buffer, max_len);
    }

    if (source == RUNTIME_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_ReadFrame(buffer, max_len);
    }

    return 0U;
}

static uint16_t FrameSource_TryReadReady(uint8_t *buffer, uint16_t max_len)
{
    uint8_t source;
    uint16_t len;

    if (buffer == 0)
    {
        return 0U;
    }

    for (source = RUNTIME_COMM_SOURCE_1; source <= RUNTIME_COMM_SOURCE_4; source++)
    {
        if (FrameSource_IsReady(source) == 0U)
        {
            continue;
        }

        len = FrameSource_Read(source, buffer, max_len);
        if (len > 0U)
        {
            return len;
        }
    }

    return 0U;
}

/*static uint16_t FrameSource_TryReadAny(uint8_t *buffer, uint16_t max_len)
{
    uint8_t source;
    uint16_t len;

    if (buffer == 0)
    {
        return 0U;
    }

    for (source = RUNTIME_COMM_SOURCE_1; source <= RUNTIME_COMM_SOURCE_4; source++)
    {
        len = FrameSource_Read(source, buffer, max_len);
        if (len > 0U)
        {
            return len;
        }
    }

    return 0U;
}*/

static uint8_t ParseAndDispatchFrame(const uint8_t *frame, uint16_t len)
{
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    protocol_core_frame_parse_result_t parse_result;
    protocol_command_t command;

    if ((frame == 0) || (len == 0U))
    {
        return 0U;
    }

    if (ProtocolCore_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        RuntimeC3_ReportFrameError();
        return 0U;
    }

    parse_result = ProtocolCore_ParseFrame(payload, payload_len, &command);
    if (parse_result == PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH)
    {
        return 0U;
    }

    if (parse_result != PROTOCOL_CORE_FRAME_PARSE_OK)
    {
        RuntimeC3_ReportFrameError();
        return 0U;
    }

    command.updated = 1U;
    return RuntimeC3_HandleCommand(&command);
}

uint8_t RuntimeC2_ProcessOneFrame(void)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    uint16_t len;

    len = FrameSource_TryReadReady(frame, (uint16_t)sizeof(frame));
    
    /*if (len == 0U)
    {
        len = FrameSource_TryReadAny(frame, (uint16_t)sizeof(frame));
    }*/

    if (len == 0U)
    {
        return 0U;
    }

    return ParseAndDispatchFrame(frame, len);
}

void RuntimeC2_Init(void)
{
    RuntimeC3_Init();
}

void RuntimeC2_UpdateSignals(const runtime_step_signal_t *signal)
{
    RuntimeC3_UpdateSignals(signal);
}

void RuntimeC2_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC3_BuildSnapshot(snapshot);
}

void RuntimeC2_Commit(void)
{
    RuntimeC3_Commit();
}
