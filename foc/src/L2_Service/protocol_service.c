#include "L2_Service/protocol_service.h"

#include "L2_Service/protocol_parser.h"
#include "L2_Service/command_manager.h"

uint8_t ProtocolService_ProcessStep(uint8_t max_frames)
{
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

    if (max_frames == 0U)
    {
        return 0U;
    }

    while ((ProtocolParser_IsParsePending() != 0U) && (consumed < max_frames))
    {
        ProtocolParser_Process();
        if (ProtocolParser_GetLastResult() == PROTOCOL_PARSER_RESULT_FRAME_ERROR)
        {
            CommandManager_ReportProtocolFrameError();
        }

        CommandManager_Process();
        if (CommandManager_GetRuntimeState()->last_exec_ok != 0U)
        {
            has_comm_activity = 1U;
        }

        consumed++;
    }

    return has_comm_activity;
}
