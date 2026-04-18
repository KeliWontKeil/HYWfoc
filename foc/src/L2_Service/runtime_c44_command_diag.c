#include "L2_Service/runtime_c44_command_diag.h"

#include <stdio.h>

#include "L3_Algorithm/protocol_core.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

const char *CommandManager_GetFaultName(command_manager_fault_code_t fault_code)
{
    switch (fault_code)
    {
    case COMMAND_MANAGER_FAULT_NONE:
        return "NONE";
    case COMMAND_MANAGER_FAULT_SENSOR_ADC_INVALID:
        return "SENSOR_ADC_INVALID";
    case COMMAND_MANAGER_FAULT_SENSOR_ENCODER_INVALID:
        return "SENSOR_ENCODER_INVALID";
    case COMMAND_MANAGER_FAULT_UNDERVOLTAGE:
        return "UNDERVOLTAGE";
    case COMMAND_MANAGER_FAULT_PROTOCOL_FRAME:
        return "PROTOCOL_FRAME";
    case COMMAND_MANAGER_FAULT_PARAM_INVALID:
        return "PARAM_INVALID";
    case COMMAND_MANAGER_FAULT_INIT_FAILED:
        return "INIT_FAILED";
    default:
        return "UNKNOWN";
    }
}

void CommandManager_OutputDiag(const char *level, const char *module, const char *detail)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "diag.level=%s module=%s detail=%s\r\n",
             (level != 0) ? level : "INFO",
             (module != 0) ? module : "general",
             (detail != 0) ? detail : "none");
    FOC_Platform_WriteDebugText(out);
#else
    (void)level;
    (void)module;
    (void)detail;
#endif
}

void CommandManager_OutputParam(char subcommand, float value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatParamLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);

    FOC_Platform_WriteDebugText(out);
}

void CommandManager_OutputState(char subcommand, uint8_t value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatStateLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);
    FOC_Platform_WriteDebugText(out);
}
