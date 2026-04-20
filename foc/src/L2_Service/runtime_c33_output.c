#include "L2_Service/runtime_c33_output.h"

#include <stdio.h>

#include "L3_Algorithm/protocol_core.h"
#include "L2_Service/runtime_internal_types.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

void RuntimeOutput_WriteText(const char *text)
{
    if (text == 0)
    {
        return;
    }

    FOC_Platform_WriteDebugText(text);
}

void RuntimeOutput_WriteStatusByte(uint8_t status)
{
    FOC_Platform_WriteStatusByte(status);
}

const char *RuntimeOutput_GetFaultName(uint8_t fault_code)
{
    switch (fault_code)
    {
    case RUNTIME_FAULT_NONE:
        return "NONE";
    case RUNTIME_FAULT_SENSOR_ADC_INVALID:
        return "SENSOR_ADC_INVALID";
    case RUNTIME_FAULT_SENSOR_ENCODER_INVALID:
        return "SENSOR_ENCODER_INVALID";
    case RUNTIME_FAULT_UNDERVOLTAGE:
        return "UNDERVOLTAGE";
    case RUNTIME_FAULT_PROTOCOL_FRAME:
        return "PROTOCOL_FRAME";
    case RUNTIME_FAULT_PARAM_INVALID:
        return "PARAM_INVALID";
    case RUNTIME_FAULT_INIT_FAILED:
        return "INIT_FAILED";
    default:
        return "UNKNOWN";
    }
}

void RuntimeOutput_OutputDiag(const char *level, const char *module, const char *detail)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "diag.level=%s module=%s detail=%s\r\n",
             (level != 0) ? level : "INFO",
             (module != 0) ? module : "general",
             (detail != 0) ? detail : "none");
    RuntimeOutput_WriteText(out);
#else
    (void)level;
    (void)module;
    (void)detail;
#endif
}

void RuntimeOutput_OutputParam(char subcommand, float value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatParamLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);

    RuntimeOutput_WriteText(out);
}

void RuntimeOutput_OutputState(char subcommand, uint8_t value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatStateLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);
    RuntimeOutput_WriteText(out);
}
