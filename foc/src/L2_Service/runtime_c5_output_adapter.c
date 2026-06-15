#include "L2_Service/runtime_c5_output_adapter.h"

#include <stdio.h>

#include "LS_Config/foc_runtime_types.h"
#include "L3_Algorithm/protocol_core.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* 输出适配器：通过调试端口写入文本 */
void RuntimeC5_WriteText(const char *text)
{
    if (text == 0)
    {
        return;
    }

    FOC_Platform_WriteDebugText(text);
}

/* 输出适配器：写入单字节状态码 */
void RuntimeC5_WriteStatusByte(uint8_t status)
{
    FOC_Platform_WriteStatusByte(status);
}

/* 输出适配器：获取故障码对应的可读名称 */
const char *RuntimeC5_GetFaultName(uint8_t fault_code)
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

/* 输出适配器：输出诊断信息（level/module/detail格式） */
void RuntimeC5_OutputDiag(const char *level, const char *module, const char *detail)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "diag.level=%s module=%s detail=%s\r\n",
             (level != 0) ? level : "INFO",
             (module != 0) ? module : "general",
             (detail != 0) ? detail : "none");
    RuntimeC5_WriteText(out);
#else
    (void)level;
    (void)module;
    (void)detail;
#endif
}

/* 输出适配器：格式化并输出参数（subcommand+value） */
void RuntimeC5_OutputParam(char subcommand, float value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatParamLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);

    RuntimeC5_WriteText(out);
}

/* 输出适配器：格式化并输出状态（subcommand+value） */
void RuntimeC5_OutputState(char subcommand, uint8_t value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatStateLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);
    RuntimeC5_WriteText(out);
}