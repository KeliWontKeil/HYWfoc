#include "L2_Core/Protocol/foc_protocol_output.h"

#include <stdio.h>

#include "L2_Core/Runtime/foc_runtime_types.h"
#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Protocol/foc_protocol_parser.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* 通过调试端口写入文本 */
void FOC_Protocol_WriteText(const char *text)
{
    if (text == 0)
    {
        return;
    }

    FOC_Platform_WriteDebugText(text);
}

/* 写入单字节状态码 */
void FOC_Protocol_WriteStatus(uint8_t status)
{
    FOC_Platform_WriteStatusByte(status);
}

/* 获取故障码对应的可读名称 */
const char *FOC_Protocol_GetFaultName(uint8_t fault_code)
{
    switch (fault_code)
    {
    case FOC_FAULT_NONE:
        return "NONE";
    case FOC_FAULT_SENSOR_ADC_INVALID:
        return "SENSOR_ADC_INVALID";
    case FOC_FAULT_SENSOR_ENCODER_INVALID:
        return "SENSOR_ENCODER_INVALID";
    case FOC_FAULT_UNDERVOLTAGE:
        return "UNDERVOLTAGE";
    case FOC_FAULT_PROTOCOL_FRAME:
        return "PROTOCOL_FRAME";
    case FOC_FAULT_PARAM_INVALID:
        return "PARAM_INVALID";
    case FOC_FAULT_INIT_FAILED:
        return "INIT_FAILED";
    default:
        return "UNKNOWN";
    }
}

/* 输出诊断信息（level/module/detail格式） */
void FOC_Protocol_OutputDiag(const char *level, const char *module, const char *detail)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "diag.level=%s module=%s detail=%s\r\n",
             (level != 0) ? level : "INFO",
             (module != 0) ? module : "general",
             (detail != 0) ? detail : "none");
    FOC_Protocol_WriteText(out);
#else
    (void)level;
    (void)module;
    (void)detail;
#endif
}

/* 格式化并输出参数（subcommand+value） */
void FOC_Protocol_OutputParam(char subcommand, float value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatParamLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);

    FOC_Protocol_WriteText(out);
}

/* 格式化并输出状态（subcommand+value） */
void FOC_Protocol_OutputState(char subcommand, uint8_t value)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    ProtocolText_FormatStateLine(out,
                                 (uint16_t)sizeof(out),
                                 subcommand,
                                 value);
    FOC_Protocol_WriteText(out);
}
