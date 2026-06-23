#ifndef FOC_PROTOCOL_OUTPUT_H
#define FOC_PROTOCOL_OUTPUT_H

#include <stdint.h>

/* 通过调试端口写入文本 */
void FOC_Protocol_WriteText(const char *text);

/* 写入单字节状态码 */
void FOC_Protocol_WriteStatus(uint8_t status);

/* 获取故障码对应的可读名称 */
const char *FOC_Protocol_GetFaultName(uint8_t fault_code);

/* 输出诊断信息（level/module/detail格式） */
void FOC_Protocol_OutputDiag(const char *level, const char *module, const char *detail);

/* 格式化并输出参数（subcommand+value） */
void FOC_Protocol_OutputParam(char subcommand, float value);

/* 格式化并输出状态（subcommand+value） */
void FOC_Protocol_OutputState(char subcommand, uint8_t value);

#endif /* FOC_PROTOCOL_OUTPUT_H */