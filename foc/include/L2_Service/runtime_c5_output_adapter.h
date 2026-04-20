#ifndef RUNTIME_C5_OUTPUT_ADAPTER_H
#define RUNTIME_C5_OUTPUT_ADAPTER_H

#include <stdint.h>

void RuntimeC5_WriteText(const char *text);
void RuntimeC5_WriteStatusByte(uint8_t status);

const char *RuntimeC5_GetFaultName(uint8_t fault_code);
void RuntimeC5_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeC5_OutputParam(char subcommand, float value);
void RuntimeC5_OutputState(char subcommand, uint8_t value);

#endif /* RUNTIME_C5_OUTPUT_ADAPTER_H */

