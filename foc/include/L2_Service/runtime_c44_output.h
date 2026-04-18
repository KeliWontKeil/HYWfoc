#ifndef RUNTIME_C44_OUTPUT_H
#define RUNTIME_C44_OUTPUT_H

#include <stdint.h>

void RuntimeC44_WriteText(const char *text);
void RuntimeC44_WriteStatusByte(uint8_t status);

const char *RuntimeC44_GetFaultName(uint8_t fault_code);
void RuntimeC44_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeC44_OutputParam(char subcommand, float value);
void RuntimeC44_OutputState(char subcommand, uint8_t value);

#endif /* RUNTIME_C44_OUTPUT_H */
