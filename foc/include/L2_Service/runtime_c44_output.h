#ifndef RUNTIME_OUTPUT_H
#define RUNTIME_OUTPUT_H

#include <stdint.h>

void RuntimeOutput_WriteText(const char *text);
void RuntimeOutput_WriteStatusByte(uint8_t status);

const char *RuntimeOutput_GetFaultName(uint8_t fault_code);
void RuntimeOutput_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeOutput_OutputParam(char subcommand, float value);
void RuntimeOutput_OutputState(char subcommand, uint8_t value);

#endif /* RUNTIME_OUTPUT_H */

