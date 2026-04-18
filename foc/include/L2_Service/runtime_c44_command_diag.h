#ifndef RUNTIME_C44_COMMAND_DIAG_H
#define RUNTIME_C44_COMMAND_DIAG_H

#include <stdint.h>

const char *CommandManager_GetFaultName(uint8_t fault_code);
void CommandManager_OutputDiag(const char *level, const char *module, const char *detail);
void CommandManager_OutputParam(char subcommand, float value);
void CommandManager_OutputState(char subcommand, uint8_t value);

#endif /* RUNTIME_C44_COMMAND_DIAG_H */
