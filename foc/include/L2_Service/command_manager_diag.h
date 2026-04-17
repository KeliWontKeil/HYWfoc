#ifndef COMMAND_MANAGER_DIAG_H
#define COMMAND_MANAGER_DIAG_H

#include <stdint.h>

#include "L2_Service/command_manager.h"

const char *CommandManager_GetFaultName(command_manager_fault_code_t fault_code);
void CommandManager_OutputDiag(const char *level, const char *module, const char *detail);
void CommandManager_OutputParam(char subcommand, float value);
void CommandManager_OutputState(char subcommand, uint8_t value);

#endif /* COMMAND_MANAGER_DIAG_H */
