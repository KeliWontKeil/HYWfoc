#ifndef RUNTIME_C42_COMMAND_DISPATCH_H
#define RUNTIME_C42_COMMAND_DISPATCH_H

/* Dispatch consumes shared protocol command types from L3 core only. */
#include "L3_Algorithm/protocol_core_types.h"

typedef enum {
    COMMAND_EXEC_RESULT_OK = 0,
    COMMAND_EXEC_RESULT_PARAM_ERROR,
    COMMAND_EXEC_RESULT_COMMAND_ERROR
} command_exec_result_t;

void CommandManager_DispatchReportInitDiag(void);
command_exec_result_t CommandManager_DispatchExecute(const protocol_command_t *cmd);

#endif /* RUNTIME_C42_COMMAND_DISPATCH_H */
