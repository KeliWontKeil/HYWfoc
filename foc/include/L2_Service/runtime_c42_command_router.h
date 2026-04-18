#ifndef RUNTIME_COMMAND_ROUTER_H
#define RUNTIME_COMMAND_ROUTER_H

#include <stdint.h>

#include "L2_Service/runtime_c43_store.h"
#include "L3_Algorithm/protocol_core_types.h"

typedef enum {
    RUNTIME_CMD_EXEC_OK = 0,
    RUNTIME_CMD_EXEC_PARAM_ERROR,
    RUNTIME_CMD_EXEC_COMMAND_ERROR
} runtime_command_exec_result_t;

typedef runtime_runtime_state_t runtime_runtime_view_t;
typedef runtime_params_t runtime_params_view_t;
typedef runtime_states_t runtime_states_view_t;

void RuntimeCommandRouter_Init(void);
runtime_runtime_view_t *RuntimeCommandRouter_Runtime(void);
runtime_params_view_t *RuntimeCommandRouter_Params(void);
runtime_states_view_t *RuntimeCommandRouter_States(void);

void RuntimeCommandRouter_UpdateReportMode(void);
runtime_command_exec_result_t RuntimeCommandRouter_Execute(const protocol_command_t *cmd);

void RuntimeCommandRouter_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeCommandRouter_ClearDirty(void);

void RuntimeCommandRouter_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeCommandRouter_OutputRuntimeSummary(void);
void RuntimeCommandRouter_OutputFaultControlSummary(void);
const char *RuntimeCommandRouter_GetFaultName(uint8_t fault_code);
void RuntimeCommandRouter_WriteText(const char *text);

void RuntimeCommandRouter_WriteStatusFrameError(void);
void RuntimeCommandRouter_WriteStatusParamInvalid(void);
void RuntimeCommandRouter_WriteStatusCmdInvalid(void);

uint8_t RuntimeCommandRouter_RecoverFaultAndReinit(void);

#endif /* RUNTIME_COMMAND_ROUTER_H */

