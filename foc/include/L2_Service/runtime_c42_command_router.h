#ifndef RUNTIME_C42_COMMAND_ROUTER_H
#define RUNTIME_C42_COMMAND_ROUTER_H

#include <stdint.h>

#include "L2_Service/runtime_c43_store.h"
#include "L3_Algorithm/protocol_core_types.h"

typedef enum {
    RUNTIME_C42_EXEC_OK = 0,
    RUNTIME_C42_EXEC_PARAM_ERROR,
    RUNTIME_C42_EXEC_COMMAND_ERROR
} runtime_c42_exec_result_t;

typedef runtime_c43_runtime_state_t runtime_c42_runtime_view_t;
typedef runtime_c43_params_t runtime_c42_params_view_t;
typedef runtime_c43_states_t runtime_c42_states_view_t;

void RuntimeC42_Init(void);
runtime_c42_runtime_view_t *RuntimeC42_Runtime(void);
runtime_c42_params_view_t *RuntimeC42_Params(void);
runtime_c42_states_view_t *RuntimeC42_States(void);

void RuntimeC42_UpdateReportMode(void);
runtime_c42_exec_result_t RuntimeC42_RouteCommand(const protocol_command_t *cmd);

void RuntimeC42_BuildSnapshot(runtime_snapshot_t *snapshot);
void RuntimeC42_ClearDirty(void);

void RuntimeC42_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeC42_OutputRuntimeSummary(void);
void RuntimeC42_OutputFaultControlSummary(void);
const char *RuntimeC42_GetFaultName(uint8_t fault_code);
void RuntimeC42_WriteText(const char *text);

void RuntimeC42_WriteStatusFrameError(void);
void RuntimeC42_WriteStatusParamInvalid(void);
void RuntimeC42_WriteStatusCmdInvalid(void);

uint8_t RuntimeC42_RecoverFaultAndReinit(void);

#endif /* RUNTIME_C42_COMMAND_ROUTER_H */
