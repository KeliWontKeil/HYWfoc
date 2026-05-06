#ifndef RUNTIME_C4_RUNTIME_CORE_H
#define RUNTIME_C4_RUNTIME_CORE_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"
#include "LS_Config/foc_protocol_types.h"
#include "LS_Config/foc_runtime_types.h"
#include "LS_Config/foc_snapshot_types.h"
#include "LS_Config/foc_config.h"

void RuntimeC4_Init(void);

void RuntimeC4_AccumulateInitChecks(uint16_t pass_mask, uint16_t fail_mask);
uint16_t RuntimeC4_GetInitCheckMask(void);
uint16_t RuntimeC4_GetInitFailMask(void);
void RuntimeC4_ResetSensorInvalidConsecutive(void);
void RuntimeC4_IncrementSensorInvalidConsecutive(void);
uint16_t RuntimeC4_GetSensorInvalidConsecutive(void);
void RuntimeC4_IncrementControlSkipCount(void);
void RuntimeC4_IncrementProtocolErrorCount(void);
void RuntimeC4_IncrementParamErrorCount(void);
void RuntimeC4_SetSystemState(uint8_t system_state);
uint8_t RuntimeC4_GetSystemState(void);
void RuntimeC4_SetCommState(uint8_t comm_state);
void RuntimeC4_SetInitDiag(uint8_t init_diag);
uint8_t RuntimeC4_GetInitDiag(void);
void RuntimeC4_SetLastFaultCode(uint8_t fault_code);
uint8_t RuntimeC4_GetLastExecOk(void);
void RuntimeC4_SetLastExecOk(uint8_t last_exec_ok);

void RuntimeC4_UpdateReportMode(void);
runtime_c4_exec_result_t RuntimeC4_ExecuteCommand(const protocol_command_t *cmd);

void RuntimeC4_ClearDirty(void);

void RuntimeC4_OutputDiag(const char *level, const char *module, const char *detail);
void RuntimeC4_OutputRuntimeSummary(void);
void RuntimeC4_OutputFaultControlSummary(void);
const char *RuntimeC4_GetFaultName(uint8_t fault_code);
void RuntimeC4_WriteText(const char *text);

void RuntimeC4_WriteStatusOK(void);
void RuntimeC4_WriteStatusFrameError(void);
void RuntimeC4_WriteStatusParamInvalid(void);
void RuntimeC4_WriteStatusCmdInvalid(void);

uint8_t RuntimeC4_RecoverFaultAndReinit(void);

#endif /* RUNTIME_C4_RUNTIME_CORE_H */
