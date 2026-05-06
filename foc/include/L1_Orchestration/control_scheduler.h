#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include <stddef.h>
#include <stdint.h>

#include "LS_Config/foc_scheduler_types.h"
#include "LS_Config/foc_config.h"

typedef void (*ControlScheduler_Callback_t)(void);

void ControlScheduler_Init(void);
void ControlScheduler_RunTick(void);
void ControlScheduler_EnableDWT(void);
uint32_t ControlScheduler_GetExecutionCycles(void);
uint16_t ControlScheduler_GetTickCounter(void);
/*
 * Unused-interface audit tag: RESERVED_TEST_HOOK.
 * Reserved for deterministic test/re-sync scenarios.
 */
void ControlScheduler_ResetTickCounter(void);

void ControlScheduler_SetCallback(FOC_TaskRate_t rate, ControlScheduler_Callback_t callback);
void ControlScheduler_ClearCallback(FOC_TaskRate_t rate);
void ControlScheduler_ClearAllCallbacks(void);

#endif /* CONTROL_SCHEDULER_H */
