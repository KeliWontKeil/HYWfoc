#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include <stddef.h>
#include <stdint.h>

#include "foc_shared_types.h"
#include "foc_config.h"

void ControlScheduler_Init(void);
void ControlScheduler_RunTick(void);
void ControlScheduler_EnableDWT(void);
uint32_t ControlScheduler_GetExecutionCycles(void);
uint16_t ControlScheduler_GetTickCounter(void);
void ControlScheduler_ResetTickCounter(void);

void ControlScheduler_SetCallback(FOC_TaskRate_t rate, FOC_SchedulerCallback_t callback);
void ControlScheduler_ClearCallback(FOC_TaskRate_t rate);
void ControlScheduler_ClearAllCallbacks(void);

#endif /* CONTROL_SCHEDULER_H */
