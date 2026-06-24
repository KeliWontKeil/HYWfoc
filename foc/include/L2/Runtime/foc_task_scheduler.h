#ifndef FOC_TASK_SCHEDULER_H

#define FOC_TASK_SCHEDULER_H


#include <stddef.h>
#include <stdint.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L2/Runtime/foc_scheduler_types.h"
#include "LS_Config/foc_config.h"

void ControlScheduler_Init(control_scheduler_t *sched);
void ControlScheduler_RunTick(control_scheduler_t *sched);
void ControlScheduler_EnableDWT(control_scheduler_t *sched);
uint32_t ControlScheduler_GetExecutionCycles(const control_scheduler_t *sched);
uint16_t ControlScheduler_GetTickCounter(const control_scheduler_t *sched);
void ControlScheduler_ResetTickCounter(control_scheduler_t *sched);

void ControlScheduler_SetCallback(control_scheduler_t *sched, FOC_TaskRate_t rate, void (*callback)(void));
void ControlScheduler_ClearCallback(control_scheduler_t *sched, FOC_TaskRate_t rate);
void ControlScheduler_ClearAllCallbacks(control_scheduler_t *sched);

#endif /* CONTROL_SCHEDULER_H */