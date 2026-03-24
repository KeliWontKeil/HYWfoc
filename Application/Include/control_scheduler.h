#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include "gd32f30x.h"
#include <stddef.h>

/* Callback type macro for portability/customization */
#define CONTROL_SCHEDULER_CALLBACK_TYPE(name) void (*name)(void)
typedef CONTROL_SCHEDULER_CALLBACK_TYPE(ControlScheduler_Callback_t);

typedef enum {
    CONTROL_SCHED_RATE_1KHZ = 0,
    CONTROL_SCHED_RATE_100HZ,
    CONTROL_SCHED_RATE_10HZ,
    CONTROL_SCHED_RATE_1HZ,
    CONTROL_SCHED_RATE_COUNT
} ControlScheduler_Rate_t;

void ControlScheduler_Init(void);
void ControlScheduler_RunTick(void);
void ControlScheduler_EnableDWT(void);
uint32_t ControlScheduler_GetExecutionCycles(void);
uint16_t ControlScheduler_GetTickCounter(void);
void ControlScheduler_ResetTickCounter(void);

void ControlScheduler_SetCallback(ControlScheduler_Rate_t rate, ControlScheduler_Callback_t callback);
void ControlScheduler_ClearCallback(ControlScheduler_Rate_t rate);
void ControlScheduler_ClearAllCallbacks(void);

#endif /* CONTROL_SCHEDULER_H */
