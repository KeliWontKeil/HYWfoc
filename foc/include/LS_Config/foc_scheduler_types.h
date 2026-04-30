#ifndef FOC_SCHEDULER_TYPES_H
#define FOC_SCHEDULER_TYPES_H

#include <stdint.h>

/* ========== Task rate identifiers for callback registration ========== */
typedef enum {
    FOC_TASK_RATE_FAST_CONTROL = 0,
    FOC_TASK_RATE_SERVICE,
    FOC_TASK_RATE_MONITOR,
    FOC_TASK_RATE_HEARTBEAT,
    FOC_TASK_RATE_COUNT
} FOC_TaskRate_t;

#endif /* FOC_SCHEDULER_TYPES_H */
