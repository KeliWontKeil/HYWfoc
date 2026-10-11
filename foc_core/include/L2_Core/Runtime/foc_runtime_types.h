#ifndef FOC_RUNTIME_TYPES_H
#define FOC_RUNTIME_TYPES_H

#include <stdint.h>

typedef enum {
    RUNTIME_INIT_CHECK_COMM = (1U << 0),
    RUNTIME_INIT_CHECK_PROTOCOL = (1U << 1),
    RUNTIME_INIT_CHECK_COMMAND = (1U << 2),
    RUNTIME_INIT_CHECK_DEBUG = (1U << 3),
    RUNTIME_INIT_CHECK_SENSOR = (1U << 4),
    RUNTIME_INIT_CHECK_PWM = (1U << 5),
    RUNTIME_INIT_CHECK_MOTOR = (1U << 6),
    RUNTIME_INIT_CHECK_VBUS = (1U << 7)
} runtime_init_check_t;

/* 就绪前静态可判定项（不含依赖对齐产物的 MOTOR 位，后者由 FOC_Control_VerifyMotor 判定） */
#define RUNTIME_INIT_CHECK_STATIC_ALL ((uint16_t)(RUNTIME_INIT_CHECK_COMM |       \
                                                  RUNTIME_INIT_CHECK_PROTOCOL |   \
                                                  RUNTIME_INIT_CHECK_COMMAND |    \
                                                  RUNTIME_INIT_CHECK_DEBUG |      \
                                                  RUNTIME_INIT_CHECK_SENSOR |     \
                                                  RUNTIME_INIT_CHECK_PWM |        \
                                                  RUNTIME_INIT_CHECK_VBUS))

/* ========== Task rate identifiers for callback registration ========== */
typedef enum {
    FOC_TASK_RATE_FAST_CONTROL = 0,
    FOC_TASK_RATE_SERVICE,
    FOC_TASK_RATE_MONITOR,
    FOC_TASK_RATE_HEARTBEAT,
    FOC_TASK_RATE_COUNT
} FOC_TaskRate_t;

typedef struct {
    uint16_t tick_counter;
    uint32_t execution_cycles;
    void (*callbacks[FOC_TASK_RATE_COUNT])(void);
    uint8_t dwt_enabled;
} control_scheduler_t;

#endif /* FOC_RUNTIME_TYPES_H */
