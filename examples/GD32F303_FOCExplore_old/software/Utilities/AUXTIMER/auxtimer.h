#ifndef _AUXTIMER_H_
#define _AUXTIMER_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER4 用作电流环独立 ISR 定时器（三 ISR 模式） */
#define AUXTIMER_CURRENT_LOOP_PERIPH   TIMER4
#define AUXTIMER_CURRENT_LOOP_RCU      RCU_TIMER4
#define AUXTIMER_CURRENT_LOOP_IRQn     TIMER4_IRQn

/* 优先级：低于 PWM ISR (1,1) 与调度 ISR (1,0)，可被两者抢占 */
#define AUXTIMER_CURRENT_LOOP_PRIO_GROUP    2
#define AUXTIMER_CURRENT_LOOP_PRIO_SUBGROUP 0

/* Interrupt handler callback type */
#define AUXTIMER_CALLBACK_TYPE(name) void (*name)(void)
typedef AUXTIMER_CALLBACK_TYPE(auxtimer_callback_t);

/* Function prototypes */
void AuxTimer_Init(uint32_t prescaler, uint32_t period);
void AuxTimer_Start(void);
void AuxTimer_Stop(void);
void AuxTimer_SetUpdateInterruptEnabled(uint8_t enable);
void AuxTimer_SetCallback(auxtimer_callback_t callback);
void AuxTimer_IRQHandler_Internal(void);

#endif /* _AUXTIMER_H_ */
