#ifndef _TIMER5_H_
#define _TIMER5_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER5 基本定时器：控制调度节拍（1kHz），接管原 TIMER1 的控制调度功能。 */
#define TIMER5_PERIPH           TIMER5
#define TIMER5_RCU              RCU_TIMER5
#define TIMER5_IRQn             TIMER5_IRQn

/* Interrupt handler callback type */
typedef void (*timer5_callback_t)(void);

void Timer5_Init(uint32_t prescaler, uint32_t period);
void Timer5_Start(void);
void Timer5_Stop(void);
void Timer5_SetUpdateInterruptEnabled(uint8_t enable);
void Timer5_SetCallback(timer5_callback_t callback);
void Timer5_IRQHandler_Internal(void);

#endif /* _TIMER5_H_ */
