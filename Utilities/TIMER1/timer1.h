#ifndef _TIMER1_H_
#define _TIMER1_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER1 peripheral definitions */
#define TIMER1_PERIPH           TIMER1
#define TIMER1_RCU              RCU_TIMER1
#define TIMER1_IRQn             TIMER1_IRQn

/* Interrupt handler callback type */
#define TIMER1_CALLBACK_TYPE(name) void (*name)(void)
typedef TIMER1_CALLBACK_TYPE(timer1_callback_t);

/* Function prototypes */
void Timer1_Init(uint32_t prescaler, uint32_t period);
void Timer1_Start(void);
void Timer1_Stop(void);

void Timer1_SetCallback(timer1_callback_t callback);
void Timer1_IRQHandler_Internal(void);

#endif /* _TIMER1_H_ */


