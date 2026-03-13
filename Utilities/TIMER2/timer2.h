#ifndef _TIMER2_H_
#define _TIMER2_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER2 peripheral definitions */
#define TIMER2_PERIPH           TIMER2
#define TIMER2_RCU              RCU_TIMER2
#define TIMER2_IRQn             TIMER2_IRQn

/* Interrupt handler callback type */
typedef void (*timer2_callback_t)(void);

/* Function prototypes */
void Timer2_Init(uint32_t prescaler, uint32_t period);
void Timer2_Start(void);
void Timer2_Stop(void);
void Timer2_SetCallback(timer2_callback_t callback);

#endif /* _TIMER2_H_ */
