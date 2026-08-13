#ifndef _TIMER2_H_
#define _TIMER2_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER2 peripheral definitions */
#define TIMER2_PERIPH           TIMER2
#define TIMER2_RCU              RCU_TIMER2
#define TIMER2_IRQn             TIMER2_IRQn

/* Synchronization: lock TIMER2 phase to PWM timer (TIMER0) */
#define TIMER2_SYNC_WITH_PWM_ENABLE      1U
#define TIMER2_SYNC_TRIGGER_SOURCE       TIMER_SMCFG_TRGSEL_ITI0

/* ADC trigger fine tuning around center point, unit: TIMER2 ticks */
#define TIMER2_ADC_TRIGGER_OFFSET_TICKS  0

/* Interrupt handler callback type */
typedef void (*timer2_callback_t)(void);

/* Function prototypes */
void Timer2_Init(uint32_t prescaler, uint32_t period);
void Timer2_Start(void);
void Timer2_Stop(void);

void Timer2_SetCallback(timer2_callback_t callback);
void Timer2_IRQHandler_Internal(void);

#endif /* _TIMER2_H_ */
