#ifndef _TIMER1_H_
#define _TIMER1_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER1：同步主定时器，TRGO=UPDATE 同步 PWM(TIMER0) 与 TIMER3(ADC 采样触发)。
 * 原控制调度节拍功能已移至 TIMER5（基本定时器）。 */
#define TIMER1_PERIPH           TIMER1
#define TIMER1_RCU              RCU_TIMER1
#define TIMER1_IRQn             TIMER1_IRQn

/* Interrupt handler callback type */
typedef void (*timer1_callback_t)(void);

/* Function prototypes */
void Timer1_Init(uint32_t prescaler, uint32_t period);
void Timer1_Start(void);
void Timer1_Stop(void);
void Timer1_SetUpdateInterruptEnabled(uint8_t enable);

void Timer1_SetCallback(timer1_callback_t callback);
void Timer1_IRQHandler_Internal(void);

#endif /* _TIMER1_H_ */


