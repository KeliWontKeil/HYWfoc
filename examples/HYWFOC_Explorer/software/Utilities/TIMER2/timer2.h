#ifndef _TIMER2_H_
#define _TIMER2_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"

/* TIMER2 peripheral definitions */
#define TIMER2_PERIPH           TIMER2
#define TIMER2_RCU              RCU_TIMER2
#define TIMER2_IRQn             TIMER2_IRQn

/* TIMER2 已从 PWM/ADC 采样同步链退出（PWM 现为主定时器，TIMER3 从属 PWM 触发 ADC）。
 * 本定时器预留为 HALL 传感器输入捕获（CH0/1/2），暂未实现。 */

/* Interrupt handler callback type */
typedef void (*timer2_callback_t)(void);

/* Function prototypes */
void Timer2_Init(uint32_t prescaler, uint32_t period);
void Timer2_Start(void);
void Timer2_Stop(void);

void Timer2_SetCallback(timer2_callback_t callback);
void Timer2_IRQHandler_Internal(void);

#endif /* _TIMER2_H_ */
