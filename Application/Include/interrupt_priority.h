#ifndef _INTERRUPT_PRIORITY_H_
#define _INTERRUPT_PRIORITY_H_

/*!
    \file    interrupt_priority.h
    \brief   Centralized interrupt priority definitions

    \version 2026-3-9, V1.0.0, user interrupt priorities
*/

#include "gd32f30x.h"

/* Interrupt priority levels (0-15, where 0 is highest priority) */
#define NVIC_PRIORITY_GROUPING NVIC_PRIGROUP_PRE4_SUB0

/* Timer interrupt priorities */
#define TIMER1_PRIORITY_GROUP      1
#define TIMER1_PRIORITY_SUBGROUP   0

#define TIMER2_PRIORITY_GROUP      2
#define TIMER2_PRIORITY_SUBGROUP   0

/* USART interrupt priorities */
#define USART1_PRIORITY_GROUP      3
#define USART1_PRIORITY_SUBGROUP   0

/* SysTick interrupt priority */
#define SYSTICK_PRIORITY_GROUP     4
#define SYSTICK_PRIORITY_SUBGROUP  0

/* Helper macros for NVIC configuration */
#define NVIC_CONFIG(irqn, pri_group, pri_subgroup) \
    nvic_irq_enable((irqn), (pri_group), (pri_subgroup))

#endif /* _INTERRUPT_PRIORITY_H_ */
