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

/* ── 定时器映射关系（配置优先级前请确认） ──
 * TIMER0 : PWM 输出 + PWM 更新 ISR
 *          - 双 ISR 模式：插值 + 守卫 + 电流环
 *          - 三 ISR 模式：仅插值 + 守卫
 * TIMER1 : 控制调度节拍 ISR（1kHz，外环/服务/监控）
 * TIMER2 : PWM 同步主定时器（24kHz 自由更新，从机重启源）
 * TIMER3 : ADC 采样触发（同步 TIMER2，CH3 比较事件）
 * TIMER4 : AUX 辅助定时器（三 ISR 模式电流环 ISR）
 *          - L3 API: FOC_Platform_AuxTimerId_t::FOC_AUX_TIMER_CURRENT_LOOP
 *          - 实例层: Utilities/AUXTIMER/auxtimer.h
 */
/* Timer interrupt priorities */
#define TIMER0_UP_PRIORITY_GROUP   1
#define TIMER0_UP_PRIORITY_SUBGROUP 0

/* AUX timer priority: 低于 PWM ISR 与调度 ISR，三 ISR 模式电流环使用 */
#define TIMER4_PRIORITY_GROUP      2
#define TIMER4_PRIORITY_SUBGROUP   0

#define TIMER1_PRIORITY_GROUP      2
#define TIMER1_PRIORITY_SUBGROUP   0

#define TIMER2_PRIORITY_GROUP      0
#define TIMER2_PRIORITY_SUBGROUP   3

#define TIMER3_PRIORITY_GROUP      0
#define TIMER3_PRIORITY_SUBGROUP   1

/* ADC interrupt priorities */
#define ADC0_1_PRIORITY_GROUP      0
#define ADC0_1_PRIORITY_SUBGROUP   2

/* ADC DMA interrupt priorities */
#define ADC_DMA_PRIORITY_GROUP     0
#define ADC_DMA_PRIORITY_SUBGROUP  0

/* USART interrupt priorities */
#define USART1_PRIORITY_GROUP      3
#define USART1_PRIORITY_SUBGROUP   0

/* USART interrupt priorities */
#define USART2_PRIORITY_GROUP      4
#define USART2_PRIORITY_SUBGROUP   0

/* SysTick interrupt priority */
#define SYSTICK_PRIORITY_GROUP     5
#define SYSTICK_PRIORITY_SUBGROUP  0

/* Helper macros for NVIC configuration */
#define NVIC_CONFIG(irqn, pri_group, pri_subgroup) \
    nvic_irq_enable((irqn), (pri_group), (pri_subgroup))

#endif /* _INTERRUPT_PRIORITY_H_ */
