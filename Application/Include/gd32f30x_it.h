/*!
    \file    gd32f30x_it.h
    \brief   the header file of the ISR

    \version 2026-2-6, V3.0.3, firmware for GD32F30x
*/

#ifndef GD32F30X_IT_H
#define GD32F30X_IT_H

#include "gd32f30x.h"
#include "main.h"
#include "foc_irq_api.h"
#include "systick.h"
#include "LED.h"
#include "usart1.h"
#include "timer1.h"
#include "timer2.h"
#include "adc.h"

/* function declarations */
/* this function handles NMI exception */
void NMI_Handler(void);
/* this function handles HardFault exception */
void HardFault_Handler(void);
/* this function handles MemManage exception */
void MemManage_Handler(void);
/* this function handles BusFault exception */
void BusFault_Handler(void);
/* this function handles UsageFault exception */
void UsageFault_Handler(void);
/* this function handles SVC exception */
void SVC_Handler(void);
/* this function handles DebugMon exception */
void DebugMon_Handler(void);
/* this function handles PendSV exception */
void PendSV_Handler(void);
/* this function handles SysTick exception */
void SysTick_Handler(void);
/* this function handles ADC0 and ADC1 interrupt */
void ADC0_1_IRQHandler(void);
/* this function handles USART1 interrupt */
void USART1_IRQHandler(void);
/* this function handles TIMER1 interrupt */
void TIMER1_IRQHandler(void);
/* this function handles TIMER2 interrupt */
void TIMER2_IRQHandler(void);
/* this function handles DMA0 Channel0 interrupt */
void DMA0_Channel0_IRQHandler(void);

#endif /* GD32F30X_IT_H */
