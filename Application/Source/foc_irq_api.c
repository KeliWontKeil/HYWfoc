#include "foc_irq_api.h"

void FOC_IRQ_OnAdc0_1(void)
{
    extern void ADC_IRQHandler_Internal(void);
    ADC_IRQHandler_Internal();
}

void FOC_IRQ_OnUsart1(void)
{
    extern void USART1_IRQHandler_Internal(void);
    USART1_IRQHandler_Internal();
}

void FOC_IRQ_OnTimer1(void)
{
    extern void Timer1_IRQHandler_Internal(void);
    Timer1_IRQHandler_Internal();
}

void FOC_IRQ_OnTimer2(void)
{
    extern void Timer2_IRQHandler_Internal(void);
    Timer2_IRQHandler_Internal();
}

void FOC_IRQ_OnDma0Channel0(void)
{
    extern void ADC_DMA_IRQHandler_Internal(void);
    ADC_DMA_IRQHandler_Internal();
}
