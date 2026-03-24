#include "foc_irq_api.h"

void FOC_IRQ_OnCurrentSampleReady(void)
{
    extern void ADC_IRQHandler_Internal(void);
    ADC_IRQHandler_Internal();
}

void FOC_IRQ_OnTelemetryEvent(void)
{
    extern void USART1_IRQHandler_Internal(void);
    USART1_IRQHandler_Internal();
}

void FOC_IRQ_OnControlTick(void)
{
    extern void Timer1_IRQHandler_Internal(void);
    Timer1_IRQHandler_Internal();
}

void FOC_IRQ_OnHighRateTick(void)
{
    extern void Timer2_IRQHandler_Internal(void);
    Timer2_IRQHandler_Internal();
}

void FOC_IRQ_OnCurrentSampleTransferDone(void)
{
    extern void ADC_DMA_IRQHandler_Internal(void);
    ADC_DMA_IRQHandler_Internal();
}
