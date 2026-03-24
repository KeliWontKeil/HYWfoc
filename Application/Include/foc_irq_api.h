#ifndef FOC_IRQ_API_H
#define FOC_IRQ_API_H

void FOC_IRQ_OnCurrentSampleReady(void);
void FOC_IRQ_OnTelemetryEvent(void);
void FOC_IRQ_OnControlTick(void);
void FOC_IRQ_OnHighRateTick(void);
void FOC_IRQ_OnCurrentSampleTransferDone(void);

#endif /* FOC_IRQ_API_H */
