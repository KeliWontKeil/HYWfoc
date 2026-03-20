#ifndef _TIMER3_H_
#define _TIMER3_H_

#include "gd32f30x.h"

/* TIMER3 peripheral definitions */
#define TIMER3_PERIPH                    TIMER3
#define TIMER3_RCU                       RCU_TIMER3

/* TIMER3 uses ITI2 to receive TIMER2 TRGO (update event) for phase-locked sampling. */
#define TIMER3_SYNC_TRIGGER_SOURCE       TIMER_SMCFG_TRGSEL_ITI2

void Timer3_Init(uint32_t prescaler, uint32_t period);
void Timer3_Start(void);
void Timer3_Stop(void);

#endif /* _TIMER3_H_ */
