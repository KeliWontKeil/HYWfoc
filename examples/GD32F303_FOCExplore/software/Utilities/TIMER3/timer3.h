#ifndef _TIMER3_H_
#define _TIMER3_H_

#include "gd32f30x.h"

#define TIMER3_VIS_TOGGLE_ENABLE 0U
#if (TIMER3_VIS_TOGGLE_ENABLE == 1U)
/* 临时调试：借 USART2_TX(PB10) 引脚作普通 IO，在 TIMER3 中断翻转电平以观察采样频率。
 * 注意：TIMER3_Init 会将该引脚覆盖为普通输出，USART2 TX 暂时不可用。 */
#define TIMER3_VIS_GPIO_RCU RCU_GPIOB
#define TIMER3_VIS_GPIO_PORT GPIOB
#define TIMER3_VIS_GPIO_PIN GPIO_PIN_10
#endif

/* TIMER3 peripheral definitions */
#define TIMER3_PERIPH                    TIMER3
#define TIMER3_RCU                       RCU_TIMER3

/* TIMER3 uses ITI1 (TIMER1 TRGO / sync master update) for phase-locked sampling. */
#define TIMER3_SYNC_TRIGGER_SOURCE       TIMER_SMCFG_TRGSEL_ITI1

void Timer3_Init(uint32_t prescaler, uint32_t period);
void Timer3_Start(void);
void Timer3_Stop(void);
void Timer3_SetSampleOffsetPercent(float percent);

#endif /* _TIMER3_H_ */
