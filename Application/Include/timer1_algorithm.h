#ifndef _TIMER1_ALGORITHM_H_
#define _TIMER1_ALGORITHM_H_

#include "gd32f30x.h"
#include "timer1.h"
#include "LED.h"
#include <stddef.h>

/* Callback function type */
typedef void (*Timer1_Callback_t)(void);

/* Callback rates */
typedef enum {
    TIMER1_CALLBACK_1KHZ = 0,
    TIMER1_CALLBACK_100HZ,
    TIMER1_CALLBACK_10HZ,
    TIMER1_CALLBACK_1HZ,
    TIMER1_CALLBACK_COUNT
} Timer1_CallbackRate_t;

void Timer1_Algorithm_Init(void);
void Timer1_Algorithm_Handler(void);
void Timer1_EnableDWT(void);
uint32_t Timer1_GetExecutionTime(void);
uint16_t Timer1_GetCounter(void);
void Timer1_ResetCounter(void);

/* Callback management functions */
void Timer1_SetAlgorithmCallback(Timer1_CallbackRate_t rate, Timer1_Callback_t callback);
void Timer1_ClearAlgorithmCallback(Timer1_CallbackRate_t rate);
void Timer1_ClearAllCallbacks(void);

#endif /* _TIMER1_ALGORITHM_H_ */
