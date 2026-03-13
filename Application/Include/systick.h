/*!
    \file    systick.h
    \brief   the header file of systick

    \version 2026-2-6, V3.0.3, firmware for GD32F30x
*/

#ifndef SYS_TICK_H
#define SYS_TICK_H

#include <stdint.h>

/* configure systick */
void systick_config(void);
/* delay a time in milliseconds */
void delay_1ms(uint32_t count);
/* delay decrement */
void delay_decrement(void);
/* get current systick tick count */
uint32_t systick_get_tick(void);

#endif /* SYS_TICK_H */
