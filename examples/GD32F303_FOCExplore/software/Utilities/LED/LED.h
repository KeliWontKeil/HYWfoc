#ifndef _LED_h_
#define _LED_h_

#include "gd32f30x.h"

/* 三色 LED 引脚：红=PB1、蓝=PB2、绿=PA7 */
#define LEDR_GPIO_RCU RCU_GPIOB
#define LEDR_GPIO GPIOB
#define LEDR_GPIO_PIN GPIO_PIN_2

#define LEDB_GPIO_RCU RCU_GPIOB
#define LEDB_GPIO GPIOB
#define LEDB_GPIO_PIN GPIO_PIN_1

#define LEDG_GPIO_RCU RCU_GPIOA
#define LEDG_GPIO GPIOA
#define LEDG_GPIO_PIN GPIO_PIN_7

void LED_Init(void);
void Set_LED(uint8_t LEDX);
void Reset_LED(uint8_t LEDX);
void LED_SetState(uint8_t led_index, uint8_t on);

#endif
