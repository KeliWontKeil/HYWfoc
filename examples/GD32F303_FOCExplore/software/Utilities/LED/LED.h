#ifndef _LED_h_
#define _LED_h_

#include "gd32f30x.h"

#define LEDR_GPIO_RCU RCU_GPIOC
#define LEDR_GPIO GPIOC
#define LEDR_GPIO_PIN GPIO_PIN_15

#define LEDB_GPIO_RCU RCU_GPIOC
#define LEDB_GPIO GPIOC
#define LEDB_GPIO_PIN GPIO_PIN_13

#define LEDG_GPIO_RCU RCU_GPIOC
#define LEDG_GPIO GPIOC
#define LEDG_GPIO_PIN GPIO_PIN_14

void LED_Init(void);
void Set_LED(uint8_t LEDX);
void Reset_LED(uint8_t LEDX);
void LED_SetState(uint8_t led_index, uint8_t on);

#endif
