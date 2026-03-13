#ifndef _LED_h_
#define _LED_h_

#include "gd32f30x.h"

#define LED1_GPIO_RCU RCU_GPIOB
#define LED1_GPIO GPIOB
#define LED1_GPIO_PIN GPIO_PIN_3

#define LED2_GPIO_RCU RCU_GPIOB
#define LED2_GPIO GPIOB
#define LED2_GPIO_PIN GPIO_PIN_4

#define LED3_GPIO_RCU RCU_GPIOB
#define LED3_GPIO GPIOB
#define LED3_GPIO_PIN GPIO_PIN_5

void LED_Init(void);
void Set_LED(uint8_t LEDX);
void Reset_LED(uint8_t LEDX);

#endif
