#include "LED.h"

void LED_Init(void)
{
	rcu_periph_clock_enable(LED1_GPIO_RCU);
	rcu_periph_clock_enable(LED2_GPIO_RCU);
	rcu_periph_clock_enable(LED3_GPIO_RCU);
	
	gpio_init(LED1_GPIO,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,LED1_GPIO_PIN);
	gpio_init(LED2_GPIO,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,LED2_GPIO_PIN);
	gpio_init(LED3_GPIO,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,LED3_GPIO_PIN);
	
}

void Set_LED(uint8_t LEDX)
{
	switch(LEDX)
	{
		case 1:
			gpio_bit_set(LED1_GPIO,LED1_GPIO_PIN);
			break;
		case 2:
			gpio_bit_set(LED2_GPIO,LED2_GPIO_PIN);
			break;
		case 3:
			gpio_bit_set(LED3_GPIO,LED3_GPIO_PIN);
			break;
		default:
			break;
	}
}

void Reset_LED(uint8_t LEDX)
{
	switch(LEDX)
	{
		case 1:
			gpio_bit_reset(LED1_GPIO,LED1_GPIO_PIN);
			break;
		case 2:
			gpio_bit_reset(LED2_GPIO,LED2_GPIO_PIN);
			break;
		case 3:
			gpio_bit_reset(LED3_GPIO,LED3_GPIO_PIN);
			break;
		default:
			break;
	}
}

void LED_SetState(uint8_t led_index, uint8_t on)
{
	if (on != 0U)
	{
		Set_LED(led_index);
	}
	else
	{
		Reset_LED(led_index);
	}
}
