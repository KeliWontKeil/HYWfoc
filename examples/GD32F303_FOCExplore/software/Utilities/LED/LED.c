#include "LED.h"

/* 逻辑索引→颜色映射：1=蓝(COMM)、2=绿(RUN)、3=红(ERROR)。 */
static void LED_Apply(uint8_t led_index, FlagStatus level)
{
    switch (led_index)
    {
    case 1:
        gpio_bit_write(LEDB_GPIO, LEDB_GPIO_PIN, level);
        break;
    case 2:
        gpio_bit_write(LEDG_GPIO, LEDG_GPIO_PIN, level);
        break;
    case 3:
        gpio_bit_write(LEDR_GPIO, LEDR_GPIO_PIN, level);
        break;
    default:
        break;
    }
}

void LED_Init(void)
{
    rcu_periph_clock_enable(LEDR_GPIO_RCU);
    rcu_periph_clock_enable(LEDB_GPIO_RCU);
    rcu_periph_clock_enable(LEDG_GPIO_RCU);

    gpio_init(LEDR_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LEDR_GPIO_PIN);
    gpio_init(LEDB_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LEDB_GPIO_PIN);
    gpio_init(LEDG_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LEDG_GPIO_PIN);
}

void Set_LED(uint8_t LEDX)
{
    LED_Apply(LEDX, SET);
}

void Reset_LED(uint8_t LEDX)
{
    LED_Apply(LEDX, RESET);
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
