#include "main.h"

int main(void)
{
    rcu_periph_clock_enable(RCU_AF);
    /* SWD remap */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    /* USART0 remap to PB6/PB7 (default PA9/PA10 conflicts with PWM) */
    gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
    /* I2C0 remap to PB8/PB9 (default PB6/PB7 now used by USART0) */
    gpio_pin_remap_config(GPIO_I2C0_REMAP, ENABLE);

    FOC_App_Init();
    FOC_App_Start();

    while (1)
    {
        FOC_App_Loop();
        /* User application logic can be inserted here without changing FOC app internals. */
    }
}
