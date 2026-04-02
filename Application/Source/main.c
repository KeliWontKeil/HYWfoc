#include "main.h"

int main(void)
{
    rcu_periph_clock_enable(RCU_AF);
    /* SWD remap */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);

    FOC_App_Init();
    FOC_App_Start();

    while (1)
    {
        FOC_App_Loop();
        /* User application logic can be inserted here without changing FOC app internals. */
    }
}
