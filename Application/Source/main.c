#include "main.h"

/* Function prototypes */
static void LED_Blink_1Hz(void);

int main(void)
{
    systick_config();
    LED_Init();

    USART1_Init();

    PWM_Init(24,2);
    PWM_Start();

    Timer1_Algorithm_Init();

    /* Initialize I2C */
    I2C0_Init();

    /* Set LED blink callback for 1Hz task */
    Timer1_SetAlgorithmCallback(TIMER1_CALLBACK_1HZ, LED_Blink_1Hz);

    Set_LED(3);

    USART1_SendString("\r\n=== GD32F303CC Framework Started ===\r\n");
    USART1_SendString("Basic peripherals initialized\r\n");
    USART1_SendString("Ready for application development\r\n\r\n");

    while (1)
    {
        /* Framework ready - add your application logic here */
    }
}

static void LED_Blink_1Hz(void)
{
    static uint8_t led3_state = 0;
    
    if (led3_state == 0) {
        Set_LED(3);
        led3_state = 1;
    } else {
        Reset_LED(3);
        led3_state = 0;
    }
}
