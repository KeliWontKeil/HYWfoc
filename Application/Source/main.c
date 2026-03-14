#include "main.h"

/* Function prototypes */
static void LED_Blink_1Hz(void);

int main(void)
{
    systick_config();
    LED_Init();
    
    USART1_Init();
    
    /* Initialize TIMER2 first (master timer) */
    Timer2_Init(0, 4999);  /* 24kHz PWM frequency*/
    
    /* Initialize PWM (TIMER0 as slave) */
    PWM_Init(24, 2);
    
    /* Start TIMER2 (master) */
    Timer2_Start();
    
    /* Start PWM (slave will be triggered by master) */
    PWM_Start();
    
    Timer1_Algorithm_Init();
    
    /* Initialize I2C and AS5600 */
    I2C0_Init();
    AS5600_Init();
    
    /* Initialize ADC for synchronous current sampling */
    ADC_Init();
    ADC_Start();
    
    /* Calibrate ADC zero offset */
    ADC_CalibrateZeroOffset();
    
    /* Set LED blink callback for 1Hz task */
    Timer1_SetAlgorithmCallback(TIMER1_CALLBACK_1HZ, LED_Blink_1Hz);
    
    Set_LED(3);
    
    USART1_SendString("\r\n=== GD32F303CC FOC System Started ===\r\n");
    USART1_SendString("USART1 Loopback Enabled\r\n");
    USART1_SendString("TIMER2: Master timer (24kHz) with TRGO output\r\n");
    USART1_SendString("TIMER0: Slave timer (central aligned PWM, 3-ch complementary)\r\n");
    USART1_SendString("Timer1: 1kHz algorithm timing with LED3 blink (callback)\r\n");
    USART1_SendString("AS5600 Magnetic Encoder I2C Test Enabled\r\n");
    USART1_SendString("ADC: Synchronous sampling (ADC0+ADC1, TIMER2_TRGO triggered)\r\n");
    USART1_SendString("UART Debug: Current and encoder monitoring\r\n");
    USART1_SendString("Type any character to echo...\r\n\r\n");
    
    PWM_SetDutyCycle(PWM_CHANNEL_0, 10);
    PWM_SetDutyCycle(PWM_CHANNEL_1, 20);
    PWM_SetDutyCycle(PWM_CHANNEL_2, 30);
    
    while (1)
    {
        UART_Debug_OutputAll();
        delay_1ms(500);
    }
}

static void LED_Blink_1Hz(void)
{
    static uint8_t led3_state = 0;
    
    if (led3_state == 0)
    {
        Set_LED(3);
        led3_state = 1;
    }
    else
    {
        Reset_LED(3);
        led3_state = 0;
    }
}