#include "main.h"

/* Function prototypes */
static void LED_Blink_1Hz(void);
static void Motor_Control_Loop(void);

int main(void)
{
    systick_config();                                      

    LED_Init();
    Set_LED(3);

    /* Initialize PWM (TIMER0 as slave) */
    PWM_Init(24, 2);
    PWM_SetDutyCycle(PWM_CHANNEL_0, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_1, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_2, 0);
    PWM_Start();

    Timer1_Algorithm_Init();
		/* Set callback for tasks */
    Timer1_SetAlgorithmCallback(TIMER1_CALLBACK_1HZ, LED_Blink_1Hz);
    Timer1_SetAlgorithmCallback(TIMER1_CALLBACK_1KHZ, Motor_Control_Loop);

    USART1_Init();
    USART1_SendString("\r\n=== GD32F303CC FOC System Started ===\r\n");
    USART1_SendString("USART1 Loopback Enabled\r\n");
    USART1_SendString("TIMER2: Master timer (24kHz) with TRGO output\r\n");
    USART1_SendString("TIMER0: Slave timer (central aligned PWM, 3-ch complementary)\r\n");
    USART1_SendString("Timer1: 1kHz algorithm timing with LED3 blink (callback)\r\n");
    USART1_SendString("AS5600 Magnetic Encoder I2C Test Enabled\r\n");
    USART1_SendString("ADC: Synchronous sampling (ADC0+ADC1, TIMER2_TRGO triggered)\r\n");
    USART1_SendString("UART Debug: Current and encoder monitoring\r\n");
    USART1_SendString("Init Sensors...\r\n\r\n");

    /* Initialize sensor module with Kalman filters */
    Sensor_Init();

    /* Initialize SVPWM simulation output (for algorithm test only). */
    SVPWM_Init(12.0f);
    
    Timer1_Algorithm_Start();

    while (1)
    {
        //UART_Debug_OutputAll();
        UART_Debug_OutputOscilloscope();
        delay_1ms(20);
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

static void Motor_Control_Loop(void)
{
    static float theta = 0.0f;
    svpwm_input_t sv_input;

    /* 5Hz rotating vector update at 1kHz loop. */
    sv_input.alpha = cosf(theta);
    sv_input.beta = sinf(theta);
    sv_input.set_voltage = 11.4f;
    SVPWM_Update(&sv_input);

    theta += 2.0f * 3.1415926f * 5.0f * 0.0001f;
    if (theta >= 2.0f * 3.1415926f)
    {
        theta = 0.0f;
    }

    Sensor_ReadAll();
}
