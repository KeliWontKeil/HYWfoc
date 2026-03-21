#include "main.h"

/* Function prototypes */
static void LED_Blink_1Hz(void);
static void Motor_Control_Loop(void);

static foc_motor_t g_motor;

int main(void)
{
    systick_config();                                      

    LED_Init();
    Set_LED(3);

    /* Initialize TIMER2 first (master timer) */
    Timer2_Init(0, 120000 / 24 - 1);  /* 24kHz PWM frequency*/
    Timer2_Start();

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
    USART1_SendString("ADC: Synchronous sampling (ADC0+ADC1, TIMER3_CH3 triggered)\r\n");
    USART1_SendString("UART Debug: Current and encoder monitoring\r\n");
    USART1_SendString("Init Sensors...\r\n\r\n");

    /* Initialize sensor module with Kalman filters */
    Sensor_Init(24);

    /* Initialize SVPWM simulation output (for algorithm test only). */
    SVPWM_Init(24, 2);

    /* Initialize open-loop motor model and targets. */
    FOC_MotorInit(&g_motor, 12.0f, 6.0f, FOC_POLE_PAIRS_UNDEFINED, FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED, FOC_DIR_UNDEFINED);
    //FOC_MotorInit(&g_motor, 12.0f, 6.0f, 7, FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED, FOC_DIR_UNDEFINED);

    printf("mech zero at elec0: %.4f rad, direction: %d ,pole pairs: %d\r\n", g_motor.mech_angle_at_elec_zero_rad, g_motor.direction, g_motor.pole_pairs);
    delay_1ms(1000);
    
    Timer1_Algorithm_Start();

    while (1)
    {
        //UART_Debug_OutputAll();
        UART_Debug_OutputOscilloscope();
        delay_1ms(200);
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
    FOC_OpenLoopStep(&g_motor, 0.5f);

    Sensor_ReadAll();
}
