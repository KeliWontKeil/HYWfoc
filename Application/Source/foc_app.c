#include "foc_app.h"

/* Internal function prototypes */
static void LED_Blink_1Hz(void);
static void Motor_Control_Loop(void);

static foc_motor_t g_motor;
static sensor_data_t *sensor;
static foc_pid_t g_torque_current_pid;
static foc_pid_t g_angle_pid;
static foc_pid_t g_speed_pid;

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetHeartbeatIndicator(1U);

    /* Initialize high-rate clock first as modulation timing source. */
    FOC_Platform_HighRateClockInit(FOC_APP_PWM_FREQ_KHZ);

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();
    FOC_Platform_BindControlTickCallback(ControlScheduler_RunTick);
    ControlScheduler_SetCallback(FOC_SCHEDULER_RATE_HEARTBEAT_1HZ, LED_Blink_1Hz);
    ControlScheduler_SetCallback(FOC_SCHEDULER_RATE_CONTROL_1KHZ, Motor_Control_Loop);

    FOC_Platform_TelemetryInit();
    FOC_Platform_TelemetryWrite("\r\n=== GD32F303CC FOC System Started ===\r\n");
    FOC_Platform_TelemetryWrite("USART1 Loopback Enabled\r\n");
    FOC_Platform_TelemetryWrite("High-rate modulation clock active at configured PWM frequency\r\n");
    FOC_Platform_TelemetryWrite("PWM bridge running in center-aligned complementary mode\r\n");
    FOC_Platform_TelemetryWrite("Control scheduler running at 1kHz with heartbeat callback\r\n");
    FOC_Platform_TelemetryWrite("Magnetic encoder feedback enabled\r\n");
    FOC_Platform_TelemetryWrite("Current sampling pipeline enabled\r\n");
    FOC_Platform_TelemetryWrite("Control debug telemetry enabled\r\n");
    FOC_Platform_TelemetryWrite("Init feedback pipeline...\r\n\r\n");

    Sensor_Init(FOC_APP_SENSOR_SAMPLE_FREQ_KHZ);

    /* Initialize SVPWM output and interpolation callback. */
    SVPWM_Init(FOC_APP_PWM_FREQ_KHZ, 2U);
    FOC_Platform_RegisterHighRateCallback(SVPWM_InterpolationISR);
    FOC_Platform_StartHighRateClock();

    /* Initialize motor model and targets. */
    FOC_MotorInit(&g_motor, 12.0f, 11.4f, 6.1f, 7, 3.22, FOC_DIR_NORMAL);

    /* Initialize current-loop PID states (safe defaults for future closed-loop enabling). */
    FOC_PIDInit(&g_torque_current_pid, 0.0f, 0.0f, 0.0f, -g_motor.set_voltage, g_motor.set_voltage);
    FOC_PIDInit(&g_angle_pid, 3.0f, 1.0f, 0.01f, -g_motor.set_voltage, g_motor.set_voltage);
    FOC_PIDInit(&g_speed_pid, 3.0f, 0.5f, 0.0f, -g_motor.set_voltage, g_motor.set_voltage);

    printf("mech zero at elec0: %.4f rad, direction: %d ,pole pairs: %d\r\n", g_motor.mech_angle_at_elec_zero_rad, g_motor.direction, g_motor.pole_pairs);
    FOC_Platform_WaitMs(1000U);
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
}

void FOC_App_Loop(void)
{
    //UART_Debug_OutputOscilloscope(sensor, &g_motor);
    UART_Debug_OutputAll();
    FOC_Platform_WaitMs(FOC_APP_MAIN_LOOP_DELAY_MS);
}

static void LED_Blink_1Hz(void)
{
    static uint8_t led3_state = 0;

    if (led3_state == 0)
    {
        FOC_Platform_SetHeartbeatIndicator(1U);
        led3_state = 1;
    }
    else
    {
        FOC_Platform_SetHeartbeatIndicator(0U);
        led3_state = 0;
    }
}

static void Motor_Control_Loop(void)
{
    Sensor_ReadAll();

    sensor = Sensor_GetData();

    if (sensor == 0)
    {
        return;
    }

    if ((sensor->adc_valid == 0U) || (sensor->encoder_valid == 0U))
    {
        return;
    }

    FOC_SpeedAngleControlStep(&g_motor,
                              &g_speed_pid,
                              &g_angle_pid,
                              &g_torque_current_pid,
                              FOC_APP_TARGET_ANGLE_RAD,
                              FOC_APP_ANGLE_POSITION_SPEED_RAD_S,
                              sensor,
                              FOC_APP_CONTROL_DT_SEC,
                              FOC_TORQUE_MODE_CURRENT_PID);
}
