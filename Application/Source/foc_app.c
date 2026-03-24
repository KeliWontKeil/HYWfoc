#include "foc_app.h"

/* Internal function prototypes */
static void LED_Blink_1Hz(void);
static void Motor_Control_Loop(void);

static foc_motor_t g_motor;
static foc_pid_t g_torque_current_pid;
static foc_pid_t g_angle_pid;
static foc_pid_t g_speed_pid;

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetHeartbeatIndicator(1U);

    /* Initialize high-rate clock first as modulation timing source. */
    FOC_Platform_HighRateClockInit(24U);

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();
    FOC_Platform_BindControlTickCallback(ControlScheduler_RunTick);
    ControlScheduler_SetCallback(FOC_SCHEDULER_RATE_HEARTBEAT_1HZ, LED_Blink_1Hz);
    ControlScheduler_SetCallback(FOC_SCHEDULER_RATE_CONTROL_1KHZ, Motor_Control_Loop);

    FOC_Platform_TelemetryInit();
    FOC_Platform_TelemetryWrite("\r\n=== GD32F303CC FOC System Started ===\r\n");
    FOC_Platform_TelemetryWrite("USART1 Loopback Enabled\r\n");
    FOC_Platform_TelemetryWrite("High-rate modulation clock active at 24kHz\r\n");
    FOC_Platform_TelemetryWrite("PWM bridge running in center-aligned complementary mode\r\n");
    FOC_Platform_TelemetryWrite("Control scheduler running at 1kHz with heartbeat callback\r\n");
    FOC_Platform_TelemetryWrite("Magnetic encoder feedback enabled\r\n");
    FOC_Platform_TelemetryWrite("Current sampling pipeline enabled\r\n");
    FOC_Platform_TelemetryWrite("Control debug telemetry enabled\r\n");
    FOC_Platform_TelemetryWrite("Init feedback pipeline...\r\n\r\n");

    Sensor_Init(24U);

    /* Initialize SVPWM output and interpolation callback. */
    SVPWM_Init(24U, 2U);
    FOC_Platform_RegisterHighRateCallback(SVPWM_InterpolationISR);
    FOC_Platform_StartHighRateClock();

    /* Initialize motor model and targets. */
    FOC_MotorInit(&g_motor, 12.0f, 11.4f, 6.1f, 7, FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED, FOC_DIR_UNDEFINED);

    /* Initialize current-loop PID states (safe defaults for future closed-loop enabling). */
    FOC_PIDInit(&g_torque_current_pid, 0.0f, 0.0f, 0.0f, -g_motor.set_voltage, g_motor.set_voltage);
    FOC_PIDInit(&g_angle_pid, 6.0f, 2.0f, 0.01f, -g_motor.set_voltage, g_motor.set_voltage);
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
    UART_Debug_OutputOscilloscope(g_motor.iq_measured);
    FOC_Platform_WaitMs(20U);
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
    sensor_data_t *sensor;

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

    FOC_AngleControlStep(&g_motor,
                         &g_angle_pid,
                         &g_torque_current_pid,
                         3.14f,
                         sensor->current_a.output_value,
                         sensor->current_b.output_value,
                         sensor->current_c.output_value,
                         sensor->mech_angle_rad.output_value,
                         0.001f,
                        FOC_TORQUE_MODE_CURRENT_PID);

    /*FOC_SpeedControlStep(&g_motor,
                         &g_speed_pid,
                         &g_torque_current_pid,
                         16.0f,
                         sensor->current_a.output_value,
                         sensor->current_b.output_value,
                         sensor->current_c.output_value,
                         sensor->mech_angle_rad.output_value,
                         0.001f,
                         FOC_TORQUE_MODE_CURRENT_PID);*/
}
