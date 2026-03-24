#include "foc_app.h"

#include <stdio.h>

#include "foc_platform_api.h"
#include "foc_control.h"
#include "svpwm.h"

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

    FOC_Platform_ControlSchedulerInit();
    FOC_Platform_RegisterControlSchedulerCallback(CONTROL_SCHED_RATE_1HZ, LED_Blink_1Hz);
    FOC_Platform_RegisterControlSchedulerCallback(CONTROL_SCHED_RATE_1KHZ, Motor_Control_Loop);

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

    FOC_Platform_FeedbackPipelineInit(24U);

    /* Initialize SVPWM output and interpolation callback. */
    FOC_Platform_ModulationInit(24U, 2U);
    FOC_Platform_RegisterHighRateCallback(SVPWM_InterpolationISR);
    FOC_Platform_StartHighRateClock();

    /* Initialize motor model and targets. */
    FOC_MotorInit(&g_motor, 12.0f, 11.4f, 6.1f, 7, FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED, FOC_DIR_UNDEFINED);

    /* Initialize current-loop PID states (safe defaults for future closed-loop enabling). */
    FOC_PIDInit(&g_torque_current_pid, 1.0f, 0.2f, 0.0f, -g_motor.set_voltage, g_motor.set_voltage);
    FOC_PIDInit(&g_angle_pid, 6.0f, 1.0f, 0.01f, -g_motor.set_voltage, g_motor.set_voltage);
    FOC_PIDInit(&g_speed_pid, 3.0f, 0.5f, 0.0f, -g_motor.set_voltage, g_motor.set_voltage);

    printf("mech zero at elec0: %.4f rad, direction: %d ,pole pairs: %d\r\n", g_motor.mech_angle_at_elec_zero_rad, g_motor.direction, g_motor.pole_pairs);
    FOC_Platform_WaitMs(1000U);
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlScheduler();
}

void FOC_App_Loop(void)
{
    FOC_Platform_PublishDebugWave(g_motor.iq_measured);
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
    foc_control_feedback_t feedback;

    FOC_Platform_RefreshFeedbackSample();

    if (FOC_Platform_ReadControlFeedback(&feedback) == 0U)
    {
        return;
    }

    if ((feedback.adc_valid == 0U) || (feedback.encoder_valid == 0U))
    {
        return;
    }

    FOC_SpeedControlStep(&g_motor,
                         &g_speed_pid,
                         &g_torque_current_pid,
                         16.0f,
                         feedback.phase_current_a,
                         feedback.phase_current_b,
                         feedback.phase_current_c,
                         feedback.mech_angle_rad,
                         0.001f,
                         FOC_TORQUE_MODE_CURRENT_PID);
}
