#include "foc_control.h"
#include "svpwm.h"
#include "pwm.h"
#include <math.h>

#define FOC_TWO_PI 6.2831852f

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   uint8_t pole_pairs,
                    float zero_electrical_angle,
                    uint8_t direction)
{
    if (motor == 0)
    {
        return;
    }

    motor->electrical_angle = 0.0f;
    motor->ud = 0.0f;
    motor->uq = 1.0f;
    motor->set_voltage = set_voltage;
    motor->vbus_voltage = vbus_voltage;
    motor->zero_electrical_angle = zero_electrical_angle;
    motor->direction = direction;
    motor->pole_pairs = pole_pairs;

    motor->alpha = 0.0f;
    motor->beta = 0.0f;
    motor->phase_a = 0.0f;
    motor->phase_b = 0.0f;
    motor->phase_c = 0.0f;
    motor->duty_a = 0.5f;
    motor->duty_b = 0.5f;
    motor->duty_c = 0.5f;
    motor->sector = 0U;
}

void FOC_OpenLoopStep(foc_motor_t *motor, float turn_speed)
{
    float delta_theta;

    if (motor == 0)
    {
        return;
    }

    delta_theta = FOC_TWO_PI * turn_speed * motor->pole_pairs * 0.001f; /* turn_speed is in Hz, dt is 1ms */
    motor->electrical_angle += delta_theta;
    while (motor->electrical_angle >= FOC_TWO_PI)
    {
        motor->electrical_angle -= FOC_TWO_PI;
    }
    while (motor->electrical_angle < 0.0f)
    {
        motor->electrical_angle += FOC_TWO_PI;
    }


    Math_InverseParkTransform(motor->ud,
                              motor->uq,
                              motor->electrical_angle,
                              &motor->alpha,
                              &motor->beta);

    Math_InverseClarkeTransform(motor->alpha,
                                motor->beta,
                                &motor->phase_a,
                                &motor->phase_b,
                                &motor->phase_c);

    SVPWM_Update(motor->phase_a,
                 motor->phase_b,
                 motor->phase_c,
                 motor->set_voltage,
                 motor->vbus_voltage,
                 &motor->sector,
                 &motor->duty_a,
                 &motor->duty_b,
                 &motor->duty_c);

    PWM_SetDutyCycle(PWM_CHANNEL_0, (uint8_t)(motor->duty_a * 100.0f));
    PWM_SetDutyCycle(PWM_CHANNEL_1, (uint8_t)(motor->duty_b * 100.0f));
    PWM_SetDutyCycle(PWM_CHANNEL_2, (uint8_t)(motor->duty_c * 100.0f));
}
