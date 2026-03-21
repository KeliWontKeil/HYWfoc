#include "foc_control.h"
#include "svpwm.h"
#include "pwm.h"
#include "as5600.h"
#include "systick.h"
#include <math.h>

static float FOC_WrapRad(float angle)
{
    while (angle >= FOC_TWO_PI)
    {
        angle -= FOC_TWO_PI;
    }
    while (angle < 0.0f)
    {
        angle += FOC_TWO_PI;
    }
    return angle;
}

static float FOC_WrapRadDelta(float angle)
{
    while (angle > 3.1415926f)
    {
        angle -= FOC_TWO_PI;
    }
    while (angle < -3.1415926f)
    {
        angle += FOC_TWO_PI;
    }
    return angle;
}

static float FOC_GetDirectionSign(uint8_t direction)
{
    return (direction == FOC_DIR_REVERSED) ? -1.0f : 1.0f;
}

static uint8_t FOC_ClampPolePairs(int32_t pole_pairs)
{
    if (pole_pairs < 1)
    {
        return 1U;
    }
    if (pole_pairs > 32)
    {
        return 32U;
    }
    return (uint8_t)pole_pairs;
}

static uint8_t FOC_ReadMechanicalAngleRad(float *angle_rad)
{
    uint16_t angle_raw;

    if ((angle_rad == 0) || (AS5600_ReadAngle(&angle_raw) != I2C_OK))
    {
        return 0U;
    }

    *angle_rad = (float)angle_raw * AS5600_ANGLE_TO_RAD;
    return 1U;
}

static void FOC_ApplyElectricalAngle(foc_motor_t *motor, float electrical_angle)
{
    float Openloop_electrical_angle = FOC_WrapRad(electrical_angle);

    motor->electrical_phase_angle = Openloop_electrical_angle;

    Math_InverseParkTransform(motor->ud,
                              motor->uq,
                              Openloop_electrical_angle,
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

static uint8_t FOC_SampleLockedMechanicalAngle(foc_motor_t *motor,
                                               float electrical_angle,
                                               uint16_t settle_ms,
                                               uint16_t sample_count,
                                               float *mech_angle_rad)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t i;

    if ((motor == 0) || (mech_angle_rad == 0) || (sample_count == 0U))
    {
        return 0U;
    }

    FOC_ApplyElectricalAngle(motor, electrical_angle);
    delay_1ms(settle_ms);

    for (i = 0U; i < sample_count; i++)
    {
        float sample_rad;

        if (FOC_ReadMechanicalAngleRad(&sample_rad) == 0U)
        {
            continue;
        }

        sin_sum += sinf(sample_rad);
        cos_sum += cosf(sample_rad);
        delay_1ms(FOC_CALIB_SETTLE_MS);
    }

    if ((fabsf(sin_sum) < 1e-6f) && (fabsf(cos_sum) < 1e-6f))
    {
        return 0U;
    }

    *mech_angle_rad = FOC_WrapRad(atan2f(sin_sum, cos_sum));
    return 1U;
}

static uint8_t FOC_EstimateDirectionAndPolePairs(foc_motor_t *motor,
                                                 uint8_t *direction_est,
                                                 uint8_t *pole_pairs_est)
{
    float prev_mech_rad = 0.0f;
    float prev_elec_rad = 0.0f;
    float sum_d_mech = 0.0f;
    float sum_d_elec = 0.0f;
    uint8_t has_prev = 0U;
    uint16_t i;

    if ((motor == 0) || (direction_est == 0) || (pole_pairs_est == 0))
    {
        return 0U;
    }

    /* Use one-way stepping over a larger electrical span to reduce inertia bias. */
    for (i = 0U; i <= FOC_CALIB_STEP_COUNT; i++)
    {
        float elec_target = FOC_CALIB_STEP_ELEC_RAD * (float)i;
        float mech_rad;

        if (FOC_SampleLockedMechanicalAngle(motor,
                                            elec_target,
                                            FOC_CALIB_STEP_SETTLE_MS,
                                            FOC_CALIB_STEP_SAMPLE_COUNT,
                                            &mech_rad) == 0U)
        {
            continue;
        }

        if (has_prev == 0U)
        {
            prev_mech_rad = mech_rad;
            prev_elec_rad = elec_target;
            has_prev = 1U;
            continue;
        }

        {
            float d_mech = FOC_WrapRadDelta(mech_rad - prev_mech_rad);
            float d_elec = elec_target - prev_elec_rad;

            if (fabsf(d_mech) >= FOC_CALIB_MIN_MECH_STEP_RAD)
            {
                sum_d_mech += d_mech;
                sum_d_elec += d_elec;
            }
        }

        prev_mech_rad = mech_rad;
        prev_elec_rad = elec_target;
    }

    if ((fabsf(sum_d_mech) < FOC_CALIB_MIN_MECH_STEP_RAD) || (fabsf(sum_d_elec) < 1e-6f))
    {
        return 0U;
    }

    *direction_est = (sum_d_mech >= 0.0f) ? FOC_DIR_NORMAL : FOC_DIR_REVERSED;
    *pole_pairs_est = FOC_ClampPolePairs((int32_t)(fabsf(sum_d_elec / sum_d_mech) + 0.5f));

    /* Return to zero electrical angle after probe sweep. */
    FOC_ApplyElectricalAngle(motor, 0.0f);
    delay_1ms(FOC_CALIB_STEP_SETTLE_MS);

    return 1U;
}

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   uint8_t direction)
{
    if (motor == 0)
    {
        return;
    }

    motor->electrical_phase_angle = 0.0f;
    motor->ud = 0.0f;
    motor->uq = 1.0f;
    motor->set_voltage = set_voltage;
    motor->vbus_voltage = vbus_voltage;
    motor->pole_pairs = pole_pairs;
    motor->mech_angle_at_elec_zero_rad = mech_angle_at_elec_zero_rad;
    motor->direction = direction;

    motor->alpha = 0.0f;
    motor->beta = 0.0f;
    motor->phase_a = 0.0f;
    motor->phase_b = 0.0f;
    motor->phase_c = 0.0f;
    motor->duty_a = 0.5f;
    motor->duty_b = 0.5f;
    motor->duty_c = 0.5f;
    motor->sector = 0U;

    FOC_CalibrateElectricalAngleAndDirection(motor);
}

void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor)
{
    float calib_uq;
    float backup_ud;
    float backup_uq;
    uint8_t direction_est;
    uint8_t pole_pairs_est;
    float mech_zero_rad_est;
    uint8_t need_zero;
    uint8_t need_direction;
    uint8_t need_pole_pairs;

    if (motor == 0)
    {
        return;
    }

    need_zero = (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) ? 1U : 0U;
    need_direction = (motor->direction == FOC_DIR_UNDEFINED) ? 1U : 0U;
    need_pole_pairs = (motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ? 1U : 0U;

    if ((need_zero == 0U) && (need_direction == 0U) && (need_pole_pairs == 0U))
    {
        return;
    }

    backup_ud = motor->ud;
    backup_uq = motor->uq;

    calib_uq = motor->set_voltage * 0.20f;
    if (calib_uq < 0.5f)
    {
        calib_uq = 0.5f;
    }
    if (calib_uq > 2.5f)
    {
        calib_uq = 2.5f;
    }

    motor->ud = 0.0f;
    motor->uq = calib_uq;

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(motor,
                                            0.0f,
                                            FOC_CALIB_LOCK_SETTLE_MS,
                                            FOC_CALIB_LOCK_SAMPLE_COUNT,
                                            &mech_zero_rad_est) != 0U)
        {
            motor->mech_angle_at_elec_zero_rad = mech_zero_rad_est;
        }
        else
        {
            motor->mech_angle_at_elec_zero_rad = 0.0f;
        }
    }

    if ((need_direction != 0U) || (need_pole_pairs != 0U))
    {
        if (FOC_EstimateDirectionAndPolePairs(motor, &direction_est, &pole_pairs_est) != 0U)
        {
            if (need_direction != 0U)
            {
                motor->direction = direction_est;
            }
            if (need_pole_pairs != 0U)
            {
                motor->pole_pairs = pole_pairs_est;
            }
        }
        else
        {
            if (need_direction != 0U)
            {
                motor->direction = FOC_DIR_NORMAL;
            }
            if (need_pole_pairs != 0U)
            {
                motor->pole_pairs = 1U;
            }
        }
    }

    motor->ud = backup_ud;
    motor->uq = backup_uq;
    FOC_ApplyElectricalAngle(motor, 0.0f);
}

void FOC_OpenLoopStep(foc_motor_t *motor, float turn_speed)
{
    float delta_theta;
    float direction_sign;

    if (motor == 0)
    {
        return;
    }

    direction_sign = FOC_GetDirectionSign(motor->direction);
    delta_theta = FOC_TWO_PI * turn_speed * motor->pole_pairs * 0.001f * direction_sign;
    motor->electrical_phase_angle = FOC_WrapRad(motor->electrical_phase_angle + delta_theta);

    /* Open-loop output uses pure electrical phase command; mechanical zero is for sensored mapping. */
    FOC_ApplyElectricalAngle(motor, motor->electrical_phase_angle);
}
