#include "algorithm/foc_control_init.h"

#include "interface/foc_platform_api.h"
#include "algorithm/foc_control_internal.h"
#include "config/foc_config.h"

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

    FOC_ControlApplyElectricalAngle(motor, electrical_angle);
    FOC_Platform_WaitMs(settle_ms);

    for (i = 0U; i < sample_count; i++)
    {
        float sample_rad;

        if (FOC_Platform_ReadMechanicalAngleRad(&sample_rad) == 0U)
        {
            continue;
        }

        sin_sum += sinf(sample_rad);
        cos_sum += cosf(sample_rad);
        FOC_Platform_WaitMs(FOC_CALIB_SETTLE_MS);
    }

    if ((fabsf(sin_sum) < 1e-6f) && (fabsf(cos_sum) < 1e-6f))
    {
        return 0U;
    }

    *mech_angle_rad = Math_WrapRad(atan2f(sin_sum, cos_sum));
    return 1U;
}

static uint8_t FOC_EstimateDirectionAndPolePairs(foc_motor_t *motor,
                                                 int8_t *direction_est,
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
            float d_mech = Math_WrapRadDelta(mech_rad - prev_mech_rad);
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

    FOC_ControlApplyElectricalAngle(motor, 0.0f);
    FOC_Platform_WaitMs(FOC_CALIB_STEP_SETTLE_MS);

    for (i = FOC_CALIB_STEP_COUNT; i > 0; i--)
    {
        float elec_target = FOC_CALIB_STEP_ELEC_RAD * (float)i;
        float mech_rad;

        FOC_SampleLockedMechanicalAngle(motor,
                                        elec_target,
                                        FOC_CALIB_STEP_SETTLE_MS,
                                        FOC_CALIB_STEP_SAMPLE_COUNT,
                                        &mech_rad);
    }

    return 1U;
}

void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor)
{
    float calib_uq;
    float backup_ud;
    float backup_uq;
    int8_t direction_est;
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

    calib_uq = motor->set_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO_DEFAULT;
    calib_uq = Math_ClampFloat(calib_uq, 0.0f, motor->set_voltage);

    motor->uq = 0.0f;
    motor->ud = calib_uq;

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

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(motor,
                                            0.0f,
                                            FOC_CALIB_LOCK_SETTLE_MS,
                                            FOC_CALIB_LOCK_SAMPLE_COUNT,
                                            &mech_zero_rad_est) != 0U)
        {
            motor->mech_angle_at_elec_zero_rad = mech_zero_rad_est;
            motor->mech_angle_accum_rad = mech_zero_rad_est;
            motor->mech_angle_prev_rad = mech_zero_rad_est;
            motor->mech_angle_prev_valid = 1U;
        }
        else
        {
            motor->mech_angle_at_elec_zero_rad = 0.0f;
            motor->mech_angle_accum_rad = 0.0f;
            motor->mech_angle_prev_rad = 0.0f;
            motor->mech_angle_prev_valid = 1U;
        }
    }

    motor->ud = backup_ud;
    motor->uq = backup_uq;
    FOC_ControlApplyElectricalAngle(motor, 0.0f);
}

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction)
{
    if (motor == 0)
    {
        return;
    }

    if (vbus_voltage < 0.0f)
    {
        vbus_voltage = 0.0f;
    }
    set_voltage = Math_ClampFloat(set_voltage, 0.0f, vbus_voltage);

    motor->electrical_phase_angle = 0.0f;
    motor->ud = 0.0f;
    motor->uq = 0.0f;
    motor->set_voltage = set_voltage;
    motor->vbus_voltage = vbus_voltage;
    motor->iq_target = 0.0f;
    motor->iq_measured = 0.0f;
    motor->mech_angle_accum_rad = 0.0f;
    motor->mech_angle_prev_rad = 0.0f;
    motor->mech_angle_prev_valid = 0U;
    motor->phase_resistance = phase_resistance;
    motor->pole_pairs = pole_pairs;
    motor->mech_angle_at_elec_zero_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_accum_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_prev_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_prev_valid = 1U;
    motor->direction = direction;

    motor->alpha = 0.0f;
    motor->beta = 0.0f;
    motor->phase_a = 0.0f;
    motor->phase_b = 0.0f;
    motor->phase_c = 0.0f;
    motor->duty_a = 0.0f;
    motor->duty_b = 0.0f;
    motor->duty_c = 0.0f;
    motor->sector = 0U;

    FOC_CalibrateElectricalAngleAndDirection(motor);
}
