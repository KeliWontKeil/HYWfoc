#include "L3_Algorithm/foc_control_c12_init.h"

#include <stdio.h>
#include <math.h>

#include "L41_Math/foc_math_lut.h"
#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "L3_Algorithm/foc_control_c11_entry.h"
#include "L3_Algorithm/foc_control_c23_motor_param_learn.h"
#include "L3_Algorithm/foc_control_c24_compensation.h"
#include "LS_Config/foc_config.h"

float Math_WrapRad(float angle);
float Math_WrapRadDelta(float angle);
float Math_ClampFloat(float value, float min_val, float max_val);

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

    calib_uq = motor->set_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO;
    calib_uq = Math_ClampFloat(calib_uq, 0.0f, motor->set_voltage);

    motor->uq = 0.0f;
    motor->ud = calib_uq;

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(motor,
                                            0.0f,
                                            FOC_CALIB_ZERO_LOCK_SETTLE_MS,
                                            FOC_CALIB_ZERO_LOCK_SAMPLE_COUNT,
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
    else
    {
        FOC_ControlApplyElectricalAngleInitBridge(motor, 0.0f);
        FOC_Platform_WaitMs(FOC_CALIB_ZERO_LOCK_SETTLE_MS);
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
    FOC_ControlApplyElectricalAngleInitBridge(motor, 0.0f);
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

#if (FOC_INIT_CALIBRATION_ENABLE == FOC_CFG_ENABLE)
    FOC_CalibrateElectricalAngleAndDirection(motor);
#endif
    FOC_ControlInitCoggingCompensation(motor);
}
