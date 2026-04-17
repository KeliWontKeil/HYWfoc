#include "L3_Algorithm/foc_control_c23_motor_param_learn.h"

#include <math.h>

#include "L41_Math/foc_math_lut.h"
#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "L3_Algorithm/foc_control_c31_actuation.h"
#include "LS_Config/foc_config.h"

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

uint8_t FOC_SampleLockedMechanicalAngle(foc_motor_t *motor,
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

    FOC_ControlApplyElectricalAngleDirect(motor, electrical_angle);
    FOC_Platform_WaitMs(settle_ms);

    for (i = 0U; i < sample_count; i++)
    {
        float sample_rad;

        if (FOC_Platform_ReadMechanicalAngleRad(&sample_rad) == 0U)
        {
            continue;
        }

        sin_sum += FOC_MathLut_Sin(sample_rad);
        cos_sum += FOC_MathLut_Sin(sample_rad + FOC_MATH_PI * 0.5f);
        FOC_Platform_WaitMs(FOC_CALIB_SETTLE_MS);
    }

    if ((fabsf(sin_sum) < 1e-6f) && (fabsf(cos_sum) < 1e-6f))
    {
        return 0U;
    }

    *mech_angle_rad = Math_WrapRad(FOC_MathLut_Atan2(sin_sum, cos_sum));
    return 1U;
}

uint8_t FOC_EstimateDirectionAndPolePairs(foc_motor_t *motor,
                                          int8_t *direction_est,
                                          uint8_t *pole_pairs_est)
{
    float prev_mech_rad = 0.0f;
    float prev_elec_rad = 0.0f;
    float sum_d_mech = 0.0f;
    float sum_d_elec = 0.0f;
    uint8_t has_prev = 0U;
    uint16_t i;
    uint16_t step_count = FOC_CALIB_COARSE_STEP_COUNT;
    float step_elec_rad = FOC_CALIB_COARSE_STEP_ELEC_RAD;

    if ((motor == 0) || (direction_est == 0) || (pole_pairs_est == 0))
    {
        return 0U;
    }

    for (i = 0U; i <= step_count; i++)
    {
        float elec_target = step_elec_rad * (float)i;
        float mech_rad;

        if (FOC_SampleLockedMechanicalAngle(motor,
                                            elec_target,
                                            FOC_CALIB_COARSE_STEP_SETTLE_MS,
                                            FOC_CALIB_COARSE_STEP_SAMPLE_COUNT,
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

    if ((fabsf(sum_d_mech) < FOC_CALIB_MIN_MECH_STEP_RAD) ||
        (fabsf(sum_d_elec) < 1e-6f))
    {
        return 0U;
    }

    *direction_est = (sum_d_mech >= 0.0f) ? FOC_DIR_NORMAL : FOC_DIR_REVERSED;
    *pole_pairs_est = FOC_ClampPolePairs((int32_t)(fabsf(sum_d_elec / sum_d_mech) + 0.5f));

    FOC_ControlApplyElectricalAngleDirect(motor, 0.0f);
    FOC_Platform_WaitMs(FOC_CALIB_COARSE_STEP_SETTLE_MS);

    for (i = step_count; i > 0U; i--)
    {
        float elec_target = step_elec_rad * (float)i;
        float mech_rad;

        FOC_SampleLockedMechanicalAngle(motor,
                                        elec_target,
                                        FOC_CALIB_COARSE_STEP_SETTLE_MS,
                                        FOC_CALIB_COARSE_STEP_SAMPLE_COUNT,
                                        &mech_rad);
    }

    return 1U;
}
