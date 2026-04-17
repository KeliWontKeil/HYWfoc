#include "L3_Algorithm/foc_control_c12_init.h"

#include <stdio.h>
#include <math.h>

#include "L41_Math/foc_math_lut.h"
#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "L3_Algorithm/foc_control_c11_entry.h"
#include "L3_Algorithm/foc_control_c21_cfg_state.h"
#include "LS_Config/foc_config.h"

float Math_WrapRad(float angle);
float Math_WrapRadDelta(float angle);
float Math_ClampFloat(float value, float min_val, float max_val);

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

static int16_t FOC_QuantizeCoggingIqToQ15(float iq_a, float iq_lsb_a)
{
    float scaled;

    if (iq_lsb_a <= 1e-6f)
    {
        return 0;
    }

    scaled = iq_a / iq_lsb_a;
    if (scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }

    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void FOC_DumpCoggingCompTable(const char *title,
                                     const int16_t *table_q15,
                                     uint16_t point_count,
                                     float iq_lsb_a)
{
#if (FOC_COGGING_DEBUG_DUMP_ENABLE == FOC_CFG_ENABLE)
    char out[192];
    uint16_t i;

    if ((title == 0) || (table_q15 == 0) || (point_count == 0U))
    {
        return;
    }

    snprintf(out,
             sizeof(out),
             "cogging.%s.points=%u lsb=%.4f gate=%.3f limit=%.3f\r\n",
             title,
             (unsigned int)point_count,
             iq_lsb_a,
             FOC_COGGING_COMP_SPEED_GATE_RAD_S,
             FOC_COGGING_COMP_IQ_LIMIT_A);
    FOC_Platform_WriteDebugText(out);

    for (i = 0U; i < point_count; i += 8U)
    {
        uint16_t j;
        uint16_t offset = 0U;
        int written;

        written = snprintf(out + offset,
                           sizeof(out) - offset,
                           "cogging.%s[%u..]=",
                           title,
                           (unsigned int)i);
        if ((written < 0) || ((uint16_t)written >= (sizeof(out) - offset)))
        {
            break;
        }
        offset += (uint16_t)written;

        for (j = i; (j < point_count) && (j < (uint16_t)(i + 8U)); j++)
        {
            written = snprintf(out + offset,
                               sizeof(out) - offset,
                               "%d%s",
                               (int)table_q15[j],
                               (j + 1U < point_count) && (j + 1U < (uint16_t)(i + 8U)) ? "," : "");
            if ((written < 0) || ((uint16_t)written >= (sizeof(out) - offset)))
            {
                break;
            }
            offset += (uint16_t)written;
        }

        if (offset < (sizeof(out) - 3U))
        {
            out[offset++] = '\r';
            out[offset++] = '\n';
            out[offset] = '\0';
            FOC_Platform_WriteDebugText(out);
        }
    }
#else
    (void)title;
    (void)table_q15;
    (void)point_count;
    (void)iq_lsb_a;
#endif
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

    FOC_ControlApplyElectricalAngleInitBridge(motor, electrical_angle);
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

    FOC_ControlApplyElectricalAngleInitBridge(motor, 0.0f);
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

static uint8_t FOC_LoadStaticCoggingTable(int16_t *table_q15,
                                          uint16_t point_count,
                                          float iq_lsb_a)
{
    uint16_t i;

    if ((table_q15 == 0) || (point_count == 0U))
    {
        return 0U;
    }

#if (FOC_COGGING_STATIC_TABLE_DEFINED == FOC_CFG_ENABLE)
    for (i = 0U; i < point_count; i++)
    {
        float angle = (FOC_MATH_TWO_PI * (float)i) / (float)point_count;
        float iq_a = FOC_COGGING_STATIC_HARMONIC_AMPLITUDE_A *
                     FOC_MathLut_Sin((float)FOC_COGGING_STATIC_HARMONIC_ORDER * angle +
                                     FOC_COGGING_STATIC_HARMONIC_PHASE_RAD);
        iq_a = Math_ClampFloat(iq_a, -FOC_COGGING_COMP_IQ_LIMIT_A, FOC_COGGING_COMP_IQ_LIMIT_A);
        table_q15[i] = FOC_QuantizeCoggingIqToQ15(iq_a, iq_lsb_a);
    }
    return 1U;
#else
    (void)i;
    (void)iq_lsb_a;
    return 0U;
#endif
}

static uint8_t FOC_LearnCoggingTable(foc_motor_t *motor,
                                     int16_t *table_q15,
                                     uint16_t point_count,
                                     float iq_lsb_a)
{
    float backup_ud;
    float backup_uq;
    float lock_ud;
    uint16_t i;
    uint16_t valid_count = 0U;
    int32_t sum = 0;
    int16_t dc_bias;

    if ((motor == 0) || (table_q15 == 0) || (point_count == 0U))
    {
        return 0U;
    }

    backup_ud = motor->ud;
    backup_uq = motor->uq;
    lock_ud = Math_ClampFloat(motor->set_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO,
                              0.0f,
                              motor->set_voltage);

    motor->uq = 0.0f;
    motor->ud = lock_ud;

    for (i = 0U; i < point_count; i++)
    {
        float target_elec_rad = (FOC_MATH_TWO_PI * (float)i) / (float)point_count;
        float target_mech_rad;
        float sampled_mech_rad;

        if ((motor->pole_pairs == 0U) || (motor->direction == FOC_DIR_UNDEFINED))
        {
            return 0U;
        }

        target_mech_rad = Math_WrapRad(motor->mech_angle_at_elec_zero_rad +
                                       ((float)motor->direction * target_elec_rad / (float)motor->pole_pairs));

        if (FOC_SampleLockedMechanicalAngle(motor,
                                            target_elec_rad,
                                            FOC_COGGING_LEARN_STEP_SETTLE_MS,
                                            FOC_COGGING_LEARN_SAMPLE_COUNT,
                                            &sampled_mech_rad) != 0U)
        {
            float mech_err_rad;
            float iq_comp_a;

            mech_err_rad = Math_WrapRadDelta(target_mech_rad - sampled_mech_rad);
            iq_comp_a = mech_err_rad * FOC_COGGING_LEARN_ERR_TO_IQ_GAIN_A_PER_RAD;
            iq_comp_a = Math_ClampFloat(iq_comp_a,
                                        -FOC_COGGING_COMP_IQ_LIMIT_A,
                                        FOC_COGGING_COMP_IQ_LIMIT_A);

            table_q15[i] = FOC_QuantizeCoggingIqToQ15(iq_comp_a, iq_lsb_a);
            valid_count++;
        }
        else
        {
            table_q15[i] = 0;
        }

        sum += (int32_t)table_q15[i];
    }

    motor->ud = backup_ud;
    motor->uq = backup_uq;
    FOC_ControlApplyElectricalAngleInitBridge(motor, 0.0f);

    if ((valid_count * 100U) < (uint16_t)(point_count * FOC_COGGING_LEARN_MIN_VALID_PERCENT))
    {
        return 0U;
    }

    dc_bias = (int16_t)(sum / (int32_t)point_count);
    for (i = 0U; i < point_count; i++)
    {
        table_q15[i] = (int16_t)(table_q15[i] - dc_bias);
    }

    return 1U;
}

static void FOC_InitOptionalCoggingCompensation(foc_motor_t *motor)
{
    int16_t table_q15[FOC_COGGING_LUT_POINT_COUNT];
    uint8_t loaded = 0U;
    char out[128];

    FOC_ControlSetCoggingCompEnable(FOC_COGGING_COMP_ENABLE);

#if (FOC_COGGING_COMP_ENABLE != FOC_CFG_ENABLE)
    FOC_ControlSetCoggingCompUnavailable(FOC_COGGING_COMP_SOURCE_DISABLED);
    FOC_Platform_WriteDebugText("cogging.init: disabled by feature switch\r\n");
    return;
#endif

    if (FOC_LoadStaticCoggingTable(table_q15,
                                   FOC_COGGING_LUT_POINT_COUNT,
                                   FOC_COGGING_LUT_IQ_LSB_A) != 0U)
    {
        if (FOC_ControlLoadCoggingCompTableQ15(table_q15,
                                               FOC_COGGING_LUT_POINT_COUNT,
                                               FOC_COGGING_LUT_IQ_LSB_A,
                                               FOC_COGGING_COMP_SOURCE_STATIC_TABLE) != 0U)
        {
            loaded = 1U;
            FOC_Platform_WriteDebugText("cogging.init: loaded static compensation table\r\n");
            FOC_DumpCoggingCompTable("static", table_q15, FOC_COGGING_LUT_POINT_COUNT, FOC_COGGING_LUT_IQ_LSB_A);
        }
    }

    if ((loaded == 0U) && (FOC_COGGING_INIT_LEARN_ENABLE == FOC_CFG_ENABLE))
    {
        FOC_Platform_WriteDebugText("cogging.init: static table not found, start learning\r\n");
        if (FOC_LearnCoggingTable(motor,
                                  table_q15,
                                  FOC_COGGING_LUT_POINT_COUNT,
                                  FOC_COGGING_LUT_IQ_LSB_A) != 0U)
        {
            if (FOC_ControlLoadCoggingCompTableQ15(table_q15,
                                                   FOC_COGGING_LUT_POINT_COUNT,
                                                   FOC_COGGING_LUT_IQ_LSB_A,
                                                   FOC_COGGING_COMP_SOURCE_INIT_LEARN) != 0U)
            {
                loaded = 1U;
                FOC_Platform_WriteDebugText("cogging.init: learning finished\r\n");
                FOC_DumpCoggingCompTable("learned", table_q15, FOC_COGGING_LUT_POINT_COUNT, FOC_COGGING_LUT_IQ_LSB_A);
            }
        }
    }

    if (loaded == 0U)
    {
        FOC_ControlSetCoggingCompUnavailable(FOC_COGGING_COMP_SOURCE_NONE);
        snprintf(out,
                 sizeof(out),
                 "cogging.init: no available source (learn=%u)\r\n",
                 (unsigned int)FOC_COGGING_INIT_LEARN_ENABLE);
        FOC_Platform_WriteDebugText(out);
    }
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
    FOC_InitOptionalCoggingCompensation(motor);
}
