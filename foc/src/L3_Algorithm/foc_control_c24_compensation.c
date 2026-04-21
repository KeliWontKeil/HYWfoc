#include "L3_Algorithm/foc_control_c24_compensation.h"

#include <stdio.h>
#include <math.h>

#include "L3_Algorithm/foc_control_c31_actuation.h"
#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_cogging_speed_est_valid = 0U;
static float g_cogging_prev_mech_angle_rad = 0.0f;

static void FOC_CoggingResetSpeedEstimator(void)
{
    g_cogging_speed_est_valid = 0U;
    g_cogging_prev_mech_angle_rad = 0.0f;
}

static uint8_t FOC_CoggingComputePhaseRad(const foc_motor_t *motor,
                                          float mech_angle_rad,
                                          float *phase_rad)
{
    float mech_delta_rad;

    if ((motor == 0) || (phase_rad == 0))
    {
        return 0U;
    }

    if ((motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ||
        (motor->direction == FOC_DIR_UNDEFINED) ||
        (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        return 0U;
    }

    mech_delta_rad = Math_WrapRadDelta(mech_angle_rad - motor->mech_angle_at_elec_zero_rad);
    *phase_rad = Math_WrapRad((float)motor->direction * mech_delta_rad * (float)motor->pole_pairs);
    return 1U;
}

static float FOC_CoggingEstimateSpeedAbsRadS(float mech_angle_rad, float speed_ref_rad_s)
{
    float dt_sec = FOC_CONTROL_DT_SEC;
    float speed_abs_rad_s;

    if (dt_sec <= 1e-6f)
    {
        return fabsf(speed_ref_rad_s);
    }

    if (g_cogging_speed_est_valid == 0U)
    {
        g_cogging_prev_mech_angle_rad = mech_angle_rad;
        g_cogging_speed_est_valid = 1U;
        return fabsf(speed_ref_rad_s);
    }

    speed_abs_rad_s = fabsf(Math_WrapRadDelta(mech_angle_rad - g_cogging_prev_mech_angle_rad) / dt_sec);
    g_cogging_prev_mech_angle_rad = mech_angle_rad;
    return speed_abs_rad_s;
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

    if ((motor->pole_pairs == 0U) ||
        (motor->direction == FOC_DIR_UNDEFINED) ||
        (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
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
        float target_mech_rad = Math_WrapRad(motor->mech_angle_at_elec_zero_rad +
                                             ((float)motor->direction * target_elec_rad / (float)motor->pole_pairs));
        float sampled_mech_rad;

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
    FOC_ControlApplyElectricalAngleDirect(motor, 0.0f);

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
#endif

float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float cogging_phase_rad,
                                 float speed_abs_rad_s)
{
    float angle_rad;
    float index_f;
    uint16_t index_0;
    uint16_t index_1;
    float frac;
    float iq_0;
    float iq_1;
    float iq_comp;
    float speed_abs;

    if ((status == 0) || (table_q15 == 0))
    {
        return 0.0f;
    }

    if ((status->enabled == 0U) ||
        (status->available == 0U) ||
        (status->point_count < 2U))
    {
        return 0.0f;
    }

    angle_rad = Math_WrapRad(cogging_phase_rad);
    index_f = angle_rad * ((float)status->point_count / FOC_MATH_TWO_PI);
    index_0 = (uint16_t)index_f;
    if (index_0 >= status->point_count)
    {
        index_0 = (uint16_t)(status->point_count - 1U);
    }
    index_1 = (uint16_t)((index_0 + 1U) % status->point_count);
    frac = index_f - (float)index_0;

    iq_0 = (float)table_q15[index_0] * status->iq_lsb_a;
    iq_1 = (float)table_q15[index_1] * status->iq_lsb_a;
    iq_comp = iq_0 + (iq_1 - iq_0) * frac;

    speed_abs = (speed_abs_rad_s >= 0.0f) ? speed_abs_rad_s : -speed_abs_rad_s;
    if (status->speed_gate_rad_s > 1e-6f)
    {
        if (speed_abs >= status->speed_gate_rad_s)
        {
            return 0.0f;
        }

        iq_comp *= (1.0f - (speed_abs / status->speed_gate_rad_s));
    }

    return Math_ClampFloat(iq_comp,
                           -status->iq_limit_a,
                           status->iq_limit_a);
}

void FOC_ControlApplyCoggingCompensation(foc_motor_t *motor,
                                         float mech_angle_rad,
                                         float speed_ref_rad_s)
{
#if (FOC_COGGING_COMP_ENABLE != FOC_CFG_ENABLE)
    (void)mech_angle_rad;
    (void)speed_ref_rad_s;
    (void)motor;
    return;
#else
    float phase_rad;
    float speed_abs_rad_s;

    if (motor == 0)
    {
        return;
    }

    if (FOC_CoggingComputePhaseRad(motor, mech_angle_rad, &phase_rad) == 0U)
    {
        return;
    }

    speed_abs_rad_s = FOC_CoggingEstimateSpeedAbsRadS(mech_angle_rad, speed_ref_rad_s);

    motor->iq_target += FOC_ControlCoggingLookupIq(&motor->cogging_comp_status,
                                                   motor->cogging_comp_table_q15,
                                                   phase_rad,
                                                   speed_abs_rad_s);
#endif
}

uint8_t FOC_ControlLoadCoggingCompTableQ15(foc_motor_t *motor,
                                           const int16_t *table_q15,
                                           uint16_t point_count,
                                           float iq_lsb_a,
                                           uint8_t source)
{
    uint16_t i;

    if ((motor == 0) || (table_q15 == 0) || (point_count < 2U) ||
        (point_count > (uint16_t)FOC_MOTOR_COGGING_LUT_CAPACITY))
    {
        return 0U;
    }

    if (iq_lsb_a <= 0.0f)
    {
        return 0U;
    }

    for (i = 0U; i < point_count; i++)
    {
        motor->cogging_comp_table_q15[i] = table_q15[i];
    }

    for (; i < (uint16_t)FOC_MOTOR_COGGING_LUT_CAPACITY; i++)
    {
        motor->cogging_comp_table_q15[i] = 0;
    }

    motor->cogging_comp_status.point_count = point_count;
    motor->cogging_comp_status.iq_lsb_a = iq_lsb_a;
    motor->cogging_comp_status.available = 1U;
    motor->cogging_comp_status.source = source;
    return 1U;
}

void FOC_ControlSetCoggingCompUnavailable(foc_motor_t *motor, uint8_t source)
{
    if (motor == 0)
    {
        return;
    }

    motor->cogging_comp_status.available = 0U;
    motor->cogging_comp_status.source = source;
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    FOC_CoggingResetSpeedEstimator();
#endif
}

void FOC_ControlInitCoggingCompensation(foc_motor_t *motor)
{
#if (FOC_COGGING_COMP_ENABLE != FOC_CFG_ENABLE)
    FOC_ControlSetCoggingCompUnavailable(motor, FOC_COGGING_COMP_SOURCE_DISABLED);
    FOC_Platform_WriteDebugText("cogging.init: disabled by feature switch\r\n");
#else
    int16_t table_q15[FOC_COGGING_LUT_POINT_COUNT];
    uint8_t loaded = 0U;
    char out[128];

    FOC_CoggingResetSpeedEstimator();

    if (FOC_LoadStaticCoggingTable(table_q15,
                                   FOC_COGGING_LUT_POINT_COUNT,
                                   FOC_COGGING_LUT_IQ_LSB_A) != 0U)
    {
        if (FOC_ControlLoadCoggingCompTableQ15(motor,
                                                table_q15,
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
        if ((motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ||
            (motor->direction == FOC_DIR_UNDEFINED) ||
            (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
        {
            FOC_Platform_WriteDebugText("cogging.init: skip learning due to invalid calibration base\r\n");
        }
        else
        {
            FOC_Platform_WriteDebugText("cogging.init: static table not found, start learning\r\n");
            if (FOC_LearnCoggingTable(motor,
                                      table_q15,
                                      FOC_COGGING_LUT_POINT_COUNT,
                                      FOC_COGGING_LUT_IQ_LSB_A) != 0U)
            {
                if (FOC_ControlLoadCoggingCompTableQ15(motor,
                                                       table_q15,
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
    }

    if (loaded == 0U)
    {
        FOC_ControlSetCoggingCompUnavailable(motor, FOC_COGGING_COMP_SOURCE_NONE);
        snprintf(out,
                 sizeof(out),
                 "cogging.init: no available source (learn=%u)\r\n",
                 (unsigned int)FOC_COGGING_INIT_LEARN_ENABLE);
        FOC_Platform_WriteDebugText(out);
    }
#endif
}
