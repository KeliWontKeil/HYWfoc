#include "L3_Algorithm/foc_control_c24_compensation.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "L3_Algorithm/foc_control_c31_actuation.h"
#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)

/* ------------------------------------------------------------------ */
/*  Internal speed estimator (static, used by Apply).                 */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/*  Calibration-internal raw table accumulator (heap/static-free).    */
/*  We store per-point sums in a local buffer passed on the stack,     */
/*  but since FOC_COGGING_LUT_POINT_COUNT can be up to 128 we use     */
/*  a single statically-allocated accumulator that lives only while   */
/*  calibration is active.  This is safe because calibration is       */
/*  mutually exclusive with normal compensation.                      */
/* ------------------------------------------------------------------ */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

static float g_calib_iq_accum[FOC_MOTOR_COGGING_LUT_CAPACITY];
static uint16_t g_calib_sample_count[FOC_MOTOR_COGGING_LUT_CAPACITY];
static float g_calib_prev_mech_rad;
static uint8_t g_calib_prev_valid;

/* Deferred request flags — set by L2 protocol, consumed on control tick. */
static uint8_t g_calib_start_request_pending = 0U;
static uint8_t g_calib_dump_request_pending = 0U;

#endif /* FOC_COGGING_CALIB_ENABLE */

/* ------------------------------------------------------------------ */
/*  Core public API.                                                  */
/* ------------------------------------------------------------------ */

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

    /*
     * Speed-dependent gain roll-off.
     *
     * At zero speed the full compensation is applied.
     * As speed increases toward speed_gate_rad_s, the gain linearly
     * decreases to zero. Beyond speed_gate_rad_s the compensation is
     * fully suppressed (cogging torque averages out at high speed).
     */
    speed_abs = (speed_abs_rad_s >= 0.0f) ? speed_abs_rad_s : -speed_abs_rad_s;
    if ((status->speed_gate_rad_s > 1e-6f) && (speed_abs < status->speed_gate_rad_s))
    {
        iq_comp *= (1.0f - (speed_abs / status->speed_gate_rad_s));
    }
    else
    {
        iq_comp = 0.0f;
    }

    return Math_ClampFloat(iq_comp,
                           -status->iq_limit_a,
                           status->iq_limit_a);
}

void FOC_ControlApplyCoggingCompensation(foc_motor_t *motor,
                                         float mech_angle_rad,
                                         float speed_ref_rad_s)
{
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
    motor->cogging_comp_status.calib_in_progress = 0U;
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
    motor->cogging_comp_status.calib_in_progress = 0U;
    FOC_CoggingResetSpeedEstimator();
}

/* ------------------------------------------------------------------ */
/*  Calibration state machine  (optional feature).                    */
/* ------------------------------------------------------------------ */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

static void FOC_CoggingCalibResetAccum(void)
{
    uint16_t i;
    for (i = 0U; i < (uint16_t)FOC_MOTOR_COGGING_LUT_CAPACITY; i++)
    {
        g_calib_iq_accum[i] = 0.0f;
        g_calib_sample_count[i] = 0U;
    }
    g_calib_prev_mech_rad = 0.0f;
    g_calib_prev_valid = 0U;
}

/*
 * Map a mechanical angle to the nearest LUT bin index [0 .. point_count-1].
 * Uses the motor's mech_angle_at_elec_zero_rad as the reference.
 */
static uint16_t FOC_CoggingCalibMechToBin(float mech_rad,
                                          float mech_zero_rad,
                                          uint8_t direction,
                                          uint8_t pole_pairs,
                                          uint16_t point_count)
{
    float delta_mech;
    float elec_rad;
    float norm;
    uint16_t bin;

    delta_mech = Math_WrapRadDelta(mech_rad - mech_zero_rad);
    elec_rad = Math_WrapRad((float)direction * delta_mech * (float)pole_pairs);
    norm = elec_rad / FOC_MATH_TWO_PI; /* [0 .. 1) */
    if (norm < 0.0f)
    {
        norm += 1.0f;
    }
    bin = (uint16_t)(norm * (float)point_count);
    if (bin >= point_count)
    {
        bin = (uint16_t)(point_count - 1U);
    }
    return bin;
}

void FOC_CoggingCalibRequestStart(void)
{
    g_calib_start_request_pending = 1U;
}

static void FOC_CoggingCalibInternalDumpTable(const foc_motor_t *motor)
{
    char out[192];
    uint16_t i;
    uint16_t total_points;

    if (motor == 0)
    {
        return;
    }

    total_points = motor->cogging_comp_status.point_count;
    if (total_points < 2U)
    {
        FOC_Platform_WriteDebugText("cogging.dump: table empty (no calibration data)\r\n");
        return;
    }

    snprintf(out, sizeof(out),
             "cogging.table.points=%u lsb=%.4f gate=%.3f limit=%.3f\r\n",
             (unsigned int)total_points,
             motor->cogging_comp_status.iq_lsb_a,
             motor->cogging_comp_status.speed_gate_rad_s,
             motor->cogging_comp_status.iq_limit_a);
    FOC_Platform_WriteDebugText(out);

    for (i = 0U; i < total_points; i += 8U)
    {
        uint16_t j;
        uint16_t offset = 0U;
        int written;

        written = snprintf(out + offset, sizeof(out) - offset,
                           "cogging.table[%u..]=", (unsigned int)i);
        if ((written < 0) || ((uint16_t)written >= (sizeof(out) - offset)))
        {
            break;
        }
        offset += (uint16_t)written;

        for (j = i; (j < total_points) && (j < (uint16_t)(i + 8U)); j++)
        {
            written = snprintf(out + offset, sizeof(out) - offset, "%d%s",
                               (int)motor->cogging_comp_table_q15[j],
                               ((j + 1U < total_points) && (j + 1U < (uint16_t)(i + 8U))) ? "," : "");
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
}

void FOC_CoggingCalibDumpTable(void)
{
    g_calib_dump_request_pending = 1U;
}

uint8_t FOC_CoggingCalibStart(foc_motor_t *motor)
{
    if (motor == 0)
    {
        return 0U;
    }

    if (motor->cogging_comp_status.calib_in_progress != 0U)
    {
        return 0U; /* already busy */
    }

    /* Require valid calibration base (pole pairs, direction, mech zero). */
    if ((motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ||
        (motor->direction == FOC_DIR_UNDEFINED) ||
        (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        FOC_Platform_WriteDebugText("cogging.calib: abort – missing pole/direction/zero\r\n");
        return 0U;
    }

    /* Motor must be enabled and spinning (coast check: speed_ref near target). */
    if (motor->cogging_speed_ref_rad_s < (FOC_COGGING_CALIB_SPEED_RAD_S * 0.5f))
    {
        FOC_Platform_WriteDebugText("cogging.calib: abort – motor not spinning (speed too low)\r\n");
        return 0U;
    }

    motor->cogging_comp_status.calib_in_progress = 1U;
    motor->cogging_comp_status.calib_progress_percent = 0U;
    motor->cogging_comp_status.calib_point_index = 0U;
    motor->cogging_comp_status.calib_pass_index = 0U;

    FOC_CoggingCalibResetAccum();

    FOC_Platform_WriteDebugText("cogging.calib: started\r\n");
    return 1U;
}

uint8_t FOC_CoggingCalibProcess(foc_motor_t *motor)
{
    float iq_floor;
    uint16_t total_points;
    uint16_t pass;
    uint16_t step;

    if (motor == 0)
    {
        return 0U;
    }

    /* ---- consume deferred request flags ---- */
    if (g_calib_dump_request_pending != 0U)
    {
        g_calib_dump_request_pending = 0U;
        FOC_CoggingCalibInternalDumpTable(motor);
    }

    if (g_calib_start_request_pending != 0U)
    {
        g_calib_start_request_pending = 0U;
        FOC_CoggingCalibStart(motor);
        /* If start just kicked off, the state machine will run below. */
    }

    if (motor->cogging_comp_status.calib_in_progress == 0U)
    {
        return 0U;
    }

    total_points = FOC_COGGING_LUT_POINT_COUNT;

    /*
     * The motor is assumed to be spinning at FOC_COGGING_CALIB_SPEED_RAD_S
     * in speed-closed / current-open (open-loop torque) mode.
     *
     * We run the current loop open (force iq_target to the baseline Id).
     * Actual Iq control is intentionally bypassed so that the measured Iq
     * reflects the cogging torque directly.
     */

    /* Override iq_target to the calibration baseline. */
    motor->iq_target = FOC_COGGING_CALIB_IQ_BASELINE_A;

    /*
     * Accumulate one sample at the current mechanical angle.
     *
     * Strategy: at each control tick, gate on the direction of angle
     * advance.  We collect a sample when the mechanical angle has
     * advanced by at least one electrical step relative to the previous
     * sample, but we bin by mechanical angle modulo one electrical
     * revolution.
     *
     * For simplicity:  accumulate iq_measured into the appropriate bin
     * on EVERY tick, and increment the sample counter.  This naturally
     * fills the LUT as the motor rotates.  After enough samples per
     * point (FOC_COGGING_CALIB_SAMPLES_PER_POINT * point_count total),
     * we advance to the next pass.
     */
    {
        uint16_t bin;
        float mech_rad;

        mech_rad = motor->mech_angle_accum_rad;
        bin = FOC_CoggingCalibMechToBin(mech_rad,
                                        motor->mech_angle_at_elec_zero_rad,
                                        (uint8_t)(motor->direction > 0 ? 1U : 0U),
                                        motor->pole_pairs,
                                        total_points);

        g_calib_iq_accum[bin] += motor->iq_measured;
        g_calib_sample_count[bin]++;
    }

    /* Check completion: all bins have enough samples. */
    {
        uint16_t i;
        uint8_t all_done = 1U;

        for (i = 0U; i < total_points; i++)
        {
            if (g_calib_sample_count[i] < FOC_COGGING_CALIB_SAMPLES_PER_POINT)
            {
                all_done = 0U;
                break;
            }
        }

        if (all_done == 0U)
        {
            /* Report rough progress. */
            {
                uint32_t total_needed = (uint32_t)total_points * (uint32_t)FOC_COGGING_CALIB_SAMPLES_PER_POINT;
                uint32_t total_got = 0U;
                uint16_t j;
                for (j = 0U; j < total_points; j++)
                {
                    total_got += (uint32_t)g_calib_sample_count[j];
                }
                motor->cogging_comp_status.calib_progress_percent =
                    (uint8_t)((total_got * 100U) / total_needed);
            }
            return 1U; /* still busy */
        }
    }

    /*
     * This pass is complete.
     * Average the accumulated iq values, convert to Q15, and keep the
     * table in motor->cogging_comp_table_q15.
     */
    {
        uint16_t i;
        float iq_mean;

        for (i = 0U; i < total_points; i++)
        {
            iq_mean = g_calib_iq_accum[i] / (float)g_calib_sample_count[i];
            /* Remove the floor = the DC component (average across all bins). */
            motor->cogging_comp_table_q15[i] = FOC_QuantizeCoggingIqToQ15(
                iq_mean, FOC_COGGING_LUT_IQ_LSB_A);
        }

        /* Remove DC bias from Q15 table. */
        {
            int32_t sum = 0;
            int16_t dc_bias;
            for (i = 0U; i < total_points; i++)
            {
                sum += (int32_t)motor->cogging_comp_table_q15[i];
            }
            dc_bias = (int16_t)(sum / (int32_t)total_points);
            for (i = 0U; i < total_points; i++)
            {
                motor->cogging_comp_table_q15[i] =
                    (int16_t)(motor->cogging_comp_table_q15[i] - dc_bias);
            }
        }
    }

    /*
     * Check if we have completed all passes.  If not, reset accumulators
     * for the next pass.
     */
    pass = motor->cogging_comp_status.calib_pass_index + 1U;
    if (pass < FOC_COGGING_CALIB_NUM_PASSES)
    {
        motor->cogging_comp_status.calib_pass_index = (uint8_t)pass;
        motor->cogging_comp_status.calib_progress_percent =
            (uint8_t)((pass * 100U) / FOC_COGGING_CALIB_NUM_PASSES);
        FOC_CoggingCalibResetAccum();
        return 1U; /* more passes to go */
    }

    /* All passes done — finalize. */
    motor->cogging_comp_status.point_count = total_points;
    motor->cogging_comp_status.iq_lsb_a = FOC_COGGING_LUT_IQ_LSB_A;
    motor->cogging_comp_status.available = 1U;
    motor->cogging_comp_status.source = FOC_COGGING_COMP_SOURCE_CALIB;
    motor->cogging_comp_status.calib_in_progress = 0U;
    motor->cogging_comp_status.calib_progress_percent = 100U;
    motor->cogging_comp_status.enabled = 1U;

    FOC_Platform_WriteDebugText("cogging.calib: completed, table loaded\r\n");

#if (FOC_COGGING_DEBUG_DUMP_ENABLE == FOC_CFG_ENABLE)
    {
        char out[192];
        uint16_t i;

        snprintf(out, sizeof(out),
                 "cogging.calib.points=%u lsb=%.4f gate=%.3f limit=%.3f\r\n",
                 (unsigned int)total_points,
                 FOC_COGGING_LUT_IQ_LSB_A,
                 FOC_COGGING_COMP_SPEED_GATE_RAD_S,
                 FOC_COGGING_COMP_IQ_LIMIT_A);
        FOC_Platform_WriteDebugText(out);

        for (i = 0U; i < total_points; i += 8U)
        {
            uint16_t j;
            uint16_t offset = 0U;
            int written;

            written = snprintf(out + offset, sizeof(out) - offset,
                               "cogging.calib[%u..]=", (unsigned int)i);
            if ((written < 0) || ((uint16_t)written >= (sizeof(out) - offset)))
            {
                break;
            }
            offset += (uint16_t)written;

            for (j = i; (j < total_points) && (j < (uint16_t)(i + 8U)); j++)
            {
                written = snprintf(out + offset, sizeof(out) - offset, "%d%s",
                                   (int)motor->cogging_comp_table_q15[j],
                                   ((j + 1U < total_points) && (j + 1U < (uint16_t)(i + 8U))) ? "," : "");
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
    }
#endif /* FOC_COGGING_DEBUG_DUMP_ENABLE */

    return 0U; /* finished */
}

#endif /* FOC_COGGING_CALIB_ENABLE */

#endif /* FOC_COGGING_COMP_ENABLE */