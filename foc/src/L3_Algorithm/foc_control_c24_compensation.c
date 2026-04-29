#include "L3_Algorithm/foc_control_c24_compensation.h"

#include <math.h>
#include <stdio.h>
#include <string.h>


#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"
#include "L3_Algorithm/foc_control_c31_actuation.h"

/* ======================================================================== *
 *  Static LUT storage and Q15 helpers (shared by lookup and calibration)   *
 * ======================================================================== */

/* Maximum LUT point count (must match FOC_MOTOR_COGGING_LUT_CAPACITY). */
#define COGGING_LUT_MAX  128U

/*
 * Q15 format: range [-1.0, 1.0) maps to [-32768, 32767].
 * iq_value_a = q15_val * status->iq_lsb_a
 */
static inline int16_t FloatToQ15(float val, float lsb)
{
    float q;
    if (lsb < 1e-12f)
    {
        return 0;
    }
    q = val / lsb;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int16_t)((int32_t)(q + (q >= 0.0f ? 0.5f : -0.5f)));
}

static inline float Q15ToFloat(int16_t q15, float lsb)
{
    return (float)q15 * lsb;
}

/* ======================================================================== *
 *  Cogging compensation lookup and application                             *
 * ======================================================================== */

float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float mech_angle_rad,
                                 float speed_ref_rad_s)
{
    uint16_t idx0;
    uint16_t idx1;
    float frac;
    float val0;
    float val1;
    float interp;
    float iq_comp;
    float speed_abs;

    if ((status == 0) || (table_q15 == 0))
    {
        return 0.0f;
    }

    /* Only compensate when running above the speed gate. */
    speed_abs = (speed_ref_rad_s >= 0.0f) ? speed_ref_rad_s : -speed_ref_rad_s;
    if (speed_abs < status->speed_gate_rad_s)
    {
        return 0.0f;
    }

    if ((status->available == 0U) || (status->point_count == 0U))
    {
        return 0.0f;
    }

    /* Normalise mechanical angle to [0, 2*pi). */
    {
        float norm = mech_angle_rad;
        while (norm < 0.0f)
        {
            norm += FOC_MATH_TWO_PI;
        }
        while (norm >= FOC_MATH_TWO_PI)
        {
            norm -= FOC_MATH_TWO_PI;
        }
        mech_angle_rad = norm;
    }

    /* Map angle to LUT index with linear interpolation. */
    {
        float step = FOC_MATH_TWO_PI / (float)status->point_count;
        float pos  = mech_angle_rad / step;
        idx0 = (uint16_t)pos;
        frac = pos - (float)idx0;

        if (idx0 >= status->point_count)
        {
            idx0 = (uint16_t)(status->point_count - 1U);
        }

        idx1 = idx0 + 1U;
        if (idx1 >= status->point_count)
        {
            idx1 = 0U;
        }
    }

    val0 = Q15ToFloat(table_q15[idx0], status->iq_lsb_a);
    val1 = Q15ToFloat(table_q15[idx1], status->iq_lsb_a);
    interp = val0 + frac * (val1 - val0);

    /* Clamp to configured limit. */
    iq_comp = interp;
    if (iq_comp > status->iq_limit_a)
    {
        iq_comp = status->iq_limit_a;
    }
    if (iq_comp < -status->iq_limit_a)
    {
        iq_comp = -status->iq_limit_a;
    }

    return iq_comp;
}

void FOC_ControlApplyCoggingCompensation(foc_motor_t *motor,
                                         float mech_angle_rad,
                                         float speed_ref_rad_s)
{
    float iq_comp;

    if (motor == 0)
    {
        return;
    }

    /* Skip if disabled or not available. */
    if ((motor->cogging_comp_status.enabled == 0U) ||
        (motor->cogging_comp_status.available == 0U))
    {
        return;
    }

    iq_comp = FOC_ControlCoggingLookupIq(&motor->cogging_comp_status,
                                          motor->cogging_comp_table_q15,
                                          mech_angle_rad,
                                          speed_ref_rad_s);

    motor->iq_target += iq_comp * motor->cogging_comp_status.calib_gain_k;
}

uint8_t FOC_ControlLoadCoggingCompTableQ15(foc_motor_t *motor,
                                            const int16_t *table_q15,
                                            uint16_t point_count,
                                            float iq_lsb_a,
                                            uint8_t source)
{
    uint16_t copy_count;

    if ((motor == 0) || (table_q15 == 0))
    {
        return 0U;
    }

    if (point_count > COGGING_LUT_MAX)
    {
        point_count = COGGING_LUT_MAX;
    }

    copy_count = point_count;
    (void)memcpy((void *)motor->cogging_comp_table_q15,
                 (const void *)table_q15,
                 (size_t)copy_count * sizeof(int16_t));

    motor->cogging_comp_status.point_count = point_count;
    motor->cogging_comp_status.iq_lsb_a    = iq_lsb_a;
    motor->cogging_comp_status.source      = source;
    motor->cogging_comp_status.available   = 1U;

    return 1U;
}

void FOC_ControlSetCoggingCompUnavailable(foc_motor_t *motor, uint8_t source)
{
    if (motor == 0)
    {
        return;
    }
    motor->cogging_comp_status.available = 0U;
    motor->cogging_comp_status.source    = source;
}

/* ======================================================================== *
 *  Runtime cogging calibration (user-triggered, self-driven)                *
 *                                                                           *
 *  Pure open-loop position-error method:                                    *
 *    - Drives the motor at constant Iq with self-maintained electrical      *
 *      angle (no outer-loop PID, no current-loop PID).                      *
 *    - Records the position error Δθ = θ_actual - θ_expected per LUT bin.  *
 *    - On completion: circular-differentiates Δθ → iq compensation table.  *
 * ======================================================================== */

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

/* --- Static state (calibration request flags, LUT accumulators) --- */

static uint8_t g_calib_request_start = 0U;
static uint8_t g_calib_request_dump  = 0U;
static uint8_t g_calib_request_export = 0U;

/* Calibration internal state machine phases (stored in calib_point_index). */
#define CALIB_PHASE_IDLE     0xFFFFU   /* idle / not started */
#define CALIB_PHASE_START    0xFFFEU   /* init accumulators and expected angle */
#define CALIB_PHASE_SETTLE   0xFFFDU   /* open-loop drive N revs to build equilibrium */
#define CALIB_PHASE_SCAN     0xFFFCU   /* accumulate Δθ per LUT bin */
#define CALIB_PHASE_CHECK    0xFFFBU   /* one scan pass complete, decide next */
#define CALIB_PHASE_FINISH   0xFFFAU   /* compute differential table and load */
#define CALIB_PHASE_DONE     0xFFF9U   /* finished */

/* Accumulator for position error Δθ (radians) per LUT bin, summed over passes. */
static float   g_calib_dtheta_accum[COGGING_LUT_MAX];
static uint16_t g_calib_count[COGGING_LUT_MAX];

/*
 * Last calibrated Q15 table (global copy for dump/export without
 * requiring a motor pointer).  Populated on calibration completion
 * by CoggingCalib_Finish().
 */
static int16_t  g_calib_last_table_q15[COGGING_LUT_MAX];
static uint16_t g_calib_last_point_count = 0U;
static float    g_calib_last_iq_lsb_a    = 0.0f;
static uint8_t  g_calib_table_available  = 0U;

/*
 * Current pass number (0-based). Incremented each time SCAN completes
 * one full mechanical revolution.
 */
static uint8_t g_calib_pass_num = 0U;

/* Self-maintained expected electrical angle for open-loop drive. */
static float g_calib_expected_elec_angle = 0.0f;

/* Self-maintained expected mechanical angle (independent of pole-pair wrap). */
static float g_calib_expected_mech_angle = 0.0f;

/* Revolution tracking: accumulated angular travel (rad) within current phase. */
static float g_calib_travel_accum_rad = 0.0f;

/* Previous sensor mechanical angle (rad) used for angular delta computation. */
static float g_calib_angle_prev_rad = -1.0f;

/*
 * Per-bin-once tracking: last LUT bin index sampled in the current SCAN pass.
 * Initialised to 0xFFFF so first crossing always triggers a sample.
 */
static uint16_t g_calib_last_lut_index = 0xFFFFU;

/* Number of unique LUT bins sampled during the current SCAN pass. */
static uint16_t g_calib_bins_collected = 0U;

/* Revolution counter within the current SETTLE or SCAN phase. */
static uint8_t g_calib_rev_count = 0U;

/*
 * Last reported progress percentage (stepped at 10 % granularity)
 * to throttle periodic debug output during SCAN phase.
 */
static uint8_t g_calib_last_reported_progress = 0U;

/*
 * Soft-switch state saved at calibration start, restored on finish.
 */
static uint8_t g_calib_saved_softswitch_enabled = 1U;
static uint8_t g_calib_saved_softswitch_mode    = 0U;

/* ------------------------------------------------------------------ */

void FOC_CoggingCalibRequestStart(void)
{
    g_calib_request_start = 1U;
}

uint8_t FOC_CoggingCalibStart(foc_motor_t *motor)
{
    uint16_t i;

    if (motor == 0)
    {
        return 0U;
    }

    /* Reject if calibration already in progress. */
    if (motor->cogging_comp_status.calib_in_progress != 0U)
    {
        return 0U;
    }

    /* Reset all accumulators. */
    for (i = 0U; i < COGGING_LUT_MAX; i++)
    {
        g_calib_dtheta_accum[i] = 0.0f;
        g_calib_count[i] = 0U;
    }

    /* Initialise calibration state. */
    motor->cogging_comp_status.calib_in_progress      = 1U;
    motor->cogging_comp_status.calib_progress_percent  = 0U;
    motor->cogging_comp_status.calib_point_index       = CALIB_PHASE_START;
    motor->cogging_comp_status.calib_pass_index        = 0U;
    g_calib_pass_num                                   = 0U;

    /* Angle tracking will be seeded in START phase from sensor data. */
    g_calib_expected_elec_angle = 0.0f;
    g_calib_expected_mech_angle = 0.0f;
    g_calib_angle_prev_rad      = -1.0f;
    g_calib_travel_accum_rad    = 0.0f;

    /* Reset per-bin-once tracking and progress throttle. */
    g_calib_last_lut_index         = 0xFFFFU;
    g_calib_last_reported_progress = 0U;
    g_calib_bins_collected         = 0U;
    g_calib_rev_count              = 0U;

    g_calib_table_available = 0U;

    /* Save current soft-switch state so it can be restored on finish. */
    g_calib_saved_softswitch_enabled = motor->current_soft_switch_status.enabled;
    g_calib_saved_softswitch_mode    = motor->current_soft_switch_status.configured_mode;

    FOC_Platform_WriteDebugText("COGGING CALIB START (open-loop position-error method)\r\n");

    return 1U;
}

/* ------------------------------------------------------------------ */

/*
 * Helper: compute angular delta (rad) between two mechanical angle readings,
 * handling the 0/2pi wrap-around.
 */
static float CoggingCalib_AngleDelta(float curr, float prev)
{
    float delta = curr - prev;
    if (delta > FOC_MATH_PI)
    {
        delta -= FOC_MATH_TWO_PI;
    }
    else if (delta < -FOC_MATH_PI)
    {
        delta += FOC_MATH_TWO_PI;
    }
    return delta;
}

/*
 * Open-loop drive step for calibration.
 *
 * This function completely bypasses the outer-loop PID and current-loop PID.
 * It self-maintains an expected electrical angle and drives Uq = I_calib * R_phase,
 * then publishes the angle and iq_target for the PWM ISR to consume.
 */
static void CoggingCalib_OpenLoopDriveStep(foc_motor_t *motor, float dt_sec)
{
    float phase_resistance;
    float uq;

    if (motor == 0)
    {
        return;
    }

    /*
     * Advance expected mechanical angle directly.
     * FOC_COGGING_CALIB_SPEED_RAD_S is the electrical angular velocity of the
     * rotating field.  The mechanical angular velocity is speed / pole_pairs.
     * We advance the mechanical angle so it stays in [0, 2π) and is directly
     * comparable to the sensor's mechanical angle reading.
     */
    g_calib_expected_mech_angle += FOC_COGGING_CALIB_SPEED_RAD_S *
                                    dt_sec *
                                    (float)motor->direction;
    g_calib_expected_mech_angle  = Math_WrapRad(g_calib_expected_mech_angle);

    /* Derive electrical angle from mechanical: θ_elec = θ_mech * pole_pairs. */
    g_calib_expected_elec_angle = g_calib_expected_mech_angle * (float)motor->pole_pairs;
    g_calib_expected_elec_angle = Math_WrapRad(g_calib_expected_elec_angle);

    /* Compute Uq = I_calib * R_phase (open-loop resistance model). */
    phase_resistance = fabsf(motor->phase_resistance);
    if (phase_resistance < 1e-6f)
    {
        phase_resistance = 1e-6f;
    }
    uq = FOC_COGGING_CALIB_IQ_A * phase_resistance;
    uq = Math_ClampFloat(uq, -motor->set_voltage, motor->set_voltage);

    /* Set motor DQ voltages for SVPWM drive. */
    motor->ud = 0.0f;
    motor->uq = uq;

    /* Publish iq_target for the fast current loop (PWM ISR). */
    motor->iq_target = FOC_COGGING_CALIB_IQ_A;

    /* Apply electrical angle and drive SVPWM. */
    motor->electrical_phase_angle = g_calib_expected_elec_angle;
    FOC_ControlApplyElectricalAngleRuntime(motor, g_calib_expected_elec_angle);
}

/* ------------------------------------------------------------------ */

/*
 * Map a mechanical angle (rad) to a LUT bin index [0, FOC_COGGING_LUT_POINT_COUNT).
 */
static uint16_t CoggingCalib_AngleToBin(float mech_angle_rad)
{
    float step_rad;
    uint16_t idx;

    while (mech_angle_rad < 0.0f)
    {
        mech_angle_rad += FOC_MATH_TWO_PI;
    }
    while (mech_angle_rad >= FOC_MATH_TWO_PI)
    {
        mech_angle_rad -= FOC_MATH_TWO_PI;
    }

    step_rad = FOC_MATH_TWO_PI / (float)FOC_COGGING_LUT_POINT_COUNT;
    idx = (uint16_t)(mech_angle_rad / step_rad);
    if (idx >= FOC_COGGING_LUT_POINT_COUNT)
    {
        idx = (uint16_t)(FOC_COGGING_LUT_POINT_COUNT - 1U);
    }
    return idx;
}

/* ------------------------------------------------------------------ */

static void CoggingCalib_Finish(foc_motor_t *motor)
{
    uint16_t i;
    uint16_t valid_count;
    float    dtheta_avg[COGGING_LUT_MAX];
    int16_t  q15_table[COGGING_LUT_MAX];
    float    mean;
    char     buf[64];

    /* ---- Step 1: Compute average Δθ per bin. ---- */
    valid_count = 0U;
    mean        = 0.0f;

    for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        if (g_calib_count[i] > 0U)
        {
            dtheta_avg[i] = g_calib_dtheta_accum[i] / (float)g_calib_count[i];
            valid_count++;
            mean += dtheta_avg[i];
        }
        else
        {
            dtheta_avg[i] = 0.0f;
        }
    }

    /* Report validity. */
    (void)snprintf(buf, sizeof(buf),
                   "CALIB DONE: %u/%u valid bins\r\n",
                   (unsigned)valid_count,
                   (unsigned)FOC_COGGING_LUT_POINT_COUNT);
    FOC_Platform_WriteDebugText(buf);

    /* ---- Step 2: Remove DC offset. ---- */
    if (valid_count > 0U)
    {
        mean /= (float)valid_count;
        for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
        {
            dtheta_avg[i] -= mean;
        }
    }

    /* ---- Step 3: Circular difference → iq compensation. ---- */
    for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        uint16_t next = (i + 1U) % FOC_COGGING_LUT_POINT_COUNT;
        //float diff    = dtheta_avg[next] - dtheta_avg[i];
        //float iq_comp = diff * motor->cogging_comp_status.calib_gain_k;
        float iq_comp = dtheta_avg[next] - dtheta_avg[i];

        q15_table[i] = FloatToQ15(iq_comp, FOC_COGGING_LUT_IQ_LSB_A);
    }

    /* ---- Step 4: Load into motor compensation table. ---- */
    (void)FOC_ControlLoadCoggingCompTableQ15(motor,
                                              q15_table,
                                              FOC_COGGING_LUT_POINT_COUNT,
                                              FOC_COGGING_LUT_IQ_LSB_A,
                                              FOC_COGGING_COMP_SOURCE_CALIB);

    /* Save global copy for dump/export without motor pointer. */
    (void)memcpy((void *)g_calib_last_table_q15,
                 (const void *)q15_table,
                 (size_t)FOC_COGGING_LUT_POINT_COUNT * sizeof(int16_t));
    g_calib_last_point_count = FOC_COGGING_LUT_POINT_COUNT;
    g_calib_last_iq_lsb_a    = FOC_COGGING_LUT_IQ_LSB_A;
    g_calib_table_available  = 1U;

    /* Restore soft-switch state saved at calibration start. */
    motor->current_soft_switch_status.enabled       = g_calib_saved_softswitch_enabled;
    motor->current_soft_switch_status.configured_mode = g_calib_saved_softswitch_mode;

    /* Mark calibration complete. */
    motor->cogging_comp_status.calib_in_progress      = 0U;
    motor->cogging_comp_status.calib_progress_percent  = 100U;
    motor->cogging_comp_status.calib_point_index       = CALIB_PHASE_DONE;

    FOC_Platform_WriteDebugText("COGGING CALIB FINISHED\r\n");

    /* Auto-dump the final table. */
    FOC_CoggingCalibDumpTable();
}

/* ------------------------------------------------------------------ */

uint8_t FOC_CoggingCalibSampleStep(foc_motor_t *motor,
                                    const sensor_data_t *sensor,
                                    float dt_sec)
{
    if ((motor == 0) || (sensor == 0))
    {
        return 0U;
    }

    /* --- Consume deferred request flags --- */
    if (g_calib_request_start != 0U)
    {
        g_calib_request_start = 0U;
        (void)FOC_CoggingCalibStart(motor);
    }

    if (g_calib_request_dump != 0U)
    {
        g_calib_request_dump = 0U;
        FOC_CoggingCalibDumpTable();
    }

    if (g_calib_request_export != 0U)
    {
        g_calib_request_export = 0U;
        FOC_CoggingCalibExportTable();
    }

    /* If not in calibration, return 0 (idle). */
    if (motor->cogging_comp_status.calib_in_progress == 0U)
    {
        return 0U;
    }

    /* Use a safe dt_sec fallback if not provided. */
    if (dt_sec <= 0.0f)
    {
        dt_sec = FOC_CONTROL_DT_SEC;
    }

    /* ---------- State machine ---------- */
    switch (motor->cogging_comp_status.calib_point_index)
    {
        /* ================================================================ */
        case CALIB_PHASE_START:
        {
            /*
             * Seed the expected electrical angle from the current motor
             * electrical phase angle (which reflects the actual rotor position).
             * This ensures the open-loop drive starts from the correct position.
             */
            g_calib_expected_elec_angle = motor->electrical_phase_angle;

            /* Seed expected mechanical angle from the sensor's actual mechanical angle. */
            g_calib_expected_mech_angle = sensor->mech_angle_rad.output_value;
            g_calib_expected_mech_angle = Math_WrapRad(g_calib_expected_mech_angle);

            /* Initialise angle tracking for revolution detection. */
            g_calib_angle_prev_rad   = sensor->mech_angle_rad.output_value;
            g_calib_travel_accum_rad = 0.0f;
            g_calib_rev_count        = 0U;

            /* Reset per-bin-once for subsequent SCAN phase. */
            g_calib_last_lut_index   = 0xFFFFU;
            g_calib_bins_collected   = 0U;

            /* Drive open-loop to get the motor moving right away. */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            /* Transition to SETTLE phase. */
            motor->cogging_comp_status.calib_point_index = CALIB_PHASE_SETTLE;
            motor->cogging_comp_status.calib_pass_index  = 0U;

            FOC_Platform_WriteDebugText("CALIB: start done, entering settle\r\n");

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_SETTLE:
        {
            float mech_angle;
            float delta;

            /* Run open-loop drive. */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            /* Track angular travel for revolution counting. */
            mech_angle = sensor->mech_angle_rad.output_value;
            delta = CoggingCalib_AngleDelta(mech_angle, g_calib_angle_prev_rad);
            g_calib_travel_accum_rad += delta;
            g_calib_angle_prev_rad = mech_angle;

            /* Check if a full revolution has completed. */
            if (g_calib_travel_accum_rad >= FOC_MATH_TWO_PI)
            {
                g_calib_rev_count++;
                g_calib_travel_accum_rad = 0.0f;

                if (g_calib_rev_count >= FOC_COGGING_CALIB_SETTLE_REV)
                {
                    /* Transition to SCAN phase. */
                    motor->cogging_comp_status.calib_point_index = CALIB_PHASE_SCAN;
                    motor->cogging_comp_status.calib_pass_index  = 0U;
                    g_calib_rev_count              = 0U;
                    g_calib_travel_accum_rad       = 0.0f;
                    g_calib_last_lut_index         = 0xFFFFU;
                    g_calib_bins_collected         = 0U;
                    g_calib_last_reported_progress = 0U;

                    /* Re-sync angle tracking to avoid a large delta on first SCAN step. */
                    g_calib_angle_prev_rad = mech_angle;

                    {
                        char buf[48];
                        (void)snprintf(buf, sizeof(buf),
                                      "CALIB: settle done (%u revs), starting scan\r\n",
                                      (unsigned)FOC_COGGING_CALIB_SETTLE_REV);
                        FOC_Platform_WriteDebugText(buf);
                    }
                }
            }

            /* Update progress: fraction of SETTLE phase. */
            {
                float settle_frac = (float)g_calib_rev_count /
                                    (float)FOC_COGGING_CALIB_SETTLE_REV;
                if (settle_frac > 1.0f)
                {
                    settle_frac = 1.0f;
                }
                motor->cogging_comp_status.calib_progress_percent =
                    (uint8_t)(settle_frac * 10.0f);  /* 0-10 % during settle */
            }

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_SCAN:
        {
            float mech_angle;
            float delta;
            uint16_t lut_index;
            float mech_expected;

            /* Run open-loop drive. */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            /* Read current mechanical angle from sensor. */
            mech_angle = sensor->mech_angle_rad.output_value;

            /* Compute angular travel since last call (handles 0/2pi wrap). */
            delta = CoggingCalib_AngleDelta(mech_angle, g_calib_angle_prev_rad);
            g_calib_travel_accum_rad += delta;
            g_calib_angle_prev_rad = mech_angle;

            /* Use self-maintained expected mechanical angle directly. */
            mech_expected = g_calib_expected_mech_angle;

            /* Compute position error Δθ. */
            {
                float dtheta = Math_WrapRadDelta(mech_angle - mech_expected);

                /* Map measured mechanical angle to LUT bin. */
                lut_index = CoggingCalib_AngleToBin(mech_angle);

                /* Per-bin-once sample: only accumulate when angle crosses into a new bin. */
                if (lut_index != g_calib_last_lut_index)
                {
                    g_calib_dtheta_accum[lut_index] += dtheta;
                    g_calib_count[lut_index]++;
                    g_calib_last_lut_index = lut_index;
                    g_calib_bins_collected++;
                }
            }

            /* --- Progress: bin-count based (immune to sensor noise backlash) --- */
            {
                float pass_progress_frac;
                uint8_t pct;

                pass_progress_frac = (float)g_calib_bins_collected /
                                     (float)FOC_COGGING_LUT_POINT_COUNT;
                if (pass_progress_frac > 1.0f)
                {
                    pass_progress_frac = 1.0f;
                }

                pct = (uint8_t)(((float)g_calib_pass_num + pass_progress_frac) *
                                100.0f / (float)FOC_COGGING_CALIB_NUM_PASSES);
                if (pct > 100U)
                {
                    pct = 100U;
                }
                motor->cogging_comp_status.calib_progress_percent = pct;
            }

            /* --- Periodic debug output every ~10 % of bins within this pass --- */
            {
                uint8_t tenth = (uint8_t)(((uint32_t)g_calib_bins_collected * 10U) /
                                          (uint32_t)FOC_COGGING_LUT_POINT_COUNT);
                if (tenth != g_calib_last_reported_progress)
                {
                    g_calib_last_reported_progress = tenth;
                    {
                        char buf[48];
                        (void)snprintf(buf, sizeof(buf),
                                      "CALIB: pass %u/%u, progress %u%%\r\n",
                                      (unsigned)(g_calib_pass_num + 1U),
                                      (unsigned)FOC_COGGING_CALIB_NUM_PASSES,
                                      (unsigned)motor->cogging_comp_status.calib_progress_percent);
                        FOC_Platform_WriteDebugText(buf);
                    }
                }
            }

            /*
             * Revolution detection (bin-count based): per-bin-once sampling
             * gives exactly one sample per bin per revolution, so collecting
             * all LUT bins means one full mechanical revolution is complete.
             */
            if (g_calib_bins_collected >= FOC_COGGING_LUT_POINT_COUNT)
            {
                g_calib_pass_num++;
                motor->cogging_comp_status.calib_pass_index = 0U;
                motor->cogging_comp_status.calib_point_index = CALIB_PHASE_CHECK;
                return 1U;
            }

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_CHECK:
        {
            /*
             * One SCAN pass completed.  Check if we need more passes.
             * If yes, reset SCAN tracking and continue.
             * If no, proceed to FINISH.
             */

            if (g_calib_pass_num < FOC_COGGING_CALIB_NUM_PASSES)
            {
                /* Reset per-bin-once and travel tracking for next SCAN pass. */
                motor->cogging_comp_status.calib_pass_index    = 0U;
                motor->cogging_comp_status.calib_point_index    = CALIB_PHASE_SCAN;
                g_calib_travel_accum_rad = 0.0f;
                g_calib_last_lut_index         = 0xFFFFU;
                g_calib_last_reported_progress = 0U;
                g_calib_bins_collected         = 0U;
                g_calib_rev_count              = 0U;

                /*
                 * Re-sync angle tracking to avoid missing angle on first
                 * step of next SCAN pass.
                 */
                g_calib_angle_prev_rad = sensor->mech_angle_rad.output_value;

                {
                    char buf[48];
                    (void)snprintf(buf, sizeof(buf),
                                  "CALIB: pass %u/%u done\r\n",
                                  (unsigned)(g_calib_pass_num),
                                  (unsigned)FOC_COGGING_CALIB_NUM_PASSES);
                    FOC_Platform_WriteDebugText(buf);
                }
            }
            else
            {
                /* All passes complete, transition to FINISH. */
                motor->cogging_comp_status.calib_point_index = CALIB_PHASE_FINISH;
                FOC_Platform_WriteDebugText("CALIB: all passes done, computing table\r\n");
            }

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_FINISH:
        {
            /* Compute differential table and load into motor. */
            CoggingCalib_Finish(motor);
            return 0U;
        }

        /* ================================================================ */
        default:
            /* IDLE, DONE or unknown state. */
            return 0U;
    }
}

/* ------------------------------------------------------------------ */

void FOC_CoggingCalibDumpTable(void)
{
    uint16_t i;
    char line_buf[128];
    uint16_t n;

    if (g_calib_table_available == 0U)
    {
        FOC_Platform_WriteDebugText("--- COGGING LUT DUMP: no table available ---\r\n");
        return;
    }

    n = g_calib_last_point_count;
    if (n == 0U)
    {
        FOC_Platform_WriteDebugText("--- COGGING LUT DUMP: empty table ---\r\n");
        return;
    }

    FOC_Platform_WriteDebugText("--- COGGING LUT DUMP (Q15) ---\r\n");

    for (i = 0U; i < n; i++)
    {
        float iq_a = Q15ToFloat(g_calib_last_table_q15[i], g_calib_last_iq_lsb_a);

        (void)snprintf(line_buf, sizeof(line_buf),
                       "cogging.table[%u]=%d  (%.6f A)\r\n",
                       (unsigned)i,
                       (int)g_calib_last_table_q15[i],
                       (double)iq_a);
        FOC_Platform_WriteDebugText(line_buf);
    }

    FOC_Platform_WriteDebugText("--- END LUT DUMP ---\r\n");
    (void)line_buf;
}

void FOC_CoggingCalibExportTable(void)
{
    uint16_t i;
    char line_buf[128];

    if (g_calib_table_available == 0U)
    {
        FOC_Platform_WriteDebugText("COGGING EXPORT: no calibrated table available\r\n");
        FOC_Platform_WriteDebugText("Run cogging calibration first (Y:G) then retry.\r\n");
        return;
    }

    FOC_Platform_WriteDebugText("--- COGGING LUT EXPORT (C code) ---\r\n");

    (void)snprintf(line_buf, sizeof(line_buf),
                   "static const int16_t cogging_table_q15[%u] = {\r\n",
                   (unsigned)g_calib_last_point_count);
    FOC_Platform_WriteDebugText(line_buf);

    for (i = 0U; i < g_calib_last_point_count; i++)
    {
        if ((i % 8U) == 0U)
        {
            FOC_Platform_WriteDebugText("    ");
        }

        (void)snprintf(line_buf, sizeof(line_buf),
                       "%5d",
                       (int)g_calib_last_table_q15[i]);

        if (i < (g_calib_last_point_count - 1U))
        {
            /* Append comma */
            size_t len = strlen(line_buf);
            if (len < (sizeof(line_buf) - 2U))
            {
                line_buf[len]     = ',';
                line_buf[len + 1U] = '\0';
            }
        }

        if (((i + 1U) % 8U) == 0U)
        {
            (void)snprintf(line_buf + strlen(line_buf),
                           sizeof(line_buf) - strlen(line_buf),
                           "\r\n");
        }
        else if (i < (g_calib_last_point_count - 1U))
        {
            (void)snprintf(line_buf + strlen(line_buf),
                           sizeof(line_buf) - strlen(line_buf),
                           " ");
        }
        else
        {
            (void)snprintf(line_buf + strlen(line_buf),
                           sizeof(line_buf) - strlen(line_buf),
                           "\r\n");
        }

        FOC_Platform_WriteDebugText(line_buf);
    }

    FOC_Platform_WriteDebugText("};\r\n");
    FOC_Platform_WriteDebugText("--- END EXPORT ---\r\n");
}

#endif /* FOC_COGGING_CALIB_ENABLE */
