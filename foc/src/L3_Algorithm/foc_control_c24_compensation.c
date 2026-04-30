#include "L3_Algorithm/foc_control_c24_compensation.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"
#include "L3_Algorithm/foc_control_c31_actuation.h"

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)

/* ======================================================================== *
 *  Q15 helpers (shared by lookup and calibration)                          *
 * ======================================================================== */

static inline float Q15ToFloat(int16_t q15, float lsb)
{
    return (float)q15 * lsb;
}

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
/*
 * Float to Q15 conversion – only used by calibration code.
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
#endif /* FOC_COGGING_CALIB_ENABLE */

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

    /* Only compensate when running below the speed gate (low-speed cogging). */
    speed_abs = (speed_ref_rad_s >= 0.0f) ? speed_ref_rad_s : -speed_ref_rad_s;
    if (speed_abs > status->speed_gate_rad_s)
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

    /*
     * Apply compensation current scaled by runtime gain K.
     * The LUT stores raw compensation currents (amps); gain_k
     * allows online fine-tuning of the compensation intensity.
     * gain_k = 0 disables, gain_k = 1 applies table values as-is.
     */
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

    if (point_count > FOC_COGGING_LUT_POINT_COUNT)
    {
        point_count = FOC_COGGING_LUT_POINT_COUNT;
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


#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

/* --- Deferred request flags (scalar statics, negligible RAM) --- */
static uint8_t g_calib_request_start  = 0U;
static uint8_t g_calib_request_dump   = 0U;
static uint8_t g_calib_request_export = 0U;

/*
 * Note: no static scratch buffers needed — calibration operates directly
 * on motor->cogging_comp_table_q15 (Q15 domain, in-place).
 */

/* Calibration internal state machine phases (stored in calib_state.point_index). */
#define CALIB_PHASE_IDLE     0xFFFFU   /* idle / not started */
#define CALIB_PHASE_START    0xFFFEU   /* init accumulators and predicted angle */
#define CALIB_PHASE_SETTLE   0xFFFDU   /* open-loop drive N revs to build equilibrium */
#define CALIB_PHASE_SCAN     0xFFFCU   /* accumulate Δθ per LUT bin */
#define CALIB_PHASE_CHECK    0xFFFBU   /* one scan pass complete, decide next */
#define CALIB_PHASE_FINISH   0xFFFAU   /* compute compensation table and load */
#define CALIB_PHASE_DONE     0xFFF9U   /* finished */

/* ------------------------------------------------------------------ */

void FOC_CoggingCalibRequestStart(void)
{
    g_calib_request_start = 1U;
}

void FOC_CoggingCalibRequestDump(void)
{
    g_calib_request_dump = 1U;
}

void FOC_CoggingCalibRequestExport(void)
{
    g_calib_request_export = 1U;
}

uint8_t FOC_CoggingCalibIsDumpPending(void)
{
    return g_calib_request_dump;
}

uint8_t FOC_CoggingCalibIsExportPending(void)
{
    return g_calib_request_export;
}

void FOC_CoggingCalibClearDumpPending(void)
{
    g_calib_request_dump = 0U;
}

void FOC_CoggingCalibClearExportPending(void)
{
    g_calib_request_export = 0U;
}

/* ------------------------------------------------------------------ */

uint8_t FOC_CoggingCalibStart(foc_motor_t *motor)
{
    uint16_t i;

    if (motor == 0)
    {
        return 0U;
    }

    /* Reject if calibration already in progress. */
    if (motor->cogging_calib_state.in_progress != 0U)
    {
        return 0U;
    }

    /*
     * Zero the compensation table — it doubles as the Q15 accumulation
     * buffer during calibration scan passes.
     */
    for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        motor->cogging_comp_table_q15[i] = 0;
    }

    /* Initialise calibration state. */
    motor->cogging_calib_state.in_progress       = 1U;
    motor->cogging_calib_state.progress_percent  = 0U;
    motor->cogging_calib_state.point_index       = CALIB_PHASE_START;
    motor->cogging_calib_state.completed_pass_count = 0U;

    /* Prediction state: will be seeded from sensor on first START step. */
    motor->cogging_calib_state.pred_mech_angle   = 0.0f;
    motor->cogging_calib_state.angle_prev_rad    = -1.0f;
    motor->cogging_calib_state.travel_accum_rad  = 0.0f;

    /* Reset per-bin-once tracking and progress throttle. */
    motor->cogging_calib_state.last_lut_index         = 0xFFFFU;
    motor->cogging_calib_state.last_reported_progress = 0U;
    motor->cogging_calib_state.bins_collected         = 0U;
    motor->cogging_calib_state.rev_count              = 0U;
    motor->cogging_calib_state.pass_num               = 0U;

    /* Save current soft-switch state so it can be restored on finish. */
    motor->cogging_calib_state.saved_softswitch_enabled = motor->current_soft_switch_status.enabled;
    motor->cogging_calib_state.saved_softswitch_mode    = motor->current_soft_switch_status.configured_mode;

    FOC_Platform_WriteDebugText("COGGING CALIB START (position-deviation method)\r\n");

    return 1U;
}

/* ------------------------------------------------------------------ */

/*
 * Helper: compute signed angular delta in [-pi, pi) between two
 * mechanical angle readings, handling 0/2pi wrap.
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

static void CoggingCalib_OpenLoopDriveStep(foc_motor_t *motor, float dt_sec)
{
    float phase_resistance;
    float uq;
    float elec_angle;
    float pred_wrapped;

    if (motor == 0)
    {
        return;
    }

    /* Wrap the predicted angle to [0, 2π). */
    pred_wrapped = Math_WrapRad(motor->cogging_calib_state.pred_mech_angle);

    /* Derive electrical angle: θ_elec = θ_mech × pole_pairs.
     * No direction factor here – θ_pred already moves in the physical
     * shaft direction (signed by motor->direction in the integrator). */
    elec_angle = pred_wrapped * (float)motor->pole_pairs;
    elec_angle = Math_WrapRad(elec_angle);

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
    motor->electrical_phase_angle = elec_angle;
    FOC_ControlApplyElectricalAngleRuntime(motor, elec_angle);
}

/* ------------------------------------------------------------------ */

/*
 * Map a mechanical angle (rad) to a LUT bin index [0, FOC_COGGING_LUT_POINT_COUNT).
 * 0 rad → bin 0, 2π → bin N-1.
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


/*
 * Q15 boundary discontinuity fix.
 * Replaces the old float version (now operating directly on Q15 counts).
 * threshold_q15 ≈ COGGING_BOUNDARY_THRESHOLD_RAD / (dtheta_scale * iq_lsb_a)
 * which at default config maps to roughly 1200 Q15 counts.
 */
#define COGGING_BOUNDARY_Q15_THRESHOLD 1200

static void CoggingCalib_FixBoundaryDiscontinuityQ15(int16_t table[], uint16_t n_points)
{
    int32_t diff;

    if (n_points < 3U)
    {
        return;
    }

    diff = (int32_t)table[0] - (int32_t)table[n_points - 1U];
    if (diff < 0) diff = -diff;

    if (diff > COGGING_BOUNDARY_Q15_THRESHOLD)
    {
        int16_t replacement = (int16_t)(((int32_t)table[0] + (int32_t)table[n_points - 1U]) / 2);
        table[0]                  = replacement;
        table[n_points - 1U]      = replacement;
    }

    if (n_points > 4U)
    {
        diff = (int32_t)table[1] - (int32_t)table[n_points - 2U];
        if (diff < 0) diff = -diff;

        if (diff > COGGING_BOUNDARY_Q15_THRESHOLD)
        {
            int16_t replacement = (int16_t)(((int32_t)table[1] + (int32_t)table[n_points - 2U]) / 2);
            table[1]                  = replacement;
            table[n_points - 2U]      = replacement;
        }
    }
}

/* ------------------------------------------------------------------ */

/*
 * Neighbour-difference outlier detection for Q15 table.
 *
 * A bin is flagged as an outlier when the absolute difference to both
 * immediate neighbours exceeds COGGING_OUTLIER_Q15_THRESHOLD.
 * Flagged bins are replaced by the average of their two neighbours.
 *
 * This replaces the old IQR+sort method, eliminating the need for a
 * large scratch buffer.
 */
#define COGGING_OUTLIER_Q15_THRESHOLD 1200

static void CoggingCalib_RemoveOutliersQ15(int16_t table[], uint16_t n)
{
    uint16_t i;
    int16_t left_val;
    int16_t right_val;
    int32_t diff_left;
    int32_t diff_right;

    if (n < 3U)
    {
        return;
    }

    for (i = 0U; i < n; i++)
    {
        left_val  = (i > 0U) ? table[i - 1U] : table[n - 1U];
        right_val = (i < (n - 1U)) ? table[i + 1U] : table[0U];

        diff_left  = (int32_t)table[i] - (int32_t)left_val;
        if (diff_left < 0) diff_left = -diff_left;

        diff_right = (int32_t)table[i] - (int32_t)right_val;
        if (diff_right < 0) diff_right = -diff_right;

        if ((diff_left > COGGING_OUTLIER_Q15_THRESHOLD) &&
            (diff_right > COGGING_OUTLIER_Q15_THRESHOLD))
        {
            table[i] = (int16_t)(((int32_t)left_val + (int32_t)right_val) / 2);
        }
    }
}

/* ------------------------------------------------------------------ */

static void CoggingCalib_Finish(foc_motor_t *motor)
{
    uint16_t i;
    int32_t  sum;
    int16_t *table;
    char     buf[64];
    foc_cogging_calib_state_t *state;

    if (motor == 0)
    {
        return;
    }

    state = &motor->cogging_calib_state;
    table = motor->cogging_comp_table_q15;   /* ← operates in-place */

    /* ---- Step 1: Compute sum for zero-mean correction (int32_t). ---- */
    sum = 0;
    for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        sum += (int32_t)table[i];
    }

    /* Report validity. */
    (void)snprintf(buf, sizeof(buf),
                   "CALIB DONE: %u bins, sum=%ld\r\n",
                   (unsigned)FOC_COGGING_LUT_POINT_COUNT,
                   (long)sum);
    FOC_Platform_WriteDebugText(buf);

    /* ---- Step 2: Zero-mean in Q15 domain. ---- */
    {
        int16_t mean_q15 = (int16_t)(sum / (int32_t)FOC_COGGING_LUT_POINT_COUNT);
        for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
        {
            table[i] -= mean_q15;
        }
    }

    /* ---- Step 2b: Outlier removal (neighbour-difference). ---- */
    CoggingCalib_RemoveOutliersQ15(table, FOC_COGGING_LUT_POINT_COUNT);

    /* ---- Step 2c: Boundary discontinuity fix. ---- */
    CoggingCalib_FixBoundaryDiscontinuityQ15(table, FOC_COGGING_LUT_POINT_COUNT);

    /* ---- Step 3: Direction compensation. ---- */
    if (motor->direction == FOC_DIR_REVERSED)
    {
        for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
        {
            table[i] = (int16_t)(-(int32_t)table[i]);
        }
    }

    /* ---- Step 4: Mark table as available (no memcpy needed). ---- */
    motor->cogging_comp_status.point_count = FOC_COGGING_LUT_POINT_COUNT;
    motor->cogging_comp_status.iq_lsb_a    = FOC_COGGING_LUT_IQ_LSB_A;
    motor->cogging_comp_status.source      = FOC_COGGING_COMP_SOURCE_CALIB;
    motor->cogging_comp_status.available   = 1U;

    /* Restore soft-switch state saved at calibration start. */
    motor->current_soft_switch_status.enabled       = state->saved_softswitch_enabled;
    motor->current_soft_switch_status.configured_mode = state->saved_softswitch_mode;

    /* Mark calibration complete. */
    state->in_progress       = 0U;
    state->progress_percent  = 100U;
    state->point_index       = CALIB_PHASE_DONE;

    FOC_Platform_WriteDebugText("COGGING CALIB FINISHED\r\n");

    /* Trigger auto-dump (flag consumed in FOC_App_Loop for serial output). */
    g_calib_request_dump = 1U;

}

/* ------------------------------------------------------------------ */

uint8_t FOC_CoggingCalibSampleStep(foc_motor_t *motor,
                                    const sensor_data_t *sensor,
                                    float dt_sec)
{
    foc_cogging_calib_state_t *state;

    if ((motor == 0) || (sensor == 0))
    {
        return 0U;
    }

    state = &motor->cogging_calib_state;

    /* --- Consume deferred request flags --- */
    if (g_calib_request_start != 0U)
    {
        g_calib_request_start = 0U;
        (void)FOC_CoggingCalibStart(motor);
    }

    /* If not in calibration, return 0 (idle). */
    if (state->in_progress == 0U)
    {
        return 0U;
    }

    /* Use a safe dt_sec fallback if not provided. */
    if (dt_sec <= 0.0f)
    {
        dt_sec = FOC_CONTROL_DT_SEC;
    }

    /* ---------- State machine ---------- */
    switch (state->point_index)
    {
        /* ================================================================ */
        case CALIB_PHASE_START:
        {
            float mech_actual;

            /*
             * Seed the predicted angle from the sensor's current position.
             * This is the ONLY time we couple θ_pred to the sensor.
             * After this, θ_pred runs as a pure integrator.
             */
            mech_actual = sensor->mech_angle_rad.output_value;
            state->pred_mech_angle = mech_actual;

            /* Initialise angle tracking for revolution detection. */
            state->angle_prev_rad    = mech_actual;
            state->travel_accum_rad  = 0.0f;
            state->rev_count         = 0U;

            /* Reset per-bin-once for subsequent SCAN phase. */
            state->last_lut_index    = 0xFFFFU;
            state->bins_collected    = 0U;

            /* Drive open-loop to get the motor moving right away. */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            /* Transition to SETTLE phase. */
            state->point_index = CALIB_PHASE_SETTLE;

            FOC_Platform_WriteDebugText("CALIB: start done, entering settle\r\n");

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_SETTLE:
        {
            state->pred_mech_angle += FOC_COGGING_CALIB_SPEED_RAD_S *
                                       (float)motor->direction *
                                       dt_sec;

            /* Drive with the predicted angle. */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);
            {
                float mech_actual = sensor->mech_angle_rad.output_value;
                float delta = CoggingCalib_AngleDelta(mech_actual, state->angle_prev_rad);
                state->travel_accum_rad += (delta >= 0.0f) ? delta : -delta;
                state->angle_prev_rad = mech_actual;
            }

            /* Check if a full revolution has completed. */
            if (state->travel_accum_rad >= FOC_MATH_TWO_PI)
            {
                state->rev_count++;
                state->travel_accum_rad = 0.0f;

                if (state->rev_count >= FOC_COGGING_CALIB_SETTLE_REV)
                {
                    /* Transition to SCAN phase. */
                    state->point_index         = CALIB_PHASE_SCAN;
                    state->rev_count           = 0U;
                    state->travel_accum_rad    = 0.0f;
                    state->last_lut_index      = 0xFFFFU;
                    state->bins_collected      = 0U;
                    state->last_reported_progress = 0U;

                    /* Re-sync angle tracking. */
                    state->angle_prev_rad = sensor->mech_angle_rad.output_value;

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
                float settle_frac = (float)state->rev_count /
                                    (float)FOC_COGGING_CALIB_SETTLE_REV;
                if (settle_frac > 1.0f)
                {
                    settle_frac = 1.0f;
                }
                state->progress_percent =
                    (uint8_t)(settle_frac * 10.0f);  /* 0-10 % during settle */
            }

            return 1U;
        }

        case CALIB_PHASE_SCAN:
        {
            float mech_actual;
            float pred_wrapped;
            float dtheta;
            uint16_t lut_index;
            int16_t *table;

            table = motor->cogging_comp_table_q15;
            mech_actual = Math_WrapRad(sensor->mech_angle_rad.output_value);

            if (motor->direction == FOC_DIR_NORMAL)
            {
                pred_wrapped = Math_WrapRad(state->pred_mech_angle);
            }
            else
            {
                pred_wrapped = Math_WrapRad(FOC_MATH_TWO_PI - state->pred_mech_angle);
            }

            dtheta = Math_WrapRadDelta(mech_actual - pred_wrapped);

            lut_index = CoggingCalib_AngleToBin(mech_actual);

            if (lut_index != state->last_lut_index)
            {
                /*
                 * Running-average update for this bin.
                 * Accumulate IQ compensation (Q15) directly into the
                 * same motor->cogging_comp_table_q15 that holds the
                 * final compensation LUT.
                 *
                 *   iq_comp = -dtheta * dtheta_scale    (Amperes)
                 *   q15_val = iq_comp / iq_lsb_a        (Q15 counts)
                 *
                 * completed_pass_count tracks how many full passes
                 * have been merged so far.
                 */
                {
                    float iq_comp  = -dtheta * FOC_COGGING_CALIB_DTHETA_SCALE;
                    int16_t q15_val = FloatToQ15(iq_comp, FOC_COGGING_LUT_IQ_LSB_A);
                    uint8_t pcnt    = state->completed_pass_count;

                    if (pcnt == 0U)
                    {
                        /* First pass: store raw Q15 value. */
                        table[lut_index] = q15_val;
                    }
                    else
                    {
                        /* Running average: (prev * pcnt + q15_val) / (pcnt + 1). */
                        int32_t accum = (int32_t)table[lut_index] * (int32_t)pcnt
                                      + (int32_t)q15_val;
                        table[lut_index] = (int16_t)(accum / (int32_t)(pcnt + 1U));
                    }
                }

                state->last_lut_index = lut_index;
                state->bins_collected++;
            }

            state->pred_mech_angle += FOC_COGGING_CALIB_SPEED_RAD_S *
                                       (float)motor->direction *
                                       dt_sec;

            /* Drive with the updated predicted angle. */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            /* Track angular travel for revolution counting. */
            {
                float delta = CoggingCalib_AngleDelta(mech_actual, state->angle_prev_rad);
                state->travel_accum_rad += delta;
                state->angle_prev_rad = mech_actual;
            }

            /* --- Progress: bin-count based --- */
            {
                float pass_progress_frac;
                uint8_t pct;

                pass_progress_frac = (float)state->bins_collected / (float)FOC_COGGING_LUT_POINT_COUNT;

                if (pass_progress_frac > 1.0f)
                {
                    pass_progress_frac = 1.0f;
                }

                pct = (uint8_t)(((float)state->pass_num + pass_progress_frac) *
                                100.0f / (float)FOC_COGGING_CALIB_NUM_PASSES);
                if (pct > 100U)
                {
                    pct = 100U;
                }
                state->progress_percent = pct;
            }

            /* --- Periodic debug output every ~10 % of bins within this pass --- */
            {
                uint8_t tenth = (uint8_t)(((uint32_t)state->bins_collected * 10U) /
                                          (uint32_t)FOC_COGGING_LUT_POINT_COUNT);
                if (tenth != state->last_reported_progress)
                {
                    state->last_reported_progress = tenth;
                    {
                        char buf[48];
                        (void)snprintf(buf, sizeof(buf),
                                      "CALIB: pass %u/%u, progress %u%%\r\n",
                                      (unsigned)(state->pass_num + 1U),
                                      (unsigned)FOC_COGGING_CALIB_NUM_PASSES,
                                      (unsigned)state->progress_percent);
                        FOC_Platform_WriteDebugText(buf);
                    }
                }
            }

            /*
             * Revolution detection: once we've collected all unique bins,
             * one full mechanical revolution of SCAN data is complete.
             */
            if (state->bins_collected >= FOC_COGGING_LUT_POINT_COUNT)
            {
                state->completed_pass_count++;
                state->pass_num++;
                state->point_index = CALIB_PHASE_CHECK;
                return 1U;
            }

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_CHECK:
        {
            /*
             * One SCAN pass completed.  Check if more passes needed.
             */
            if (state->pass_num < FOC_COGGING_CALIB_NUM_PASSES)
            {
                state->point_index           = CALIB_PHASE_SCAN;
                state->travel_accum_rad     = 0.0f;
                state->last_lut_index       = 0xFFFFU;
                state->last_reported_progress = 0U;
                state->bins_collected       = 0U;
                state->rev_count            = 0U;

                state->angle_prev_rad = sensor->mech_angle_rad.output_value;

                {
                    char buf[48];
                    (void)snprintf(buf, sizeof(buf),
                                  "CALIB: pass %u done\r\n",
                                  (unsigned)(state->pass_num));
                    FOC_Platform_WriteDebugText(buf);
                }
            }
            else
            {
                state->point_index = CALIB_PHASE_FINISH;
                FOC_Platform_WriteDebugText("CALIB: all passes done, computing table\r\n");
            }

            return 1U;
        }

        /* ================================================================ */
        case CALIB_PHASE_FINISH:
        {
            CoggingCalib_Finish(motor);
            return 0U;
        }

        /* ================================================================ */
        default:
            return 0U;
    }
}

/* ------------------------------------------------------------------ */

/*
 * Dump and export are called from FOC_App_Loop (not ISR context) to
 * avoid blocking the control chain.  The request flags are set by
 * protocol handlers (Y:T / Y:D) and consumed by these functions.
 */

void FOC_CoggingCalibDumpTable(const foc_motor_t *motor)
{
    uint16_t i;
    char line_buf[72];
    uint16_t n;
    const int16_t *table;

    if (motor == 0)
    {
        return;
    }

    if (motor->cogging_comp_status.available == 0U)
    {
        FOC_Platform_WriteDebugText("--- COGGING LUT DUMP: no table available ---\r\n");
        return;
    }

    n = motor->cogging_comp_status.point_count;
    table = motor->cogging_comp_table_q15;

    if ((n == 0U) || (table == 0))
    {
        FOC_Platform_WriteDebugText("--- COGGING LUT DUMP: empty table ---\r\n");
        return;
    }

    FOC_Platform_WriteDebugText("--- COGGING LUT DUMP (Q15) ---\r\n");

    for (i = 0U; i < n; i++)
    {
        (void)snprintf(line_buf, sizeof(line_buf),
                       "%d,",
                       (int)table[i]);
        FOC_Platform_WriteDebugText(line_buf);
    }

    FOC_Platform_WriteDebugText("--- END LUT DUMP ---\r\n");
    (void)line_buf;
}

void FOC_CoggingCalibExportTable(const foc_motor_t *motor)
{
    uint16_t i;
    char line_buf[72];
    uint16_t n;
    const int16_t *table;

    if (motor == 0)
    {
        return;
    }

    if (motor->cogging_comp_status.available == 0U)
    {
        FOC_Platform_WriteDebugText("COGGING EXPORT: no calibrated table available\r\n");
        FOC_Platform_WriteDebugText("Run cogging calibration first (Y:G) then retry.\r\n");
        return;
    }

    n = motor->cogging_comp_status.point_count;
    table = motor->cogging_comp_table_q15;

    FOC_Platform_WriteDebugText("--- COGGING LUT EXPORT (C code) ---\r\n");

    (void)snprintf(line_buf, sizeof(line_buf),
                   "static const int16_t cogging_table_q15[%u] = {\r\n",
                   (unsigned)n);
    FOC_Platform_WriteDebugText(line_buf);

    for (i = 0U; i < n; i++)
    {
        if ((i % 8U) == 0U)
        {
            FOC_Platform_WriteDebugText("    ");
        }

        (void)snprintf(line_buf, sizeof(line_buf),
                       "%5d",
                       (int)table[i]);

        if (i < (n - 1U))
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
        else if (i < (n - 1U))
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

#else /* FOC_COGGING_CALIB_ENABLE not set — provide stub */

uint8_t FOC_CoggingCalibSampleStep(foc_motor_t *motor,
                                    const sensor_data_t *sensor,
                                    float dt_sec)
{
    (void)motor;
    (void)sensor;
    (void)dt_sec;
    return 0U;
}

#endif /* FOC_COGGING_CALIB_ENABLE */

#endif /* FOC_COGGING_COMP_ENABLE */
