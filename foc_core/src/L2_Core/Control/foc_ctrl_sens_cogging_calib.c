#include "L2_Core/foc_motor_aggregate.h"
#include "L2_Core/Control/foc_ctrl_sens_cogging_calib.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

/* ========== Internal helpers ========== */

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

#define CALIB_PHASE_IDLE     0xFFFFU
#define CALIB_PHASE_START    0xFFFEU
#define CALIB_PHASE_SETTLE   0xFFFDU
#define CALIB_PHASE_SCAN     0xFFFCU
#define CALIB_PHASE_CHECK    0xFFFBU
#define CALIB_PHASE_FINISH   0xFFFAU
#define CALIB_PHASE_DONE     0xFFF9U

#define COGGING_OUTLIER_Q15_THRESHOLD 1200

/* ========== Public API ========== */

uint8_t FOC_CoggingCalibIsBusy(const foc_cogging_calib_state_t *state)
{
    return (state->in_progress != 0U) ||
           (state->request_start != 0U);
}

void FOC_CoggingCalib_RequestStart(foc_cogging_calib_state_t *state)
{
    state->request_start = 1U;
}

void FOC_CoggingCalib_Abort(foc_motor_t *motor)
{
    motor->cogging_calib_state.point_index = CALIB_PHASE_IDLE;
    motor->cogging_calib_state.request_start = 0U;
    motor->cogging_calib_state.pass_num = 0U;
    motor->cogging_calib_state.in_progress = 0U;
}

void FOC_CoggingCalib_RequestDump(foc_cogging_calib_state_t *state)
{
    state->request_dump = 1U;
}

void FOC_CoggingCalib_RequestExport(foc_cogging_calib_state_t *state)
{
    state->request_export = 1U;
}

uint8_t FOC_CoggingCalibIsDumpPending(const foc_cogging_calib_state_t *state)
{
    return state->request_dump;
}

uint8_t FOC_CoggingCalibIsExportPending(const foc_cogging_calib_state_t *state)
{
    return state->request_export;
}

void FOC_CoggingCalibClearDumpPending(foc_cogging_calib_state_t *state)
{
    state->request_dump = 0U;
}

void FOC_CoggingCalibClearExportPending(foc_cogging_calib_state_t *state)
{
    state->request_export = 0U;
}

/* ========== Internal helpers ========== */

static void CoggingCalib_OpenLoopDriveStep(foc_motor_t *motor, float dt_sec)
{
    float elec_angle;
    float mech_for_elec;

    mech_for_elec = motor->cogging_calib_state.pred_mech_angle + motor->params.mech_angle_at_elec_zero_rad;
    elec_angle = Math_WrapRad(mech_for_elec * (float)motor->params.pole_pairs * (float)motor->params.direction);

    motor->ctrl.ud = 0.0f;
    motor->ctrl.uq = FOC_COGGING_CALIB_IQ_A * motor->params.phase_resistance * (float)motor->params.direction;
    motor->ctrl.iq_target = 0.0f;

    FOC_ControlRecordPhaseOutputDqAngle(motor,
                                        FOC_CONTROL_PHASE_COGGING_CALIB,
                                        motor->cogging_calib_state.pass_num,
                                        elec_angle,
                                        motor->ctrl.ud,
                                        motor->ctrl.uq);
}

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

static void CoggingCalib_FixBoundaryDiscontinuityQ15(int16_t table[], uint16_t n_points)
{
    int32_t diff;

    if (n_points < 3U)
    {
        return;
    }

    /* Stage 1: original simple fix for the immediate boundary pair */
    diff = (int32_t)table[0] - (int32_t)table[n_points - 1U];
    if (diff < 0) diff = -diff;

    if (diff > FOC_COGGING_BOUNDARY_Q15_THRESHOLD)
    {
        int16_t replacement = (int16_t)(((int32_t)table[0] + (int32_t)table[n_points - 1U]) / 2);
        table[0]                  = replacement;
        table[n_points - 1U]      = replacement;
    }

    if (n_points > 4U)
    {
        diff = (int32_t)table[1] - (int32_t)table[n_points - 2U];
        if (diff < 0) diff = -diff;

        if (diff > FOC_COGGING_BOUNDARY_Q15_THRESHOLD)
        {
            int16_t replacement = (int16_t)(((int32_t)table[1] + (int32_t)table[n_points - 2U]) / 2);
            table[1]                  = replacement;
            table[n_points - 2U]      = replacement;
        }
    }

    /* Stage 2: window-based boundary drift correction. */
    {
        uint16_t win = FOC_COGGING_BOUNDARY_BLEND_WIN;
        uint16_t i = 0;
        int32_t sum_head = 0;
        int32_t sum_tail = 0;
        int32_t avg_head;
        int32_t avg_tail;
        int32_t blend_val;

        if (win >= n_points / 4U)
        {
            win = n_points / 4U;
        }
        if (win < 2U)
        {
            return;
        }

        for (i = 0U; i < win; i++)
        {
            sum_head += (int32_t)table[i];
            sum_tail += (int32_t)table[n_points - 1U - i];
        }

        avg_head = sum_head / (int32_t)win;
        avg_tail = sum_tail / (int32_t)win;

        diff = avg_head - avg_tail;
        if (diff < 0) diff = -diff;

        if (diff > (int32_t)(FOC_COGGING_BOUNDARY_Q15_THRESHOLD / 2U))
        {
            blend_val = (avg_head + avg_tail) / 2;

            for (i = 0U; i < win; i++)
            {
                /* Head side: linear from blend_val (i=0) to original (i=win-1) */
                int32_t hv = ((int32_t)table[i] * (int32_t)i
                            + blend_val * (int32_t)(win - 1U - i))
                           / (int32_t)(win - 1U);
                if (hv > 32767) hv = 32767;
                if (hv < -32768) hv = -32768;
                table[i] = (int16_t)hv;

                /* Tail side: linear from original (i=0) to blend_val (i=win-1) */
                {
                    uint16_t j = n_points - win + i;
                    int32_t tv = ((int32_t)table[j] * (int32_t)(win - 1U - i)
                                + blend_val * (int32_t)i)
                               / (int32_t)(win - 1U);
                    if (tv > 32767) tv = 32767;
                    if (tv < -32768) tv = -32768;
                    table[j] = (int16_t)tv;
                }
            }
        }
    }
}

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

static void CoggingCalib_Finish(foc_motor_t *motor)
{
    uint16_t i;
    int32_t  sum;
    int16_t *table;
    char     buf[64];

    table = motor->cogging_comp_table_q15;

    sum = 0;
    for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        sum += (int32_t)table[i];
    }

    (void)snprintf(buf, sizeof(buf),
                   "CALIB DONE: %u bins, sum=%ld\r\n",
                   (unsigned)FOC_COGGING_LUT_POINT_COUNT,
                   (long)sum);
    FOC_Platform_WriteDebugFast(buf);

    /* Zero-mean in Q15 domain */
    {
        int16_t mean_q15 = (int16_t)(sum / (int32_t)FOC_COGGING_LUT_POINT_COUNT);
        for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
        {
            table[i] -= mean_q15;
        }
    }

    CoggingCalib_RemoveOutliersQ15(table, FOC_COGGING_LUT_POINT_COUNT);
    CoggingCalib_FixBoundaryDiscontinuityQ15(table, FOC_COGGING_LUT_POINT_COUNT);

    /* Flip sign for reversed direction */
    if (motor->params.direction == FOC_DIR_REVERSED)
    {
        for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
        {
            table[i] = (int16_t)(-(int32_t)table[i]);
        }
    }

    motor->cogging_comp_status.point_count = FOC_COGGING_LUT_POINT_COUNT;
    motor->cogging_comp_status.iq_lsb_a    = FOC_COGGING_LUT_IQ_LSB_A;
    motor->cogging_comp_status.source      = FOC_COGGING_COMP_SOURCE_CALIB;
    motor->cogging_comp_status.available   = 1U;
    motor->cogging_comp_status.enabled     = 1U;

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->current_soft_switch_status.enabled       = motor->cogging_calib_state.saved_softswitch_enabled;
    motor->current_soft_switch_status.configured_mode = motor->cogging_calib_state.saved_softswitch_mode;
#endif

    motor->cogging_calib_state.in_progress       = 0U;
    motor->cogging_calib_state.progress_percent  = 100U;
    motor->cogging_calib_state.point_index       = CALIB_PHASE_DONE;

    FOC_Platform_WriteDebugFast("COGGING CALIB FINISHED\r\n");

    /* 由主循环输出 dump 表（不阻塞 ISR） */
    motor->cogging_calib_state.request_dump = 1U;
}

static uint8_t CoggingCalib_Start(foc_motor_t *motor)
{
    uint16_t i;

    if (motor->cogging_calib_state.in_progress != 0U)
    {
        return 0U;
    }

    for (i = 0U; i < FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        motor->cogging_comp_table_q15[i] = 0;
    }

    motor->cogging_calib_state.in_progress       = 1U;
    motor->cogging_calib_state.progress_percent  = 0U;
    motor->cogging_calib_state.point_index       = CALIB_PHASE_START;
    motor->cogging_calib_state.completed_pass_count = 0U;

    motor->cogging_calib_state.pred_mech_angle   = 0.0f;
    motor->cogging_calib_state.settle_counter    = 0U;

    motor->cogging_calib_state.last_lut_index         = 0xFFFFU;
    motor->cogging_calib_state.last_reported_progress = 0U;
    motor->cogging_calib_state.bins_collected         = 0U;
    motor->cogging_calib_state.pass_num               = 0U;

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->cogging_calib_state.saved_softswitch_enabled = motor->current_soft_switch_status.enabled;
    motor->cogging_calib_state.saved_softswitch_mode    = motor->current_soft_switch_status.configured_mode;
#endif

    FOC_Platform_WriteDebugFast("COGGING CALIB START\r\n");

    return 1U;
}

/* ========== Main state-machine step ========== */

uint8_t FOC_CoggingCalib_RunStep(foc_motor_t *motor,
                                 const sensor_data_t *sensor,
                                 float dt_sec)
{

    if (sensor == 0)
    {
        return 0U;
    }

    (void)sensor;  /* 直接读取硬件，不依赖缓存数据 */


    /* 检查启动请求 */
    if (motor->cogging_calib_state.request_start != 0U)
    {
        motor->cogging_calib_state.request_start = 0U;
        (void)CoggingCalib_Start(motor);
    }

    if (motor->cogging_calib_state.in_progress == 0U)
    {
        return 0U;
    }

    if (dt_sec <= 0.0f)
    {
        dt_sec = FOC_CONTROL_DT_SEC;
    }

    switch (motor->cogging_calib_state.point_index)
    {
        case CALIB_PHASE_START:
        {
            float mech_actual;

            (void)FOC_Platform_ReadMechanicalAngleRad(&mech_actual);
            motor->cogging_calib_state.pred_mech_angle = mech_actual;

            motor->cogging_calib_state.last_lut_index    = 0xFFFFU;
            motor->cogging_calib_state.bins_collected    = 0U;

            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            motor->cogging_calib_state.point_index = CALIB_PHASE_SETTLE;

            FOC_Platform_WriteDebugFast("entering settle\r\n");

            return 1U;
        }

        case CALIB_PHASE_SETTLE:
        {
            /*
             * 锁定在当前机械角度（pred_mech_angle 不增加），
             * 保持开环电流驱动 settle_cycles 个控制周期，
             * 使电流建立稳定。
             */
            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);
            motor->cogging_calib_state.settle_counter++;

            motor->cogging_calib_state.progress_percent =
                (uint8_t)((uint32_t)motor->cogging_calib_state.settle_counter * 10U /
                          (uint32_t)FOC_COGGING_CALIB_SETTLE_CYCLES);

            if (motor->cogging_calib_state.settle_counter >= FOC_COGGING_CALIB_SETTLE_CYCLES)
            {
                motor->cogging_calib_state.point_index       = CALIB_PHASE_SCAN;
                motor->cogging_calib_state.settle_counter    = 0U;
                motor->cogging_calib_state.last_lut_index    = 0xFFFFU;
                motor->cogging_calib_state.bins_collected    = 0U;
                motor->cogging_calib_state.last_reported_progress = 0U;

                {
                    char buf[48];
                    (void)snprintf(buf, sizeof(buf),
                                  "starting scan\r\n");
                    FOC_Platform_WriteDebugFast(buf);
                }
            }

            return 1U;
        }

        case CALIB_PHASE_SCAN:
        {
            float mech_actual;
            float pred_wrapped;
            float dtheta;
            int16_t *table;

            table = motor->cogging_comp_table_q15;
            (void)FOC_Platform_ReadMechanicalAngleRad(&mech_actual);
            mech_actual = Math_WrapRad(mech_actual);

            /* pred_mech_angle 已包含方向符号（direction × dt），直接归约 */
            pred_wrapped = Math_WrapRad(motor->cogging_calib_state.pred_mech_angle);

            dtheta = Math_WrapRadDelta(mech_actual - pred_wrapped);

            {
                uint16_t lut_index = CoggingCalib_AngleToBin(mech_actual);
                uint8_t accept = 0U;

                if (motor->cogging_calib_state.last_lut_index == 0xFFFFU)
                {
                    accept = 1U;
                }
                else if (motor->params.direction == FOC_DIR_NORMAL)
                {
                    if ((lut_index > motor->cogging_calib_state.last_lut_index) ||
                        (motor->cogging_calib_state.last_lut_index >= (uint16_t)(FOC_COGGING_LUT_POINT_COUNT * 3U / 4U) &&
                         lut_index < (uint16_t)(FOC_COGGING_LUT_POINT_COUNT / 4U)))
                    {
                        accept = 1U;
                    }
                }
                else
                {
                    if ((lut_index < motor->cogging_calib_state.last_lut_index) ||
                        (motor->cogging_calib_state.last_lut_index < (uint16_t)(FOC_COGGING_LUT_POINT_COUNT / 4U) &&
                         lut_index >= (uint16_t)(FOC_COGGING_LUT_POINT_COUNT * 3U / 4U)))
                    {
                        accept = 1U;
                    }
                }

                if (accept != 0U)
                {
                    float iq_comp  = -dtheta * FOC_COGGING_CALIB_DTHETA_SCALE;
                    /* 反向时补偿也需反向 */
                    if (motor->params.direction == FOC_DIR_REVERSED)
                    {
                        iq_comp = -iq_comp;
                    }
                    int16_t q15_val = FloatToQ15(iq_comp, FOC_COGGING_LUT_IQ_LSB_A);
                    uint8_t pcnt    = motor->cogging_calib_state.completed_pass_count;

                    if (pcnt == 0U)
                    {
                        table[lut_index] = q15_val;
                    }
                    else
                    {
                        int32_t accum = (int32_t)table[lut_index] * (int32_t)pcnt
                                      + (int32_t)q15_val;
                        table[lut_index] = (int16_t)(accum / (int32_t)(pcnt + 1U));
                    }

                    motor->cogging_calib_state.last_lut_index = lut_index;
                    motor->cogging_calib_state.bins_collected++;
                }
            }

            motor->cogging_calib_state.pred_mech_angle += FOC_COGGING_CALIB_SPEED_RAD_S *
                                       (float)motor->params.direction *
                                       dt_sec;

            CoggingCalib_OpenLoopDriveStep(motor, dt_sec);

            {
                float pass_progress_frac;
                uint8_t pct;

                pass_progress_frac = (float)motor->cogging_calib_state.bins_collected / (float)FOC_COGGING_LUT_POINT_COUNT;

                if (pass_progress_frac > 1.0f)
                {
                    pass_progress_frac = 1.0f;
                }

                pct = (uint8_t)(((float)motor->cogging_calib_state.pass_num + pass_progress_frac) *
                                100.0f / (float)FOC_COGGING_CALIB_NUM_PASSES);
                if (pct > 100U)
                {
                    pct = 100U;
                }
                motor->cogging_calib_state.progress_percent = pct;
            }

            {
                uint8_t tenth = (uint8_t)(((uint32_t)motor->cogging_calib_state.bins_collected * 10U) /
                                          (uint32_t)FOC_COGGING_LUT_POINT_COUNT);
                if (tenth != motor->cogging_calib_state.last_reported_progress)
                {
                    motor->cogging_calib_state.last_reported_progress = tenth;
                    {
                        char buf[48];
                        (void)snprintf(buf, sizeof(buf),
                                      "CALIB:%u/%u,%u%%\r\n",
                                      (unsigned)(motor->cogging_calib_state.pass_num + 1U),
                                      (unsigned)FOC_COGGING_CALIB_NUM_PASSES,
                                      (unsigned)motor->cogging_calib_state.progress_percent);
                        FOC_Platform_WriteDebugFast(buf);
                    }
                }
            }

            if (motor->cogging_calib_state.bins_collected >= FOC_COGGING_LUT_POINT_COUNT)
            {
                motor->cogging_calib_state.completed_pass_count++;
                motor->cogging_calib_state.pass_num++;
                motor->cogging_calib_state.point_index = CALIB_PHASE_CHECK;
                return 1U;
            }

            return 1U;
        }

        case CALIB_PHASE_CHECK:
        {
            if (motor->cogging_calib_state.pass_num < FOC_COGGING_CALIB_NUM_PASSES)
            {
                motor->cogging_calib_state.point_index           = CALIB_PHASE_SCAN;
                motor->cogging_calib_state.last_lut_index        = 0xFFFFU;
                motor->cogging_calib_state.last_reported_progress = 0U;
                motor->cogging_calib_state.bins_collected        = 0U;
            }
            else
            {
                motor->cogging_calib_state.point_index = CALIB_PHASE_FINISH;
                FOC_Platform_WriteDebugFast("CALIB: all passes done, computing table\r\n");
            }

            return 1U;
        }

        case CALIB_PHASE_FINISH:
        {
            CoggingCalib_Finish(motor);
            /* 完成时自动切回 NORMAL 控制阶段 */
            motor->state.control_phase = FOC_CONTROL_PHASE_NORMAL;
            return 0U;
        }

        default:
            return 0U;
    }
}

/* ========== Dump / Export ========== */

void FOC_CoggingCalibDumpTable(const foc_motor_t *motor)
{
    uint16_t i;
    char line_buf[72];
    uint16_t n;
    const int16_t *table;

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
}

void FOC_CoggingCalibExportTable(const foc_motor_t *motor)
{
    uint16_t i;
    char line_buf[72];
    uint16_t n;
    const int16_t *table;

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

#else /* FOC_COGGING_CALIB_ENABLE not set -- stub */

uint8_t FOC_CoggingCalib_RunStep(foc_motor_t *motor,
                                 const sensor_data_t *sensor,
                                 float dt_sec)
{
    (void)motor;
    (void)sensor;
    (void)dt_sec;
    return 0U;
}

#endif /* FOC_COGGING_CALIB_ENABLE */
