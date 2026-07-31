#include "L2_Core/Runtime/foc_debug_stream.h"

#include <stdio.h>
#include <string.h>

#include "L3_Hal/foc_platform_api.h"

/*
 * 语义遥测最多生成 9 行（current_a/b/c + angle_raw/filtered + vbus_raw/filtered +
 * 各 invalid 行 + exec_time + current_loop_exec_time）
 */
#define SEMANTIC_LINE_COUNT 9U

/* 状态标记：已经过语义行数检查 */
#define SEMANTIC_PHASE_COUNTER_CHECK 0xFFU

static uint16_t DebugStream_ComputePeriodTicks(uint16_t report_freq_hz);
static uint8_t  DebugStream_IsOscInputValid(const foc_motor_t *motor);
static uint8_t  DebugStream_PollSemantic(debug_stream_state_t *ds,
                                          const foc_motor_t *motor,
                                          const foc_report_config_t *report,
                                          monitor_element_t *elem_out);

/* ========== 初始化 ========== */

void DebugStream_Init(debug_stream_state_t *ds)
{
    uint8_t i;

    if (ds == 0) return;
    ds->semantic_report_counter = 0U;
    ds->osc_report_counter = 0U;
    ds->semantic_last_freq_hz = 0U;
    ds->osc_last_freq_hz = 0U;
    ds->semantic_period_ticks = 1U;
    ds->osc_period_ticks = 1U;
    ds->last_exec_cycles = 0U;
    ds->line_index = 0U;
    ds->osc_param_bit_idx = 0U;
    for (i = 0U; i < OSC_SNAPSHOT_CHANNEL_COUNT; i++)
        ds->snapshot.channel[i] = 0.0f;
    ds->snapshot.valid = 0U;
}

void DebugStream_CaptureOscSnapshot(debug_stream_state_t *ds, const foc_motor_t *motor)
{
    if ((ds == 0) || (motor == 0)) return;

    ds->snapshot.channel[0] = motor->sensor.current_a.output_value;
    ds->snapshot.channel[1] = motor->sensor.current_b.output_value;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    ds->snapshot.channel[2] = motor->sensor.current_c.output_value;
#else
    ds->snapshot.channel[2] = -(motor->sensor.current_a.output_value + motor->sensor.current_b.output_value);
#endif
    ds->snapshot.channel[3] = motor->active_source_state.mech_angle_rad;

    /* bit 4: standby angle */
    {
        uint8_t stby = motor->source_mgr_state.standby_source;
        float angle = 0.0f;
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
        if (stby == FOC_SOURCE_TYPE_ENCODER && motor->sensor.encoder_valid != 0U)
            angle = motor->sensor.mech_angle_rad.output_value;
        else
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
        if (stby == FOC_SOURCE_TYPE_SMO && motor->estim_smo_state.lock_counter < FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE && motor->params.pole_pairs > 0U)
            angle = motor->estim_smo_state.pll_angle_rad / (float)motor->params.pole_pairs;
        else
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
        if (stby == FOC_SOURCE_TYPE_OPENLOOP && motor->openloop_state.phase != FOC_OPENLOOP_STATE_FAILED && motor->params.pole_pairs > 0U)
            angle = motor->openloop_state.virtual_angle_rad / (float)motor->params.pole_pairs;
#endif
        ds->snapshot.channel[4] = angle;
    }

    ds->snapshot.channel[5] = (float)ds->last_exec_cycles / 120.0f;
    ds->snapshot.channel[6] = (motor->sensor.vbus_valid != 0U) ? motor->sensor.vbus.filtered : 0.0f;
    ds->snapshot.channel[7] = motor->ctrl.iq_target;
    ds->snapshot.channel[8] = motor->ctrl.iq_measured;

    /* bit 9, 10 unused */
    ds->snapshot.channel[9] = 0.0f;
    ds->snapshot.channel[10] = 0.0f;

    /* bit 11: active speed */
    {
        uint8_t active = motor->source_mgr_state.active_source;
        float speed = 0.0f;
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
        if (active == FOC_SOURCE_TYPE_ENCODER && motor->sensor.encoder_valid != 0U)
            speed = motor->sensor.mech_speed_rad_s;
        else
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
        if (active == FOC_SOURCE_TYPE_SMO && motor->estim_smo_state.lock_counter < FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
            speed = motor->estim_smo_state.mech_speed_rad_s;
        else
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
        if (active == FOC_SOURCE_TYPE_OPENLOOP && motor->openloop_state.phase != FOC_OPENLOOP_STATE_FAILED)
            speed = motor->openloop_state.mech_speed_rad_s;
#endif
        ds->snapshot.channel[11] = speed;
    }

    /* bit 12: standby speed */
    {
        uint8_t stby = motor->source_mgr_state.standby_source;
        float speed = 0.0f;
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
        if (stby == FOC_SOURCE_TYPE_ENCODER && motor->sensor.encoder_valid != 0U)
            speed = motor->sensor.mech_speed_rad_s;
        else
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
        if (stby == FOC_SOURCE_TYPE_SMO && motor->estim_smo_state.lock_counter < FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE)
            speed = motor->estim_smo_state.mech_speed_rad_s;
        else
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
        if (stby == FOC_SOURCE_TYPE_OPENLOOP && motor->openloop_state.phase != FOC_OPENLOOP_STATE_FAILED)
            speed = motor->openloop_state.mech_speed_rad_s;
#endif
        ds->snapshot.channel[12] = speed;
    }

    ds->snapshot.channel[13] = 0.0f;
    ds->snapshot.channel[14] = 0.0f;
    ds->snapshot.channel[15] = 0.0f;
    ds->snapshot.valid = 1U;
}

void DebugStream_SetExecutionCycles(debug_stream_state_t *ds, uint32_t exec_cycles)
{
    if (ds == 0) return;
    ds->last_exec_cycles = exec_cycles;
}

/* ========== 工具函数 ========== */

static uint8_t DebugStream_IsOscInputValid(const foc_motor_t *motor)
{
    if (motor == 0) return 0U;
    if (motor->sensor.adc_valid == 0U) return 0U;
    return 1U;
}

static uint16_t DebugStream_ComputePeriodTicks(uint16_t report_freq_hz)
{
    uint16_t effective_freq_hz = report_freq_hz;
    uint16_t period_ticks;

    if (effective_freq_hz == 0U)
        effective_freq_hz = 1U;
    if (effective_freq_hz > FOC_SCHEDULER_MONITOR_HZ)
        effective_freq_hz = FOC_SCHEDULER_MONITOR_HZ;

    period_ticks = (uint16_t)((FOC_SCHEDULER_MONITOR_HZ + (effective_freq_hz / 2U))
                              / effective_freq_hz);
    if (period_ticks == 0U) period_ticks = 1U;
    return period_ticks;
}

/* ========== 语义 + 示波器 poll 子函数 ========== */

static uint8_t DebugStream_PollSemantic(debug_stream_state_t *ds,
                                         const foc_motor_t *motor,
                                         const foc_report_config_t *report,
                                         monitor_element_t *elem_out)
{
#if (DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE)
    uint16_t semantic_freq_hz;

    if ((report == 0) || (report->semantic_enabled == 0U))
        return 0U;

    semantic_freq_hz = report->semantic_freq_hz;

    if (semantic_freq_hz != ds->semantic_last_freq_hz)
    {
        ds->semantic_last_freq_hz = semantic_freq_hz;
        ds->semantic_period_ticks = DebugStream_ComputePeriodTicks(semantic_freq_hz);
    }

    if (ds->semantic_report_counter < SEMANTIC_PHASE_COUNTER_CHECK)
    {
        ds->semantic_report_counter++;
        if (ds->semantic_report_counter < ds->semantic_period_ticks)
            return 0U;

        ds->semantic_report_counter = SEMANTIC_PHASE_COUNTER_CHECK;
        ds->line_index = 0U;
    }

    if (ds->line_index < SEMANTIC_LINE_COUNT)
    {
        uint8_t idx = ds->line_index;
        ds->line_index++;

        elem_out->tag = (uint8_t)(MONITOR_ELEM_SEMANTIC_0 + idx);
        elem_out->aux = 1U;

        switch (idx)
        {
        case 0U:
            if ((motor != 0) && (motor->sensor.adc_valid != 0U))
                elem_out->value = motor->sensor.current_a.output_value;
            else
                elem_out->aux = 0U;
            break;
        case 1U:
            if ((motor != 0) && (motor->sensor.adc_valid != 0U))
                elem_out->value = motor->sensor.current_b.output_value;
            else
                return DebugStream_PollSemantic(ds, motor, report, elem_out);
            break;
        case 2U:
#if (FOC_CURRENT_SENSE_PHASES == 3U)
            if ((motor != 0) && (motor->sensor.adc_valid != 0U))
                elem_out->value = motor->sensor.current_c.output_value;
            else
                return DebugStream_PollSemantic(ds, motor, report, elem_out);
#else
            if ((motor != 0) && (motor->sensor.adc_valid != 0U))
                elem_out->value = -(motor->sensor.current_a.output_value + motor->sensor.current_b.output_value);
            else
                return DebugStream_PollSemantic(ds, motor, report, elem_out);
#endif
            break;
        case 3U:
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
            if ((motor != 0) && (motor->sensor.encoder_valid != 0U))
                elem_out->value = motor->sensor.mech_angle_rad.output_value;
            else
                elem_out->aux = 0U;
            break;
#else
            return DebugStream_PollSemantic(ds, motor, report, elem_out);
#endif
        case 4U:
            if ((motor != 0) && (motor->active_source_state.valid != 0U))
                elem_out->value = motor->active_source_state.mech_angle_rad;
            else
                return DebugStream_PollSemantic(ds, motor, report, elem_out);
            break;
        case 5U:
            if ((motor != 0) && (motor->sensor.vbus_valid != 0U))
                elem_out->value = motor->sensor.vbus.raw;
            else
                elem_out->aux = 0U;
            break;
        case 6U:
            if ((motor != 0) && (motor->sensor.vbus_valid != 0U))
                elem_out->value = motor->sensor.vbus.filtered;
            else
                return DebugStream_PollSemantic(ds, motor, report, elem_out);
            break;
        case 7U:
            elem_out->value = (float)ds->last_exec_cycles / ( FOC_PLATFORM_BASE_CLOCK_KHZ / 1000.0 );
            elem_out->aux = 2U;
            break;
        case 8U:
            if (motor != 0)
                elem_out->value = (float)motor->isr_timing.current_loop_cycles / ( FOC_PLATFORM_BASE_CLOCK_KHZ / 1000.0 );
            else
                elem_out->value = 0.0f;
            elem_out->aux = 2U;
            break;
        default:
            break;
        }

        if (elem_out->aux == 0U)
        {
            elem_out->tag = MONITOR_ELEM_SEMANTIC_END;
        }
        return 1U;
    }

    if (ds->line_index >= SEMANTIC_LINE_COUNT)
    {
        elem_out->tag = MONITOR_ELEM_SEMANTIC_END;
        ds->semantic_report_counter = 0U;
        ds->line_index = 0U;
        return 1U;
    }
#else
    (void)motor;
#endif
    return 0U;
}

/* ========== ISR 级接口：逐元素输出 ========== */

uint8_t DebugStream_PollNextValue(debug_stream_state_t *ds,
                                   const foc_motor_t *motor,
                                   const foc_report_config_t *report,
                                   monitor_element_t *elem_out)
{
    if ((ds == 0) || (elem_out == 0)) return 0U;
    if ((motor != 0) && (motor->state.system_fault != 0U)) return 0U;

    /* 语义遥测 */
    if (DebugStream_PollSemantic(ds, motor, report, elem_out) != 0U)
    {
        return 1U;
    }

    /* ---- 示波器值 ---- */
#if (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE)
    if ((report != 0) && (report->osc_enabled != 0U) &&
        (DebugStream_IsOscInputValid(motor) != 0U))
    {
        uint16_t osc_freq_hz;

        if (ds->line_index == 0U)
        {
            osc_freq_hz = report->osc_freq_hz;
            if (osc_freq_hz != ds->osc_last_freq_hz)
            {
                ds->osc_last_freq_hz = osc_freq_hz;
                ds->osc_period_ticks = DebugStream_ComputePeriodTicks(osc_freq_hz);
            }

            ds->osc_report_counter++;
            if (ds->osc_report_counter < ds->osc_period_ticks)
                return 0U;

            ds->osc_report_counter = 0U;
            ds->osc_param_bit_idx = 0U;
            ds->line_index = 1U;
        }

        if (ds->osc_param_bit_idx < 16U)
        {
            uint16_t mask = report->osc_param_mask;

            while (ds->osc_param_bit_idx < 16U)
            {
                if ((mask & (1U << ds->osc_param_bit_idx)) != 0U)
                {
                    uint8_t bit = ds->osc_param_bit_idx;
                    ds->osc_param_bit_idx++;

                    elem_out->tag = MONITOR_ELEM_OSC_VALUE;
                    elem_out->aux  = bit;

                    if (ds->snapshot.valid != 0U && bit < OSC_SNAPSHOT_CHANNEL_COUNT)
                        elem_out->value = ds->snapshot.channel[bit];
                    else
                        elem_out->value = 0.0f;

                    return 1U;
                }
                ds->osc_param_bit_idx++;
            }
        }

        elem_out->tag   = MONITOR_ELEM_OSC_END;
        elem_out->value = 0.0f;
        elem_out->aux   = 0U;
        ds->osc_param_bit_idx = 0U;
        ds->line_index = 0U;
        return 1U;
    }
#endif

    return 0U;
}

/* ========== 主循环级接口：格式化 ========== */

void DebugStream_FormatSemanticLine(uint8_t tag, float value,
                                     char *line_out, uint16_t line_max)
{
    uint8_t idx;

    if ((line_out == 0) || (line_max == 0U)) return;

    idx = tag & 0x0FU;

    switch (idx)
    {
    case 0U:
        snprintf(line_out, line_max,
            "measurement.phase_current_a_ampere=%.3f\r\n", value);
        break;
    case 1U:
        snprintf(line_out, line_max,
            "measurement.phase_current_b_ampere=%.3f\r\n", value);
        break;
    case 2U:
        snprintf(line_out, line_max,
            "measurement.phase_current_c_ampere=%.3f\r\n", value);
        break;
    case 3U:
        snprintf(line_out, line_max,
            "measurement.mech_angle_raw_rad=%.3f\r\n", value);
        break;
    case 4U:
        snprintf(line_out, line_max,
            "measurement.mech_angle_filtered_rad=%.3f\r\n", value);
        break;
    case 5U:
        snprintf(line_out, line_max,
            "measurement.vbus_voltage_raw_v=%.3f\r\n", value);
        break;
    case 6U:
        snprintf(line_out, line_max,
            "measurement.vbus_voltage_filtered_v=%.3f\r\n", value);
        break;
    case 7U:
        snprintf(line_out, line_max,
            "control.execution_time_us=%.3f\r\n", value);
        break;
    case 8U:
        snprintf(line_out, line_max,
            "control.current_loop_execution_time_us=%.3f\r\n\r\n", value);
        break;
    default:
        snprintf(line_out, line_max, "measurement.status=invalid\r\n");
        break;
    }
}

uint8_t DebugStream_FormatInvalidLine(uint8_t tag, char *line_out, uint16_t line_max)
{
    uint8_t idx;

    if ((line_out == 0) || (line_max == 0U)) return 0U;

    idx = tag & 0x0FU;
    switch (idx)
    {
    case 0U:
        snprintf(line_out, line_max, "measurement.current.status=invalid\r\n");
        break;
    case 3U:
        snprintf(line_out, line_max, "measurement.mech_angle.status=invalid\r\n");
        break;
    case 5U:
        snprintf(line_out, line_max, "measurement.vbus.status=invalid\r\n");
        break;
    default:
        return 0U;
    }
    return 1U;
}

void DebugStream_AppendOscValue(char *osc_buffer, uint16_t *offset,
                                 float value)
{
    int written;

    if ((osc_buffer == 0) || (offset == 0) || (*offset >=
        (uint16_t)(DEBUG_STREAM_OSC_PAYLOAD_LEN - 12U))) return;

    written = snprintf(osc_buffer + *offset,
                       (size_t)(DEBUG_STREAM_OSC_PAYLOAD_LEN - *offset),
                       " %.3f", value);
    if (written < 0) return;
    if ((uint16_t)written >= (DEBUG_STREAM_OSC_PAYLOAD_LEN - *offset))
    {
        *offset = (uint16_t)(DEBUG_STREAM_OSC_PAYLOAD_LEN - 1U);
        return;
    }
    *offset += (uint16_t)written;
}

uint16_t DebugStream_FormatOscLine(char *osc_buffer, uint16_t max_len)
{
    int written;
    uint16_t head_len;
    char temp[DEBUG_STREAM_OSC_PAYLOAD_LEN];

    if (osc_buffer == 0) return 0U;

    (void)memcpy(temp, osc_buffer, (max_len < sizeof(temp)) ? max_len : sizeof(temp));

    written = snprintf(osc_buffer, max_len, "%c",
                       (char)DEBUG_STREAM_OSC_HEAD_BYTE);
    if ((written < 0) || ((uint16_t)written >= max_len)) return 0U;
    head_len = (uint16_t)written;

    {
        uint16_t copy_len = (max_len - head_len > sizeof(temp)) ?
                             (uint16_t)sizeof(temp) : (uint16_t)(max_len - head_len);
        if (copy_len > 1U)
        {
            (void)memcpy(osc_buffer + head_len, temp, copy_len - 1U);
            osc_buffer[head_len + copy_len - 1U] = '\0';
        }
    }

    {
        uint16_t cur = (uint16_t)strlen(osc_buffer);
        if ((cur + 6U) < max_len)
        {
            written = snprintf(osc_buffer + cur, max_len - cur,
                               " %c ", (char)DEBUG_STREAM_OSC_TAIL_BYTE);
            if (written < 0) return 0U;
            cur += (uint16_t)written;
        }
        return cur;
    }
}

/* ========== 向后兼容：原生成器 ========== */

uint8_t DebugStream_GenerateLine(debug_stream_state_t *ds,
                                  const foc_motor_t *motor,
                                  const foc_report_config_t *report,
                                  char *line_out,
                                  uint16_t line_max)
{
    monitor_element_t elem;
    static char compat_osc_buf[DEBUG_STREAM_OSC_PAYLOAD_LEN];
    static uint16_t compat_osc_off;
    static uint8_t  compat_osc_active;

    if ((ds == 0) || (line_out == 0) || (line_max == 0U)) return 0U;

    if (DebugStream_PollNextValue(ds, motor, report, &elem) == 0U) return 0U;

    if (elem.tag == MONITOR_ELEM_FRAME_START)
    {
        return DebugStream_GenerateLine(ds, motor, report,
                                         line_out, line_max);
    }

    if (elem.tag <= MONITOR_ELEM_SEMANTIC_8)
    {
        if (elem.aux == 0U)
        {
            uint8_t idx = (uint8_t)(elem.tag - MONITOR_ELEM_SEMANTIC_0);
            switch (idx)
            {
            case 0U:
                snprintf(line_out, line_max,
                    "measurement.current.status=invalid\r\n");
                break;
            case 3U:
                snprintf(line_out, line_max,
                    "measurement.mech_angle.status=invalid\r\n");
                break;
            case 5U:
                snprintf(line_out, line_max,
                    "measurement.vbus.status=invalid\r\n");
                break;
            default:
                return DebugStream_GenerateLine(ds, motor, report,
                                                 line_out, line_max);
            }
            return 1U;
        }
        DebugStream_FormatSemanticLine(elem.tag, elem.value,
                                        line_out, line_max);
        return 1U;
    }

    if (elem.tag == MONITOR_ELEM_SEMANTIC_END)
    {
        return DebugStream_GenerateLine(ds, motor, report,
                                         line_out, line_max);
    }

    if (elem.tag == MONITOR_ELEM_OSC_VALUE)
    {
        if (compat_osc_active == 0U)
        {
            compat_osc_off = (uint16_t)snprintf(compat_osc_buf,
                               sizeof(compat_osc_buf), "%c",
                               (char)DEBUG_STREAM_OSC_HEAD_BYTE);
            compat_osc_active = 1U;
        }

        DebugStream_AppendOscValue(compat_osc_buf, &compat_osc_off, elem.value);

        return DebugStream_GenerateLine(ds, motor, report,
                                         line_out, line_max);
    }

    if (elem.tag == MONITOR_ELEM_OSC_END)
    {
        uint16_t cur = (uint16_t)strlen(compat_osc_buf);
        int osc_written = snprintf(compat_osc_buf + cur,
                                   sizeof(compat_osc_buf) - cur,
                                   " %c ", (char)DEBUG_STREAM_OSC_TAIL_BYTE);
        if (osc_written < 0) return 0U;
        cur += (uint16_t)osc_written;
        (void)memcpy(line_out, compat_osc_buf,
                     (cur < line_max) ? cur : line_max);
        if (cur < line_max)
            line_out[cur] = '\0';
        else
            line_out[line_max - 1U] = '\0';
        compat_osc_active = 0U;
        return 1U;
    }

    return 0U;
}
