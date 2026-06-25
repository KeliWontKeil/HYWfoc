#include "L2_Core/Runtime/foc_debug_stream.h"

#include <stdio.h>
#include <string.h>

#include "L3_Hal/foc_platform_api.h"

/*
 * 语义遥测最多生成 10 行（current_a/b/c + angle_raw/filtered + vbus_raw/filtered +
 * 各 invalid 行 + exec_time）
 */
#define SEMANTIC_LINE_COUNT 10U

/* 状态标记：已经过语义行数检查 */
#define SEMANTIC_PHASE_COUNTER_CHECK 0xFFU

static void DebugStream_AppendOscParam(char *buffer, uint16_t buffer_len,
                                        uint16_t *offset, float value);

static uint16_t DebugStream_ComputePeriodTicks(uint16_t report_freq_hz);
static uint8_t  DebugStream_IsOscInputValid(const sensor_data_t *sensor,
                                             const foc_motor_t *motor);

/* ========== 初始化 ========== */

void DebugStream_Init(debug_stream_state_t *ds)
{
    if (ds == 0) return;
    ds->semantic_report_counter = 0U;
    ds->osc_report_counter = 0U;
    ds->semantic_last_freq_hz = 0U;
    ds->osc_last_freq_hz = 0U;
    ds->semantic_period_ticks = 1U;
    ds->osc_period_ticks = 1U;
    ds->last_exec_cycles = 0U;
    ds->line_index = 0U;
}

void DebugStream_SetExecutionCycles(debug_stream_state_t *ds, uint32_t exec_cycles)
{
    if (ds == 0) return;
    ds->last_exec_cycles = exec_cycles;
}

/* ========== 工具函数 ========== */

static uint8_t DebugStream_IsOscInputValid(const sensor_data_t *sensor,
                                            const foc_motor_t *motor)
{
    if ((sensor == 0) || (motor == 0)) return 0U;
    if ((sensor->adc_valid == 0U) || (sensor->encoder_valid == 0U)) return 0U;
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

static void DebugStream_AppendOscParam(char *buffer, uint16_t buffer_len,
                                        uint16_t *offset, float value)
{
    int written;
    if ((buffer == 0) || (offset == 0) || (*offset >= buffer_len)) return;
    if ((buffer_len - *offset) < 12U) return;

    written = snprintf(buffer + *offset, buffer_len - *offset, " %.3f", value);
    if (written < 0) return;
    if ((uint16_t)written >= (buffer_len - *offset))
    {
        *offset = (uint16_t)(buffer_len - 1U);
        return;
    }
    *offset += (uint16_t)written;
}

/* ========== 生成器：单行输出 ========== */

uint8_t DebugStream_GenerateLine(debug_stream_state_t *ds,
                                  const sensor_data_t *sensor,
                                  const foc_motor_t *motor,
                                  const telemetry_policy_snapshot_t *telemetry,
                                  char *line_out,
                                  uint16_t line_max)
{
    uint16_t osc_freq_hz;
    int written;

    if ((ds == 0) || (line_out == 0) || (line_max == 0U)) return 0U;
    if ((motor != 0) && (motor->state.system_fault != 0U)) return 0U;

    /* ---- 语义遥测（多行，通过 line_index 逐行生成） ---- */
#if (DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE)
    uint16_t semantic_freq_hz;

    if ((telemetry != 0) && (telemetry->semantic_report_enabled != 0U))
    {
        semantic_freq_hz = telemetry->semantic_report_freq_hz;

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

            /* 达到输出时刻，重置 counter 并开始输出 */
            ds->semantic_report_counter = SEMANTIC_PHASE_COUNTER_CHECK;
            ds->line_index = 0U;
        }

        if (ds->line_index < SEMANTIC_LINE_COUNT)
        {
            uint8_t idx = ds->line_index;
            ds->line_index++;

            /* 根据索引输出不同行 */
            if (sensor == 0)
            {
                if (idx == 0U)
                {
                    written = snprintf(line_out, line_max,
                                       "feedback.status=invalid\r\n");
                    if (written < 0) return 0U;
                    return 1U;
                }
                return 0U;
            }

            switch (idx)
            {
            case 0U:
                if (sensor->adc_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.phase_current_a_ampere=%.3f\r\n",
                        sensor->current_a.output_value);
                else
                    written = snprintf(line_out, line_max,
                        "measurement.current.status=invalid\r\n");
                break;
            case 1U:
                if (sensor->adc_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.phase_current_b_ampere=%.3f\r\n",
                        sensor->current_b.output_value);
                else
                    return 0U; /* already output invalid in case 0 */
                break;
            case 2U:
                if (sensor->adc_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.phase_current_c_ampere=%.3f\r\n",
                        sensor->current_c.output_value);
                else
                    return 0U;
                break;
            case 3U:
                if (sensor->encoder_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.encoder_angle_raw_rad=%.3f\r\n",
                        sensor->mech_angle_rad.raw_value);
                else
                    written = snprintf(line_out, line_max,
                        "measurement.encoder.status=invalid\r\n");
                break;
            case 4U:
                if (sensor->encoder_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.encoder_angle_filtered_rad=%.3f\r\n",
                        sensor->mech_angle_rad.output_value);
                else
                    return 0U;
                break;
            case 5U:
                if (sensor->vbus_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.vbus_voltage_raw_v=%.3f\r\n",
                        sensor->vbus_voltage_raw);
                else
                    written = snprintf(line_out, line_max,
                        "measurement.vbus.status=invalid\r\n");
                break;
            case 6U:
                if (sensor->vbus_valid != 0U)
                    written = snprintf(line_out, line_max,
                        "measurement.vbus_voltage_filtered_v=%.3f\r\n",
                        sensor->vbus_voltage_filtered);
                else
                    return 0U;
                break;
            case 7U:
            {
                uint32_t cycles = ds->last_exec_cycles;
                written = snprintf(line_out, line_max,
                    "control.execution_time_us=%.3f\r\n\r\n",
                    (float)cycles / 120.0f);
                break;
            }
            default:
                return 0U;
            }

            if (written < 0) return 0U;
            return 1U;
        }

        /* 语义行输出完毕，标记为下一次检查 */
        if (ds->line_index >= SEMANTIC_LINE_COUNT)
        {
            ds->semantic_report_counter = 0U;
            ds->line_index = 0U;
        }
    }
#endif /* DEBUG_STREAM_ENABLE_SEMANTIC_REPORT */

    /* ---- 示波器输出（单帧单行） ---- */
#if (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE)
    if ((telemetry != 0) && (telemetry->osc_report_enabled != 0U) &&
        (DebugStream_IsOscInputValid(sensor, motor) != 0U))
    {
        /* 只有 line_index == 0 时检查周期 */
        if (ds->line_index == 0U)
        {
            osc_freq_hz = telemetry->osc_report_freq_hz;
            if (osc_freq_hz != ds->osc_last_freq_hz)
            {
                ds->osc_last_freq_hz = osc_freq_hz;
                ds->osc_period_ticks = DebugStream_ComputePeriodTicks(osc_freq_hz);
            }

            ds->osc_report_counter++;
            if (ds->osc_report_counter < ds->osc_period_ticks)
                return 0U;

            ds->osc_report_counter = 0U;
        }

        /* 生成示波器帧（只在 line_index == 0 时生成） */
        if (ds->line_index == 0U)
        {
            char payload[DEBUG_STREAM_OSC_PAYLOAD_LEN];
            uint16_t offset = 0U;
            uint16_t mask = telemetry->osc_parameter_mask;

            written = snprintf(payload + offset, sizeof(payload) - offset, "%c",
                               (char)DEBUG_STREAM_OSC_HEAD_BYTE);
            if ((written < 0) || ((uint16_t)written >= (sizeof(payload) - offset)))
                return 0U;
            offset += (uint16_t)written;

            if ((mask & DEBUG_STREAM_OSC_PARAM_CURRENT_A) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           sensor->current_a.output_value);
            if ((mask & DEBUG_STREAM_OSC_PARAM_CURRENT_B) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           sensor->current_b.output_value);
            if ((mask & DEBUG_STREAM_OSC_PARAM_CURRENT_C) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           sensor->current_c.output_value);

            if ((mask & DEBUG_STREAM_OSC_PARAM_ANGLE_FILTERED) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           sensor->mech_angle_rad.output_value);

            if ((mask & DEBUG_STREAM_OSC_PARAM_ANGLE_ACCUM) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           motor->mech_angle_accum_rad);

            if ((mask & DEBUG_STREAM_OSC_PARAM_EXEC_TIME_US) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           (float)ds->last_exec_cycles / 120.0f);

            if ((mask & DEBUG_STREAM_OSC_PARAM_VBUS_VOLTAGE) != 0U)
            {
                float vbus = (sensor->vbus_valid != 0U) ?
                              sensor->vbus_voltage_filtered : 0.0f;
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset, vbus);
            }

            if ((mask & DEBUG_STREAM_OSC_PARAM_IQ_TARGET) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           motor->iq_target);

            if ((mask & DEBUG_STREAM_OSC_PARAM_IQ_MEASURED) != 0U)
                DebugStream_AppendOscParam(payload, sizeof(payload), &offset,
                                           motor->iq_measured);

            if ((offset + 6U) < sizeof(payload))
            {
                written = snprintf(payload + offset, sizeof(payload) - offset,
                                   " %c ", (char)DEBUG_STREAM_OSC_TAIL_BYTE);
                if ((written < 0) || ((uint16_t)written >= (sizeof(payload) - offset)))
                    return 0U;
                offset += (uint16_t)written;
            }

            (void)memcpy(line_out, payload, (offset < line_max) ? offset : line_max);
            if (offset < line_max)
                line_out[offset] = '\0';
            else
                line_out[line_max - 1U] = '\0';

            ds->line_index = 1U;
            return 1U;
        }

        /* 第二次调用返回 0（单行输出结束） */
        if (ds->line_index == 1U)
        {
            ds->line_index = 0U;
            return 0U;
        }
    }
#endif /* DEBUG_STREAM_ENABLE_OSC_REPORT */

    return 0U;
}