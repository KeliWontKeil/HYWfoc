#include "debug_stream.h"

#include <stdio.h>

#include "command_manager.h"
#include "control_scheduler.h"
#include "foc_platform_api.h"

static void DebugStream_OutputSemanticTelemetry(const sensor_data_t *sensor);
static void DebugStream_OutputOscilloscopeFrame(const sensor_data_t *sensor, const foc_motor_t *motor);
static void DebugStream_AppendOscParam(char *buffer, uint16_t buffer_len, uint16_t *offset, float value);
static uint16_t DebugStream_ComputePeriodTicks(uint16_t report_freq_hz);
static uint8_t DebugStream_IsOscInputValid(const sensor_data_t *sensor, const foc_motor_t *motor);

static uint16_t g_semantic_report_counter = 0U;
static uint16_t g_osc_report_counter = 0U;
static uint16_t g_semantic_last_freq_hz = 0U;
static uint16_t g_osc_last_freq_hz = 0U;
static uint16_t g_semantic_period_ticks = 1U;
static uint16_t g_osc_period_ticks = 1U;

void DebugStream_Init(void)
{
    g_semantic_report_counter = 0U;
    g_osc_report_counter = 0U;
    g_semantic_last_freq_hz = 0U;
    g_osc_last_freq_hz = 0U;
    g_semantic_period_ticks = 1U;
    g_osc_period_ticks = 1U;
}

void DebugStream_Process(const sensor_data_t *sensor, const foc_motor_t *motor)
{
#if (DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE)
    if (CommandManager_IsSemanticReportEnabled() != 0U)
    {
        uint16_t semantic_freq_hz = CommandManager_GetSemanticReportFrequencyHz();

        if (semantic_freq_hz != g_semantic_last_freq_hz)
        {
            g_semantic_last_freq_hz = semantic_freq_hz;
            g_semantic_period_ticks = DebugStream_ComputePeriodTicks(semantic_freq_hz);
        }

        g_semantic_report_counter++;
        if (g_semantic_report_counter >= g_semantic_period_ticks)
        {
            g_semantic_report_counter = 0U;
            DebugStream_OutputSemanticTelemetry(sensor);
        }
    }
#endif

#if (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE)
    if (CommandManager_IsOscilloscopeReportEnabled() != 0U)
    {
        uint16_t osc_freq_hz;

        if (DebugStream_IsOscInputValid(sensor, motor) != 0U)
        {
            osc_freq_hz = CommandManager_GetOscilloscopeReportFrequencyHz();
            if (osc_freq_hz != g_osc_last_freq_hz)
            {
                g_osc_last_freq_hz = osc_freq_hz;
                g_osc_period_ticks = DebugStream_ComputePeriodTicks(osc_freq_hz);
            }

            g_osc_report_counter++;
            if (g_osc_report_counter >= g_osc_period_ticks)
            {
                g_osc_report_counter = 0U;
                DebugStream_OutputOscilloscopeFrame(sensor, motor);
            }
        }
    }
#endif
}

static uint8_t DebugStream_IsOscInputValid(const sensor_data_t *sensor, const foc_motor_t *motor)
{
    if ((sensor == 0) || (motor == 0))
    {
        return 0U;
    }

    if ((sensor->adc_valid == 0U) || (sensor->encoder_valid == 0U))
    {
        return 0U;
    }

    return 1U;
}

static uint16_t DebugStream_ComputePeriodTicks(uint16_t report_freq_hz)
{
    uint16_t effective_freq_hz = report_freq_hz;
    uint16_t period_ticks;

    if (effective_freq_hz == 0U)
    {
        effective_freq_hz = 1U;
    }
    if (effective_freq_hz > FOC_SCHEDULER_MONITOR_HZ)
    {
        effective_freq_hz = FOC_SCHEDULER_MONITOR_HZ;
    }

    period_ticks = (uint16_t)((FOC_SCHEDULER_MONITOR_HZ + (effective_freq_hz / 2U)) / effective_freq_hz);
    if (period_ticks == 0U)
    {
        period_ticks = 1U;
    }

    return period_ticks;
}

static void DebugStream_OutputOscilloscopeFrame(const sensor_data_t *sensor, const foc_motor_t *motor)
{
    char payload[DEBUG_STREAM_OSC_PAYLOAD_LEN];
    uint16_t offset = 0U;
    uint16_t mask = CommandManager_GetOscilloscopeParameterMask();
    uint32_t exec_cycles = ControlScheduler_GetExecutionCycles();
    int written;

    if ((sensor == 0) || (motor == 0))
    {
        return;
    }

    written = snprintf(payload + offset,
                       sizeof(payload) - offset,
                       "%c",
                       (char)DEBUG_STREAM_OSC_HEAD_BYTE);
    if ((written < 0) || ((uint16_t)written >= (sizeof(payload) - offset)))
    {
        return;
    }
    offset += (uint16_t)written;

    if ((mask & DEBUG_STREAM_OSC_PARAM_CURRENT_A) != 0U)
    {
        DebugStream_AppendOscParam(payload,
                                   sizeof(payload),
                                   &offset,
                                   sensor->current_a.output_value);
    }

    if ((mask & DEBUG_STREAM_OSC_PARAM_CURRENT_B) != 0U)
    {
        DebugStream_AppendOscParam(payload,
                                   sizeof(payload),
                                   &offset,
                                   sensor->current_b.output_value);
    }

    if ((mask & DEBUG_STREAM_OSC_PARAM_CURRENT_C) != 0U)
    {
        DebugStream_AppendOscParam(payload,
                                   sizeof(payload),
                                   &offset,
                                   sensor->current_c.output_value);
    }

    if ((mask & DEBUG_STREAM_OSC_PARAM_ANGLE_FILTERED) != 0U)
    {
        DebugStream_AppendOscParam(payload,
                                   sizeof(payload),
                                   &offset,
                                   sensor->mech_angle_rad.output_value);
    }

    if ((mask & DEBUG_STREAM_OSC_PARAM_ANGLE_ACCUM) != 0U)
    {
        DebugStream_AppendOscParam(payload,
                                   sizeof(payload),
                                   &offset,
                                   motor->mech_angle_accum_rad);
    }

    if ((mask & DEBUG_STREAM_OSC_PARAM_EXEC_TIME_US) != 0U)
    {
        DebugStream_AppendOscParam(payload,
                                   sizeof(payload),
                                   &offset,
                                   (float)exec_cycles / 120.0f);
    }

    if ((offset + 6U) < sizeof(payload))
    {
        written = snprintf(payload + offset,
                           sizeof(payload) - offset,
                           " %c ",
                           (char)DEBUG_STREAM_OSC_TAIL_BYTE);
        if ((written < 0) || ((uint16_t)written >= (sizeof(payload) - offset)))
        {
            return;
        }
        FOC_Platform_DebugOutput(payload);
    }
}

static void DebugStream_AppendOscParam(char *buffer, uint16_t buffer_len, uint16_t *offset, float value)
{
    int written;

    if ((buffer == 0) || (offset == 0) || (*offset >= buffer_len))
    {
        return;
    }

    if ((buffer_len - *offset) < 12U)
    {
        return;
    }

    written = snprintf(buffer + *offset, buffer_len - *offset, " %.3f", value);
    if (written < 0)
    {
        return;
    }

    if ((uint16_t)written >= (buffer_len - *offset))
    {
        *offset = (uint16_t)(buffer_len - 1U);
        return;
    }

    *offset += (uint16_t)written;
}

static void DebugStream_OutputSemanticTelemetry(const sensor_data_t *sensor)
{
    char buffer[96];
    uint32_t exec_time;

    if (sensor == 0)
    {
        FOC_Platform_DebugOutput("feedback.status=invalid\r\n");
        return;
    }

    if (sensor->adc_valid != 0U)
    {
        snprintf(buffer, sizeof(buffer), "measurement.phase_current_a_ampere=%.3f\r\n", sensor->current_a.output_value);
        FOC_Platform_DebugOutput(buffer);
        snprintf(buffer, sizeof(buffer), "measurement.phase_current_b_ampere=%.3f\r\n", sensor->current_b.output_value);
        FOC_Platform_DebugOutput(buffer);
        snprintf(buffer, sizeof(buffer), "measurement.phase_current_c_ampere=%.3f\r\n", sensor->current_c.output_value);
        FOC_Platform_DebugOutput(buffer);
    }
    else
    {
        FOC_Platform_DebugOutput("measurement.current.status=invalid\r\n");
    }

    if (sensor->encoder_valid != 0U)
    {
        snprintf(buffer, sizeof(buffer), "measurement.encoder_angle_raw_rad=%.3f\r\n", sensor->mech_angle_rad.raw_value);
        FOC_Platform_DebugOutput(buffer);
        snprintf(buffer, sizeof(buffer), "measurement.encoder_angle_filtered_rad=%.3f\r\n", sensor->mech_angle_rad.output_value);
        FOC_Platform_DebugOutput(buffer);
    }
    else
    {
        FOC_Platform_DebugOutput("measurement.encoder.status=invalid\r\n");
    }

    exec_time = ControlScheduler_GetExecutionCycles();
    snprintf(buffer, sizeof(buffer), "control.execution_time_us=%.3f\r\n\r\n", exec_time / 120.0f);
    FOC_Platform_DebugOutput(buffer);
}
