#include "L2_Core/foc_motor_aggregate.h"
#include "L1_Orchestration/foc_output_mgr.h"

#include <stdio.h>
#include <string.h>

#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Runtime/foc_monitor_queue_types.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_codec.h"
#include "LS_Config/foc_config.h"

void FOC_OutputMgr_Init(foc_system_t *sys)
{
    if (sys == 0) return;

    /* 初始化 TX 队列（L2/Runtime FIFO） */
    FIFO_Init(&sys->runtime.output.tx_fifo,
              (uint8_t *)sys->runtime.output.tx_buffer,
              FOC_OUTPUT_FRAME_MAX_LEN,
              FOC_OUTPUT_QUEUE_DEPTH);

    /* 初始化 RX 队列（L2/Runtime FIFO） */
    FIFO_Init(&sys->runtime.comm.rx_fifo,
              (uint8_t *)sys->runtime.comm.rx_buffer,
              PROTOCOL_PARSER_RX_MAX_LEN,
              FOC_RX_QUEUE_DEPTH);
}

void FOC_OutputMgr_WriteDirect(const char *text)
{
    if (text == 0) return;
    FOC_Platform_WriteDebugText(text);
}

void FOC_OutputMgr_WriteStatus(uint8_t status)
{
    FOC_Platform_WriteStatusByte(status);
}

void FOC_OutputMgr_FlushQueue(foc_system_t *sys)
{
    uint8_t sent = 0U;

    if (sys == 0) return;

    while (sent < FOC_OUTPUT_MAX_PER_CYCLE)
    {
        char buf[FOC_OUTPUT_FRAME_MAX_LEN];

        if (FIFO_Count(&sys->runtime.output.tx_fifo) == 0U) break;

        (void)FIFO_Dequeue(&sys->runtime.output.tx_fifo, (uint8_t *)buf);
        FOC_Platform_WriteDebugText(buf);
        sent++;
    }
}

uint8_t FOC_OutputMgr_GetOverflowCount(const foc_system_t *sys)
{
    if (sys == 0) return 0U;
    return sys->runtime.output.tx_fifo.overflow_count;
}

static void FOC_OutputMgr_PollOneSource(foc_system_t *sys, uint8_t source_idx)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    uint16_t len;
    uint16_t max_frames;

    max_frames = (FOC_COMM_MAX_FRAMES_PER_SERVICE == 0U) ? 4U : FOC_COMM_MAX_FRAMES_PER_SERVICE;
    if (max_frames > 4U) max_frames = 4U;

    for (uint8_t i = 0; i < 4U; i++)
    {
        uint8_t idx = (uint8_t)((source_idx + i) % 4U);

        len = FOC_Platform_CommSource_ReadFrame((FOC_Platform_CommSourceId_t)idx,
                                               frame,
                                               sizeof(frame));

        if (len == 0U) continue;

        if (FIFO_Enqueue(&sys->runtime.comm.rx_fifo, frame) != 0U)
        {
            sys->runtime.comm.source_rr = (uint8_t)((idx + 1U) % 4U);
        }

        if (i >= (max_frames - 1U)) break;
    }
}

void FOC_OutputMgr_PollSources(foc_system_t *sys)
{
    if (sys == 0) return;
    FOC_OutputMgr_PollOneSource(sys, sys->runtime.comm.source_rr);
}

void FOC_OutputMgr_WriteStartupInfo(foc_motor_t *motor)
{
    char buf[160];
    int32_t ip_mech_zero;
    int32_t fp_mech_zero;
    int32_t ip_vbus;
    int32_t fp_vbus;
    int32_t ip_max_phase;
    int32_t fp_max_phase;
    int32_t ip_duty;
    int32_t fp_duty;
    int32_t ip_true_vbus;
    int32_t fp_true_vbus;

    Math_FloatToFixed(motor->params.mech_angle_at_elec_zero_rad, 4, &ip_mech_zero, &fp_mech_zero);
    Math_FloatToFixed(motor->params.vbus_voltage, 2, &ip_vbus, &fp_vbus);
    Math_FloatToFixed(motor->ctrl.max_phase_voltage, 2, &ip_max_phase, &fp_max_phase);
    Math_FloatToFixed((motor->params.vbus_voltage > 0.0f) ?
                      (motor->ctrl.max_phase_voltage / motor->params.vbus_voltage) : 0.0f,
                      2, &ip_duty, &fp_duty);
    Math_FloatToFixed(motor->sensor.vbus.filtered, 2, &ip_true_vbus, &fp_true_vbus);

    snprintf(buf, sizeof(buf),
             "mech zero at elec0: %d.%04d rad, direction: %d, pole pairs: %d, vbus: %d.%02dV, max_phase_voltage: %d.%02dV, duty_max: %d.%02d\r\n true_vbus: %d.%02dV\r\n",
             (int)ip_mech_zero, (int)fp_mech_zero,
             (int)motor->params.direction,
             (int)motor->params.pole_pairs,
             (int)ip_vbus, (int)fp_vbus,
             (int)ip_max_phase, (int)fp_max_phase,
             (int)ip_duty, (int)fp_duty,
             (int)ip_true_vbus, (int)fp_true_vbus);
    FOC_OutputMgr_WriteDirect(buf);
}

void FOC_OutputMgr_ProcessMonitorElements(foc_system_t *sys)
{
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    uint8_t consumed = 0U;
    uint8_t in_frame = 0U;
    uint8_t collecting_osc = 0U;

    if (sys == 0) return;

    while (consumed < FOC_MONITOR_MAX_DEQUEUE_PER_CYCLE)
    {
        monitor_element_t elem;

        if (FIFO_Dequeue(&sys->runtime.monitor.elem_fifo, (uint8_t *)&elem) == 0U)
            break;
        consumed++;

        if (elem.tag == MONITOR_ELEM_FRAME_START)
        {
            collecting_osc = 0U;
            in_frame = 1U;
            continue;
        }
        if (!in_frame) continue;

        /* 语义行 */
        if (elem.tag <= MONITOR_ELEM_SEMANTIC_9)
        {
            char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];

            if (elem.aux == 0U)
            {
                if (DebugStream_FormatInvalidLine(elem.tag, line, sizeof(line)) == 0U)
                    continue;
            }
            else
            {
                DebugStream_FormatSemanticLine(elem.tag, elem.value, line, sizeof(line));
            }
            (void)FIFO_Enqueue(&sys->runtime.output.tx_fifo, (uint8_t *)line);
            continue;
        }

        if (elem.tag == MONITOR_ELEM_SEMANTIC_END)
        {
            in_frame = 0U;
            continue;
        }

        /* 示波器累积 */
        if (elem.tag == MONITOR_ELEM_OSC_VALUE)
        {
            if (collecting_osc == 0U)
            {
                sys->runtime.monitor.osc_collect_count = 0U;
                collecting_osc = 1U;
            }
            if (sys->runtime.monitor.osc_collect_count < OSC_SNAPSHOT_CHANNEL_COUNT)
            {
                uint8_t idx = sys->runtime.monitor.osc_collect_count;
                sys->runtime.monitor.osc_collect_val[idx] = elem.value;
                sys->runtime.monitor.osc_collect_bit[idx] = elem.aux;
                sys->runtime.monitor.osc_collect_count++;
            }
            continue;
        }

        if (elem.tag == MONITOR_ELEM_OSC_END)
        {
            char out[FOC_OUTPUT_FRAME_MAX_LEN];

            (void)Codec_OscEncodeFrame(sys->runtime.monitor.osc_collect_bit,
                                       sys->runtime.monitor.osc_collect_val,
                                       sys->runtime.monitor.osc_collect_count,
                                       out, sizeof(out));
            (void)FIFO_Enqueue(&sys->runtime.output.tx_fifo, (uint8_t *)out);
            collecting_osc = 0U;
            in_frame = 0U;
            continue;
        }
    }
#else
    (void)sys;
#endif
}

