#include "L1_Orchestration/foc_output_mgr.h"

#include <stdio.h>
#include <string.h>

#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Runtime/foc_monitor_queue_types.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

void FOC_OutputMgr_Init(foc_system_t *sys)
{
    if (sys == 0) return;

    /* 初始化 TX 队列（L2/Runtime FIFO） */
    FIFO_Init(&sys->runtime.tx_fifo,
              (uint8_t *)sys->runtime.tx_fifo_buffer,
              FOC_OUTPUT_FRAME_MAX_LEN,
              FOC_OUTPUT_QUEUE_DEPTH);

    /* 初始化 RX 队列（L2/Runtime FIFO） */
    FIFO_Init(&sys->runtime.rx_fifo,
              (uint8_t *)sys->runtime.rx_fifo_buffer,
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

        if (FIFO_Count(&sys->runtime.tx_fifo) == 0U) break;

        (void)FIFO_Dequeue(&sys->runtime.tx_fifo, (uint8_t *)buf);
        FOC_Platform_WriteDebugText(buf);
        sent++;
    }
}

uint8_t FOC_OutputMgr_GetOverflowCount(const foc_system_t *sys)
{
    if (sys == 0) return 0U;
    return sys->runtime.tx_fifo.overflow_count;
}

static uint8_t FOC_OutputMgr_PollOneSource(foc_system_t *sys, uint8_t source_idx)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    uint16_t len;
    uint16_t max_frames;

    max_frames = (FOC_COMM_MAX_FRAMES_PER_SERVICE == 0U) ? 4U : FOC_COMM_MAX_FRAMES_PER_SERVICE;
    if (max_frames > 4U) max_frames = 4U;

    for (uint8_t i = 0; i < 4U; i++)
    {
        uint8_t idx = (uint8_t)((source_idx + i) % 4U);

        switch (idx)
        {
        case 0U: len = FOC_Platform_CommSource1_ReadFrame(frame, sizeof(frame)); break;
        case 1U: len = FOC_Platform_CommSource2_ReadFrame(frame, sizeof(frame)); break;
        case 2U: len = FOC_Platform_CommSource3_ReadFrame(frame, sizeof(frame)); break;
        case 3U: len = FOC_Platform_CommSource4_ReadFrame(frame, sizeof(frame)); break;
        default: len = 0U; break;
        }

        if (len == 0U) continue;

        if (FIFO_Enqueue(&sys->runtime.rx_fifo, frame) != 0U)
        {
            sys->runtime.comm_source_rr = (uint8_t)((idx + 1U) % 4U);
        }

        if (i >= (max_frames - 1U)) break;
    }
    return 0U;
}

void FOC_OutputMgr_PollSources(foc_system_t *sys)
{
    if (sys == 0) return;
    FOC_OutputMgr_PollOneSource(sys, sys->runtime.comm_source_rr);
}

void FOC_OutputMgr_WriteStartupInfo(foc_motor_t *motor)
{
    char buf[160];

    if (motor == 0) return;

    snprintf(buf, sizeof(buf),
             "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, max_phase_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
             (double)motor->mech_angle_at_elec_zero_rad,
             (int)motor->direction,
             (int)motor->pole_pairs,
             (double)motor->vbus_voltage,
             (double)motor->max_phase_voltage,
             (double)(motor->vbus_voltage > 0.0f ? motor->max_phase_voltage / motor->vbus_voltage : 0.0f),
             (double)motor->sensor.vbus_voltage_filtered);
    FOC_OutputMgr_WriteDirect(buf);
}

void FOC_OutputMgr_ProcessMonitorElements(foc_system_t *sys)
{
    uint8_t consumed = 0U;
    uint8_t in_frame = 0U;
    uint8_t collecting_osc = 0U;

    if (sys == 0) return;

    while (consumed < FOC_MONITOR_MAX_DEQUEUE_PER_CYCLE)
    {
        monitor_element_t elem;

        if (FIFO_Dequeue(&sys->runtime.monitor_elem_q, (uint8_t *)&elem) == 0U)
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
        if (elem.tag <= MONITOR_ELEM_SEMANTIC_8)
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
            (void)FIFO_Enqueue(&sys->runtime.tx_fifo, (uint8_t *)line);
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
                sys->runtime.osc.collect_offset = 0U;
                sys->runtime.osc.collect_buf[0] = '\0';
                collecting_osc = 1U;
            }
            DebugStream_AppendOscValue(sys->runtime.osc.collect_buf,
                                        &sys->runtime.osc.collect_offset,
                                        elem.value);
            continue;
        }

        if (elem.tag == MONITOR_ELEM_OSC_END)
        {
            char line[DEBUG_STREAM_OSC_PAYLOAD_LEN];
            uint16_t off = 0U;
            int written;

            written = snprintf(line, sizeof(line), "%c", (char)DEBUG_STREAM_OSC_HEAD_BYTE);
            if (written > 0) off = (uint16_t)written;

            if ((int)(sizeof(line)) > (int)(off + 1))
            {
                uint16_t copy_len = (uint16_t)(sizeof(line) - off - 1);
                uint16_t src_len = (uint16_t)strlen(sys->runtime.osc.collect_buf);
                if (copy_len > src_len) copy_len = src_len;
                if (copy_len > 0U)
                {
                    (void)memcpy(line + off, sys->runtime.osc.collect_buf, copy_len);
                    off += copy_len;
                    line[off] = '\0';
                }
            }

            if ((off + 6U) < sizeof(line))
            {
                written = snprintf(line + off, sizeof(line) - off,
                                   " %c ", (char)DEBUG_STREAM_OSC_TAIL_BYTE);
                if (written > 0) off += (uint16_t)written;
            }
            (void)FIFO_Enqueue(&sys->runtime.tx_fifo, (uint8_t *)line);
            collecting_osc = 0U;
            in_frame = 0U;
            continue;
        }
    }
}

