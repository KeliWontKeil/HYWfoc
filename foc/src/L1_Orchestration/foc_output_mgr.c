#include "L1_Orchestration/foc_output_mgr.h"

#include <string.h>

#include "L3/foc_platform_api.h"

/* 内部全局队列指针（L1 Init 时设置，供 L2 无参调用使用） */
static foc_output_queue_t *g_queue_ptr = 0;

void FOC_OutputMgr_Init(foc_system_t *sys)
{
    if (sys == 0) return;
    g_queue_ptr = &sys->runtime.output_queue;
    g_queue_ptr->write_idx = 0U;
    g_queue_ptr->read_idx = 0U;
    g_queue_ptr->count = 0U;
    g_queue_ptr->overflow_count = 0U;
}

void FOC_OutputMgr_WriteDirect(const char *text)
{
    if (text == 0) return;
    FOC_Platform_WriteDebugText(text);
}

void FOC_OutputMgr_WriteQueue(const char *text)
{
    size_t len;
    uint8_t next;

    if ((g_queue_ptr == 0) || (text == 0)) return;

    if (g_queue_ptr->count >= FOC_OUTPUT_QUEUE_DEPTH)
    {
        g_queue_ptr->read_idx = (g_queue_ptr->read_idx + 1U) % FOC_OUTPUT_QUEUE_DEPTH;
        if (g_queue_ptr->count > 0U) g_queue_ptr->count--;
        g_queue_ptr->overflow_count++;
    }

    len = strlen(text);
    if (len >= FOC_OUTPUT_FRAME_MAX_LEN) len = FOC_OUTPUT_FRAME_MAX_LEN - 1U;
    (void)memcpy(g_queue_ptr->buffer[g_queue_ptr->write_idx], text, len);
    g_queue_ptr->buffer[g_queue_ptr->write_idx][len] = '\0';

    next = (g_queue_ptr->write_idx + 1U) % FOC_OUTPUT_QUEUE_DEPTH;
    g_queue_ptr->write_idx = next;
    g_queue_ptr->count++;
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
        const char *msg;

        if (sys->runtime.output_queue.count == 0U) break;

        msg = sys->runtime.output_queue.buffer[sys->runtime.output_queue.read_idx];
        sys->runtime.output_queue.read_idx = (sys->runtime.output_queue.read_idx + 1U) % FOC_OUTPUT_QUEUE_DEPTH;
        sys->runtime.output_queue.count--;

        FOC_Platform_WriteDebugText(msg);
        sent++;
    }
}

uint8_t FOC_OutputMgr_GetOverflowCount(const foc_system_t *sys)
{
    if (sys == 0) return 0U;
    return sys->runtime.output_queue.overflow_count;
}