#include "L1_Orchestration/foc_output_mgr.h"

#include <string.h>

#include "L3/foc_platform_api.h"

void FOC_OutputMgr_Init(foc_system_t *sys)
{
    if (sys == 0) return;
    sys->runtime.output_queue.write_idx = 0U;
    sys->runtime.output_queue.read_idx = 0U;
    sys->runtime.output_queue.count = 0U;
    sys->runtime.output_queue.overflow_count = 0U;
}

void FOC_OutputMgr_WriteDirect(foc_system_t *sys, const char *text)
{
    (void)sys;
    if (text == 0) return;
    FOC_Platform_WriteDebugText(text);
}

void FOC_OutputMgr_WriteQueue(foc_system_t *sys, const char *text)
{
    size_t len;
    uint8_t next;

    if ((sys == 0) || (text == 0)) return;

    if (sys->runtime.output_queue.count >= FOC_OUTPUT_QUEUE_DEPTH)
    {
        /* 满队列：丢弃最旧帧 */
        sys->runtime.output_queue.read_idx = (sys->runtime.output_queue.read_idx + 1U) % FOC_OUTPUT_QUEUE_DEPTH;
        if (sys->runtime.output_queue.count > 0U) sys->runtime.output_queue.count--;
        sys->runtime.output_queue.overflow_count++;
    }

    len = strlen(text);
    if (len >= FOC_OUTPUT_FRAME_MAX_LEN) len = FOC_OUTPUT_FRAME_MAX_LEN - 1U;
    (void)memcpy(sys->runtime.output_queue.buffer[sys->runtime.output_queue.write_idx], text, len);
    sys->runtime.output_queue.buffer[sys->runtime.output_queue.write_idx][len] = '\0';

    next = (sys->runtime.output_queue.write_idx + 1U) % FOC_OUTPUT_QUEUE_DEPTH;
    sys->runtime.output_queue.write_idx = next;
    sys->runtime.output_queue.count++;
}

void FOC_OutputMgr_WriteStatus(foc_system_t *sys, uint8_t status)
{
    (void)sys;
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