#include "L1_Orchestration/foc_output_mgr.h"

#include "L2_Core/Runtime/foc_queue.h"
#include "L3_Hal/foc_platform_api.h"

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