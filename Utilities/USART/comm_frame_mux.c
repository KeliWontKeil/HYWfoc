#include "comm_frame_mux.h"

#include "usart1.h"
#include "usart2.h"

static uint8_t g_source_mask = COMM_FRAME_MUX_SOURCE_ALL;
static uint8_t g_arb_policy = (uint8_t)COMM_FRAME_MUX_ARB_ROUND_ROBIN;
static uint8_t g_rr_cursor = 0U;
static comm_frame_mux_rx_trigger_callback_t g_rx_trigger_callback = 0;

static uint16_t CommFrameMux_ReadSource(uint8_t source, uint8_t *buffer, uint16_t max_len);
static uint8_t CommFrameMux_SourceHasFrame(uint8_t source);
static void CommFrameMux_OnRxIdleEvent(void);

void CommFrameMux_Init(const comm_frame_mux_config_t *config)
{
    g_source_mask = COMM_FRAME_MUX_SOURCE_ALL;
    g_arb_policy = (uint8_t)COMM_FRAME_MUX_ARB_ROUND_ROBIN;
    g_rr_cursor = 0U;

    if (config != NULL)
    {
        if ((config->source_mask & COMM_FRAME_MUX_SOURCE_ALL) != 0U)
        {
            g_source_mask = (uint8_t)(config->source_mask & COMM_FRAME_MUX_SOURCE_ALL);
        }
        g_arb_policy = config->arbitration_policy;
    }

    USART1_SetIdleCallback(CommFrameMux_OnRxIdleEvent);
    USART2_SetIdleCallback(CommFrameMux_OnRxIdleEvent);
}

void CommFrameMux_SetRxTriggerCallback(comm_frame_mux_rx_trigger_callback_t callback)
{
    g_rx_trigger_callback = callback;
}

uint8_t CommFrameMux_HasPendingFrame(void)
{
    if ((g_source_mask & COMM_FRAME_MUX_SOURCE_USART1) != 0U)
    {
        if (CommFrameMux_SourceHasFrame(COMM_FRAME_MUX_SOURCE_USART1) != 0U)
        {
            return 1U;
        }
    }

    if ((g_source_mask & COMM_FRAME_MUX_SOURCE_USART2) != 0U)
    {
        if (CommFrameMux_SourceHasFrame(COMM_FRAME_MUX_SOURCE_USART2) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

uint16_t CommFrameMux_TryDequeueFrame(uint8_t *buffer, uint16_t max_len)
{
    uint16_t len = 0U;
    uint8_t first = COMM_FRAME_MUX_SOURCE_USART1;
    uint8_t second = COMM_FRAME_MUX_SOURCE_USART2;

    if ((buffer == NULL) || (max_len == 0U))
    {
        return 0U;
    }

    if (g_arb_policy == (uint8_t)COMM_FRAME_MUX_ARB_ROUND_ROBIN)
    {
        if (g_rr_cursor != 0U)
        {
            first = COMM_FRAME_MUX_SOURCE_USART2;
            second = COMM_FRAME_MUX_SOURCE_USART1;
        }
    }

    if ((g_source_mask & first) != 0U)
    {
        len = CommFrameMux_ReadSource(first, buffer, max_len);
        if (len > 0U)
        {
            g_rr_cursor = (first == COMM_FRAME_MUX_SOURCE_USART1) ? 1U : 0U;
            return len;
        }
    }

    if ((g_source_mask & second) != 0U)
    {
        len = CommFrameMux_ReadSource(second, buffer, max_len);
        if (len > 0U)
        {
            g_rr_cursor = (second == COMM_FRAME_MUX_SOURCE_USART1) ? 1U : 0U;
            return len;
        }
    }

    g_rr_cursor = (uint8_t)(1U - g_rr_cursor);
    return 0U;
}

static uint16_t CommFrameMux_ReadSource(uint8_t source, uint8_t *buffer, uint16_t max_len)
{
    if (source == COMM_FRAME_MUX_SOURCE_USART1)
    {
        return USART1_ReadFrame(buffer, max_len);
    }

    if (source == COMM_FRAME_MUX_SOURCE_USART2)
    {
        return USART2_ReadFrame(buffer, max_len);
    }

    return 0U;
}

static uint8_t CommFrameMux_SourceHasFrame(uint8_t source)
{
    if (source == COMM_FRAME_MUX_SOURCE_USART1)
    {
        return USART1_IsFrameReady();
    }

    if (source == COMM_FRAME_MUX_SOURCE_USART2)
    {
        return USART2_IsFrameReady();
    }

    return 0U;
}

static void CommFrameMux_OnRxIdleEvent(void)
{
    if (g_rx_trigger_callback != 0)
    {
        g_rx_trigger_callback();
    }
}
