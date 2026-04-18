#include "L2_Service/runtime_c31_frame_source.h"

#include "L2_Service/runtime_c32_frame_adapter.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define RUNTIME_C31_COMM_SOURCE_1 1U
#define RUNTIME_C31_COMM_SOURCE_2 2U
#define RUNTIME_C31_COMM_SOURCE_3 3U
#define RUNTIME_C31_COMM_SOURCE_4 4U

static uint8_t g_preferred_source = 0U;

static uint8_t RuntimeC31_IsFrameReadyFromSource(uint8_t source)
{
    if (source == RUNTIME_C31_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_IsFrameReady();
    }

    if (source == RUNTIME_C31_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_IsFrameReady();
    }

    if (source == RUNTIME_C31_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_IsFrameReady();
    }

    if (source == RUNTIME_C31_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_IsFrameReady();
    }

    return 0U;
}

static uint16_t RuntimeC31_ReadFrameFromSource(uint8_t source, uint8_t *buffer, uint16_t max_len)
{
    if (source == RUNTIME_C31_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_ReadFrame(buffer, max_len);
    }

    if (source == RUNTIME_C31_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_ReadFrame(buffer, max_len);
    }

    if (source == RUNTIME_C31_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_ReadFrame(buffer, max_len);
    }

    if (source == RUNTIME_C31_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_ReadFrame(buffer, max_len);
    }

    return 0U;
}

static uint16_t RuntimeC31_TryReadReadySources(uint8_t *buffer, uint16_t max_len)
{
    uint8_t source;
    uint16_t len;

    if (buffer == 0)
    {
        return 0U;
    }

    if ((g_preferred_source != 0U) &&
        (RuntimeC31_IsFrameReadyFromSource(g_preferred_source) != 0U))
    {
        len = RuntimeC31_ReadFrameFromSource(g_preferred_source, buffer, max_len);
        if (len > 0U)
        {
            return len;
        }
    }

    for (source = RUNTIME_C31_COMM_SOURCE_1; source <= RUNTIME_C31_COMM_SOURCE_4; source++)
    {
        if (source == g_preferred_source)
        {
            continue;
        }

        if (RuntimeC31_IsFrameReadyFromSource(source) == 0U)
        {
            continue;
        }

        len = RuntimeC31_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            g_preferred_source = source;
            return len;
        }
    }

    return 0U;
}

static uint16_t RuntimeC31_TryReadAnySource(uint8_t *buffer, uint16_t max_len)
{
    uint8_t source;
    uint16_t len;

    if (buffer == 0)
    {
        return 0U;
    }

    for (source = RUNTIME_C31_COMM_SOURCE_1; source <= RUNTIME_C31_COMM_SOURCE_4; source++)
    {
        len = RuntimeC31_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            g_preferred_source = source;
            return len;
        }
    }

    return 0U;
}

void RuntimeC31_Init(void)
{
    g_preferred_source = 0U;
    RuntimeC32_Init();
}

void RuntimeC31_ApplySignals(const runtime_c31_step_signal_t *signal)
{
    runtime_c32_step_signal_t c32_signal;

    if (signal == 0)
    {
        RuntimeC32_ApplySignals(0);
        return;
    }

    c32_signal.init_checks_pass_mask = signal->init_checks_pass_mask;
    c32_signal.init_checks_fail_mask = signal->init_checks_fail_mask;
    c32_signal.finalize_init = signal->finalize_init;
    c32_signal.sensor_state_updated = signal->sensor_state_updated;
    c32_signal.adc_valid = signal->adc_valid;
    c32_signal.encoder_valid = signal->encoder_valid;
    c32_signal.control_loop_skipped = signal->control_loop_skipped;
    c32_signal.undervoltage_fault = signal->undervoltage_fault;
    c32_signal.undervoltage_vbus = signal->undervoltage_vbus;

    RuntimeC32_ApplySignals(&c32_signal);
}

uint8_t RuntimeC31_ProcessOneFrame(void)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    uint16_t len;

    len = RuntimeC31_TryReadReadySources(frame, (uint16_t)sizeof(frame));
    if (len == 0U)
    {
        len = RuntimeC31_TryReadAnySource(frame, (uint16_t)sizeof(frame));
    }

    if (len == 0U)
    {
        return 0U;
    }

    return RuntimeC32_HandleFrame(frame, len);
}

void RuntimeC31_Snapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC32_Snapshot(snapshot);
}

void RuntimeC31_Commit(void)
{
    RuntimeC32_Commit();
}
