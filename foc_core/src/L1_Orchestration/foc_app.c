#include "L1_Orchestration/foc_app.h"

#include <stdio.h>
#include <string.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L1_Orchestration/foc_output_mgr.h"
#include "L1_Orchestration/foc_indicator.h"
#include "L1_Orchestration/foc_init_check.h"
#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_task_scheduler.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Control/foc_ctrl_executor.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_init.h"
#include "L2_Core/Control/foc_ctrl_cogging_calib.h"
#include "L2_Core/Protocol/foc_protocol_handler.h"
#include "L2_Core/Protocol/foc_protocol_output.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 全局实例
 * ================================================================ */

static foc_system_t g_sys;
static foc_motor_t motor;

/* ================================================================
 * 内部工具
 * ================================================================ */

static uint8_t FOC_App_IsCalibrating(void)
{
    return (motor.state.control_phase == FOC_CONTROL_PHASE_COGGING_CALIB) ? 1U : 0U;
}

static void FOC_App_SampleSensors(void)
{
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    Sensor_ReadEncoder(&motor, &motor.sensor);
#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
    motor.ecycle_ref_angle_rad = motor.sensor.mech_angle_rad.output_value;
    motor.ecycle_ref_angle_valid = motor.sensor.encoder_valid;
#endif
#else
    motor.sensor.mech_angle_rad = motor.sensor_fast.mech_angle_rad;
    motor.sensor.encoder_valid = motor.sensor_fast.encoder_valid;
#endif
    Sensor_ReadVBUS(&motor.sensor);
    Sensor_SyncCurrentSnapshot(&motor);
}

static void FOC_App_SchedTickBridge(void)
{
    ControlScheduler_RunTick(&g_sys.runtime.scheduler);
    DebugStream_SetExecutionCycles(&g_sys.runtime.debug_stream,
        ControlScheduler_GetExecutionCycles(&g_sys.runtime.scheduler));
}

static void FOC_App_HandleResult(uint8_t cycle_result)
{
    switch (cycle_result)
    {
    case FOC_CYCLE_OK:
        motor.state.system_running = 1U;
        break;
    case FOC_CYCLE_FAULT_SENSOR:
        motor.state.system_fault = 1U;
        motor.state.system_running = 0U;
        FOC_OutputMgr_WriteDirect("sensor invalid threshold reached\r\n");
        FOC_ControlExecutor_SafeOutput(&motor);
        break;
    case FOC_CYCLE_FAULT_UVLO:
        motor.state.system_fault = 1U;
        motor.state.system_running = 0U;
        FOC_ControlExecutor_SafeOutput(&motor);
        break;
    default:
        break;
    }
}

static void FOC_App_ApplyCfgDirty(void)
{
    if (motor.state.cfg_dirty != 0U)
    {
        FOC_Control_ApplyConfig(&motor);
        motor.state.cfg_dirty = 0U;
    }
}

/* ================================================================
 * 初始化
 * ================================================================ */

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, 1U);

    g_sys.runtime.service_task_pending = 0U;
    g_sys.runtime.monitor_task_pending = 0U;
    g_sys.runtime.monitor_frame_active = 0U;
    g_sys.runtime.indicator.comm_pulse_counter = 0U;
    g_sys.runtime.indicator.led_run_blink_counter = 0U;
    g_sys.runtime.comm_source_rr = 0U;
    g_sys.runtime.osc.collect_offset = 0U;
    g_sys.runtime.osc.collect_buf[0] = '\0';

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init(&g_sys.runtime.scheduler);
    FOC_Platform_SetControlTickCallback(FOC_App_SchedTickBridge);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_SERVICE, FOC_App_ServiceTrigger);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_FAST_CONTROL, FOC_App_ControlTrigger);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_MONITOR, FOC_App_MonitorTrigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    FOC_Platform_CommInit();
    FOC_OutputMgr_Init(&g_sys);

    FIFO_Init(&g_sys.runtime.monitor_elem_q,
              (uint8_t *)g_sys.runtime.monitor_elem_buffer,
              sizeof(monitor_element_t),
              FOC_MONITOR_ELEM_QUEUE_DEPTH);

    FOC_Protocol_Init(&g_sys.cfg.telemetry);
    DebugStream_Init(&g_sys.runtime.debug_stream);
    FOC_ControlPlatform_InitHardware(&motor);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    FOC_MotorInit(&motor,
                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);
    FOC_Control_ApplyConfig(&motor);

    FOC_InitCheck_Verify(&motor, &motor.sensor);
    FOC_OutputMgr_WriteStartupInfo(&motor);
    FOC_Indicator_Update(&motor, &g_sys.runtime);
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlRuntimeInterrupts(1U);
}

/* ================================================================
 * 主循环
 * ================================================================ */

void FOC_App_Loop(void)
{
    if (FOC_App_IsCalibrating() != 0U)
    {
        FOC_OutputMgr_FlushQueue(&g_sys);
        return;
    }

    /* ---- Monitor 段 ---- */
    if (g_sys.runtime.monitor_task_pending != 0U)
    {
        g_sys.runtime.monitor_task_pending = 0U;

        if (g_sys.runtime.monitor_frame_active != 0U)
        {
            g_sys.runtime.monitor_task_pending = 1U;
            return;
        }

        uint8_t consumed = 0U;
        uint8_t in_frame = 0U;
        uint8_t collecting_osc = 0U;

        while (consumed < FOC_MONITOR_MAX_DEQUEUE_PER_CYCLE)
        {
            monitor_element_t elem;

            if (FIFO_Dequeue(&g_sys.runtime.monitor_elem_q, (uint8_t *)&elem) == 0U)
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
            if (elem.tag <= MONITOR_ELEM_SEMANTIC_7)
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
                (void)FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)line);
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
                    g_sys.runtime.osc.collect_offset = 0U;
                    g_sys.runtime.osc.collect_buf[0] = '\0';
                    collecting_osc = 1U;
                }
                DebugStream_AppendOscValue(g_sys.runtime.osc.collect_buf,
                                            &g_sys.runtime.osc.collect_offset,
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
                    uint16_t src_len = (uint16_t)strlen(g_sys.runtime.osc.collect_buf);
                    if (copy_len > src_len) copy_len = src_len;
                    if (copy_len > 0U)
                    {
                        (void)memcpy(line + off, g_sys.runtime.osc.collect_buf, copy_len);
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
                (void)FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)line);
                collecting_osc = 0U;
                in_frame = 0U;
                continue;
            }

            /* 协议摘要行 */
            if (elem.tag == MONITOR_ELEM_PROTOCOL_SUMMARY)
            {
                char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                FOC_Protocol_FormatSummaryLine(&motor, line, sizeof(line));
                (void)FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)line);
                in_frame = 0U;
                continue;
            }
        }
    }

    /* ---- Service 段 ---- */
    if (g_sys.runtime.service_task_pending != 0U)
    {
        g_sys.runtime.service_task_pending = 0U;

        while (FIFO_Count(&g_sys.runtime.rx_fifo) > 0U)
        {
            uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
            foc_protocol_frame_result_t result;

            (void)FIFO_Dequeue(&g_sys.runtime.rx_fifo, frame);
            result = FOC_Protocol_ProcessSingle(&motor, frame, PROTOCOL_PARSER_RX_MAX_LEN);

            if (result.comm_active != 0U)
                g_sys.runtime.indicator.comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;

            if (result.needs_summary != 0U)
            {
                monitor_element_t elem;
                elem.tag   = MONITOR_ELEM_PROTOCOL_SUMMARY;
                elem.aux   = 0U;
                elem.value = 0.0f;
                (void)FIFO_Enqueue(&g_sys.runtime.monitor_elem_q, (uint8_t *)&elem);
                g_sys.runtime.monitor_task_pending = 1U;
            }
        }

        FOC_App_ApplyCfgDirty();

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        if (FOC_CoggingCalibIsExportPending(&motor) != 0U)
        {
            FOC_CoggingCalibClearExportPending(&motor);
            FOC_CoggingCalibExportTable(&motor);
        }
#endif
    }

    FOC_OutputMgr_FlushQueue(&g_sys);
}

/* ================================================================
 * 回调桥接
 * ================================================================ */

void FOC_App_ServiceTrigger(void)
{
    FOC_Indicator_Update(&motor, &g_sys.runtime);
    FOC_OutputMgr_PollSources(&g_sys);
    g_sys.runtime.service_task_pending = 1U;
}

void FOC_App_MonitorTrigger(void)
{
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    g_sys.runtime.monitor_frame_active = 1U;

    {
        monitor_element_t start_elem;
        start_elem.tag   = MONITOR_ELEM_FRAME_START;
        start_elem.aux   = 0U;
        start_elem.value = 0.0f;
        (void)FIFO_Enqueue(&g_sys.runtime.monitor_elem_q, (uint8_t *)&start_elem);
    }

    {
        monitor_element_t elem;
        while (DebugStream_PollNextValue(&g_sys.runtime.debug_stream,
                                          &motor,
                                          FOC_Protocol_GetTelemetry(),
                                          &elem) != 0U)
        {
            (void)FIFO_Enqueue(&g_sys.runtime.monitor_elem_q, (uint8_t *)&elem);
        }
    }

    g_sys.runtime.monitor_frame_active = 0U;
    g_sys.runtime.monitor_task_pending = 1U;
#endif
}

void FOC_App_ControlTrigger(void)
{
    uint8_t phase;
    uint8_t cycle_result = FOC_CYCLE_OK;

    phase = motor.state.control_phase;
    if (motor.state.system_fault != 0U) return;

    FOC_App_SampleSensors();

    if ((motor.sensor.adc_valid == 0U) || (motor.sensor.encoder_valid == 0U))
    {
        motor.state.sensor_invalid_consecutive++;
        motor.state.control_skip_count++;
        motor.state.last_fault_code = (motor.sensor.adc_valid == 0U) ?
            (uint8_t)FOC_FAULT_SENSOR_ADC_INVALID : (uint8_t)FOC_FAULT_SENSOR_ENCODER_INVALID;

        if (motor.state.sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
            FOC_App_HandleResult((uint8_t)FOC_CYCLE_FAULT_SENSOR);
        return;
    }
    motor.state.sensor_invalid_consecutive = 0U;
    motor.state.last_fault_code = (uint8_t)FOC_FAULT_NONE;

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (motor.sensor.vbus_voltage_filtered < FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        motor.state.last_fault_code = (uint8_t)FOC_FAULT_UNDERVOLTAGE;
        FOC_App_HandleResult((uint8_t)FOC_CYCLE_FAULT_UVLO);
        return;
    }
#endif

    switch (phase)
    {
    case FOC_CONTROL_PHASE_NORMAL:
        if (motor.state.motor_enabled == 0U) return;
        cycle_result = FOC_ControlExecutor_RunCycle(&motor, &motor.sensor, FOC_CONTROL_DT_SEC);
        FOC_App_HandleResult(cycle_result);
        break;

    case FOC_CONTROL_PHASE_COGGING_CALIB:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        (void)FOC_CoggingCalib_RunStep(&motor, &motor.sensor, FOC_CONTROL_DT_SEC);
#endif
        break;

    case FOC_CONTROL_PHASE_REINIT:
#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
        (void)FOC_ReInit_RunStep(&motor, FOC_CONTROL_DT_SEC);
#endif
        break;

    default:
        break;
    }
}

void FOC_App_OnPwmUpdateISR(void)
{
    FOC_ControlExecutor_RunISR(&motor);
}