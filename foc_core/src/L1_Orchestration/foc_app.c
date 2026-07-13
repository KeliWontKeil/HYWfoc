#include "L1_Orchestration/foc_app.h"

#include <stdio.h>
#include <string.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L1_Orchestration/foc_output_mgr.h"
#include "L1_Orchestration/foc_indicator.h"
#include "L1_Orchestration/foc_init.h"
#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_task_scheduler.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Control/foc_ctrl_executor.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
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

static void FOC_App_SchedTickBridge(void)
{
    ControlScheduler_RunTick(&g_sys.runtime.scheduler);
    DebugStream_SetExecutionCycles(&g_sys.runtime.debug_stream,
        ControlScheduler_GetExecutionCycles(&g_sys.runtime.scheduler));
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

    FOC_Init_Runtime(&g_sys, &motor,
                     FOC_App_SchedTickBridge,
                     FOC_App_ServiceTrigger,
                     FOC_App_ControlTrigger,
                     FOC_App_MonitorTrigger,
                     FOC_App_OnPwmUpdateISR);
    FOC_Init_MotorAndCalib(&motor);
    FOC_Init_Verify(&motor, &motor.sensor);
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
    if (motor.state.control_phase == FOC_CONTROL_PHASE_COGGING_CALIB)
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

        FOC_OutputMgr_ProcessMonitorElements(&g_sys);
    }

    /* ---- Service 段 ---- */
    if (g_sys.runtime.service_task_pending != 0U)
    {
        uint8_t needs_param_dump   = 0U;
        uint8_t needs_config_dump  = 0U;
        uint8_t needs_state_dump   = 0U;
        uint8_t needs_system_info  = 0U;

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
                char summary_line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                FOC_Protocol_FormatSummaryLine(&motor, summary_line, sizeof(summary_line));
                (void)FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)summary_line);
            }

            needs_param_dump   |= result.needs_param_dump;
            needs_config_dump  |= result.needs_config_dump;
            needs_state_dump   |= result.needs_state_dump;
            needs_system_info  |= result.needs_system_info;
        }

        if (motor.state.cfg_dirty != 0U)
        {
            FOC_Control_ApplyConfig(&motor);
            motor.state.cfg_dirty = 0U;
        }

        if (needs_param_dump   != 0U) FOC_Protocol_QueueParams(&motor, &g_sys.runtime.tx_fifo);
        if (needs_config_dump  != 0U) FOC_Protocol_QueueConfigs(&motor, &g_sys.runtime.tx_fifo);
        if (needs_state_dump   != 0U) FOC_Protocol_QueueStates(&motor, &g_sys.runtime.tx_fifo);
        if (needs_system_info  != 0U) FOC_Protocol_QueueSystemInfo(&motor, &g_sys.runtime.tx_fifo);

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        if (FOC_CoggingCalibIsDumpPending(&motor) != 0U)
        {
            FOC_CoggingCalibClearDumpPending(&motor);
            FOC_CoggingCalibDumpTable(&motor);
        }
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

#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    Sensor_ReadEncoder(&motor, &motor.sensor);
#else
    motor.sensor.mech_angle_rad = motor.sensor_fast.mech_angle_rad;
    motor.sensor.encoder_valid = motor.sensor_fast.encoder_valid;
#endif
    Sensor_ReadVBUS(&motor.sensor);
    Sensor_SyncCurrentSnapshot(&motor);

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