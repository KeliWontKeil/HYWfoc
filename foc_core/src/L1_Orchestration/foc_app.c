#include "L2_Core/foc_motor_aggregate.h"
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
#include "L2_Core/Control/foc_ctrl_init.h"
#include "L2_Core/Control/foc_ctrl_sens_cogging_calib.h"
#include "L2_Core/Control/foc_ctrl_sens_reinit.h"
#include "L2_Core/Control/foc_ctrl_openloop.h"
#include "L2_Core/Control/foc_ctrl_source_mgr.h"
#include "L2_Core/Protocol/foc_protocol_handler.h"
#include "L2_Core/Protocol/foc_protocol_output.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

static foc_system_t g_sys;
static foc_motor_t motor;

/* 主循环 fault 补发用本地缓存（不进 motor 状态） */
static uint8_t s_prev_system_fault = 0U;

static void FOC_App_SchedTickBridge(void)
{
    ControlScheduler_RunTick(&g_sys.runtime.scheduler);
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    DebugStream_SetExecutionCycles(&g_sys.runtime.monitor.stream,
        ControlScheduler_GetExecutionCycles(&g_sys.runtime.scheduler));
#endif
}

void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();

    FOC_Platform_IndicatorInit();
    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_ERROR_INDEX, 1U);

    FOC_Init_Runtime(&g_sys, &motor,
                     FOC_App_SchedTickBridge,
                     FOC_App_ServiceTrigger,
                     FOC_App_ControlTrigger,
                     FOC_App_MonitorTrigger,
                     FOC_App_OnPwmUpdateISR,
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
                     FOC_App_OnCurrentLoopISR
#else
                     0
#endif
                     );
    FOC_Init_MotorAndCalib(&motor);

    motor.state.control_phase = FOC_CONTROL_PHASE_NORMAL;
#if (FOC_CONTROL_LOW_SOURCE == FOC_CONTROL_SRC_OPENLOOP)
    FOC_OpenLoop_Init(&motor.openloop_state, &motor.params, &motor.cfg);
#endif

    {
        foc_source_mgr_ctx_t sm_ctx;

        FOC_ControlExecutor_BuildSourceMgrCtx(&motor, &motor.ctrl_ref, &sm_ctx);
        FOC_SourceMgr_Init(&sm_ctx,
                           (uint8_t)FOC_CONTROL_LOW_SOURCE,
                           (uint8_t)FOC_CONTROL_HIGH_SOURCE);
    }

    FOC_Init_Verify(&motor, &motor.sensor);
    FOC_OutputMgr_WriteStartupInfo(&motor);
    FOC_Indicator_Update(&motor, &g_sys.runtime);
}

void FOC_App_Start(void)
{
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    FOC_Platform_AuxTimerStart(FOC_AUX_TIMER_CURRENT_LOOP);
#endif
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlInterruptsEnabled(1U);
}

/* 故障码 → 可读自然描述（主循环补发日志用） */
static const char *FOC_App_FaultDescription(uint8_t code)
{
    switch (code)
    {
    case FOC_FAULT_SENSOR_ADC_INVALID:     return "adc current sampling invalid";
    case FOC_FAULT_SENSOR_ENCODER_INVALID: return "encoder feedback invalid";
    case FOC_FAULT_UNDERVOLTAGE:           return "bus undervoltage";
    case FOC_FAULT_PROTOCOL_FRAME:         return "protocol frame error";
    case FOC_FAULT_PARAM_INVALID:          return "invalid parameter";
    case FOC_FAULT_INIT_FAILED:            return "initialization failed";
    case FOC_FAULT_ESTIMATOR_INVALID:      return "estimator invalid";
    default:                               return "unknown fault";
    }
}

/* fault 0→1 跃迁当轮，主循环补发完整详情（慢路径可靠输出） */
static void FOC_App_ReportFaultTransition(void)
{
    if ((motor.state.system_fault != 0U) && (s_prev_system_fault == 0U))
    {
        char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
        snprintf(line, sizeof(line), "fault: %s\r\n",
                 FOC_App_FaultDescription(motor.state.last_fault_code));
        FOC_Platform_WriteDebugText(line);
    }
    s_prev_system_fault = motor.state.system_fault;
}

void FOC_App_Loop(void)
{
    FOC_App_ReportFaultTransition();

    if (g_sys.runtime.tasks.monitor_pending != 0U)
    {
        g_sys.runtime.tasks.monitor_pending = 0U;

        if (g_sys.runtime.monitor.frame_active != 0U)
        {
            g_sys.runtime.tasks.monitor_pending = 1U;
            return;
        }

        FOC_OutputMgr_ProcessMonitorElements(&g_sys);
    }

    if (g_sys.runtime.tasks.service_pending != 0U)
    {
        uint8_t needs_param_dump   = 0U;
        uint8_t needs_config_dump  = 0U;
        uint8_t needs_state_dump   = 0U;
        uint8_t needs_system_info  = 0U;

        g_sys.runtime.tasks.service_pending = 0U;

        while (FIFO_Count(&g_sys.runtime.comm.rx_fifo) > 0U)
        {
            uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
            foc_protocol_frame_result_t result;

            (void)FIFO_Dequeue(&g_sys.runtime.comm.rx_fifo, frame);
            result = FOC_Protocol_ProcessSingle(&motor, frame, PROTOCOL_PARSER_RX_MAX_LEN);

            if (result.comm_active != 0U)
                g_sys.runtime.indicator.comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;

            if (result.needs_summary != 0U)
            {
                char summary_line[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                FOC_Protocol_FormatSummaryLine(&motor, summary_line, sizeof(summary_line));
                (void)FIFO_Enqueue(&g_sys.runtime.output.tx_fifo, (uint8_t *)summary_line);
            }

            needs_param_dump   |= result.needs_param_dump;
            needs_config_dump  |= result.needs_config_dump;
            needs_state_dump   |= result.needs_state_dump;
            needs_system_info  |= result.needs_system_info;
        }

        if (needs_param_dump   != 0U) FOC_Protocol_QueueParams(&motor, &g_sys.runtime.output.tx_fifo);
        if (needs_config_dump  != 0U) FOC_Protocol_QueueConfigs(&motor, &g_sys.runtime.output.tx_fifo);
        if (needs_state_dump   != 0U) FOC_Protocol_QueueStates(&motor, &g_sys.runtime.output.tx_fifo);
        if (needs_system_info  != 0U) FOC_Protocol_QueueSystemInfo(&motor, &g_sys.runtime.output.tx_fifo);

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        if (FOC_CoggingCalibIsDumpPending(&motor.cogging_calib_state) != 0U)
        {
            FOC_CoggingCalibClearDumpPending(&motor.cogging_calib_state);
            FOC_CoggingCalibDumpTable(&motor);
        }
        if (FOC_CoggingCalibIsExportPending(&motor.cogging_calib_state) != 0U)
        {
            FOC_CoggingCalibClearExportPending(&motor.cogging_calib_state);
            FOC_CoggingCalibExportTable(&motor);
        }
#endif
    }

    FOC_OutputMgr_FlushQueue(&g_sys);
}

void FOC_App_ServiceTrigger(void)
{
    
    FOC_OutputMgr_PollSources(&g_sys);
    g_sys.runtime.tasks.service_pending = 1U;
}

void FOC_App_MonitorTrigger(void)
{
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    g_sys.runtime.monitor.frame_active = 1U;

    {
        monitor_element_t start_elem;
        start_elem.tag   = MONITOR_ELEM_FRAME_START;
        start_elem.aux   = 0U;
        start_elem.value = 0.0f;
        (void)FIFO_Enqueue(&g_sys.runtime.monitor.elem_fifo, (uint8_t *)&start_elem);
    }

    {
        monitor_element_t elem;
        while (DebugStream_PollNextValue(&g_sys.runtime.monitor.stream,
                                          &motor,
                                          FOC_Protocol_GetReportConfig(),
                                          &elem) != 0U)
        {
            (void)FIFO_Enqueue(&g_sys.runtime.monitor.elem_fifo, (uint8_t *)&elem);
        }
    }

    g_sys.runtime.monitor.frame_active = 0U;
    g_sys.runtime.tasks.monitor_pending = 1U;
#endif
}

#if (FOC_SPECIAL_PHASE_ABORT_ENABLE == FOC_CFG_ENABLE)
void FOC_App_AbortSpecialPhase(void)
{
    const char *aborted_phase = "UNKNOWN";

    switch (motor.state.control_phase)
    {
    case FOC_CONTROL_PHASE_COGGING_CALIB:
        aborted_phase = "COGGING_CALIB";
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        FOC_CoggingCalib_Abort(&motor);
#endif
        break;
    case FOC_CONTROL_PHASE_REINIT:
        aborted_phase = "REINIT";
#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
        FOC_ReInit_Abort(&motor);
#endif
        break;
    default:
        break;
    }

    /* 突发通告（fast）：本函数可由 ISR(自动退出)或主循环(Y:A)调用，慢路径文本禁用于 ISR */
    {
        char msg[40];
        snprintf(msg, sizeof(msg), "abort:%s\r\n", aborted_phase);
        FOC_OutputMgr_WriteFastEvent(msg);
    }
    motor.state.control_phase = FOC_CONTROL_PHASE_NORMAL;
    motor.mode_transition.prev_control_mode_check = motor.state.control_mode;
    FOC_ControlExecutor_FullStop(&motor);
    FOC_Control_RebuildControlBasis(&motor);
}
#endif

void FOC_App_ControlTrigger(void)
{
    uint8_t phase;
    uint8_t cycle_result = FOC_CYCLE_OK;
    FOC_Indicator_Update(&motor, &g_sys.runtime);
    phase = motor.state.control_phase;
    if (motor.state.system_fault != 0U) return;

#if (FOC_SPECIAL_PHASE_ABORT_ENABLE == FOC_CFG_ENABLE)
    if (phase != FOC_CONTROL_PHASE_NORMAL)
    {
        if ((motor.state.motor_enabled == 0U) ||
            (motor.state.control_mode != motor.mode_transition.prev_control_mode_check))
        {
            FOC_App_AbortSpecialPhase();
            phase = FOC_CONTROL_PHASE_NORMAL;
        }
    }
#endif

#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE) && (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    Sensor_ReadEncoder(&motor.sensor, FOC_CONTROL_DT_SEC);
#endif
    Sensor_ReadVBUS(&motor.sensor);


#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((motor.sensor.adc_valid == 0U) || (motor.sensor.encoder_valid == 0U))
#else
    if (motor.sensor.adc_valid == 0U)
#endif
    {
        motor.state.sensor_invalid_consecutive++;
        if (motor.state.control_skip_count < UINT32_MAX)
        {
            motor.state.control_skip_count++;
        }
        motor.state.last_fault_code = (motor.sensor.adc_valid == 0U) ?
            (uint8_t)FOC_FAULT_SENSOR_ADC_INVALID : (uint8_t)FOC_FAULT_SENSOR_ENCODER_INVALID;

        if (motor.state.sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
            FOC_ControlExecutor_OnCycleResult(&motor, (uint8_t)FOC_CYCLE_FAULT_SENSOR);
        return;
    }
    motor.state.sensor_invalid_consecutive = 0U;
    motor.state.last_fault_code = (uint8_t)FOC_FAULT_NONE;


#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
      if (motor.sensor.vbus.filtered < FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        motor.state.last_fault_code = (uint8_t)FOC_FAULT_UNDERVOLTAGE;
        FOC_ControlExecutor_OnCycleResult(&motor, (uint8_t)FOC_CYCLE_FAULT_UVLO);
        return;
    }
#endif


    switch (phase)
    {
    case FOC_CONTROL_PHASE_NORMAL:
        if (motor.state.motor_enabled == 0U) return;
        cycle_result = FOC_ControlExecutor_RunCycle(&motor, FOC_CONTROL_DT_SEC);
        FOC_ControlExecutor_OnCycleResult(&motor, cycle_result);
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

    /* 控制过程末尾：单点发布控制参考，供电流环 ISR 原子获取 */
    FOC_ControlExecutor_PublishControlRef(&motor);
}

void FOC_App_OnPwmUpdateISR(void)
{
    if (motor.state.system_fault != 0U)
    {
        FOC_ControlExecutor_FullStop(&motor);
        return;
    }

    if (motor.state.system_running == 0U) return;

#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    {
        uint32_t pwm_start = FOC_Platform_ReadCycleCounter();
        FOC_ControlExecutor_RunISR_PwmOnly(&motor);
        motor.isr_timing.pwm_isr_cycles = FOC_Platform_ReadCycleCounter() - pwm_start;
    }
#else
    FOC_ControlExecutor_RunISR(&motor);
#endif

#if (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE) && (FOC_CURRENT_LOOP_ISR_MODE != FOC_ISR_MODE_3ISR)
    DebugStream_CaptureOscSnapshot(&g_sys.runtime.monitor.stream, &motor);
#endif
}

#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
void FOC_App_OnCurrentLoopISR(void)
{
    if (motor.state.system_fault != 0U) return;
    if (motor.state.system_running == 0U) return;

    FOC_ControlExecutor_RunISR_CurrentLoop(&motor);

#if (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE)
    DebugStream_CaptureOscSnapshot(&g_sys.runtime.monitor.stream, &motor);
#endif
}
#endif
