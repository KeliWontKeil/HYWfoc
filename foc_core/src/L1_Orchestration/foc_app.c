#include "L2_Core/foc_motor_aggregate.h"
#include "L1_Orchestration/foc_app.h"

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
#include "L2_Core/Control/foc_ctrl_align.h"
#include "L2_Core/Control/foc_ctrl_openloop.h"
#include "L2_Core/Control/foc_ctrl_source_mgr.h"
#include "L2_Core/Control/foc_ctrl_injection.h"
#include "L2_Core/Control/foc_ctrl_acoustic.h"
#include "LS_Config/foc_ringtone_table.h"
#include "L2_Core/Protocol/foc_protocol_handler.h"
#include "L2_Core/Protocol/foc_protocol_output.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

static foc_system_t g_sys;
static foc_motor_t motor;

/* 主循环 fault 补发用本地缓存（不进 motor 状态） */
static uint8_t s_prev_system_fault = 0U;

/* STARTUP 对齐完成 → 主循环补发启动信息（ISR 内不做长文本） */
static uint8_t s_startup_done_pending = 0U;

/* 启动阶段选择：自检通过且电机参数待标定（有编码器且允许对齐）时进入 STARTUP 阶段 */
static uint8_t FOC_App_ShouldStartupAlign(const foc_motor_t *motor)
{
#if (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE) && (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((motor->state.system_running == 0U) || (motor->state.system_fault != 0U))
    {
        return 0U;
    }

    if (FOC_Control_IsMotorParamCalibrated(&motor->params) == 0U)
    {
        return 1U;
    }

    return 0U;
#else
    (void)motor;
    return 0U;
#endif
}

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
    uint8_t vbus_ok;

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

    /* 就绪前：只写配置初值，不做任何功率动作（PWM 占空比恒 0） */
    FOC_Init_Motor(&motor);

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

    /* 就绪前静态校验：母线电压门 + 通信/协议/命令/调试/PWM/传感器 */
    vbus_ok = FOC_Init_VbusGate(&motor.sensor);
    FOC_Init_Verify_Static(&motor, vbus_ok);

    /* 启动阶段选择：参数待标定 → 就绪后由 STARTUP 阶段完成对齐（含电压监护与中止能力） */
    if (FOC_App_ShouldStartupAlign(&motor) != 0U)
    {
        FOC_Align_RequestStartup(&motor);
    }
    else
    {
        motor.state.control_phase = FOC_CONTROL_PHASE_NORMAL;
        FOC_OutputMgr_WriteStartupInfo(&motor);
    }

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

/* fault 0→1 跃迁当轮，主循环补发完整详情（慢路径可靠输出；无故障音——
 * fault 保护功率输出，声学需 NORMAL/ACOUSTIC 相位） */
static void FOC_App_ReportFaultTransition(void)
{
    if ((motor.state.system_fault != 0U) && (s_prev_system_fault == 0U))
    {
        FOC_OutputMgr_WriteFaultReport(&motor);
    }
    s_prev_system_fault = motor.state.system_fault;
}

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
/* 就绪播报状态（L1 私有）：就绪沿触发；首次就绪 = 长鸣，其后进入正常态 = 两短鸣 */
static uint8_t s_ready_prev = 0U;
static uint8_t s_ready_announced = 0U;

/* 声学回报触发收口：进入 ACOUSTIC 相位（与对齐/标定同级——控制环停止输出）。
 * 仅 NORMAL/ACOUSTIC 且无 fault、已使能时接受；播放中再次调用 = 同相覆盖（重入）。 */
uint8_t FOC_App_PlayTune(uint8_t tune_id)
{
    if ((motor.state.system_fault != 0U) || (motor.state.motor_enabled == 0U))
    {
        return 0U;
    }
    if ((motor.state.control_phase != FOC_CONTROL_PHASE_NORMAL) &&
        (motor.state.control_phase != FOC_CONTROL_PHASE_ACOUSTIC))
    {
        return 0U;
    }

    if (FOC_Acoustic_PlayTune(&motor.acoustic_state, tune_id) == 0U)
    {
        return 0U;
    }

    /* 切换 control_phase 前同步自动退出检查基准，避免第一拍被误判模式变化而中止 */
    motor.mode_transition.prev_control_mode_check = motor.state.control_mode;
    motor.state.control_phase = FOC_CONTROL_PHASE_ACOUSTIC;
    return 1U;
}

void FOC_App_StopTune(void)
{
    /* 只发起停止（进入包络释放段）；相位由序列结束后在 ControlTrigger 自动回 NORMAL */
    FOC_Acoustic_Stop(&motor.acoustic_state);
}

/* 就绪播报：上电（从无到有）= 长鸣；其他状态 → 正常态（恢复 / 正常推进）= 两短鸣。
 * ACOUSTIC 视为"仍就绪"（它是通知，不改变控制态），避免曲终回转被误判为进入正常态。 */
static void FOC_App_AnnounceReadySound(void)
{
    uint8_t ready;

    ready = ((motor.state.system_running != 0U) &&
             (motor.state.system_fault == 0U) &&
             (motor.state.motor_enabled != 0U) &&
             ((motor.state.control_phase == FOC_CONTROL_PHASE_NORMAL) ||
              (motor.state.control_phase == FOC_CONTROL_PHASE_ACOUSTIC))) ? 1U : 0U;

    if ((ready != 0U) && (s_ready_prev == 0U))
    {
        uint8_t tune_id = (s_ready_announced == 0U) ? (uint8_t)FOC_RINGTONE_ID_LONG_BEEP
                                                    : (uint8_t)FOC_RINGTONE_ID_TWO_SHORT_BEEPS;

        s_ready_announced = 1U;
        (void)FOC_App_PlayTune(tune_id);
    }

    s_ready_prev = ready;
}
#endif /* FOC_ACOUSTIC_ENABLE */

void FOC_App_Loop(void)
{
    FOC_App_ReportFaultTransition();

    /* 对齐/重新对齐完成后补发启动信息（ISR 内不做长文本） */
    {
        uint8_t report_ready = FOC_Align_TakeReport(&motor);

        if (s_startup_done_pending != 0U)
        {
            s_startup_done_pending = 0U;
            report_ready = 1U;
        }

        if (report_ready != 0U)
        {
            FOC_OutputMgr_WriteStartupInfo(&motor);
        }
    }

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
    FOC_App_AnnounceReadySound();
#endif

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
    const char *aborted_msg = "abort:UNKNOWN\r\n";

    switch (motor.state.control_phase)
    {
    case FOC_CONTROL_PHASE_COGGING_CALIB:
        aborted_msg = "abort:COGGING_CALIB\r\n";
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        FOC_CoggingCalib_Abort(&motor);
#endif
        break;
    case FOC_CONTROL_PHASE_STARTUP:
        aborted_msg = "abort:STARTUP\r\n";
#if (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE)
        FOC_Align_Abort(&motor);
        /* 对齐被中止：参数可能仍未定义 → 立即判定（未定义则置初始化失败并停机） */
        (void)FOC_Init_Verify_Motor(&motor);
#endif
        break;
    case FOC_CONTROL_PHASE_REINIT:
        aborted_msg = "abort:REINIT\r\n";
#if (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE)
        FOC_Align_Abort(&motor);
#endif
        break;
    case FOC_CONTROL_PHASE_ACOUSTIC:
        aborted_msg = "abort:ACOUSTIC\r\n";
#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
        FOC_Acoustic_Reset(&motor.acoustic_state);
#endif
        break;
    default:
        break;
    }

    /* 突发通告（fast）：本函数可由 ISR(自动退出)或主循环(Y:A)调用，慢路径文本禁用于 ISR；
     * 消息为字面量，避免在 ISR 路径做 snprintf 格式化 */
    FOC_OutputMgr_WriteFastEvent(aborted_msg);
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

    /* 阶段1：传感器读取。fault 期间也持续采样，使母线电压/有效性基准不冻结——
     * 否则 fault 时采样暂停，filtered 停在欠压旧值，恢复电压后 Y:C 恢复仍被判欠压（死锁）。 */
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE) && (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    Sensor_ReadEncoder(&motor.sensor, FOC_CONTROL_DT_SEC);
#endif
    Sensor_ReadVBUS(&motor.sensor);

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


    /* 母线电压安全判定（与上电门共用单一判定入口；保护关闭时恒为安全） */
    if (FOC_Init_IsVbusSafe(&motor.sensor) == 0U)
    {
        motor.state.last_fault_code = (uint8_t)FOC_FAULT_UNDERVOLTAGE;
        FOC_ControlExecutor_OnCycleResult(&motor, (uint8_t)FOC_CYCLE_FAULT_UVLO);
        return;
    }


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

    case FOC_CONTROL_PHASE_STARTUP:
#if (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE)
        /* 上电对齐：状态机驱动，全程受本函数阶段1的电压/有效性检查监护 */
        if (FOC_Align_RunStep(&motor, FOC_CONTROL_DT_SEC) == 0U)
        {
            /* 对齐结束 → 电机参数就绪判定（就绪后判定组） */
            if (FOC_Init_Verify_Motor(&motor) != 0U)
            {
                s_startup_done_pending = 1U;
            }
        }
#endif
        break;

    case FOC_CONTROL_PHASE_REINIT:
#if (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE)
        (void)FOC_Align_RunStep(&motor, FOC_CONTROL_DT_SEC);
#endif
        break;

    case FOC_CONTROL_PHASE_ACOUSTIC:
#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
        /* 曲终（含包络释放）→ 回 NORMAL，重建控制基准防恢复突跳 */
        if (FOC_Acoustic_IsActive(&motor.acoustic_state) == 0U)
        {
            motor.state.current_loop_ready = 0U;
            motor.state.control_phase = FOC_CONTROL_PHASE_NORMAL;
            motor.mode_transition.prev_control_mode_check = motor.state.control_mode;
            FOC_ControlExecutor_FullStop(&motor);
            FOC_Control_RebuildControlBasis(&motor);
        }
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
