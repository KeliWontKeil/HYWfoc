#include "L1_Orchestration/foc_app.h"

#include <stdio.h>
#include <string.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L1_Orchestration/foc_service_handler.h"
#include "L1_Orchestration/foc_output_mgr.h"
#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_task_scheduler.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Control/foc_ctrl_executor.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_init.h"
#include "L2_Core/Control/foc_ctrl_cogging_calib.h"
#include "L2_Core/Control/foc_ctrl_reinit.h"
#include "L2_Core/Protocol/foc_protocol_handler.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 全局实例
 * ================================================================ */

static foc_system_t g_sys;
static foc_motor_t motor;

/* 示波器行累积缓冲区（主循环用） */
static char g_osc_collect_buf[DEBUG_STREAM_OSC_PAYLOAD_LEN];
static uint16_t g_osc_collect_offset;

/* ================================================================
 * 内部工具
 * ================================================================ */

/* 调度器 tick 回调桥接（C 函数指针，无参数）
 * 在 RunTick 之后立即保存本 ISR 总执行周期数到 debug_stream，
 * 确保 MonitorTrigger 后续读到的 exec_time 是刚完成的本轮 ISR 的值。
 */
static void FOC_App_SchedTickBridge(void)
{
    ControlScheduler_RunTick(&g_sys.runtime.scheduler);
    DebugStream_SetExecutionCycles(&g_sys.runtime.debug_stream,
        ControlScheduler_GetExecutionCycles(&g_sys.runtime.scheduler));
}

static void FOC_App_UpdateIndicators(void)
{
    FOC_Service_UpdateIndicators(&motor, &g_sys.runtime);
}

/* 启动信息输出 */
static void FOC_App_OutputStartupInfo(void)
{
    char startup_info[160];
    snprintf(startup_info, sizeof(startup_info),
             "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, set_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
             (double)motor.mech_angle_at_elec_zero_rad,
             (int)motor.direction,
             (int)motor.pole_pairs,
             (double)motor.vbus_voltage,
             (double)motor.set_voltage,
             (double)(motor.vbus_voltage > 0.0f ? motor.set_voltage / motor.vbus_voltage : 0.0f),
             (double)motor.sensor.vbus_voltage_filtered);
    FOC_OutputMgr_WriteDirect(startup_info);
}

/* 4 源公平轮询：从 round-robin 起始偏移遍历所有通信源，入队最多一帧 */
static void FOC_App_PollCommSources(void)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    uint16_t len;
    uint8_t start = g_sys.runtime.comm_source_rr;
    uint8_t max_frames;

    max_frames = (FOC_COMM_MAX_FRAMES_PER_SERVICE == 0U) ? 4U : FOC_COMM_MAX_FRAMES_PER_SERVICE;
    if (max_frames > 4U) max_frames = 4U;

    for (uint8_t i = 0; i < 4U; i++)
    {
        uint8_t idx = (uint8_t)((start + i) % 4U);

        switch (idx)
        {
        case 0U: len = FOC_Platform_CommSource1_ReadFrame(frame, sizeof(frame)); break;
        case 1U: len = FOC_Platform_CommSource2_ReadFrame(frame, sizeof(frame)); break;
        case 2U: len = FOC_Platform_CommSource3_ReadFrame(frame, sizeof(frame)); break;
        case 3U: len = FOC_Platform_CommSource4_ReadFrame(frame, sizeof(frame)); break;
        default: len = 0U; break;
        }

        if (len == 0U) continue;

        if (FIFO_Enqueue(&g_sys.runtime.rx_fifo, frame) != 0U)
        {
            g_sys.runtime.comm_source_rr = (uint8_t)((idx + 1U) % 4U);
        }

        /* 每 ServiceTrigger 处理的帧数上限 */
        if (i >= (max_frames - 1U)) break;
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

    /* 初始化系统运行时 */
    g_sys.runtime.service_task_pending = 0U;
    g_sys.runtime.monitor_task_pending = 0U;
    g_sys.runtime.monitor_frame_active = 0U;
    g_sys.runtime.indicator.comm_pulse_counter = 0U;
    g_sys.runtime.indicator.led_run_blink_counter = 0U;
    g_sys.runtime.comm_source_rr = 0U;

    /* 调度器初始化 */
    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init(&g_sys.runtime.scheduler);
    FOC_Platform_SetControlTickCallback(FOC_App_SchedTickBridge);

    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_SERVICE, FOC_App_ServiceTrigger);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_FAST_CONTROL, FOC_App_ControlTrigger);
    ControlScheduler_SetCallback(&g_sys.runtime.scheduler, FOC_TASK_RATE_MONITOR, FOC_App_MonitorTrigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    /* 通讯初始化 */
    FOC_Platform_CommInit();

    /* 输出管理器初始化 */
    FOC_OutputMgr_Init(&g_sys);

    /* Monitor 元素队列初始化 */
    FIFO_Init(&g_sys.runtime.monitor_elem_q,
              (uint8_t *)g_sys.runtime.monitor_elem_buffer,
              sizeof(monitor_element_t),
              FOC_MONITOR_ELEM_QUEUE_DEPTH);

    /* 协议初始化（传入遥测策略） */
    FOC_Protocol_Init(&g_sys.cfg.telemetry);

    /* 调试流初始化 */
    DebugStream_Init(&g_sys.runtime.debug_stream);

    /* L2 硬件初始化收口：传感器/SVPWM/快速电流环 */
    FOC_ControlPlatform_InitHardware(&motor);

    /* PWM 更新回调注册（回调指向 L1 的 ISR 桥接） */
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    /* 电机初始化（含控制初始化、阻塞标定等） */
    FOC_Service_InitMotor(&motor);
    FOC_Control_ApplyConfig(&motor);

    /* 初始化检查完整性校验 */
    FOC_Service_VerifyInitChecks(&motor, &motor.sensor);

    /* 启动信息输出 */
    FOC_App_OutputStartupInfo();

    FOC_App_UpdateIndicators();
}

void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlRuntimeInterrupts(1U);
}

/* ================================================================
 * 主循环（由 main() 持续调用）
 * ================================================================ */

void FOC_App_Loop(void)
{
    /* Monitor 任务：从 monitor_elem_q 出队 → 格式化 → 入 TX FIFO
     * 检查 monitor_frame_active：若 ISR 正在写入帧，跳过本轮消费，
     * 避免读取半帧导致 tag/value 错位。
     */
    if (g_sys.runtime.monitor_task_pending != 0U)
    {
        g_sys.runtime.monitor_task_pending = 0U;

        if (g_sys.runtime.monitor_frame_active != 0U)
        {
            /* ISR 正在写入帧，下一轮再消费 */
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
            {
                break;
            }
            consumed++;

            if (elem.tag == MONITOR_ELEM_FRAME_START)
            {
                /* 新帧开始：丢弃上一帧未完成的残余 */
                collecting_osc = 0U;
                in_frame = 1U;
                continue;
            }

            if (!in_frame)
            {
                /* 安全兜底：未遇到 FRAME_START 的元素丢弃 */
                continue;
            }

            /* ---- 语义行 ---- */
            if (elem.tag <= MONITOR_ELEM_SEMANTIC_7)
            {
                char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];

                if (elem.aux == 0U)
                {
                    /* 无效行，输出状态行 */
                    uint8_t idx = elem.tag - MONITOR_ELEM_SEMANTIC_0;
                    switch (idx)
                    {
                    case 0U:
                        snprintf(line, sizeof(line),
                            "measurement.current.status=invalid\r\n");
                        break;
                    case 3U:
                        snprintf(line, sizeof(line),
                            "measurement.encoder.status=invalid\r\n");
                        break;
                    case 5U:
                        snprintf(line, sizeof(line),
                            "measurement.vbus.status=invalid\r\n");
                        break;
                    default:
                        continue;
                    }
                }
                else
                {
                    DebugStream_FormatSemanticLine(elem.tag, elem.value,
                                                    line, sizeof(line));
                }

                (void)FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)line);
                continue;
            }

            /* 语义帧结束 */
            if (elem.tag == MONITOR_ELEM_SEMANTIC_END)
            {
                in_frame = 0U;
                continue;
            }

            /* ---- 示波器值 (累积) ---- */
            if (elem.tag == MONITOR_ELEM_OSC_VALUE)
            {
                if (collecting_osc == 0U)
                {
                    g_osc_collect_offset = 0U;
                    g_osc_collect_buf[0] = '\0';
                    collecting_osc = 1U;
                }
                DebugStream_AppendOscValue(g_osc_collect_buf,
                                            &g_osc_collect_offset,
                                            elem.value);
                continue;
            }

            /* 示波器行结束 */
            if (elem.tag == MONITOR_ELEM_OSC_END)
            {
                char line[DEBUG_STREAM_OSC_PAYLOAD_LEN];
                uint16_t off = 0U;
                int written;

                written = snprintf(line, sizeof(line), "%c",
                                   (char)DEBUG_STREAM_OSC_HEAD_BYTE);
                if (written > 0) off = (uint16_t)written;

                /* 追加已累积的 value */
                {
                    uint16_t copy_len = (sizeof(line) > off + 1U) ?
                                         (uint16_t)(sizeof(line) - off - 1U) : 0U;
                    uint16_t src_len = (uint16_t)strlen(g_osc_collect_buf);
                    if (copy_len > src_len) copy_len = src_len;
                    if (copy_len > 0U)
                    {
                        (void)memcpy(line + off, g_osc_collect_buf, copy_len);
                        off += copy_len;
                        line[off] = '\0';
                    }
                }

                /* 追加尾字节 */
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

            /* ---- 协议摘要行 ---- */
            if (elem.tag == MONITOR_ELEM_PROTOCOL_SUMMARY)
            {
                char line[COMMAND_MANAGER_REPLY_BUFFER_LEN];

                snprintf(line, sizeof(line),
                         "STATE RUN=%u FLT=%u INIT=0x%04X/0x%04X "
                         "SENS_INV=%u PROTO_ERR=%lu PARAM_ERR=%lu "
                         "CTRL_SKIP=%lu\r\n",
                         (unsigned int)motor.state.system_running,
                         (unsigned int)motor.state.system_fault,
                         (unsigned int)motor.state.init_check_mask,
                         (unsigned int)motor.state.init_fail_mask,
                         (unsigned int)motor.state.sensor_invalid_consecutive,
                         (unsigned long)motor.state.protocol_error_count,
                         (unsigned long)motor.state.param_error_count,
                         (unsigned long)motor.state.control_skip_count);

                (void)FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)line);
                in_frame = 0U;
                continue;
            }
        }
    }

    /* Service 任务：从 RX 队列取帧 → 协议处理 → 编排结果 */
    if (g_sys.runtime.service_task_pending != 0U)
    {
        g_sys.runtime.service_task_pending = 0U;

        while (FIFO_Count(&g_sys.runtime.rx_fifo) > 0U)
        {
            uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
            foc_protocol_frame_result_t result;
            uint16_t len;

            (void)FIFO_Dequeue(&g_sys.runtime.rx_fifo, frame);
            len = PROTOCOL_PARSER_RX_MAX_LEN;

            result = FOC_Protocol_ProcessSingle(&motor, frame, len);

            if (result.comm_active != 0U)
            {
                g_sys.runtime.indicator.comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;
            }

            if (result.needs_summary != 0U)
            {
                /*
                 * 协议摘要行通过 monitor_elem_q 统一输出，
                 * 避免 Service 段直入 TX FIFO 与 Monitor 段交错。
                 */
                monitor_element_t elem;
                elem.tag   = MONITOR_ELEM_PROTOCOL_SUMMARY;
                elem.aux   = 0U;
                elem.value = 0.0f;

                (void)FIFO_Enqueue(&g_sys.runtime.monitor_elem_q,
                                   (uint8_t *)&elem);
                g_sys.runtime.monitor_task_pending = 1U;
            }

            /* status 已在 ProcessSingle 内部直写，L1 无需处理 */
            /* param_changed 由 cfg_dirty 检查处理 */
        }

        /* 配置脏检查 */
        FOC_Service_ApplyCfgDirty(&motor);

        /* 数据输出动作（dump/export） */
        (void)FOC_Service_Process(&motor);
    }

    /* 输出队列消费任务（主循环唯一阻塞点） */
    FOC_OutputMgr_FlushQueue(&g_sys);
}

/* ================================================================
 * 回调桥接
 * ================================================================ */

void FOC_App_ServiceTrigger(void)
{
    FOC_App_UpdateIndicators();

    /* ISR 上下文中轮询所有通信源，入队到 RX 队列 */
    FOC_App_PollCommSources();

    g_sys.runtime.service_task_pending = 1U;
}

void FOC_App_MonitorTrigger(void)
{
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))

    /* ISR 上下文：按需读取 motor 字段 + 逐元素入 monitor_elem_q */

    g_sys.runtime.monitor_frame_active = 1U;

    /* Push 帧起始标记 */
    {
        monitor_element_t start_elem;
        start_elem.tag   = MONITOR_ELEM_FRAME_START;
        start_elem.aux   = 0U;
        start_elem.value = 0.0f;
        (void)FIFO_Enqueue(&g_sys.runtime.monitor_elem_q,
                           (uint8_t *)&start_elem);
    }

    /* Poll 所有元素并入队 */
    {
        monitor_element_t elem;
        while (DebugStream_PollNextValue(&g_sys.runtime.debug_stream,
                                          &motor,
                                          FOC_Protocol_GetTelemetry(),
                                          &elem) != 0U)
        {
            (void)FIFO_Enqueue(&g_sys.runtime.monitor_elem_q,
                               (uint8_t *)&elem);
        }
    }

    g_sys.runtime.monitor_frame_active = 0U;
    g_sys.runtime.monitor_task_pending = 1U;
#else
    /* Both semantic and osc reports disabled: no need to wake Monitor task. */
#endif
}

void FOC_App_ControlTrigger(void)
{
    uint8_t phase;
    uint8_t cycle_result = FOC_CYCLE_OK;

    phase = motor.state.control_phase;

    /* L1 系统级屏障：故障状态下不触发任何控制 */
    if (motor.state.system_fault != 0U) return;

    /* === 公共安全检查 === */

    /* 编码器角度采样：按 FOC_SENSOR_ANGLE_FAST_ENABLE 决定读取位置 */
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    Sensor_ReadEncoder(&motor, &motor.sensor);
    /* 更新 e-cycle volatile 桥接缓存（PWM ISR 原子读取） */
#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
    motor.ecycle_ref_angle_rad = motor.sensor.mech_angle_rad.output_value;
    motor.ecycle_ref_angle_valid = motor.sensor.encoder_valid;
#endif
#else
    /* 快速编码器：角度已在 PWM ISR 中读入 motor->sensor_fast，复制到 motor->sensor */
    motor.sensor.mech_angle_rad = motor.sensor_fast.mech_angle_rad;
    motor.sensor.encoder_valid = motor.sensor_fast.encoder_valid;
#endif

    /* VBUS 采样（总是控制周期读） */
    Sensor_ReadVBUS(&motor.sensor);

    /* 电流快照同步到 motor->sensor（供 DebugStream 等消费者） */
    Sensor_SyncCurrentSnapshot(&motor);

    /* 传感器有效性检查 */
    if ((motor.sensor.adc_valid == 0U) || (motor.sensor.encoder_valid == 0U))
    {
        motor.state.sensor_invalid_consecutive++;
        motor.state.control_skip_count++;

        if (motor.sensor.adc_valid == 0U)
            motor.state.last_fault_code = (uint8_t)FOC_FAULT_SENSOR_ADC_INVALID;
        else
            motor.state.last_fault_code = (uint8_t)FOC_FAULT_SENSOR_ENCODER_INVALID;

        if (motor.state.sensor_invalid_consecutive >= FOC_DIAG_SENSOR_FAULT_THRESHOLD)
        {
            cycle_result = (uint8_t)FOC_CYCLE_FAULT_SENSOR;
            FOC_Service_HandleControlResult(&motor, cycle_result);
        }
        return;
    }
    motor.state.sensor_invalid_consecutive = 0U;
    motor.state.last_fault_code = (uint8_t)FOC_FAULT_NONE;

    /* 欠压保护 */
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (motor.sensor.vbus_voltage_filtered < FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        motor.state.last_fault_code = (uint8_t)FOC_FAULT_UNDERVOLTAGE;
        cycle_result = (uint8_t)FOC_CYCLE_FAULT_UVLO;
        FOC_Service_HandleControlResult(&motor, cycle_result);
        return;
    }
#endif

    /* === 阶段路由 === */
    switch (phase)
    {
    case FOC_CONTROL_PHASE_NORMAL:
    {
        /* NORMAL 控制：仅在电机使能时执行 */
        if (motor.state.motor_enabled == 0U)
        {
            return;
        }

        /* 委托 L2 执行正常控制循环，传入已采样的传感器快照 */
        cycle_result = FOC_ControlExecutor_RunCycle(&motor, &motor.sensor, FOC_CONTROL_DT_SEC);

        /* L1 根据执行结果更新系统状态 */
        FOC_Service_HandleControlResult(&motor, cycle_result);
        break;
    }

    case FOC_CONTROL_PHASE_COGGING_CALIB:
    {
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        (void)FOC_CoggingCalib_RunStep(&motor, &motor.sensor, FOC_CONTROL_DT_SEC);
        /* 标定状态机内部在完成时自动将 control_phase 切回 NORMAL */
#endif
        break;
    }

    case FOC_CONTROL_PHASE_REINIT:
    {
#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
        (void)FOC_ReInit_RunStep(&motor, FOC_CONTROL_DT_SEC);
#endif
        /* 重初始化状态机内部在完成时自动将 control_phase 切回 NORMAL */
        break;
    }

    default:
        break;
    }
}

void FOC_App_OnPwmUpdateISR(void)
{
    FOC_ControlExecutor_RunISR(&motor);
}
