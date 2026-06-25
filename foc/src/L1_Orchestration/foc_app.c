#include "L1_Orchestration/foc_app.h"

#include <stdio.h>

#include "L1_Orchestration/foc_system_types.h"
#include "L1_Orchestration/foc_service_handler.h"
#include "L1_Orchestration/foc_output_mgr.h"
#include "L2/Runtime/foc_queue.h"
#include "L3/foc_platform_api.h"
#include "L2/Control/foc_ctrl_executor.h"
#include "L2/Control/foc_ctrl_cfg.h"
#include "L2/Control/foc_ctrl_init.h"
#include "L2/Protocol/foc_protocol_handler.h"
#include "L2/Runtime/foc_task_scheduler.h"
#include "L2/Runtime/foc_debug_stream.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 全局实例
 * ================================================================ */

static foc_system_t g_sys;
static foc_motor_t motor;

/* ================================================================
 * 内部工具
 * ================================================================ */

/* 调度器 tick 回调桥接（C 函数指针，无参数） */
static void FOC_App_SchedTickBridge(void)
{
    ControlScheduler_RunTick(&g_sys.runtime.scheduler);
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

    /* 协议初始化（传入遥测策略） */
    FOC_Protocol_Init(&g_sys.cfg.telemetry);

    /* 调试流初始化 */
    DebugStream_Init(&g_sys.runtime.debug_stream);

    /* L2 硬件初始化收口：传感器/SVPWM/快速电流环 */
    FOC_ControlPlatform_InitHardware(&motor);

    /* PWM 更新回调注册（回调指向 L1 的 ISR 桥接） */
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    /* 电机初始化（含控制初始化、标定等） */
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
    /* Monitor 任务：调试流生成 + 入队 */
    if (g_sys.runtime.monitor_task_pending != 0U)
    {
        g_sys.runtime.monitor_task_pending = 0U;

#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
        DebugStream_SetExecutionCycles(&g_sys.runtime.debug_stream,
                                       ControlScheduler_GetExecutionCycles(&g_sys.runtime.scheduler));
        {
            char line[DEBUG_STREAM_OSC_PAYLOAD_LEN];
            while (DebugStream_GenerateLine(&g_sys.runtime.debug_stream,
                                             &motor.sensor, &motor,
                                             FOC_Protocol_GetTelemetry(),
                                             line, sizeof(line)) != 0U)
            {
                FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)line);
            }
        }
#endif
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
                char summary[COMMAND_MANAGER_REPLY_BUFFER_LEN];
                snprintf(summary, sizeof(summary),
                         "STATE RUN=%u FLT=%u INIT=0x%04X/0x%04X SENS_INV=%u PROTO_ERR=%lu PARAM_ERR=%lu CTRL_SKIP=%lu\r\n",
                         (unsigned int)motor.state.system_running,
                         (unsigned int)motor.state.system_fault,
                         (unsigned int)motor.state.init_check_mask,
                         (unsigned int)motor.state.init_fail_mask,
                         (unsigned int)motor.state.sensor_invalid_consecutive,
                         (unsigned long)motor.state.protocol_error_count,
                         (unsigned long)motor.state.param_error_count,
                         (unsigned long)motor.state.control_skip_count);
                FIFO_Enqueue(&g_sys.runtime.tx_fifo, (uint8_t *)summary);
            }

            /* status 已在 ProcessSingle 内部直写，L1 无需处理 */
            /* param_changed 由 cfg_dirty 检查处理 */
        }

        /* 配置脏检查 */
        FOC_Service_ApplyCfgDirty(&motor);

        /* 阻塞动作（reinit / pending_system_action） */
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
    g_sys.runtime.monitor_task_pending = 1U;
}

void FOC_App_ControlTrigger(void)
{
    uint8_t cycle_result;

    /* L1 系统级屏障：重初始化进行中时不触发控制 */
    if (motor.state.reinit_pending != 0U) return;

    /* L1 系统级屏障：故障状态下不触发控制（ISR 会处理安全输出） */
    if (motor.state.system_fault != 0U) return;

    /* 委托 L2 执行控制循环 */
    cycle_result = FOC_ControlExecutor_RunCycle(&motor, FOC_CONTROL_DT_SEC);

    /* L1 根据执行结果更新系统状态 */
    FOC_Service_HandleControlResult(&motor, cycle_result);
}

void FOC_App_OnPwmUpdateISR(void)
{
    FOC_ControlExecutor_RunISR(&motor);
}