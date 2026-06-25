#ifndef FOC_SYSTEM_TYPES_H
#define FOC_SYSTEM_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L2/Runtime/foc_scheduler_types.h"
#include "L2/Runtime/foc_queue.h"
#include "L2/Runtime/foc_debug_stream.h"
#include "L2/Protocol/foc_snapshot_types.h"
#include "L2/foc_ctrl_types.h"

/*
 * ================================================================
 * 系统配置（不随 reinit 重置）
 *
 * 协议遥测策略、报告模式等配置跨越电机 reinit 保持。
 * ================================================================
 */
typedef struct foc_system_cfg {
    telemetry_policy_snapshot_t telemetry;  /* 遥测策略（协议设置） */
    uint8_t report_mode;                    /* 报告模式 */
} foc_system_cfg_t;

/*
 * ================================================================
 * 调度器运行状态
 * ================================================================
 */
typedef struct {
    uint16_t tick_counter;
    uint32_t execution_cycles;
    void (*callbacks[FOC_TASK_RATE_COUNT])(void);
    uint8_t dwt_enabled;
} control_scheduler_t;

/*
 * ================================================================
 * 系统运行时状态（每次 reinit 重置）
 *
 * 调度器、调试流、指示器等运行时可变状态。
 * ================================================================
 */
typedef struct {
    /* 调度器 */
    control_scheduler_t scheduler;

    /* 调试流 */
    debug_stream_state_t debug_stream;

    /* 任务触发标志 */
    volatile uint8_t service_task_pending;
    volatile uint8_t monitor_task_pending;

    /* 指示器 */
    struct {
        uint16_t comm_pulse_counter;
        uint16_t led_run_blink_counter;
    } indicator;

    /* 通信轮询公平调度：下一次轮询起始源索引（round-robin 偏移） */
    uint8_t comm_source_rr;

    /* RX 帧队列：ISR 入队帧数据，主循环出队解析 */
    fifo_queue_t rx_fifo;
    uint8_t rx_fifo_buffer[FOC_RX_QUEUE_DEPTH][PROTOCOL_PARSER_RX_MAX_LEN];

    /* TX 文本队列：主循环入队调试/协议行，主循环同周期出队发送 */
    fifo_queue_t tx_fifo;
    uint8_t tx_fifo_buffer[FOC_OUTPUT_QUEUE_DEPTH][FOC_OUTPUT_FRAME_MAX_LEN];
} foc_runtime_ctx_t;

/*
 * ================================================================
 * 系统顶层聚合
 * ================================================================
 */
typedef struct {
    foc_system_cfg_t cfg;        /* 持久配置 */
    foc_runtime_ctx_t runtime;   /* 运行时状态 */
} foc_system_t;

#endif /* FOC_SYSTEM_TYPES_H */
