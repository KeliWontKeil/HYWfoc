#ifndef FOC_SYSTEM_TYPES_H
#define FOC_SYSTEM_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Runtime/foc_runtime_types.h"
#include "L2_Core/Protocol/foc_protocol_types.h"
#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Runtime/foc_monitor_queue_types.h"

/*
 * ================================================================
 * 系统配置（不随 reinit 重置）
 *
 * 报告配置等系统级设置跨越电机 reinit 保持。
 * ================================================================
 */
typedef struct foc_system_cfg {
    foc_report_config_t report;
} foc_system_cfg_t;

typedef struct {
    volatile uint8_t service_pending;
    volatile uint8_t monitor_pending;
} foc_system_task_flags_t;

typedef struct {
    uint8_t source_rr;
    fifo_queue_t rx_fifo;
    uint8_t rx_buffer[FOC_RX_QUEUE_DEPTH][PROTOCOL_PARSER_RX_MAX_LEN];
} foc_comm_runtime_t;

typedef struct {
    fifo_queue_t tx_fifo;
    uint8_t tx_buffer[FOC_OUTPUT_QUEUE_DEPTH][FOC_OUTPUT_FRAME_MAX_LEN];
} foc_output_runtime_t;

typedef struct {
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    debug_stream_state_t stream;
    volatile uint8_t frame_active;
    fifo_queue_t elem_fifo;
    uint8_t elem_buffer[FOC_MONITOR_ELEM_QUEUE_DEPTH][sizeof(monitor_element_t)];
    char osc_collect_buf[DEBUG_STREAM_OSC_PAYLOAD_LEN];
    uint16_t osc_collect_offset;
#else
    uint8_t reserved;
#endif
} foc_monitor_runtime_t;

typedef struct {
    uint16_t comm_pulse_counter;
    uint16_t run_blink_counter;
} foc_indicator_runtime_t;

/*
 * ================================================================
 * 系统运行时状态（每次 reinit 重置）
 *
 * 调度器、任务触发标志、通信输入、输出队列、监控输出、指示器等
 * 系统级运行时可变状态。
 * ================================================================
 */
typedef struct {
    control_scheduler_t scheduler;
    foc_system_task_flags_t tasks;
    foc_comm_runtime_t comm;
    foc_output_runtime_t output;
    foc_monitor_runtime_t monitor;
    foc_indicator_runtime_t indicator;
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
