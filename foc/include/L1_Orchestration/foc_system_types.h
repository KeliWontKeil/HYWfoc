#ifndef FOC_SYSTEM_TYPES_H
#define FOC_SYSTEM_TYPES_H

#include <stdint.h>

#include "L2/Runtime/foc_scheduler_types.h"
#include "L2/Protocol/foc_snapshot_types.h"
#include "L2/foc_ctrl_types.h"

#ifndef FOC_OUTPUT_QUEUE_DEPTH
#define FOC_OUTPUT_QUEUE_DEPTH      8U
#endif

#ifndef FOC_OUTPUT_FRAME_MAX_LEN
#define FOC_OUTPUT_FRAME_MAX_LEN    96U
#endif

/* ========== 输出队列类型 ========== */
typedef struct {
    char buffer[FOC_OUTPUT_QUEUE_DEPTH][FOC_OUTPUT_FRAME_MAX_LEN];
    uint8_t write_idx;
    uint8_t read_idx;
    uint8_t count;
    uint8_t overflow_count;
} foc_output_queue_t;

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
 * 调试流状态
 * ================================================================
 */
typedef struct {
    uint16_t semantic_report_counter;
    uint16_t osc_report_counter;
    uint16_t semantic_last_freq_hz;
    uint16_t osc_last_freq_hz;
    uint16_t semantic_period_ticks;
    uint16_t osc_period_ticks;
    uint32_t last_exec_cycles;
} debug_stream_state_t;

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
        uint8_t  led_run_on;
        uint16_t led_run_blink_counter;
    } indicator;

    /* 输出队列 */
    foc_output_queue_t output_queue;
} foc_runtime_ctx_t;

/*
 * ================================================================
 * 系统顶层聚合
 * ================================================================
 */
typedef struct {
    foc_system_cfg_t cfg;        /* 持久配置 */
    foc_runtime_ctx_t runtime;   /* 运行时状态 */
    foc_motor_t motor;           /* 电机实例 */
} foc_system_t;

#endif /* FOC_SYSTEM_TYPES_H */