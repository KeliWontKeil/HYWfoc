#ifndef FOC_DEBUG_STREAM_H
#define FOC_DEBUG_STREAM_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Protocol/foc_snapshot_types.h"
#include "L2_Core/Runtime/foc_scheduler_types.h"
#include "L1_Orchestration/foc_monitor_queue_types.h"
#include "LS_Config/foc_config.h"

/*
 * 调试流运行时状态（实例由 L1 在 foc_runtime_ctx_t 中分配）
 */
typedef struct {
    uint16_t semantic_report_counter;
    uint16_t osc_report_counter;
    uint16_t semantic_last_freq_hz;
    uint16_t osc_last_freq_hz;
    uint16_t semantic_period_ticks;
    uint16_t osc_period_ticks;
    uint32_t last_exec_cycles;
    uint8_t  line_index;
    uint8_t  osc_param_bit_idx;   /* 当前示波器掩码位索引 */
} debug_stream_state_t;

/*
 * L2/Runtime — 调试流生成器
 *
 * 本模块提供两个层级的接口：
 *
 * 1. ISR 级（L1 MonitorTrigger 调用）
 *    DebugStream_PollNextValue — 跑 state machine，决定下一个要输出的值
 *    以及它的 tag。调用者循环调用直到返回 0。
 *
 * 2. 主循环级（L1 FOC_App_Loop Monitor 段调用）
 *    DebugStream_FormatSemanticLine — 格式化一条语义行 → 文本
 *    DebugStream_FormatOscLine     — 根据之前 append 的值组装示波器帧
 *    DebugStream_AppendOscValue   — 追加一个示波器参数值
 *
 * 原有 DebugStream_GenerateLine 保留为向后兼容的封装。
 */

void DebugStream_Init(debug_stream_state_t *ds);
void DebugStream_SetExecutionCycles(debug_stream_state_t *ds, uint32_t exec_cycles);

/* ---- ISR 级接口 ---- */

/* 获取下一个待输出的 Monitor 元素。
 * 调用者循环调用直到返回 0。
 * 返回 0 = 本轮无更多输出，1 = elem_out 有效。
 *
 * 参数 motor 是 ISR 中可直接访问的电机状态，
 * 本函数按需读取各字段，不做整体快照。
 */
uint8_t DebugStream_PollNextValue(debug_stream_state_t *ds,
                                   const foc_motor_t *motor,
                                   const telemetry_policy_snapshot_t *telemetry,
                                   monitor_element_t *elem_out);

/* ---- 主循环级接口 ---- */

/* 格式化一条语义行 */
void DebugStream_FormatSemanticLine(uint8_t tag, float value,
                                     char *line_out, uint16_t line_max);

/* 追加一个示波器参数值（内部维护 buffer，由 FormatOscLine 完成） */
void DebugStream_AppendOscValue(char *osc_buffer, uint16_t *offset,
                                 float value);

/* 组装示波器行（加头尾标记），完成行。返回写入的总字符数。 */
uint16_t DebugStream_FormatOscLine(char *osc_buffer, uint16_t max_len);

/* ---- 向后兼容接口 ---- */

uint8_t DebugStream_GenerateLine(debug_stream_state_t *ds,
                                  const foc_motor_t *motor,
                                  const telemetry_policy_snapshot_t *telemetry,
                                  char *line_out,
                                  uint16_t line_max);

#endif /* FOC_DEBUG_STREAM_H */
