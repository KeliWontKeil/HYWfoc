#ifndef FOC_DEBUG_STREAM_H
#define FOC_DEBUG_STREAM_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Protocol/foc_snapshot_types.h"
#include "L2_Core/Runtime/foc_scheduler_types.h"
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
} debug_stream_state_t;

/*
 * L2/Runtime — 调试流生成器
 *
 * 本模块只负责"判断是否需要输出 + 生成一行文本"，
 * **不碰任何队列操作**。生成的文本由 L1 决定是否入 TX 队列。
 *
 * 生成模式：每调用一次 DebugStream_GenerateLine 最多返回一行文本。
 * L1 在 Monitor 任务中循环调用直到返回 0，每次得到一行则入队。
 */

void DebugStream_Init(debug_stream_state_t *ds);
void DebugStream_SetExecutionCycles(debug_stream_state_t *ds, uint32_t exec_cycles);

/* 生成一行输出文本。
 * ds —— 调试流状态（内部维护行索引以逐行输出语义遥测）
 * sensor/motor/telemetry —— 数据源
 * line_out —— 输出缓冲区
 * line_max —— 输出缓冲区大小
 * 返回 1 = 成功填充 line_out，0 = 本轮无更多输出
 */
uint8_t DebugStream_GenerateLine(debug_stream_state_t *ds,
                                  const sensor_data_t *sensor,
                                  const foc_motor_t *motor,
                                  const telemetry_policy_snapshot_t *telemetry,
                                  char *line_out,
                                  uint16_t line_max);

#endif /* FOC_DEBUG_STREAM_H */