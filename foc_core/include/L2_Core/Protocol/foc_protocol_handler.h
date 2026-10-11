#ifndef FOC_PROTOCOL_HANDLER_H
#define FOC_PROTOCOL_HANDLER_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Protocol/foc_protocol_types.h"
#include "L2_Core/Runtime/foc_queue.h"

/*
 * L2/Protocol — 协议处理器
 *
 * 本模块只负责"接收单帧→解析→修改 motor 字段→返回结果"，
 * **不碰任何队列操作**。帧数据由 L1 从 RX 队列取出后传入。
 * 输出通过两种方式：
 *   快路径 — 状态码/参数行/报错等短数据在协议内部直写 L3 平台 API
 *   慢路径 — 摘要等大数据通过返回值告诉 L1，由 L1 入 TX 队列
 */

/* 处理一帧数据（纯解析 + 执行）
 * motor — 电机对象指针
 * frame — 原始帧数据（已从 RX 队列取出）
 * len   — 帧长度
 * 返回处理结果，L1 据此编排后续动作
 */
foc_protocol_frame_result_t FOC_Protocol_ProcessSingle(
    foc_motor_t *motor,
    foc_report_config_t *report,
    const uint8_t *frame,
    uint16_t len);

/* 写入报告配置默认值（L1 初始化时调用）。
 * 协议层不持有 report 指针（无模块级静态状态），运行期由调用方逐次显式传入。 */
void FOC_Protocol_InitReport(foc_report_config_t *report);

/* 批量输出（入 TX FIFO，供 X 指令使用） */
void FOC_Protocol_QueueParams(const foc_motor_t *motor, const foc_report_config_t *report, fifo_queue_t *tx_fifo);
void FOC_Protocol_QueueConfigs(const foc_motor_t *motor, fifo_queue_t *tx_fifo);
void FOC_Protocol_QueueStates(const foc_motor_t *motor, const foc_report_config_t *report, fifo_queue_t *tx_fifo);
void FOC_Protocol_QueueSystemInfo(const foc_motor_t *motor, fifo_queue_t *tx_fifo);

#endif /* FOC_PROTOCOL_HANDLER_H */
