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
    const uint8_t *frame,
    uint16_t len);

/* 初始化协议内部状态
 * report — 报告配置指针（L1 提供的不随 reinit 重置的配置空间）
 * 协议内部保存此指针用于访问报告策略等系统配置。
 */
void FOC_Protocol_Init(foc_report_config_t *report);

/* 获取报告配置（供 L1/debug 流使用） */
const foc_report_config_t *FOC_Protocol_GetReportConfig(void);

/* 批量输出（入 TX FIFO，供 X 指令使用） */
void FOC_Protocol_QueueParams(const foc_motor_t *motor, fifo_queue_t *tx_fifo);
void FOC_Protocol_QueueConfigs(const foc_motor_t *motor, fifo_queue_t *tx_fifo);
void FOC_Protocol_QueueStates(const foc_motor_t *motor, fifo_queue_t *tx_fifo);
void FOC_Protocol_QueueSystemInfo(const foc_motor_t *motor, fifo_queue_t *tx_fifo);

#endif /* FOC_PROTOCOL_HANDLER_H */
