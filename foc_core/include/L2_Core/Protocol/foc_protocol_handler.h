#ifndef FOC_PROTOCOL_HANDLER_H
#define FOC_PROTOCOL_HANDLER_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Protocol/foc_protocol_types.h"
#include "L2_Core/Protocol/foc_snapshot_types.h"

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
 * telemetry — 遥测策略快照指针（L1 提供的不随 reinit 重置的配置空间）
 * 协议内部保存此指针用于访问遥测策略等系统配置。
 */
void FOC_Protocol_Init(telemetry_policy_snapshot_t *telemetry);

/* 清除配置脏标志（L1 应用配置后调用） */
void FOC_Protocol_Commit(foc_motor_t *motor);

/* 获取遥测策略（供 L1/debug 流使用） */
const telemetry_policy_snapshot_t *FOC_Protocol_GetTelemetry(void);

#endif /* FOC_PROTOCOL_HANDLER_H */
