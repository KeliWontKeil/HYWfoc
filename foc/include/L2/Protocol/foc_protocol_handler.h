#ifndef FOC_PROTOCOL_HANDLER_H
#define FOC_PROTOCOL_HANDLER_H

#include <stdint.h>

#include "L2/foc_ctrl_types.h"
#include "L2/Runtime/foc_runtime_types.h"
#include "L2/Protocol/foc_snapshot_types.h"

/* 处理一帧通讯（从通讯源读取帧→解析→执行→写入motor）
 * 返回 1=有通讯活动，0=无帧
 */
uint8_t FOC_Protocol_Process(foc_motor_t *motor, uint8_t frame_budget);

/* 初始化协议内部状态
 * cfg — 系统遥测策略指针（L1 提供的不随 reinit 重置的配置空间）
 */
void FOC_Protocol_Init(telemetry_policy_snapshot_t *telemetry_cfg);

/* 清除配置脏标志（L1应用配置后调用） */
void FOC_Protocol_Commit(foc_motor_t *motor);

/* 获取遥测策略（供调试流使用） */
const telemetry_policy_snapshot_t *FOC_Protocol_GetTelemetry(void);

#endif /* FOC_PROTOCOL_HANDLER_H */