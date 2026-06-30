#ifndef FOC_OUTPUT_MGR_H
#define FOC_OUTPUT_MGR_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L1_Orchestration/foc_system_types.h"

/*
 * ================================================================
 * 输出管理器（L1）
 *
 * 队列操作（WriteQueue/FlushQueue）内部使用 L2/Runtime FIFO 模块。
 * L2 层不再调用本模块函数，所有输出数据通过返回值/回调由 L1 编排。
 * ================================================================
 */

void FOC_OutputMgr_Init(foc_system_t *sys);
void FOC_OutputMgr_FlushQueue(foc_system_t *sys);
uint8_t FOC_OutputMgr_GetOverflowCount(const foc_system_t *sys);

/* 直写（无缓冲，通过 L3 平台 API 立即输出） */
void FOC_OutputMgr_WriteDirect(const char *text);

/* 写状态字节（直写） */
void FOC_OutputMgr_WriteStatus(uint8_t status);

/* 轮询所有通信源帧数据，入 RX 队列 */
void FOC_OutputMgr_PollSources(foc_system_t *sys);

/* 输出启动信息 */
void FOC_OutputMgr_WriteStartupInfo(foc_motor_t *motor);

#endif /* FOC_OUTPUT_MGR_H */
