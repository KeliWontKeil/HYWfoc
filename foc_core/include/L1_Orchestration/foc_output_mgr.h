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

/* 轮询所有通信源帧数据，入 RX 队列 */
void FOC_OutputMgr_PollSources(foc_system_t *sys);

/* 输出启动信息 */
void FOC_OutputMgr_WriteStartupInfo(foc_motor_t *motor);

/* 输出 fault 完整详情（主循环 fault 0→1 跃迁补发；慢路径可靠文本） */
void FOC_OutputMgr_WriteFaultReport(const foc_motor_t *motor);

/* 处理 Monitor 元素队列（元素出队→格式化→入 TX 队列） */
void FOC_OutputMgr_ProcessMonitorElements(foc_system_t *sys);

#endif /* FOC_OUTPUT_MGR_H */
