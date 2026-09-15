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

/* 突发一次性事件通告：底层 fast 通道，主循环/ISR 均可调用。
 * 约束：仅突发/一次性信息使用；文本须简短、关键字段前置、允许尾部截断；
 *      常规长文本禁用（走主循环慢路径）。 */
void FOC_OutputMgr_WriteFastEvent(const char *text);

/* 轮询所有通信源帧数据，入 RX 队列 */
void FOC_OutputMgr_PollSources(foc_system_t *sys);

/* 输出启动信息 */
void FOC_OutputMgr_WriteStartupInfo(foc_motor_t *motor);

/* 处理 Monitor 元素队列（元素出队→格式化→入 TX 队列） */
void FOC_OutputMgr_ProcessMonitorElements(foc_system_t *sys);

#endif /* FOC_OUTPUT_MGR_H */
