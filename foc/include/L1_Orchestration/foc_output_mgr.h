#ifndef FOC_OUTPUT_MGR_H
#define FOC_OUTPUT_MGR_H

#include <stdint.h>

#include "L1_Orchestration/foc_system_types.h"

/*
 * ================================================================
 * 双通道输出管理器
 *
 * 队列类型 foc_output_queue_t 定义在 foc_system_types.h 中。
 * ================================================================
 */

#ifndef FOC_OUTPUT_MAX_PER_CYCLE
#define FOC_OUTPUT_MAX_PER_CYCLE    4U
#endif

void FOC_OutputMgr_Init(foc_system_t *sys);
void FOC_OutputMgr_WriteDirect(foc_system_t *sys, const char *text);
void FOC_OutputMgr_WriteQueue(foc_system_t *sys, const char *text);
void FOC_OutputMgr_WriteStatus(foc_system_t *sys, uint8_t status);
void FOC_OutputMgr_FlushQueue(foc_system_t *sys);
uint8_t FOC_OutputMgr_GetOverflowCount(const foc_system_t *sys);

#endif /* FOC_OUTPUT_MGR_H */