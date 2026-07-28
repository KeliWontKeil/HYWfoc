#ifndef FOC_CTRL_SOURCE_MGR_H
#define FOC_CTRL_SOURCE_MGR_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

void FOC_SourceMgr_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source);
void FOC_SourceMgr_Select(foc_motor_t *motor);
void FOC_SourceMgr_Publish(foc_motor_t *motor);
const foc_active_source_state_t *FOC_SourceMgr_GetActive(const foc_motor_t *motor);

#endif /* FOC_CTRL_SOURCE_MGR_H */
