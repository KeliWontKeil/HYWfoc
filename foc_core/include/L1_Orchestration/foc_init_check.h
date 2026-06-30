#ifndef FOC_INIT_CHECK_H
#define FOC_INIT_CHECK_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

/*
 * L1 初始化完整性校验
 *
 * 校验所有初始化步骤是否完成，设置 system_running / system_fault。
 * 在 FOC_App_Init 的最后调用一次。
 */
void FOC_InitCheck_Verify(foc_motor_t *motor, const sensor_data_t *sensor);

#endif /* FOC_INIT_CHECK_H */