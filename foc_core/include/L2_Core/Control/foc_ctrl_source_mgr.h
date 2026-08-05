#ifndef FOC_CTRL_SOURCE_MGR_H
#define FOC_CTRL_SOURCE_MGR_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

typedef struct foc_motor_t foc_motor_t;

/* ========== Source Manager 私有状态（per-motor） ========== */
typedef enum {
    FOC_REGION_STATE_FULL_ACTIVE = 0U,
    FOC_REGION_STATE_LOW_ACTIVE,
    FOC_REGION_STATE_HIGH_ACQUIRE,
    FOC_REGION_STATE_HIGH_ACTIVE,
    FOC_REGION_STATE_HIGH_SUSPECT,
    FOC_REGION_STATE_LOW_RECOVERY
} foc_region_state_t;

typedef struct {
    uint8_t active_source;
    uint8_t standby_source;
    uint8_t control_region;
    uint8_t region_state;
    uint8_t switch_in_progress;
    uint32_t switch_counter;
    uint8_t config_valid;
    /* 降域(回LOW)去抖计数：SUSPECT→HIGH 恢复需连续满足高速判据 */
    uint32_t degrade_hold_counter;
} foc_source_mgr_state_t;

typedef struct {
    uint8_t  low_source;
    uint8_t  high_source;
    float    speed_threshold_high_rad_s;
    float    speed_threshold_low_rad_s;
} foc_source_switch_state_t;

void FOC_SourceMgr_Init(foc_motor_t *motor, uint8_t low_source, uint8_t high_source);
void FOC_SourceMgr_Select(foc_motor_t *motor);
void FOC_SourceMgr_Publish(foc_motor_t *motor);
const foc_active_source_state_t *FOC_SourceMgr_GetActive(const foc_motor_t *motor);

/* 统一源读取：任意源组合的角度/机械速度读取（供调试/遥测复用） */
uint8_t FOC_SourceMgr_ReadSourceAngle(const foc_motor_t *motor, uint8_t source,
                                      float *mech_out, float *elec_out);
uint8_t FOC_SourceMgr_ReadSourceSpeed(const foc_motor_t *motor, uint8_t source,
                                      float *speed_out);

#endif /* FOC_CTRL_SOURCE_MGR_H */