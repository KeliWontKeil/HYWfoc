#ifndef FOC_CTRL_SOURCE_MGR_H
#define FOC_CTRL_SOURCE_MGR_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Control/foc_ctrl_estim_smo.h"
#include "L2_Core/Control/foc_ctrl_openloop.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L3_Hal/foc_sensor.h"

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

/* ========== 跨层只读窗：源角度/速度统一读取（调试/遥测复用） ========== */
typedef struct {
    const sensor_data_t *sensor;
    const foc_motor_params_t *params;
    const foc_control_runtime_t *ctrl;
    const foc_control_ref_t *ref;
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    const foc_estim_smo_state_t *smo_state;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    const foc_openloop_state_t *openloop_state;
#endif
} foc_source_read_ctx_t;

/* ========== Source Manager 上下文视图（由 L1/Executor 构造并分发） ========== */
typedef struct {
    /* 输入（只读） */
    const sensor_data_t *sensor;
    const foc_motor_params_t *params;
    const foc_control_cfg_t *cfg;
    uint8_t control_mode;
    const foc_control_ref_t *ref;
    /* 输入/输出（SourceMgr 读取或重写） */
    foc_source_switch_state_t *switch_cfg;
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    foc_estim_smo_state_t *smo_state;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    foc_openloop_state_t *openloop_state;
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    foc_cogging_comp_status_t *cogging_status;
#endif
    foc_source_mgr_state_t *state;
    foc_active_source_state_t *active;
    foc_control_runtime_t *ctrl;
    foc_outer_loop_private_t *outer_loop;
    foc_pid_t *speed_pid;
    foc_pid_t *torque_current_pid;
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    foc_current_soft_switch_status_t *soft_switch;
#endif
    foc_encoder_services_state_t *encoder_services;
} foc_source_mgr_ctx_t;

void FOC_SourceMgr_Init(foc_source_mgr_ctx_t *ctx, uint8_t low_source, uint8_t high_source);
void FOC_SourceMgr_Select(foc_source_mgr_ctx_t *ctx);
void FOC_SourceMgr_Publish(foc_source_mgr_ctx_t *ctx);

/* 统一源读取：任意源组合的角度/机械速度读取（供调试/遥测复用） */
uint8_t FOC_SourceMgr_ReadSourceAngle(const foc_source_read_ctx_t *ctx, uint8_t source,
                                      float *mech_out, float *elec_out);
uint8_t FOC_SourceMgr_ReadSourceSpeed(const foc_source_read_ctx_t *ctx, uint8_t source,
                                      float *speed_out);

#endif /* FOC_CTRL_SOURCE_MGR_H */
