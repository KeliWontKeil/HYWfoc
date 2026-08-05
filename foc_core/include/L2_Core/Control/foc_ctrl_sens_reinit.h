#ifndef FOC_CTRL_REINIT_H
#define FOC_CTRL_REINIT_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "LS_Config/foc_config.h"

typedef struct foc_motor_t foc_motor_t;

/*
 * =====================================================================
 * 非阻塞重初始化模块（L2/Control）
 *
 * 职责：将运行时重初始化改写为非阻塞状态机，由 L1 控制任务
 * 在每个控制周期步进，避免阻塞主循环。
 *
 * 上电初始化保持阻塞（FOC_MotorInit → FOC_CalibrateElectricalAngleAndDirection），
 * 不受本模块影响。
 *
 * 编译期裁剪：由 FOC_REINIT_ENABLE 宏控制。
 * =====================================================================
 */

#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)

/* ========== 非阻塞重初始化状态 ========== */
#define FOC_REINIT_PHASE_IDLE          0U
#define FOC_REINIT_PHASE_STOP          1U
#define FOC_REINIT_PHASE_ZERO_SAMPLE   2U
#define FOC_REINIT_PHASE_ZERO_CALC     3U
#define FOC_REINIT_PHASE_ALIGN_SETTLE  4U
#define FOC_REINIT_PHASE_ALIGN_SAMPLE  5U
#define FOC_REINIT_PHASE_ALIGN_CALC    6U
#define FOC_REINIT_PHASE_DIR_STEP      7U
#define FOC_REINIT_PHASE_DIR_SAMPLE    8U
#define FOC_REINIT_PHASE_DIR_CALC      9U
#define FOC_REINIT_PHASE_DIR_REV_STEP  10U
#define FOC_REINIT_PHASE_DIR_REV_SAMPLE 11U
#define FOC_REINIT_PHASE_FINALIZE      12U
#define FOC_REINIT_PHASE_DONE          13U

typedef struct {
    uint16_t phase;
    uint16_t settle_cycles;
    uint16_t sample_count;
    uint16_t sample_target;
    float    sin_sum;
    float    cos_sum;
    float    elec_angle_rad;
    float    calib_uq;
    float    prev_mech_rad;
    float    prev_elec_rad;
    float    sum_d_mech;
    float    sum_d_elec;
    uint8_t  has_prev;
    uint8_t  step_index;
    uint8_t  step_count;
    uint8_t  reverse_pass;
} foc_reinit_state_t;

/* 请求重初始化（由协议 Y:R 命令调用） */
void FOC_ReInit_Request(foc_motor_t *motor);

/*
 * 重初始化状态机步进，由 L1 控制任务在每个控制周期调用。
 * 返回 1 表示仍在进行中，0 表示已完成（且 control_phase 已切回 NORMAL）。
 */
uint8_t FOC_ReInit_RunStep(foc_motor_t *motor, float dt_sec);

void FOC_ReInit_Abort(foc_motor_t *motor);

#else /* FOC_REINIT_ENABLE == FOC_CFG_DISABLE */

static inline void FOC_ReInit_Request(foc_motor_t *motor) { (void)motor; }
static inline uint8_t FOC_ReInit_RunStep(foc_motor_t *motor, float dt_sec) { (void)motor; (void)dt_sec; return 0U; }

#endif /* FOC_REINIT_ENABLE */

#endif /* FOC_CTRL_REINIT_H */