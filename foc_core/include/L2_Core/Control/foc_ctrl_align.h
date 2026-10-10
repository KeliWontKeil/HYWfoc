#ifndef FOC_CTRL_ALIGN_H
#define FOC_CTRL_ALIGN_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "LS_Config/foc_config.h"

typedef struct foc_motor_t foc_motor_t;

/*
 * =====================================================================
 * 有感对齐/标定状态机（L2/Control）
 *
 * 职责：以非阻塞状态机完成"电机零点/方向/极对数"的辨识（D 轴对齐 → 零点采样 →
 * 方向/极对数粗扫 + 回扫），由 L1 控制任务在每个控制周期步进，避免阻塞主循环。
 *
 * 触发源（同一状态机、收尾判定不同）：
 *   - 上电启动对齐（STARTUP）：上电自检通过但参数未标定时由 L1 请求；
 *   - 命令触发的重新对齐（aaYI）：运行期由协议请求。
 *
 * 编译期裁剪：由 FOC_ALIGN_ENABLE 宏控制。
 * =====================================================================
 */

#if (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE)

/* ========== 对齐/标定状态机状态 ========== */
/* 触发源：上电启动对齐（STARTUP）与协议命令重新对齐（aaYI）共用同一状态机，收尾判定不同 */
#define FOC_ALIGN_TRIGGER_COMMAND     0U
#define FOC_ALIGN_TRIGGER_STARTUP     1U

#define FOC_ALIGN_PHASE_IDLE          0U
#define FOC_ALIGN_PHASE_STOP          1U
#define FOC_ALIGN_PHASE_ZERO_SAMPLE   2U
#define FOC_ALIGN_PHASE_ZERO_CALC     3U
#define FOC_ALIGN_PHASE_ALIGN_SETTLE  4U
#define FOC_ALIGN_PHASE_ALIGN_SAMPLE  5U
#define FOC_ALIGN_PHASE_ALIGN_CALC    6U
#define FOC_ALIGN_PHASE_DIR_STEP      7U
#define FOC_ALIGN_PHASE_DIR_SAMPLE    8U
#define FOC_ALIGN_PHASE_DIR_CALC      9U
#define FOC_ALIGN_PHASE_DIR_REV_STEP  10U
#define FOC_ALIGN_PHASE_DIR_REV_SAMPLE 11U
#define FOC_ALIGN_PHASE_FINALIZE      12U
#define FOC_ALIGN_PHASE_DONE          13U

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
    uint8_t  trigger_source;
    uint8_t  report_pending;
} foc_align_state_t;

/* 请求重新对齐/标定（由协议 aaYI 命令调用） */
void FOC_Align_Request(foc_motor_t *motor);

/* 请求上电启动对齐（由 L1 在启动阶段选择时调用；收尾就绪判定交由 L1） */
void FOC_Align_RequestStartup(foc_motor_t *motor);

/* 取走一次"完成报告"（ISR 内只置位，主循环调用本函数输出详情） */
uint8_t FOC_Align_TakeReport(foc_motor_t *motor);

/*
 * 对齐状态机步进，由 L1 控制任务在每个控制周期调用。
 * 返回 1 表示仍在进行中，0 表示已完成（且 control_phase 已切回 NORMAL）。
 */
uint8_t FOC_Align_RunStep(foc_motor_t *motor, float dt_sec);

void FOC_Align_Abort(foc_motor_t *motor);

#else /* FOC_ALIGN_ENABLE == FOC_CFG_DISABLE */

static inline void FOC_Align_Request(foc_motor_t *motor) { (void)motor; }
static inline void FOC_Align_RequestStartup(foc_motor_t *motor) { (void)motor; }
static inline uint8_t FOC_Align_TakeReport(foc_motor_t *motor) { (void)motor; return 0U; }
static inline uint8_t FOC_Align_RunStep(foc_motor_t *motor, float dt_sec) { (void)motor; (void)dt_sec; return 0U; }

#endif /* FOC_ALIGN_ENABLE */

#endif /* FOC_CTRL_ALIGN_H */
