#ifndef FOC_CTRL_COGGING_CALIB_H
#define FOC_CTRL_COGGING_CALIB_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

typedef struct foc_motor_t foc_motor_t;

/*
 * =====================================================================
 * 齿槽标定模块（L2/Control）
 *
 * 职责：非阻塞步进式齿槽补偿标定状态机。
 * 由 L1 控制任务在每个控制周期调用 RunStep，直至完成。
 * 完成时自动调用 FOC_CoggingCalibDumpTable 输出标定结果。
 *
 * 调用关系：L1 ControlTrigger → FOC_CoggingCalib_RunStep()
 * =====================================================================
 */

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)

/* ========== Cogging calibration runtime state ========== */
typedef struct {
    uint8_t in_progress;
    uint8_t progress_percent;
    uint16_t point_index;
    uint8_t completed_pass_count;
    float pred_mech_angle;
    uint16_t settle_counter;
    uint16_t last_lut_index;
    uint16_t bins_collected;
    uint8_t pass_num;
    uint8_t last_reported_progress;
    uint8_t saved_softswitch_enabled;
    uint8_t saved_softswitch_mode;
    uint8_t request_start;
    uint8_t request_dump;
    uint8_t request_export;
} foc_cogging_calib_state_t;

/* 1 if calibration in progress or start requested */
uint8_t FOC_CoggingCalibIsBusy(const foc_cogging_calib_state_t *state);

/* 协议命令入口：标记启动请求并切换控制阶段（跨 cogging_calib_state + state.control_phase 两域） */
void FOC_CoggingCalib_RequestStart(foc_motor_t *motor);

/* 协议命令入口：标记 dump 请求，下一控制周期消费（单域写） */
void FOC_CoggingCalib_RequestDump(foc_cogging_calib_state_t *state);

/* 协议命令入口：标记 export 请求，下一控制周期消费（单域写） */
void FOC_CoggingCalib_RequestExport(foc_cogging_calib_state_t *state);

/* 协议查询（非标定模块调用） */
uint8_t FOC_CoggingCalibIsDumpPending(const foc_cogging_calib_state_t *state);
uint8_t FOC_CoggingCalibIsExportPending(const foc_cogging_calib_state_t *state);
void    FOC_CoggingCalibClearDumpPending(foc_cogging_calib_state_t *state);
void    FOC_CoggingCalibClearExportPending(foc_cogging_calib_state_t *state);

/*
 * 标定状态机步进，由 L1 控制任务在每个控制周期调用。
 * 返回 1 表示仍在进行中，0 表示已完成（或未开始）。
 * 完成时自动 dump 标定表。
 */
uint8_t FOC_CoggingCalib_RunStep(foc_motor_t *motor,
                                 const sensor_data_t *sensor,
                                 float dt_sec);

void FOC_CoggingCalib_Abort(foc_motor_t *motor);

/* 标定表输出（dump / export） */
void FOC_CoggingCalibDumpTable(const foc_motor_t *motor);
void FOC_CoggingCalibExportTable(const foc_motor_t *motor);

#else /* FOC_COGGING_CALIB_ENABLE == FOC_CFG_DISABLE */

static inline uint8_t FOC_CoggingCalibIsBusy(const void *state) { (void)state; return 0U; }
static inline uint8_t FOC_CoggingCalibIsDumpPending(const void *state) { (void)state; return 0U; }
static inline uint8_t FOC_CoggingCalibIsExportPending(const void *state) { (void)state; return 0U; }
static inline void    FOC_CoggingCalibClearDumpPending(void *state) { (void)state; }
static inline void    FOC_CoggingCalibClearExportPending(void *state) { (void)state; }

#endif /* FOC_COGGING_CALIB_ENABLE */

#endif /* FOC_CTRL_COGGING_CALIB_H */