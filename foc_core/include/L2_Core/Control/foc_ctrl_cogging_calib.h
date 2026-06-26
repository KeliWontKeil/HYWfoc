#ifndef FOC_CTRL_COGGING_CALIB_H
#define FOC_CTRL_COGGING_CALIB_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

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

/* 1 if calibration in progress or start requested (safe inline for all build configs). */
static inline uint8_t FOC_CoggingCalibIsBusy(const foc_motor_t *motor)
{
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    return (motor->cogging_calib_state.in_progress != 0U) ||
           (motor->cogging_calib_state.request_start != 0U);
#else
    (void)motor;
    return 0U;
#endif
}

/* Progress percentage [0..100] (safe inline for all build configs). */
static inline uint8_t FOC_CoggingCalibGetProgressPercent(const foc_motor_t *motor)
{
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    return motor->cogging_calib_state.progress_percent;
#else
    (void)motor;
    return 0U;
#endif
}

/* 协议命令入口：标记启动请求，下一控制周期 RunStep 消费 */
void FOC_CoggingCalib_RequestStart(foc_motor_t *motor);

/* 协议命令入口：标记 dump 请求，下一控制周期消费 */
void FOC_CoggingCalib_RequestDump(foc_motor_t *motor);

/* 协议命令入口：标记 export 请求，下一控制周期消费 */
void FOC_CoggingCalib_RequestExport(foc_motor_t *motor);

/* 协议查询（非标定模块调用） */
uint8_t FOC_CoggingCalibIsDumpPending(const foc_motor_t *motor);
uint8_t FOC_CoggingCalibIsExportPending(const foc_motor_t *motor);
void    FOC_CoggingCalibClearDumpPending(foc_motor_t *motor);
void    FOC_CoggingCalibClearExportPending(foc_motor_t *motor);

/*
 * 标定状态机步进，由 L1 控制任务在每个控制周期调用。
 * 返回 1 表示仍在进行中，0 表示已完成（或未开始）。
 * 完成时自动 dump 标定表。
 */
uint8_t FOC_CoggingCalib_RunStep(foc_motor_t *motor,
                                 const sensor_data_t *sensor,
                                 float dt_sec);

/* 标定表输出（dump / export） */
void FOC_CoggingCalibDumpTable(const foc_motor_t *motor);
void FOC_CoggingCalibExportTable(const foc_motor_t *motor);

#endif /* FOC_CTRL_COGGING_CALIB_H */

