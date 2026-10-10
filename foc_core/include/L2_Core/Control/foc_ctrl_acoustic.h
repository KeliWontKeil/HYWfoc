#ifndef FOC_CTRL_ACOUSTIC_H

#define FOC_CTRL_ACOUSTIC_H


#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_ringtone.h"

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)

/* ========== 声学模式状态（ACOUSTIC 相位的序列引擎，实例由 L1 持有） ==========
 * 调用契约：指针为 motor 内嵌地址（构造上非 NULL），不做空指针校验。
 * 不变量（PlayTune 建立、ModeStep 只读）：
 *   steps[0..step_len-1] 已按可用音域钳位且 ticks > 0；
 *   phase_inc_rad 与当前步频率一致（切步时更新，增量 ≤ π/2）；
 *   axis ∈ {FOC_ACOUSTIC_AXIS_D, FOC_ACOUSTIC_AXIS_Q}。
 * active=1 且 step_len=0 表示序列结束后的包络释放段。 */
typedef struct {
    uint8_t  active;        /* 1 = 播放中（含释放段），0 = 静默 */
    uint8_t  axis;          /* FOC_ACOUSTIC_AXIS_*（本曲输出轴） */
    uint8_t  tune_id;       /* 当前曲目 ID（铃声表索引） */
    uint8_t  env_target;    /* 包络目标：1 = 发声，0 = 释放 */
    uint16_t step_len;      /* 本曲有效步数（≤ FOC_ACOUSTIC_SEQ_CAPACITY） */
    uint16_t step_idx;      /* 当前步索引 */
    uint16_t step_ticks;    /* 当前步剩余拍数 */
    float    phase_inc_rad; /* 派生量：当前步每拍相位增量 */
    float    amplitude_v;   /* 峰值电压 [V] */
    float    env;           /* 包络 0..1 */
    float    env_step;      /* 派生量：包络每拍步进 */
    foc_ringtone_step_t steps[FOC_ACOUSTIC_SEQ_CAPACITY];
} foc_acoustic_state_t;

void     FOC_Acoustic_Init(foc_acoustic_state_t *ac);
void     FOC_Acoustic_Reset(foc_acoustic_state_t *ac);
uint8_t  FOC_Acoustic_PlayTune(foc_acoustic_state_t *ac, uint8_t tune_id);
void     FOC_Acoustic_Stop(foc_acoustic_state_t *ac);
uint8_t  FOC_Acoustic_IsActive(const foc_acoustic_state_t *ac);
uint16_t FOC_Acoustic_GetTuneCount(void);

/* 模式步进：推进序列 + 包络，产出本拍波形规格（相位推进 / 正弦 / 叠加由注入基础设施完成）。
 * 返回 1 = 本拍仍在发声（含包络释放段），0 = 已结束（active=0）。 */
uint8_t  FOC_Acoustic_ModeStep(foc_acoustic_state_t *ac,
                               float *phase_inc_rad_out,
                               float *amplitude_v_out,
                               uint8_t *axis_out);

#endif /* FOC_ACOUSTIC_ENABLE */

#endif /* FOC_CTRL_ACOUSTIC_H */
