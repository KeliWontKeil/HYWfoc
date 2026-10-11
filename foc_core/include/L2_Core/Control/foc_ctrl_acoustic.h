#ifndef FOC_CTRL_ACOUSTIC_H

#define FOC_CTRL_ACOUSTIC_H


#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_ringtone.h"

typedef struct foc_motor_t foc_motor_t;

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)

/* ========== 声学模式状态（ACOUSTIC 相位的序列引擎，实例由 L1 持有） ==========
 * 调用契约：指针为 motor 内嵌地址（构造上非 NULL），不做空指针校验。
 * 不变量（PlayTune 建立、ModeStep 只读）：
 *   steps[0..step_len-1] 已按可用音域钳位且 ticks > 0；
 *   phase_inc_rad 与当前步频率一致（切步时更新，增量 ≤ π/2）；
 *   axis ∈ {FOC_INJECTION_AXIS_D, FOC_INJECTION_AXIS_Q}（与注入共用轴掩码符号）。
 * active=1 且 step_len=0 表示序列结束后的包络释放段。 */
typedef struct {
    uint8_t  active;        /* 1 = 播放中（含释放段），0 = 静默 */
    uint8_t  axis;          /* 声学输出轴掩码（编码取 FOC_INJECTION_AXIS_*；默认 FOC_ACOUSTIC_OUTPUT_AXIS）
                             * 选轴由声学功能决定：预留运行时入口，当前无协议/命令写入，仅 Init 取编译期值 */
    uint8_t  tune_id;       /* 当前曲目 ID（铃声表索引；预留给状态上报，当前无消费者） */
    uint8_t  env_target;    /* 包络目标：1 = 发声，0 = 释放 */
    uint16_t step_len;      /* 本曲有效步数（≤ FOC_ACOUSTIC_SEQ_CAPACITY） */
    uint16_t step_idx;      /* 当前步索引 */
    uint16_t step_ticks;    /* 当前步剩余拍数 */
    uint16_t bpm;           /* 当前曲目节拍（RTTTL b=）：供指示灯按拍速闪烁，不参与控制 */
    float    phase_inc_rad; /* 派生量：当前步每拍相位增量 */
    float    amplitude_v;   /* 峰值电压 [V]（预留运行时入口，同 axis） */
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

/* 触发收口（协议 A 组与 L1 内部事件共用）：进入 ACOUSTIC 相位（与对齐/标定同级——控制环停止输出）。
 * 守卫：无故障且已使能，且当前为 NORMAL/ACOUSTIC；播放中再次调用 = 同相覆盖（重入）。
 * 返回 1 = 已接受，0 = 拒绝（守卫不通过 / 曲目非法）。 */
uint8_t  FOC_Acoustic_RequestTune(foc_motor_t *motor, uint8_t tune_id);

/* 请求停止（只发起停止，进入包络释放段）；相位由曲终在 L1 控制触发收回 NORMAL。 */
void     FOC_Acoustic_RequestStop(foc_motor_t *motor);

#else /* FOC_ACOUSTIC_ENABLE == FOC_CFG_DISABLE */

static inline uint8_t FOC_Acoustic_RequestTune(foc_motor_t *motor, uint8_t tune_id) { (void)motor; (void)tune_id; return 0U; }
static inline void FOC_Acoustic_RequestStop(foc_motor_t *motor) { (void)motor; }

#endif /* FOC_ACOUSTIC_ENABLE */

#endif /* FOC_CTRL_ACOUSTIC_H */
