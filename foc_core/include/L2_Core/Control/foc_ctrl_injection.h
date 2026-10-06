#ifndef FOC_CTRL_INJECTION_H

#define FOC_CTRL_INJECTION_H


#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L2_Core/foc_ctrl_types.h"

#if (FOC_INJECTION_ENABLE == FOC_CFG_ENABLE)

/* ========== 注入解调状态（与注入成对的被动工具：从 d/q 实测电流解出注入响应 I/Q 与幅值）
 * 不变量（Init / Configure 建立，ISR 只读）：
 *   window_len    > 0（相干模式 = 一个注入周期 N；任意频率模式 = 结算抽拍间隔）
 *   acc_scale     相关累加 → 响应分量 的归一化系数（相干 2/N；任意频率 2，平均由 I/Q LPF 完成）
 *   iq_lpf_alpha  相干模式为 0（无 LPF，整周期累加）；任意频率模式为 f_inj/K 对应系数
 *   delay_sin/cos = sin/cos(每拍相位增量)，用于补偿"激励→采样 1 拍延迟"的等效相移 */
typedef struct {
    uint16_t window_len;
    uint16_t tick_count;
    float    acc_scale;
    float    iq_lpf_alpha;
    float    mag_lpf_alpha;
    float    delay_sin;
    float    delay_cos;
    float    d_sin;
    float    d_cos;
    float    q_sin;
    float    q_cos;
    float    i_d;
    float    q_d;
    float    i_q;
    float    q_q;
    float    mag_d;
    float    mag_q;
} foc_injection_demod_state_t;

/* ========== 注入状态（被动工具，实例由 L1 持有） ==========
 * 调用契约：所有指针均为 motor 内嵌状态地址（构造上非 NULL），故不做空指针校验。
 * 不变量（Init / Configure 建立，ISR 只读）：
 *   axis      ∈ 可用轴掩码且非零
 *   mode      ∈ {COHERENT, ARBITRARY}（无"未判定"态）
 *   phase_rad ∈ [0, 2π)，phase_inc_rad ∈ (0, π] → ISR 单次条件回卷即成立 */
typedef struct {
    uint8_t  enabled;       /* 由调用方置位；0 时不产生注入（输出保持 0） */
    uint8_t  axis;          /* FOC_INJECTION_AXIS_* 位掩码 */
    uint8_t  mode;          /* FOC_INJECTION_ACTIVE_*（配置后必为相干或任意频率） */
    uint16_t n_div;         /* 相干分频比（任意频率模式为 0） */
    float    freq_act_hz;   /* 实际生效频率（消费方应使用该值计算） */
    float    amplitude_v;   /* 归一化幅值 [V]（≥0，≤ FOC_INJECTION_AMPLITUDE_LIMIT_V） */
    float    phase_inc_rad; /* 派生量：每电流环拍相位增量 */
    float    phase_rad;     /* 当前注入相位 [0, 2π) */
    float    inj_d;         /* 本拍注入 d 分量 [V] */
    float    inj_q;         /* 本拍注入 q 分量 [V] */
    float    inj_wave;      /* 观测用主分量 [V] */
    foc_injection_demod_state_t demod;
} foc_injection_state_t;

void FOC_Injection_Init(foc_injection_state_t *inj);
void FOC_Injection_Reset(foc_injection_state_t *inj);
void FOC_Injection_Configure(foc_injection_state_t *inj,
                             float freq_hz,
                             float amplitude_v,
                             uint8_t axis);
void FOC_Injection_SetEnable(foc_injection_state_t *inj, uint8_t enable);
void FOC_ControlInjectionStep(foc_injection_state_t *inj,
                              foc_control_runtime_t *ctrl);

#endif /* FOC_INJECTION_ENABLE */

#endif /* FOC_CTRL_INJECTION_H */
