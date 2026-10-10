#ifndef FOC_CTRL_INJECTION_H

#define FOC_CTRL_INJECTION_H


#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L2_Core/foc_ctrl_types.h"

#if (FOC_INJECTION_ENABLE == FOC_CFG_ENABLE)

/* ========== 注入解调状态（从 d/q 实测电流解出注入响应 I/Q 与幅值，供 HFI 观测）
 * 不变量（HFI 配置建立时确定，ISR 只读）：
 *   window_len    > 0（相干模式 = 一个注入周期 N；任意频率模式 = 结算抽拍间隔）
 *   acc_scale     相关累加 → 响应分量 的归一化系数（相干 2/N；任意频率 2）
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

/* ========== 注入基础设施状态（共享 sink：单一叠加点；HFI 由控制态驱动、声学由 ACOUSTIC 相位驱动）
 * 调用契约：指针为 motor 内嵌状态地址（构造上非 NULL），故不做空指针校验。 */
typedef struct {
    uint8_t  enabled;       /* HFI 使能；0 时不产生注入（输出保持 0） */
    uint8_t  axis;          /* HFI 轴掩码（FOC_INJECTION_AXIS_*，运行时参数） */
    uint8_t  mode;          /* FOC_INJECTION_ACTIVE_*（相干/任意频率） */
    uint16_t n_div;         /* 相干分频比（任意频率模式为 0） */
    float    freq_act_hz;   /* 实际生效频率 */
    float    amplitude_v;   /* 归一化幅值 [V]（上限由输出级统一钳位） */
    float    phase_inc_rad; /* 派生量：每拍相位增量 */
    float    phase_rad;     /* 当前注入相位 [0, 2π) */
    float    inj_d;         /* 本拍注入 d 分量 [V] */
    float    inj_q;         /* 本拍注入 q 分量 [V] */
    float    inj_wave;      /* 观测用主分量 [V] */
    foc_injection_demod_state_t demod;
} foc_injection_state_t;

void    FOC_Injection_Init(foc_injection_state_t *inj);
void    FOC_Injection_Reset(foc_injection_state_t *inj);

/* HFI 载波（由控制态驱动）：配置 + 使能 + 每拍叠加（含同拍解调） */
void    FOC_Injection_Configure(foc_injection_state_t *inj, float freq_hz, float amplitude_v, uint8_t axis);
void    FOC_Injection_SetEnable(foc_injection_state_t *inj, uint8_t enable);
void    FOC_Injection_HfiStep(foc_injection_state_t *inj, foc_control_runtime_t *ctrl);

/* 共享注入 sink（由 ACOUSTIC 相位驱动）：注入一个样本（相位推进 + 正弦 + 单点叠加；不解调） */
void    FOC_Injection_InjectSample(foc_injection_state_t *inj, foc_control_runtime_t *ctrl,
                                   float phase_inc_rad, float amplitude_v, uint8_t axis);

/* HFI 是否在输出（供 NORMAL 的直写判定） */
uint8_t FOC_Injection_IsActive(const foc_injection_state_t *inj);

#endif /* FOC_INJECTION_ENABLE */

#endif /* FOC_CTRL_INJECTION_H */
