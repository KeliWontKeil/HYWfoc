#include "L2_Core/Control/foc_ctrl_injection.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

#if (FOC_INJECTION_ENABLE == FOC_CFG_ENABLE)

static void Injection_ClearOutput(foc_injection_state_t *inj)
{
    inj->inj_d = 0.0f;
    inj->inj_q = 0.0f;
    inj->inj_wave = 0.0f;
}

/* 解调：清空累加器与输出（含 LPF 状态） */
static void Injection_DemodClear(foc_injection_demod_state_t *dm)
{
    dm->tick_count = 0U;
    dm->d_sin = 0.0f;
    dm->d_cos = 0.0f;
    dm->q_sin = 0.0f;
    dm->q_cos = 0.0f;
    dm->i_d = 0.0f;
    dm->q_d = 0.0f;
    dm->i_q = 0.0f;
    dm->q_q = 0.0f;
    dm->mag_d = 0.0f;
    dm->mag_q = 0.0f;
}

/* 单轴结算：归一化 → 延迟补偿旋转；返回响应幅值 [A] */
static float Injection_DemodSettleAxis(const foc_injection_demod_state_t *dm,
                                       float acc_sin,
                                       float acc_cos,
                                       float *i_out,
                                       float *q_out)
{
    float i_raw = dm->acc_scale * acc_sin;
    float q_raw = dm->acc_scale * acc_cos;
    float i = (i_raw * dm->delay_cos) - (q_raw * dm->delay_sin);
    float q = (i_raw * dm->delay_sin) + (q_raw * dm->delay_cos);

    *i_out = i;
    *q_out = q;

    return sqrtf((i * i) + (q * q));
}

/* 结算：更新两轴 I/Q 与幅值；相干模式随后清空累加器（开启下一个整周期窗口） */
static void Injection_DemodSettle(foc_injection_state_t *inj)
{
    foc_injection_demod_state_t *dm = &inj->demod;
    float mag;

    mag = Injection_DemodSettleAxis(dm, dm->d_sin, dm->d_cos, &dm->i_d, &dm->q_d);
    dm->mag_d += dm->mag_lpf_alpha * (mag - dm->mag_d);

    mag = Injection_DemodSettleAxis(dm, dm->q_sin, dm->q_cos, &dm->i_q, &dm->q_q);
    dm->mag_q += dm->mag_lpf_alpha * (mag - dm->mag_q);

    if (inj->mode == FOC_INJECTION_ACTIVE_COHERENT)
    {
        dm->d_sin = 0.0f;
        dm->d_cos = 0.0f;
        dm->q_sin = 0.0f;
        dm->q_cos = 0.0f;
    }
}

/* 本拍解调累加：d/q 实测电流与注入参考正交相关（相干整周期累加 / 任意频率 I/Q 低通）。
 * 轴掩码决定累加哪些轴，未启用轴保持 0。 */
static void Injection_DemodAccumulate(foc_injection_state_t *inj,
                                      const foc_control_runtime_t *ctrl,
                                      float ref_sin,
                                      float ref_cos,
                                      uint8_t axis)
{
    foc_injection_demod_state_t *dm = &inj->demod;
    float alpha = dm->iq_lpf_alpha;

    if ((axis & FOC_INJECTION_AXIS_D) != 0U)
    {
        if (alpha > 0.0f)
        {
            dm->d_sin += alpha * ((ctrl->id_measured * ref_sin) - dm->d_sin);
            dm->d_cos += alpha * ((ctrl->id_measured * ref_cos) - dm->d_cos);
        }
        else
        {
            dm->d_sin += ctrl->id_measured * ref_sin;
            dm->d_cos += ctrl->id_measured * ref_cos;
        }
    }

    if ((axis & FOC_INJECTION_AXIS_Q) != 0U)
    {
        if (alpha > 0.0f)
        {
            dm->q_sin += alpha * ((ctrl->iq_measured * ref_sin) - dm->q_sin);
            dm->q_cos += alpha * ((ctrl->iq_measured * ref_cos) - dm->q_cos);
        }
        else
        {
            dm->q_sin += ctrl->iq_measured * ref_sin;
            dm->q_cos += ctrl->iq_measured * ref_cos;
        }
    }

    dm->tick_count++;
    if (dm->tick_count >= dm->window_len)
    {
        dm->tick_count = 0U;
        Injection_DemodSettle(inj);
    }
}

/* HFI 配置建立：轴运行时收敛、幅值/频率限幅、模式判定、派生量缓存。
 * 完成后状态必然合法（轴非零、模式必为相干或任意频率、频率在可表达区间内）。 */
void FOC_Injection_Configure(foc_injection_state_t *inj, float freq_hz, float amplitude_v, uint8_t axis)
{
    inj->axis = axis & FOC_INJECTION_AXIS_DQ;
    if (inj->axis == 0U)
    {
        inj->axis = FOC_INJECTION_AXIS_D;
    }

    /* 幅值仅取幅；上限由输出级统一钳位（不在此重复设限） */
    inj->amplitude_v = fabsf(amplitude_v);
    freq_hz = Math_ClampFloat(freq_hz, FOC_INJECTION_FREQ_MIN_HZ, FOC_INJECTION_FREQ_MAX_HZ);

#if (FOC_INJECTION_MODE == FOC_INJECTION_MODE_ARBITRARY_ONLY)
    /* 强制任意频率：相位累加器连续可调，无量化 */
    inj->mode = FOC_INJECTION_ACTIVE_ARBITRARY;
    inj->n_div = 0U;
    inj->freq_act_hz = freq_hz;
#else
    {
        float f_loop = (float)FOC_CURRENT_LOOP_FREQ_HZ;
        uint16_t n = (uint16_t)((f_loop / freq_hz) + 0.5f);
        float f_coh = f_loop / (float)n;

#if (FOC_INJECTION_MODE == FOC_INJECTION_MODE_COHERENT_ONLY)
        inj->mode = FOC_INJECTION_ACTIVE_COHERENT;
        inj->n_div = n;
        inj->freq_act_hz = f_coh;
#else
        if (fabsf(f_coh - freq_hz) <= (FOC_INJECTION_QUANT_ERROR_MAX * freq_hz))
        {
            inj->mode = FOC_INJECTION_ACTIVE_COHERENT;
            inj->n_div = n;
            inj->freq_act_hz = f_coh;
        }
        else
        {
            /* 量化误差过大 → 任意频率 */
            inj->mode = FOC_INJECTION_ACTIVE_ARBITRARY;
            inj->n_div = 0U;
            inj->freq_act_hz = freq_hz;
        }
#endif
    }
#endif

    /* 每拍相位增量：相干 = 2π/N（与电流环率严格锁定）；任意频率 = 2π·f·T_loop */
    inj->phase_inc_rad = (inj->mode == FOC_INJECTION_ACTIVE_COHERENT)
                       ? (FOC_MATH_TWO_PI / (float)inj->n_div)
                       : (FOC_MATH_TWO_PI * inj->freq_act_hz * FOC_CURRENT_LOOP_DT_SEC);

    inj->phase_rad = 0.0f;
    Injection_ClearOutput(inj);

    /* 解调派生量：窗口长度、归一化系数、I/Q LPF 系数、延迟补偿旋转量 */
    if (inj->mode == FOC_INJECTION_ACTIVE_COHERENT)
    {
        /* 相干：整周期（N 拍）相关累加 → 零泄漏、无需 LPF */
        inj->demod.window_len = inj->n_div;
        inj->demod.acc_scale = 2.0f / (float)inj->n_div;
        inj->demod.iq_lpf_alpha = 0.0f;
    }
    else
    {
        inj->demod.window_len = (uint16_t)FOC_INJECTION_DEMOD_SETTLE_DIV;
        inj->demod.acc_scale = 2.0f;
        inj->demod.iq_lpf_alpha = Math_ClampFloat(
            FOC_MATH_TWO_PI * (inj->freq_act_hz / (float)FOC_INJECTION_DEMOD_LPF_FC_DIV) * FOC_CURRENT_LOOP_DT_SEC,
            0.0f,
            1.0f);
    }
    inj->demod.mag_lpf_alpha = Math_ClampFloat(FOC_INJECTION_DEMOD_MAG_LPF_ALPHA, 0.0f, 1.0f);

    /* 激励→采样 1 拍延迟：相关结果等效被反向旋转 Δ = 每拍相位增量，配置期算好旋转量 */
    FOC_MathLut_SinCos(inj->phase_inc_rad, &inj->demod.delay_sin, &inj->demod.delay_cos);

    Injection_DemodClear(&inj->demod);
}

/* 基础设施（唯一叠加点）：相位推进 → 单位正弦（复用 L3 通用查表）→ 按轴叠加 ctrl.ud/uq → 按需解调 */
static void Injection_ApplyWave(foc_injection_state_t *inj,
                                foc_control_runtime_t *ctrl,
                                float phase_inc_rad,
                                float amplitude_v,
                                uint8_t axis,
                                uint8_t demod)
{
    float ref_sin;
    float ref_cos;
    float wave;

    inj->phase_rad += phase_inc_rad;
    if (inj->phase_rad >= FOC_MATH_TWO_PI)
    {
        inj->phase_rad -= FOC_MATH_TWO_PI;
    }

    FOC_MathLut_SinCos(inj->phase_rad, &ref_sin, &ref_cos);
    wave = ref_sin * amplitude_v;
    inj->inj_wave = wave;

    if ((axis & FOC_INJECTION_AXIS_D) != 0U)
    {
        inj->inj_d = wave;
        ctrl->ud += wave;
    }
    else
    {
        inj->inj_d = 0.0f;
    }

    if ((axis & FOC_INJECTION_AXIS_Q) != 0U)
    {
        inj->inj_q = wave;
        ctrl->uq += wave;
    }
    else
    {
        inj->inj_q = 0.0f;
    }

    if (demod != 0U)
    {
        Injection_DemodAccumulate(inj, ctrl, ref_sin, ref_cos, axis);
    }
}

void FOC_Injection_Init(foc_injection_state_t *inj)
{
    inj->enabled = 0U;
    FOC_Injection_Configure(inj, FOC_INJECTION_DEFAULT_FREQ_HZ,
                            FOC_INJECTION_DEFAULT_AMPLITUDE_V, FOC_INJECTION_AXIS_D);
}

/* 复位瞬态（停止/恢复路径）；保留频率/幅值与派生量配置 */
void FOC_Injection_Reset(foc_injection_state_t *inj)
{
    inj->enabled = 0U;
    inj->phase_rad = 0.0f;
    Injection_ClearOutput(inj);
    Injection_DemodClear(&inj->demod);
}

void FOC_Injection_SetEnable(foc_injection_state_t *inj, uint8_t enable)
{
    inj->enabled = (enable != 0U) ? 1U : 0U;
    Injection_ClearOutput(inj);
    /* 启停均从干净窗口开始，避免跨启停拼接半窗 */
    Injection_DemodClear(&inj->demod);
}

/* NORMAL 相位：HFI 载波叠加 + 同拍解调（未使能时无副作用） */
void FOC_Injection_HfiStep(foc_injection_state_t *inj, foc_control_runtime_t *ctrl)
{
    if (inj->enabled == 0U)
    {
        return;
    }

    Injection_ApplyWave(inj, ctrl, inj->phase_inc_rad, inj->amplitude_v, inj->axis, 1U);
}

/* 共享 sink：ACOUSTIC 相位驱动（控制环不输出，本函数产生的样本即最终 dq 电压；不解调） */
void FOC_Injection_InjectSample(foc_injection_state_t *inj, foc_control_runtime_t *ctrl,
                                float phase_inc_rad, float amplitude_v, uint8_t axis)
{
    Injection_ApplyWave(inj, ctrl, phase_inc_rad, amplitude_v, axis, 0U);
}

uint8_t FOC_Injection_IsActive(const foc_injection_state_t *inj)
{
    return (inj->enabled != 0U) ? 1U : 0U;
}

#endif /* FOC_INJECTION_ENABLE */
