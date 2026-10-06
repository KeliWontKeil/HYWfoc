#include "L2_Core/Control/foc_ctrl_acoustic.h"

#include "LS_Config/foc_ringtone_table.h"
#include "L3_Hal/foc_math_lut.h"

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)

/* 每拍相位增量系数（2π · dt）：切步时一次乘法，ISR 内无除法 */
#define FOC_ACOUSTIC_PHASE_PER_HZ (FOC_MATH_TWO_PI * FOC_CURRENT_LOOP_DT_SEC)

/* 装载当前步：更新相位增量与步计时（频率 0 = 休止 → 释放包络） */
static void Acoustic_StartStep(foc_acoustic_state_t *ac)
{
    const foc_ringtone_step_t *step = &ac->steps[ac->step_idx];

    ac->step_ticks = step->ticks;
    ac->phase_inc_rad = (float)step->freq_hz * FOC_ACOUSTIC_PHASE_PER_HZ;
    ac->env_target = (step->freq_hz != 0U) ? 1U : 0U;
}

void FOC_Acoustic_Init(foc_acoustic_state_t *ac)
{
    ac->axis = FOC_ACOUSTIC_AXIS;
    ac->amplitude_v = FOC_ACOUSTIC_AMPLITUDE_V;
    ac->env_step = 1.0f;

    FOC_Acoustic_Reset(ac);
}

void FOC_Acoustic_Reset(foc_acoustic_state_t *ac)
{
    ac->active = 0U;
    ac->tune_id = 0U;
    ac->env_target = 0U;
    ac->step_len = 0U;
    ac->step_idx = 0U;
    ac->step_ticks = 0U;
    ac->phase_rad = 0.0f;
    ac->phase_inc_rad = 0.0f;
    ac->env = 0.0f;
    ac->wave = 0.0f;
}

/* 播放铃声表条目（调用方上下文：查表 → 解码 → 音域收敛 → 启动） */
uint8_t FOC_Acoustic_PlayTune(foc_acoustic_state_t *ac, uint8_t tune_id)
{
    const foc_ringtone_entry_t *entry;
    uint16_t count = 0U;
    uint16_t i;

    if ((uint32_t)tune_id >= (uint32_t)FOC_RINGTONE_COUNT)
    {
        return 0U;
    }

    entry = &foc_ringtone_table[tune_id];
    if (FOC_Ringtone_Parse(entry->rtttl, ac->steps,
                           (uint16_t)FOC_ACOUSTIC_SEQ_MAX_STEPS, &count) == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < count; i++)
    {
        if (ac->steps[i].freq_hz == 0U)
        {
            continue;
        }
        if ((float)ac->steps[i].freq_hz < (float)FOC_ACOUSTIC_TONE_MIN_HZ)
        {
            ac->steps[i].freq_hz = (uint16_t)FOC_ACOUSTIC_TONE_MIN_HZ;
        }
        if ((float)ac->steps[i].freq_hz > FOC_ACOUSTIC_TONE_MAX_HZ)
        {
            ac->steps[i].freq_hz = (uint16_t)FOC_ACOUSTIC_TONE_MAX_HZ;
        }
    }

    ac->axis = entry->axis;
    ac->amplitude_v = (entry->amplitude_v > FOC_ACOUSTIC_AMPLITUDE_LIMIT_V)
                    ? FOC_ACOUSTIC_AMPLITUDE_LIMIT_V : entry->amplitude_v;
    /* 包络步进 = 1 / 包络时长拍数（时长为 0 的情形由编译期校验阻断） */
    ac->env_step = 1.0f / ((float)entry->envelope_ms * 0.001f * (float)FOC_CURRENT_LOOP_FREQ_HZ);
    if (ac->env_step > 1.0f)
    {
        ac->env_step = 1.0f;
    }

    ac->tune_id = tune_id;
    ac->step_len = count;
    ac->step_idx = 0U;
    ac->phase_rad = 0.0f;
    ac->env = 0.0f;
    ac->wave = 0.0f;
    ac->active = 1U;

    Acoustic_StartStep(ac);
    return 1U;
}

void FOC_Acoustic_Stop(foc_acoustic_state_t *ac)
{
    /* 走释放段而非硬切：避免爆音与直流尾 */
    ac->step_len = 0U;
    ac->step_idx = 0U;
    ac->env_target = 0U;
}

uint8_t FOC_Acoustic_IsActive(const foc_acoustic_state_t *ac)
{
    return ac->active;
}

uint8_t FOC_Acoustic_GetTuneId(const foc_acoustic_state_t *ac)
{
    return ac->tune_id;
}

uint16_t FOC_Acoustic_GetTuneCount(void)
{
    return (uint16_t)FOC_RINGTONE_COUNT;
}

/* 阶段 4b：音频波形叠加（与电流环同拍）。未播放时直接返回（输出已清零）。 */
void FOC_ControlAcousticStep(foc_acoustic_state_t *ac, foc_control_runtime_t *ctrl)
{
    float wave;

    if (ac->active == 0U)
    {
        return;
    }

    /* 包络逼近目标（起停斜坡） */
    if (ac->env_target != 0U)
    {
        if (ac->env < 1.0f)
        {
            ac->env += ac->env_step;
            if (ac->env > 1.0f)
            {
                ac->env = 1.0f;
            }
        }
    }
    else
    {
        if (ac->env > 0.0f)
        {
            ac->env -= ac->env_step;
            if (ac->env < 0.0f)
            {
                ac->env = 0.0f;
            }
        }
    }

    /* 相位推进 + 单次条件回卷（增量 ≤ π/2，由音域上限保证） */
    ac->phase_rad += ac->phase_inc_rad;
    if (ac->phase_rad >= FOC_MATH_TWO_PI)
    {
        ac->phase_rad -= FOC_MATH_TWO_PI;
    }

    wave = FOC_MathLut_Sin(ac->phase_rad) * ac->amplitude_v * ac->env;
    ac->wave = wave;

    if (ac->axis == FOC_ACOUSTIC_AXIS_Q)
    {
        ctrl->uq += wave;
    }
    else
    {
        ctrl->ud += wave;
    }

    /* 步进推进（切步保持相位连续，仅换频率） */
    if (ac->step_ticks > 0U)
    {
        ac->step_ticks--;
    }
    if (ac->step_ticks == 0U)
    {
        ac->step_idx++;
        if (ac->step_idx >= ac->step_len)
        {
            ac->step_len = 0U;
            ac->step_idx = 0U;
            ac->env_target = 0U;
        }
        else
        {
            Acoustic_StartStep(ac);
        }
    }

    /* 释放完成 → 静默 */
    if ((ac->step_len == 0U) && (ac->env <= 0.0f))
    {
        ac->active = 0U;
        ac->wave = 0.0f;
    }
}

#endif /* FOC_ACOUSTIC_ENABLE */
