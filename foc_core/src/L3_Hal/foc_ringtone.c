#include "L3_Hal/foc_ringtone.h"

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)

/* 音名频率表（十二平均律，八度 4 基准）：C C# D D# E F F# G G# A A# B */
static const uint16_t g_ringtone_note_hz[12] =
{
    262U, 277U, 294U, 311U, 330U, 349U,
    370U, 392U, 415U, 440U, 466U, 494U
};

/* 音名 → 半音索引（C=0..B=11）；非音名返回 0xFF */
static uint8_t Ringtone_NoteIndex(char c)
{
    switch ((char)(c | 0x20))
    {
    case 'c': return 0U;
    case 'd': return 2U;
    case 'e': return 4U;
    case 'f': return 5U;
    case 'g': return 7U;
    case 'a': return 9U;
    case 'b': return 11U;
    default:  return 0xFFU;
    }
}

static uint8_t Ringtone_IsDurationValid(uint16_t dur)
{
    return ((dur == 1U) || (dur == 2U) || (dur == 4U) ||
            (dur == 8U) || (dur == 16U) || (dur == 32U)) ? 1U : 0U;
}

/* 十进制无符号数读取（不跳过空白；无数字时返回 0 且不前进） */
static uint16_t Ringtone_ReadUint(const char **pp)
{
    uint16_t value = 0U;
    const char *p = *pp;

    while ((*p >= '0') && (*p <= '9'))
    {
        value = (uint16_t)((value * 10U) + (uint16_t)(*p - '0'));
        p++;
    }

    *pp = p;
    return value;
}

uint8_t FOC_Ringtone_Parse(const char *text,
                           foc_ringtone_step_t *steps_out,
                           uint16_t max_steps,
                           uint16_t *count_out)
{
    const char *p;
    uint16_t bpm = 63U;
    uint8_t default_dur = 4U;
    uint8_t default_oct = 6U;
    uint16_t count = 0U;

    if ((text == 0) || (steps_out == 0) || (count_out == 0) || (max_steps == 0U))
    {
        return 0U;
    }

    /* 段1：名称（丢弃） */
    p = text;
    while ((*p != '\0') && (*p != ':'))
    {
        p++;
    }
    if (*p != ':')
    {
        return 0U;
    }
    p++;

    /* 段2：缺省值 d=/o=/b=（逗号分隔，顺序不限，可缺省） */
    while ((*p != '\0') && (*p != ':'))
    {
        char key = (char)(*p | 0x20);
        uint16_t value;

        if (*p == ',')
        {
            p++;
            continue;
        }

        p++;
        if (*p != '=')
        {
            return 0U;
        }
        p++;
        value = Ringtone_ReadUint(&p);

        if (key == 'd')
        {
            if (Ringtone_IsDurationValid(value) == 0U)
            {
                return 0U;
            }
            default_dur = (uint8_t)value;
        }
        else if (key == 'o')
        {
            default_oct = (uint8_t)((value > 8U) ? 8U : value);
        }
        else if (key == 'b')
        {
            bpm = (uint16_t)((value < 25U) ? 25U : ((value > 900U) ? 900U : value));
        }
    }
    if (*p != ':')
    {
        return 0U;
    }
    p++;

    /* 段3：音符序列 */
    while (*p != '\0')
    {
        uint8_t note;
        uint8_t octave = default_oct;
        uint8_t dotted = 0U;
        uint16_t dur;
        uint32_t ticks;
        uint32_t freq = 0U;

        if (*p == ',')
        {
            p++;
            continue;
        }

        /* 时值（缺省用 d=） */
        dur = default_dur;
        if ((*p >= '0') && (*p <= '9'))
        {
            dur = Ringtone_ReadUint(&p);
            if (Ringtone_IsDurationValid(dur) == 0U)
            {
                return 0U;
            }
        }

        /* 音名或休止 */
        note = Ringtone_NoteIndex(*p);
        if ((*p | 0x20) == 'p')
        {
            note = 0xFFU;
            p++;
        }
        else if (note != 0xFFU)
        {
            p++;
            if (*p == '#')
            {
                note = (uint8_t)((note + 1U) % 12U);
                p++;
            }
        }
        else
        {
            return 0U;
        }

        /* 八度 */
        if ((*p >= '0') && (*p <= '9'))
        {
            uint16_t oct = Ringtone_ReadUint(&p);
            octave = (uint8_t)((oct > 8U) ? 8U : oct);
        }

        /* 附点 */
        if (*p == '.')
        {
            dotted = 1U;
            p++;
        }

        /* 时值 → 毫秒 → 电流环拍（分步计算，避免中间量溢出） */
        ticks = ((uint32_t)60000U * 4U) / ((uint32_t)bpm * (uint32_t)dur);
        ticks = (ticks * (uint32_t)FOC_CURRENT_LOOP_FREQ_HZ) / 1000U;
        if (dotted != 0U)
        {
            ticks = (ticks * 3U) / 2U;
        }
        if (ticks == 0U)
        {
            ticks = 1U;
        }
        if (ticks > 65535U)
        {
            ticks = 65535U;
        }

        /* 音名 → 频率（八度移位，休止保持 0） */
        if (note != 0xFFU)
        {
            uint8_t index = (uint8_t)(note % 12U);

            freq = (uint32_t)g_ringtone_note_hz[index];
            if (octave >= 4U)
            {
                uint8_t shift = (uint8_t)(octave - 4U);
                if (shift > 4U)
                {
                    shift = 4U;
                }
                freq <<= shift;
            }
            else
            {
                freq >>= (uint8_t)(4U - octave);
            }
            if (freq == 0U)
            {
                freq = 1U;
            }
            if (freq > 65535U)
            {
                freq = 65535U;
            }
        }

        if (count >= max_steps)
        {
            return 0U;
        }

        steps_out[count].freq_hz = (uint16_t)freq;
        steps_out[count].ticks = (uint16_t)ticks;
        count++;
    }

    if (count == 0U)
    {
        return 0U;
    }

    *count_out = count;
    return 1U;
}

#endif /* FOC_ACOUSTIC_ENABLE */
