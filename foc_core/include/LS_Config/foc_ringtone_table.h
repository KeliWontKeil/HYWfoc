#ifndef FOC_RINGTONE_TABLE_H

#define FOC_RINGTONE_TABLE_H

#include "LS_Config/foc_cfg_feature_switches.h"
#include "LS_Config/foc_cfg_init_values.h"

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)

/* LS 数据表 — 铃声表（声学回报的曲目资源，随固件版本管理）
 *
 * 每条铃声 = RTTTL 文本（音高与节奏，见 foc_ringtone.h 的子集说明）+ 元数据
 * （输出轴 / 峰值电压 / 起停包络时长）。协议只携带条目 ID，音频资源不下发，
 * 使协议保持"仅数字参数"的紧凑契约。
 *
 * ID 约定（索引即 ID，协议 `A:P<id>` 播放、`A:L` 读条目总数）：
 *   0..2  内部事件固定 ID（见下方 FOC_RINGTONE_ID_*，L1 就绪播报引用）——勿插入/删除；
 *   >= 3  自定义铃声：**在表尾追加条目即可**，用 `A:P<id>` 播放（id = 追加后的索引）。
 *         协议接受任意「整数且 < 条目总数」的 id，不限于宏定义的项。
 *
 * 追加须知：
 *   - 解码后步数（音符 + 休止）须 ≤ FOC_ACOUSTIC_SEQ_CAPACITY，否则 `A:P` 返回参数错误（不播半曲）；
 *   - 每条可独立指定输出轴 / 峰值电压 / 包络时长（见 foc_ringtone_entry_t）；
 *   - id 以 uint8_t 寻址 ⇒ 条目总数上限 256；
 *   - 一律在表尾追加，勿在中间插入或删除，否则改变既有 ID 含义（上位机与内部事件按 ID 引用）。 */

typedef struct {
    const char *rtttl;
    uint8_t     axis;
    float       amplitude_v;
    uint16_t    envelope_ms;
} foc_ringtone_entry_t;

/* 内部事件引用的固定 ID（按"声音是什么"命名；"哪个场景用哪个声音"由 L1 决定） */
#define FOC_RINGTONE_ID_LONG_BEEP       0U
#define FOC_RINGTONE_ID_TWO_SHORT_BEEPS 1U
#define FOC_RINGTONE_ID_SCALE           2U

static const foc_ringtone_entry_t foc_ringtone_table[] =
{
    /* ID 0：一声长鸣 */
    { "long:d=2,o=6,b=240:c",
      FOC_ACOUSTIC_DEFAULT_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },

    /* ID 1：两短鸣（中间一短休止） */
    { "double:d=16,o=6,b=240:8c6,8p,8c6",
      FOC_ACOUSTIC_DEFAULT_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },

    /* ID 2：音阶（验证音域与音量，供协议 A:P2 触发） */
    { "scale:d=8,o=6,b=200:c,d,e,f,g",
      FOC_ACOUSTIC_DEFAULT_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },

    /* ID 3：音阶（验证音域与音量，供协议 A:P2 触发） */
    { "Jianpu:d=4,o=4,b=175:8f5,8d#5,8p,8c5,8a#4,8p,8g4,8g#4,8p,8d#5,8d#4,16g#4,16a#4,8c5,8a#4,4g#4,8f5,8d#5,8p,8c5,8a#4,8p,8g4,8g#4,8p,8d#5,8d#4,16g#4,16a#4,8c5,8a#4,4g#4,8f5,8d#5,8p,8c5,8a#4,8p,8g4,8g#4,8p,8d#5,8d#4,16g#4,16a#4,8g#5,8g5,4d#5,8f5,8d#5,8p,8c5,8a#4,8p,8g4,8g#4,8p,8d#5,8d#4,16g#4,16a#4,8c5,8a#4,4g#4,8c5,8d#4,8d#4,8a#4,8d#4,8g4,8g#4,8d#5,2d#5,8d#5,8c4,8a#3,8g#3,8g#3,8a#3,8c4,8d#4,8p,4g#3,2a#3,8c4,8p,8c4,8a#3,8g#3,8g#3,4a#3,4g4,8g#3,8g#3,4a#3,4g#3,8g3,4g#3,4p,4g#4,8g#4,8f4,8d#4,8d#4,4d#4,8g#4,8g#4,8a#4,8c5,8p,8d#4,8d#4,8d#4,8g#4,8a#4,4c5,4a#4,4d#4,8g#4,8g#4,8a#4,8a#4,8p,4g#4,4p,8p,8g#4,8g#4,8g4,8d#4,8d#4,4d#4,8d#4,4d#4,8c#4,8c4,8g#3,8g#3,8a#3,8c4,8g#3,8g#3,8a#3,8c4,8g#3,8g#3,8a#3,8g#3,8g#3,8g#3,4g#4,8g4,4g#4,8p,8g#4,4g#4,8c5,4a#4,8g#4,8p,8g#4,8a#4,4g#4,8g#4,4g4,8p,4g4,8g#4,1a#4,8a#4,4p,4p,4p,8e4,8e4,4a4,4b4,4c#5,4b4,4a4,2a4,4a4,4b4,2a4,8c#5,8b4,4c#5,8d#4,8p,4p,8d#4,4a4,4b4,4c#5,4b4,4a4,4a4,4p,4a4,4b4,4a4,4d5,8c#5,8b4,2b4,4c#5,4p,4a4,4b4,4e5,8b4,8a4,8a4,8p,4e4,4e4,2e4,8b4,8a4,4a4,4b4,4d5,4c#5,4d5,4c#5,8b4,8a4,4a4,4p,2a4,4a4,8p,2c#5,2b4,4a4,4g#4,8p,4p,8e4,8e4,4a4,4b4,4c#5,4b4,4a4,2a4,4a4,8f#4,8g#4,2g#4,8f#4,8e4,2e4,4p,8e4,8e4,4a4,4b4,4c#5,4b4,4a4,4a4,4p,4a4,2e4,8b4,8c#5,4c#5,8p,4p,8a4,8a4,4a4,4b4,8e5,8a4,2a4,4e5,8e5,8p,8a4,8a4,4a4,4e5,8e5,8b4,4g#4,4a4,4e5,8e5,8p,8e4,8e4,4e4,4a4,4p,4e4,4e4,4a4,4a4,8c#5,8b4,4b4,4p,4a4,8c#5,8d5,4d5,4c#5,4a4,4g#4,1a4,1a4,4p,4p,4p,4p,4p,4p,4p,4p",
      FOC_ACOUSTIC_DEFAULT_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },
      /* ID 3：音阶（验证音域与音量，供协议 A:P2 触发） */
    { "1:",
      FOC_ACOUSTIC_DEFAULT_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },
};

#define FOC_RINGTONE_COUNT (sizeof(foc_ringtone_table) / sizeof(foc_ringtone_table[0]))

#endif /* FOC_ACOUSTIC_ENABLE */

#endif /* FOC_RINGTONE_TABLE_H */


