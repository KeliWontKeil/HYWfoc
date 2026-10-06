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
 * 索引即 ID（协议 A 组参数）：新增曲目在表尾追加，不要在中间插入或删除，
 * 否则会改变既有 ID 的含义（上位机与内部事件均按 ID 引用）。 */

typedef struct {
    const char *rtttl;
    uint8_t     axis;
    float       amplitude_v;
    uint16_t    envelope_ms;
} foc_ringtone_entry_t;

/* 内部事件引用的固定 ID */
#define FOC_RINGTONE_ID_BOOT   0U
#define FOC_RINGTONE_ID_FAULT  1U
#define FOC_RINGTONE_ID_TEST   2U

static const foc_ringtone_entry_t foc_ringtone_table[] =
{
    /* ID 0：上电自检通过提示音（两短音） */
    { "boot:d=16,o=5,b=240:8e,8p,8e",
      FOC_ACOUSTIC_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },

    /* ID 1：故障报警音（三短音） */
    { "fault:d=16,o=4,b=200:8c,8p,8c,8p,8c",
      FOC_ACOUSTIC_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },

    /* ID 2：音阶测试（验证音域与音量，供协议 A:P2 触发） */
    { "test:d=4,o=5,b=200:c,d,e,f,g",
      FOC_ACOUSTIC_AXIS, FOC_ACOUSTIC_AMPLITUDE_V, FOC_ACOUSTIC_ENVELOPE_MS },
};

#define FOC_RINGTONE_COUNT (sizeof(foc_ringtone_table) / sizeof(foc_ringtone_table[0]))

#endif /* FOC_ACOUSTIC_ENABLE */

#endif /* FOC_RINGTONE_TABLE_H */
