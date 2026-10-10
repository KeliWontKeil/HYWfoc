#ifndef FOC_RINGTONE_H

#define FOC_RINGTONE_H


#include <stdint.h>

#include "LS_Config/foc_config.h"

/* L3 — RTTTL（Ring Tone Text Transfer Language）铃声编解码（纯函数）
 *
 * 把 RTTTL 文本解码为与平台无关的"事件步"序列：
 *   步 = { 频率 [Hz]，时长 [电流环拍] }；freq_hz == 0 表示休止。
 * 输出时长以电流环拍为单位，使消费方（声学引擎）在 ISR 内只需递减计数，
 * 不在实时路径做时间换算。频率不做音域钳位（由消费方按自身可用音域收敛）。
 *
 * 支持的子集：name:d=,o=,b=:note,... / 音符 [时值][音名][#][八度][.] / 休止 p /
 * 时值 {1,2,4,8,16,32} 与附点 / 三处缺省值由解析器兜底。
 *
 * 契约：输出容量不足或文本非法时返回 0 且不产生部分结果（不写越界）。
 */

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)

typedef struct {
    uint16_t freq_hz;
    uint16_t ticks;
} foc_ringtone_step_t;

/* 解码 RTTTL 文本 → 事件步序列。
 * in:  text       以 '\0' 结尾的 RTTTL 文本
 *      max_steps  steps_out 容量（步数）
 * out: steps_out   事件步序列（仅在返回 1 时有效）
 *      count_out   实际步数
 * 返回：1 = 成功，0 = 失败（非法文本 / 超容量 / 空序列）。 */
uint8_t FOC_Ringtone_Parse(const char *text,
                           foc_ringtone_step_t *steps_out,
                           uint16_t max_steps,
                           uint16_t *count_out);

#endif /* FOC_ACOUSTIC_ENABLE */

#endif /* FOC_RINGTONE_H */
