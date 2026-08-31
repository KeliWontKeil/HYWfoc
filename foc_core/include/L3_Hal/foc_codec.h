#ifndef FOC_CODEC_H
#define FOC_CODEC_H

#include <stdint.h>

#include "LS_Config/foc_config.h"

/* L3 — 协议编解码（纯数据/字符串封装，无业务类型依赖，可替换）
 *
 * 本模块是把 "wire 语法 ↔ 裸数据" 解耦的单一 seam，覆盖三类可被不同
 * 上位机 / 不同传输协议替换的帧：
 *   1) 指令帧（RX）：外部传入的指令（随接口变化）
 *   2) OSC 回报帧（TX）：示波器浮点数据（随上位机解析变化）
 *   3) 短回报行（TX）：parameter/config/state 行（机器解析）
 * 大字符串回报（系统信息 / 批读 dump / 摘要）为"人读"，不抽象，保持 L2。
 *
 * 接口契约（与 foc_platform_api 同风格）：
 *   - 接口面稳定：输入/输出为结构化数据，不随格式裁剪。
 *   - 可替换：换接口/上位机只需修改对应函数。
 *   - 无业务类型依赖：本模块不 include 任何 L2/L1 业务类型。
 */

typedef enum {
    FOC_CODEC_FRAME_PARSE_INVALID = 0,
    FOC_CODEC_FRAME_PARSE_OK,
    FOC_CODEC_FRAME_PARSE_ADDRESS_MISMATCH
} foc_codec_frame_parse_result_t;

/* ================= 指令帧（RX） ================= */

/* 从原始字节流提取一帧并解析信封（driver/cmd/sub + param 文本）。
 * in:  rx        原始字节（可含一帧或多帧，取第一个完整 a...b 帧）
 *      rx_len    字节数
 * out: out_driver driver_id
 *      out_cmd    command  ('P'/'C'/'S'/'Y')
 *      out_sub    subcommand
 *      param_out  param 文本（含 '\0'；无 param 为空串）
 *      param_max  param_out 容量（param 超出则判 INVALID）
 *      has_param  1=带 param，0=不带
 * 返回：OK / INVALID / ADDRESS_MISMATCH（对 MISMATCH 调用方应静默丢弃，无响应）。
 *
 * 【修改指导】换接口/协议（CAN、二进制、已有总线协议）时只需替换本函数：
 *   - 帧边界（一帧如何界定）见 Codec_ExtractFrame；
 *   - 信封/编址策略见 Codec_IsDriverIdFormatValid / Codec_IsDriverAddressedToLocal；
 *   - 本函数只把 param 原样拷入 param_out，数值解析见 Codec_ParseSignedFloat（L2 按需调用）。
 */
foc_codec_frame_parse_result_t Codec_ParseCommandFrame(
    const uint8_t *rx, uint16_t rx_len,
    uint8_t *out_driver, uint8_t *out_cmd, uint8_t *out_sub,
    char *param_out, uint16_t param_max, uint8_t *has_param);

/* 从字节流提取第一个完整帧边界（a...b）。 */
uint8_t Codec_ExtractFrame(const uint8_t *rx_data, uint16_t rx_len,
                           const uint8_t **frame_start, uint16_t *frame_len);

/* 编址策略（默认 ASCII 单播/广播）。不同接口可改写。 */
uint8_t Codec_IsDriverIdFormatValid(uint8_t driver_id);
uint8_t Codec_IsDriverAddressedToLocal(uint8_t driver_id);

/* 有符号浮点文本解析。L2 业务层对 param 做数值解析时调用。 */
uint8_t Codec_ParseSignedFloat(const char *text, float *value_out);

/* 状态值 0/1 → 使能状态字节。 */
uint8_t Codec_ParseStateValue(float value, uint8_t *state_out);

/* ================= OSC 回报帧（TX） ================= */

/* 把一组示波器浮点数据编码为回报帧。
 * in:  bit_index[]  每个值对应的通道 bit（0~15；自描述格式可用，位置格式可忽略）
 *      value[]      通道浮点值（原始数据，不含格式）
 *      count        值个数
 * out: out          完整帧文本（含头尾，'\0' 终止）
 *      max_len      out 容量
 * 返回：写入字符数。
 *
 * 【修改指导】换上位机示波器时只需修改本函数：
 *   - 默认位置格式（a <v1> <v2> ... b，定点 3 位小数）；
 *   - 需要自描述/二进制/CSV/带 CRC 时，用 bit_index 携带通道语义重新编码。
 */
uint16_t Codec_OscEncodeFrame(const uint8_t *bit_index, const float *value,
                              uint16_t count, char *out, uint16_t max_len);

/* ================= 短回报行（TX） ================= */

/* 把单个数值编码为 "prefix.name=value\r\n" 行。
 * in:  prefix  行前缀（"parameter"/"config"）
 *      name    业务名（由 L2 传入，如 "target_angle_rad"）
 *      value   数值
 *      is_int  1=整数输出（%u），0=定点 3 位小数
 * out: out     行文本（含 '\r\n'），'\0' 终止
 * 返回：1=成功，0=失败。
 * 【修改指导】改行格式（key 命名/精度/分隔）只需修改本函数。 */
uint8_t Codec_FormatValueLine(char *out, uint16_t max,
                              const char *prefix, const char *name,
                              float value, uint8_t is_int);

/* 把状态值编码为 "state.name=ENABLE/DISABLE\r\n" 行。 */
uint8_t Codec_FormatStateLine(char *out, uint16_t max,
                              const char *name, uint8_t value);

#endif /* FOC_CODEC_H */

