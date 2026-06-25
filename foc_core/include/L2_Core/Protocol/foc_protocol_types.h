#ifndef FOC_PROTOCOL_TYPES_H
#define FOC_PROTOCOL_TYPES_H

#include <stdint.h>

typedef struct {
    uint8_t driver_id;
    char command;
    char subcommand;
    char param_text[32];
    float param_value;
    uint8_t has_param;
    uint8_t updated;
    uint8_t frame_valid;
} protocol_command_t;

typedef enum {
    PROTOCOL_PARSER_RESULT_NO_FRAME = 0,
    PROTOCOL_PARSER_RESULT_OK,
    PROTOCOL_PARSER_RESULT_FRAME_ERROR
} protocol_parser_result_t;

typedef enum {
    PROTOCOL_CORE_FRAME_PARSE_INVALID = 0,
    PROTOCOL_CORE_FRAME_PARSE_OK,
    PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH
} protocol_core_frame_parse_result_t;

/*
 * 单帧处理结果：
 * 协议层解析并执行一帧数据后，告诉 L1 需要做哪些后续动作。
 * comm_active   = 1: 当前帧有通信活动（用于 LED 指示）
 * needs_status  = 1: 状态码已在协议内部直写完成
 * needs_summary = 1: 请求 L1 生成 RUNTIME_SUMMARY 并发送
 * param_changed = 1: 参数被修改，需要 L1 调用 ApplyConfig
 */
typedef struct {
    uint8_t comm_active   : 1;
    uint8_t needs_status  : 1;
    uint8_t needs_summary : 1;
    uint8_t param_changed : 1;
} foc_protocol_frame_result_t;

#endif /* FOC_PROTOCOL_TYPES_H */
