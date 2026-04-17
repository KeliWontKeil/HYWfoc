#ifndef PROTOCOL_CORE_TYPES_H
#define PROTOCOL_CORE_TYPES_H

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

#endif /* PROTOCOL_CORE_TYPES_H */
