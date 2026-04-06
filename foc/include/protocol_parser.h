#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include <stdint.h>

typedef struct {
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

void ProtocolParser_Init(void);
void ProtocolParser_TriggerParse(void);
uint8_t ProtocolParser_IsParsePending(void);
void ProtocolParser_Process(void);
const protocol_command_t *ProtocolParser_GetLatestCommand(void);
void ProtocolParser_ClearUpdatedFlag(void);
protocol_parser_result_t ProtocolParser_GetLastResult(void);

#endif /* PROTOCOL_PARSER_H */
