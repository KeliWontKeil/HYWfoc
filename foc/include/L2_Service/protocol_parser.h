#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include <stdint.h>

/* Reuse L3 core protocol types; avoid duplicated type ownership in L2. */
#include "L3_Algorithm/protocol_core_types.h"

void ProtocolParser_Init(void);
uint8_t ProtocolParser_IsParsePending(void);
void ProtocolParser_Process(void);
const protocol_command_t *ProtocolParser_GetLatestCommand(void);
void ProtocolParser_ClearUpdatedFlag(void);
protocol_parser_result_t ProtocolParser_GetLastResult(void);

#endif /* PROTOCOL_PARSER_H */
