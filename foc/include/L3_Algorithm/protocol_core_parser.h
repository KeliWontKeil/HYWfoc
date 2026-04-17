#ifndef PROTOCOL_CORE_PARSER_H
#define PROTOCOL_CORE_PARSER_H

#include <stdint.h>

#include "L3_Algorithm/protocol_core_types.h"

uint8_t ProtocolCore_ParseSignedFloat(const char *text, float *value_out);
uint8_t ProtocolCore_ExtractFrame(const uint8_t *rx_data,
                                  uint16_t rx_len,
                                  const uint8_t **frame_start,
                                  uint16_t *frame_len);
protocol_core_frame_parse_result_t ProtocolCore_ParseFrame(const uint8_t *frame,
                                                           uint16_t len,
                                                           protocol_command_t *out_cmd);
uint8_t ProtocolCore_IsDriverIdFormatValid(uint8_t driver_id);
uint8_t ProtocolCore_IsDriverAddressedToLocal(uint8_t driver_id);

#endif /* PROTOCOL_CORE_PARSER_H */
