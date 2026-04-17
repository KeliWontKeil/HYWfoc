#ifndef PROTOCOL_TEXT_CODEC_H
#define PROTOCOL_TEXT_CODEC_H

#include <stdint.h>

const char *ProtocolText_GetParamName(char subcommand);
const char *ProtocolText_GetStateName(char subcommand);
uint8_t ProtocolText_IsIntegerParam(char subcommand);
void ProtocolText_FormatParamLine(char *out,
                                  uint16_t out_len,
                                  char subcommand,
                                  float value);
void ProtocolText_FormatStateLine(char *out,
                                  uint16_t out_len,
                                  char subcommand,
                                  uint8_t value);

#endif /* PROTOCOL_TEXT_CODEC_H */
