#ifndef FOC_MONITOR_QUEUE_TYPES_H
#define FOC_MONITOR_QUEUE_TYPES_H

#include <stdint.h>

/*
 * Monitor element tag: marks each queued monitor item.
 * FRAME_START lets the main loop discard a partially consumed previous frame.
 */
typedef enum {
    MONITOR_ELEM_FRAME_START      = 0xFD,
    MONITOR_ELEM_SEMANTIC_0       = 0x00,
    MONITOR_ELEM_SEMANTIC_1       = 0x01,
    MONITOR_ELEM_SEMANTIC_2       = 0x02,
    MONITOR_ELEM_SEMANTIC_3       = 0x03,
    MONITOR_ELEM_SEMANTIC_4       = 0x04,
    MONITOR_ELEM_SEMANTIC_5       = 0x05,
    MONITOR_ELEM_SEMANTIC_6       = 0x06,
    MONITOR_ELEM_SEMANTIC_7       = 0x07,
    MONITOR_ELEM_SEMANTIC_8       = 0x08,
    MONITOR_ELEM_SEMANTIC_9       = 0x09,
    MONITOR_ELEM_SEMANTIC_END     = 0x7F,
    MONITOR_ELEM_OSC_VALUE        = 0x80,
    MONITOR_ELEM_OSC_END          = 0x81,
    MONITOR_ELEM_PROTOCOL_SUMMARY = 0xC0,
} monitor_elem_tag_t;

typedef struct {
    uint8_t  tag;
    uint8_t  aux;
    uint16_t _pad;
    float    value;
} monitor_element_t;

#endif /* FOC_MONITOR_QUEUE_TYPES_H */
