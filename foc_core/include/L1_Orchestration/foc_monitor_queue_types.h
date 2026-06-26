#ifndef FOC_MONITOR_QUEUE_TYPES_H
#define FOC_MONITOR_QUEUE_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"

/*
 * ================================================================
 * Monitor element tag — 标记每个队列元素属于哪种输出帧
 *
 * 帧起始标记 (FRAME_START) 用于主循环的帧隔离机制：
 * 当主循环遇到 FRAME_START 时，如果上一个帧未消费完则丢弃残余，
 * 防止不同 MonitorTrigger 生成的帧元素交错。
 * ================================================================
 */
typedef enum {
    MONITOR_ELEM_FRAME_START      = 0xFD,  /* 帧起始分隔符 */
    MONITOR_ELEM_SEMANTIC_0       = 0x00,  /* 语义行 0: phase current A */
    MONITOR_ELEM_SEMANTIC_1       = 0x01,  /* 语义行 1: phase current B */
    MONITOR_ELEM_SEMANTIC_2       = 0x02,  /* 语义行 2: phase current C */
    MONITOR_ELEM_SEMANTIC_3       = 0x03,  /* 语义行 3: encoder angle raw */
    MONITOR_ELEM_SEMANTIC_4       = 0x04,  /* 语义行 4: encoder angle filtered */
    MONITOR_ELEM_SEMANTIC_5       = 0x05,  /* 语义行 5: vbus raw */
    MONITOR_ELEM_SEMANTIC_6       = 0x06,  /* 语义行 6: vbus filtered */
    MONITOR_ELEM_SEMANTIC_7       = 0x07,  /* 语义行 7: execution time */
    MONITOR_ELEM_SEMANTIC_END     = 0x7F,  /* 语义帧结束标记 */
    MONITOR_ELEM_OSC_VALUE        = 0x80,  /* 示波器参数值（aux 记录掩码位索引） */
    MONITOR_ELEM_OSC_END          = 0x81,  /* 示波器行结束标记 */
    MONITOR_ELEM_PROTOCOL_SUMMARY = 0xC0,  /* 协议摘要行 */
} monitor_elem_tag_t;

/*
 * 队列元素
 *   tag   — 元素类型
 *   value — 原始数据值（对 FRAME_START 无意义）
 *   aux   — 辅助信息（示波器掩码位索引、协议摘要子类型等）
 *   共计 8 字节（2 字节填充后 8 字节对齐）
 */
typedef struct {
    uint8_t  tag;
    uint8_t  aux;
    uint16_t _pad;      /* 对齐填充 */
    float    value;
} monitor_element_t;

#endif /* FOC_MONITOR_QUEUE_TYPES_H */
