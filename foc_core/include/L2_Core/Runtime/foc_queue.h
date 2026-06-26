#ifndef FOC_QUEUE_H
#define FOC_QUEUE_H

#include <stdint.h>

/*
 * ================================================================
 * 轻量级环形字节队列（L2/Runtime）
 *
 * 本模块仅提供类型定义与操作方法，**不持有任何队列实例**。
 * 队列存储空间与 fifo_queue_t 实例由调用者（L1）分配，
 * 通过指针传入本模块的纯函数进行操作。
 * ================================================================
 */

typedef struct {
    uint8_t *buffer;           /* 环形缓冲区基址（连续内存，每 element_size 一个槽） */
    uint16_t element_size;     /* 每个槽的字节数 */
    uint8_t  depth;            /* 槽数量（缓冲区大小 = depth * element_size，调用者分配） */
    volatile uint8_t write_idx;
    volatile uint8_t read_idx;
    volatile uint8_t count;
    uint8_t  overflow_count;
} fifo_queue_t;

/* 初始化队列：绑定缓冲区、设置参数、清零索引 */
void FIFO_Init(fifo_queue_t *q, uint8_t *buf, uint16_t elem_size, uint8_t depth);

/* 入队：复制 data（element_size 字节）到队列尾部
 * 返回 1=成功，0=队列已满（overflow_count 递增） */
uint8_t FIFO_Enqueue(fifo_queue_t *q, const uint8_t *data);

/* 出队：从队列头部复制 element_size 字节到 data
 * 返回 1=成功，0=队列为空 */
uint8_t FIFO_Dequeue(fifo_queue_t *q, uint8_t *data);

/* 获取队列中当前元素个数 */
uint8_t FIFO_Count(const fifo_queue_t *q);

#endif /* FOC_QUEUE_H */
