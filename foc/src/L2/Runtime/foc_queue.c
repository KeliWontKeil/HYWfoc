#include "L2/Runtime/foc_queue.h"

#include <string.h>

void FIFO_Init(fifo_queue_t *q, uint8_t *buf, uint16_t elem_size, uint8_t depth)
{
    if (q == 0) return;
    q->buffer        = buf;
    q->element_size  = elem_size;
    q->depth         = depth;
    q->write_idx     = 0U;
    q->read_idx      = 0U;
    q->count         = 0U;
    q->overflow_count = 0U;
}

uint8_t FIFO_Enqueue(fifo_queue_t *q, const uint8_t *data)
{
    uint8_t next;

    if ((q == 0) || (data == 0)) return 0U;

    if (q->count >= q->depth)
    {
        q->overflow_count++;
        return 0U;
    }

    (void)memcpy(q->buffer + (q->write_idx * q->element_size),
                 data,
                 q->element_size);

    next = (uint8_t)((q->write_idx + 1U) % q->depth);
    q->write_idx = next;
    q->count++;

    return 1U;
}

uint8_t FIFO_Dequeue(fifo_queue_t *q, uint8_t *data)
{
    if ((q == 0) || (data == 0)) return 0U;

    if (q->count == 0U) return 0U;

    (void)memcpy(data,
                 q->buffer + (q->read_idx * q->element_size),
                 q->element_size);

    q->read_idx = (uint8_t)((q->read_idx + 1U) % q->depth);
    q->count--;

    return 1U;
}

uint8_t FIFO_Count(const fifo_queue_t *q)
{
    if (q == 0) return 0U;
    return q->count;
}