/**
 * @file    ringbuf.c
 * @brief   Lock-free SPSC ring buffer implementation
 * @note    Single producer / single consumer — safe on Cortex-M3 without atomics
 *          because head is only written by producer, tail only by consumer
 */

#include "ringbuf.h"

void RingBuf_Init(RingBuf *rb, uint8_t *buf, uint16_t capacity)
{
    rb->buf      = buf;
    rb->capacity = capacity;
    rb->head     = 0;
    rb->tail     = 0;
}

uint16_t RingBuf_Available(RingBuf *rb)
{
    return (rb->head + rb->capacity - rb->tail) % rb->capacity;
}

uint16_t RingBuf_Free(RingBuf *rb)
{
    return rb->capacity - 1 - RingBuf_Available(rb);
}

int RingBuf_Put(RingBuf *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % rb->capacity;
    while (next == rb->tail);   /* blocking wait */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 0;
}

int RingBuf_PutNB(RingBuf *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % rb->capacity;
    if (next == rb->tail) return -1;   /* full — drop */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 0;
}

int RingBuf_Get(RingBuf *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) return -1;   /* empty */
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    return 0;
}

void RingBuf_Flush(RingBuf *rb)
{
    rb->head = 0;
    rb->tail = 0;
}
