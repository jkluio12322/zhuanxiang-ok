/**
 * @file    ringbuf.h
 * @brief   Lock-free SPSC ring buffer for bare-metal inter-module IPC
 * @note    Single producer, single consumer — no locks needed on Cortex-M3
 *          Multi-byte element push/pop via memcpy
 *
 *          Usage:
 *            RingBuf rb;
 *            uint8_t buf[256];
 *            RingBuf_Init(&rb, buf, sizeof(buf));
 *            RingBuf_Put(&rb, 0xAA);           // blocking or non-blocking
 *            RingBuf_PutNB(&rb, 0xAA);         // non-blocking, returns -1 if full
 *            RingBuf_Get(&rb, &byte);          // returns 0 or -1
 */

#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

/* ---- Generic byte-level ring buffer ---- */
typedef struct {
    uint8_t          *buf;
    uint16_t          capacity;    /* total buffer size in bytes */
    volatile uint16_t head;        /* write index (producer: ISR or task) */
    volatile uint16_t tail;        /* read index  (consumer: main loop or ISR) */
} RingBuf;

void     RingBuf_Init(RingBuf *rb, uint8_t *buf, uint16_t capacity);
uint16_t RingBuf_Available(RingBuf *rb);
uint16_t RingBuf_Free(RingBuf *rb);
int      RingBuf_Put(RingBuf *rb, uint8_t byte);        /* blocking (busy-wait) */
int      RingBuf_PutNB(RingBuf *rb, uint8_t byte);      /* non-blocking, -1=full */
int      RingBuf_Get(RingBuf *rb, uint8_t *byte);       /* non-blocking, -1=empty */
void     RingBuf_Flush(RingBuf *rb);

#endif /* __RINGBUF_H */
