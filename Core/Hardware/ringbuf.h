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
#define __RINGBUF_H  /* 头文件保护宏，防止环形缓冲接口重复声明。 */

#include <stdint.h>

/* ---- Generic byte-level ring buffer ---- */
typedef struct {
    uint8_t          *buf;
    uint16_t          capacity;    /* total buffer size in bytes */
    volatile uint16_t head;        /* write index (producer: ISR or task) */
    volatile uint16_t tail;        /* read index  (consumer: main loop or ISR) */
} RingBuf;

/**
 * @brief  初始化环形缓冲区。
 * @param  rb 环形缓冲区对象。
 * @param  buf 用户提供的数据存储区。
 * @param  capacity 存储区总容量，单位字节。
 */
void RingBuf_Init(RingBuf *rb, uint8_t *buf, uint16_t capacity);

/**
 * @brief  获取当前可读字节数量。
 * @param  rb 环形缓冲区对象。
 * @return 尚未读取的数据字节数。
 */
uint16_t RingBuf_Available(RingBuf *rb);

/**
 * @brief  获取当前剩余可写空间。
 * @param  rb 环形缓冲区对象。
 * @return 还能写入的数据字节数。
 */
uint16_t RingBuf_Free(RingBuf *rb);

/**
 * @brief  阻塞写入一个字节。
 * @param  rb 环形缓冲区对象。
 * @param  byte 待写入字节。
 * @return 固定返回 0。
 */
int RingBuf_Put(RingBuf *rb, uint8_t byte);

/**
 * @brief  非阻塞写入一个字节。
 * @param  rb 环形缓冲区对象。
 * @param  byte 待写入字节。
 * @return 0 表示成功，-1 表示缓冲区已满。
 */
int RingBuf_PutNB(RingBuf *rb, uint8_t byte);

/**
 * @brief  非阻塞读取一个字节。
 * @param  rb 环形缓冲区对象。
 * @param  byte 输出字节指针。
 * @return 0 表示成功，-1 表示缓冲区为空。
 */
int RingBuf_Get(RingBuf *rb, uint8_t *byte);

/**
 * @brief  清空环形缓冲区。
 * @param  rb 环形缓冲区对象。
 */
void RingBuf_Flush(RingBuf *rb);

#endif /* __RINGBUF_H */
