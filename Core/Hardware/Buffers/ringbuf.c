/**
 * @file    ringbuf.c
 * @brief   单生产者单消费者环形缓冲区实现。
 * @note    head 只由生产者写入，tail 只由消费者写入；在 Cortex-M3 上按
 *          约定的唯一生产者和唯一消费者访问时可无锁使用。
 */

#include "ringbuf.h"
#include <stddef.h>

/**
 * @brief  初始化环形缓冲区并清空读写索引。
 * @param  rb 环形缓冲区对象。
 * @param  buf 用户提供的数据存储区。
 * @param  capacity 存储区总容量，单位字节。
 */
void RingBuf_Init(RingBuf *rb, uint8_t *buf, uint16_t capacity)
{
    rb->buf      = buf;
    rb->capacity = capacity;
    rb->head     = 0;
    rb->tail     = 0;
}

/**
 * @brief  计算环形缓冲区当前可读字节数。
 * @param  rb 环形缓冲区对象。
 * @return 当前可读字节数。
 */
uint16_t RingBuf_Available(RingBuf *rb)
{
    return (rb->head + rb->capacity - rb->tail) % rb->capacity;
}

/**
 * @brief  计算环形缓冲区剩余可写空间。
 * @param  rb 环形缓冲区对象。
 * @return 当前剩余可写字节数。
 */
uint16_t RingBuf_Free(RingBuf *rb)
{
    return rb->capacity - 1 - RingBuf_Available(rb);
}

/**
 * @brief  阻塞写入一个字节，缓冲区满时等待消费者释放空间。
 * @param  rb 环形缓冲区对象。
 * @param  byte 待写入字节。
 * @return 固定返回 0。
 * @note   等待期间不会释放 CPU；禁止在中断上下文或生产者与消费者相同的
 *         调用链中使用，否则可能永久阻塞。
 */
int RingBuf_Put(RingBuf *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % rb->capacity;
    while (next == rb->tail);   /* blocking wait */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 0;
}

/**
 * @brief  非阻塞写入一个字节，缓冲区满时返回 -1。
 * @param  rb 环形缓冲区对象。
 * @param  byte 待写入字节。
 * @return 0 表示成功，-1 表示缓冲区已满。
 */
int RingBuf_PutNB(RingBuf *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % rb->capacity;
    if (next == rb->tail) return -1;   /* full — drop */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 0;
}

/**
 * @brief  非阻塞读取一个字节，缓冲区为空时返回 -1。
 * @param  rb 环形缓冲区对象。
 * @param  byte 输出字节指针。
 * @return 0 表示成功，-1 表示缓冲区为空。
 */
int RingBuf_Get(RingBuf *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) return -1;   /* empty */
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    return 0;
}

/**
 * @brief  读取指定偏移处的字节，但不移动读索引。
 * @param  rb 环形缓冲区对象。
 * @param  offset 相对当前读索引的偏移，0 表示最早可读字节。
 * @param  byte 输出字节指针。
 * @return 0 表示成功，-1 表示偏移超出当前可读数据范围。
 */
int RingBuf_Peek(const RingBuf *rb, uint16_t offset, uint8_t *byte)
{
    uint16_t available;

    if ((rb == NULL) || (byte == NULL)) return -1;

    available = (rb->head + rb->capacity - rb->tail) % rb->capacity;
    if (offset >= available) return -1;

    *byte = rb->buf[(rb->tail + offset) % rb->capacity];
    return 0;
}

/**
 * @brief  清空环形缓冲区并复位读写索引。
 * @param  rb 环形缓冲区对象。
 */
void RingBuf_Flush(RingBuf *rb)
{
    rb->head = 0;
    rb->tail = 0;
}
