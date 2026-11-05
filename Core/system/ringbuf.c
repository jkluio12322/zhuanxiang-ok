/* 单生产者单消费者环形缓冲：head 只由生产者写、tail 只由消费者写，唯一生产/消费时可无锁。 */

#include "ringbuf.h"
#include <stddef.h>

void ringbuf_init(RingBuf *rb, uint8_t *buf, uint16_t capacity)   /* 初始化并清空读写索引 */
{
    rb->buf      = buf;
    rb->capacity = capacity;
    rb->head     = 0;
    rb->tail     = 0;
}

uint16_t ringbuf_available(RingBuf *rb)   /* 可读字节数 */
{
    return (rb->head + rb->capacity - rb->tail) % rb->capacity;
}

uint16_t ringbuf_free(RingBuf *rb)   /* 剩余可写空间 */
{
    return rb->capacity - 1 - ringbuf_available(rb);
}

/* 阻塞式写入已移除：满时忙等会在中断或主循环里直接挂死，
   全工程统一只用 ringbuf_put_nb()（放不下就返回 -1，由调用方决定丢帧）。 */

int ringbuf_put_nb(RingBuf *rb, uint8_t byte)   /* 非阻塞写，满时返回 -1 */
{
    uint16_t next = (rb->head + 1) % rb->capacity;
    if (next == rb->tail) return -1;   /* 满 */
    rb->buf[rb->head] = byte;
    rb->head = next;
    return 0;
}

int ringbuf_get(RingBuf *rb, uint8_t *byte)   /* 非阻塞读，空时返回 -1 */
{
    if (rb->head == rb->tail) return -1;   /* 空 */
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    return 0;
}

int ringbuf_peek(const RingBuf *rb, uint16_t offset, uint8_t *byte)   /* 读偏移处字节但不移动读索引 */
{
    uint16_t available;

    if ((rb == NULL) || (byte == NULL)) return -1;

    available = (rb->head + rb->capacity - rb->tail) % rb->capacity;
    if (offset >= available) return -1;

    *byte = rb->buf[(rb->tail + offset) % rb->capacity];
    return 0;
}

void ringbuf_flush(RingBuf *rb)   /* 清空并复位读写索引 */
{
    rb->head = 0;
    rb->tail = 0;
}
