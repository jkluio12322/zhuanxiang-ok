/* 单生产者单消费者环形缓冲：head 只由生产者写、tail 只由消费者写。 */

#ifndef __RINGBUF_H
#define __RINGBUF_H  /* 头文件保护 */
#include <stdint.h>

typedef struct {
    uint8_t          *buf;
    uint16_t          capacity;    /* 总容量，字节 */
    volatile uint16_t head;        /* 写索引，只由生产者改 */
    volatile uint16_t tail;        /* 读索引，只由消费者改 */
} RingBuf;

void ringbuf_init(RingBuf *rb, uint8_t *buf, uint16_t capacity);            /* 初始化 */
uint16_t ringbuf_available(RingBuf *rb);                                    /* 可读字节数 */
uint16_t ringbuf_free(RingBuf *rb);                                         /* 剩余可写空间 */
int ringbuf_put(RingBuf *rb, uint8_t byte);                                 /* 阻塞写一个字节；禁止在中断上下文调用 */
int ringbuf_put_nb(RingBuf *rb, uint8_t byte);                               /* 非阻塞写，满时返回 -1 */
int ringbuf_get(RingBuf *rb, uint8_t *byte);                                /* 非阻塞读，空时返回 -1 */
int ringbuf_peek(const RingBuf *rb, uint16_t offset, uint8_t *byte);        /* 读偏移处字节，不移动读索引 */
void ringbuf_flush(RingBuf *rb);                                            /* 清空 */

#endif /* __RINGBUF_H */
