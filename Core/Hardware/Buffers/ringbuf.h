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

void RingBuf_Init(RingBuf *rb, uint8_t *buf, uint16_t capacity);            /* 初始化 */
uint16_t RingBuf_Available(RingBuf *rb);                                    /* 可读字节数 */
uint16_t RingBuf_Free(RingBuf *rb);                                         /* 剩余可写空间 */
int RingBuf_Put(RingBuf *rb, uint8_t byte);                                 /* 阻塞写一个字节；禁止在中断上下文调用 */
int RingBuf_PutNB(RingBuf *rb, uint8_t byte);                               /* 非阻塞写，满时返回 -1 */
int RingBuf_Get(RingBuf *rb, uint8_t *byte);                                /* 非阻塞读，空时返回 -1 */
int RingBuf_Peek(const RingBuf *rb, uint16_t offset, uint8_t *byte);        /* 读偏移处字节，不移动读索引 */
void RingBuf_Flush(RingBuf *rb);                                            /* 清空 */

#endif /* __RINGBUF_H */
