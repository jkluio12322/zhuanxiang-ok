/* 单生产者 / 单消费者环形缓冲。
 *
 * 设计约束（保护措施都写在这里，实现不再重复判断）：
 *  - head 只由生产者推进，tail 只由消费者推进，两个指针各自单写；
 *  - 不关中断即可安全使用：中断里 put、主循环 get（或反过来）都不会撕裂；
 *  - 容量自动向下取 2 的幂，用掩码代替取模；
 *  - 留一格区分"满"和"空"，因此实际可用容量是 容量-1；
 *  - 写满时丢弃新数据并累加 overflow，读空时返回 0 并累加 dropped，绝不越界。
 */
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

typedef struct {
    uint8_t *buf;               /* 由调用方提供的存储区 */
    uint16_t mask;              /* 容量 - 1，容量是 2 的幂 */
    volatile uint16_t head;     /* 生产者写 */
    volatile uint16_t tail;     /* 消费者写 */
    volatile uint32_t overflow; /* 因满而丢弃的字节数 */
    volatile uint32_t dropped;  /* 因读空被截断的请求次数 */
} RingBuf;

void ringbuf_init(RingBuf *rb, uint8_t *storage, uint16_t capacity);
void ringbuf_reset(RingBuf *rb);

uint16_t ringbuf_capacity(const RingBuf *rb);
uint16_t ringbuf_used(const RingBuf *rb);
uint16_t ringbuf_free(const RingBuf *rb);

uint16_t ringbuf_put(RingBuf *rb, const uint8_t *src, uint16_t len);
uint16_t ringbuf_get(RingBuf *rb, uint8_t *dst, uint16_t len);

uint8_t ringbuf_put_byte(RingBuf *rb, uint8_t byte);
uint8_t ringbuf_get_byte(RingBuf *rb, uint8_t *byte);
uint8_t ringbuf_peek_byte(const RingBuf *rb, uint8_t *byte);

/* 消费者把刚刚取出的 n 个字节退回队列：仅消费者调用，用于发送失败重试 */
void ringbuf_unget(RingBuf *rb, uint16_t n);

#endif /* __RINGBUF_H */
