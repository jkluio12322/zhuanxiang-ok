#include "ringbuf.h"
#include "main.h"

void ringbuf_init(RingBuf *rb, uint8_t *storage, uint16_t capacity)
{
    uint16_t cap = 1U;

    if ((rb == 0) || (storage == 0)) {
        return;
    }

    /* 向下取到 2 的幂，保证掩码运算成立 */
    while ((uint32_t)cap * 2U <= (uint32_t)capacity) {
        cap = (uint16_t)(cap << 1U);
    }

    rb->buf = storage;
    rb->mask = (uint16_t)(cap - 1U);
    rb->head = 0U;
    rb->tail = 0U;
    rb->overflow = 0U;
    rb->dropped = 0U;
    __DMB();
}

void ringbuf_reset(RingBuf *rb)
{
    if (rb == 0) {
        return;
    }
    rb->tail = rb->head;
    rb->overflow = 0U;
    rb->dropped = 0U;
    __DMB();
}

uint16_t ringbuf_capacity(const RingBuf *rb)
{
    if (rb == 0) {
        return 0U;
    }
    return (uint16_t)(rb->mask + 1U);
}

uint16_t ringbuf_used(const RingBuf *rb)
{
    uint16_t head;
    uint16_t tail;

    if (rb == 0) {
        return 0U;
    }
    head = rb->head;
    __DMB();
    tail = rb->tail;
    return (uint16_t)((uint16_t)(head - tail) & rb->mask);
}

void ringbuf_unget(RingBuf *rb, uint16_t n)
{
    if ((rb == 0) || (n == 0U)) {
        return;
    }
    if (n > rb->mask) {
        return;
    }
    /* tail 只由消费者写，回退不与生产者竞争 */
    rb->tail = (uint16_t)((rb->tail - n) & rb->mask);
}

uint16_t ringbuf_free(const RingBuf *rb)
{
    uint16_t cap;

    if (rb == 0) {
        return 0U;
    }
    cap = (uint16_t)(rb->mask + 1U);
    return (uint16_t)((uint16_t)(cap - 1U) - ringbuf_used(rb));
}

uint16_t ringbuf_put(RingBuf *rb, const uint8_t *src, uint16_t len)
{
    uint16_t head;
    uint16_t tail;
    uint16_t free_cnt;
    uint16_t n;
    uint16_t i;

    if ((rb == 0) || (src == 0) || (len == 0U)) {
        return 0U;
    }

    head = rb->head;
    __DMB();
    tail = rb->tail;

    /* 保留一个空位：(tail - head - 1) & mask 就是还能写的字节数 */
    free_cnt = (uint16_t)((uint16_t)(tail - head - 1U) & rb->mask);
    n = (len < free_cnt) ? len : free_cnt;

    for (i = 0U; i < n; i++) {
        rb->buf[(uint16_t)((uint16_t)(head + i) & rb->mask)] = src[i];
    }

    __DMB();
    rb->head = (uint16_t)((uint16_t)(head + n) & rb->mask);

    if (n < len) {
        rb->overflow += (uint32_t)(len - n);
    }
    return n;
}

uint16_t ringbuf_get(RingBuf *rb, uint8_t *dst, uint16_t len)
{
    uint16_t tail;
    uint16_t head;
    uint16_t avail;
    uint16_t n;
    uint16_t i;

    if ((rb == 0) || (dst == 0) || (len == 0U)) {
        return 0U;
    }

    tail = rb->tail;
    __DMB();
    head = rb->head;

    avail = (uint16_t)((uint16_t)(head - tail) & rb->mask);
    n = (len < avail) ? len : avail;

    for (i = 0U; i < n; i++) {
        dst[i] = rb->buf[(uint16_t)((uint16_t)(tail + i) & rb->mask)];
    }

    __DMB();
    rb->tail = (uint16_t)((uint16_t)(tail + n) & rb->mask);

    if (n < len) {
        rb->dropped++;
    }
    return n;
}

uint8_t ringbuf_put_byte(RingBuf *rb, uint8_t byte)
{
    uint16_t head;
    uint16_t tail;

    if (rb == 0) {
        return 0U;
    }

    head = rb->head;
    __DMB();
    tail = rb->tail;

    if ((uint16_t)((uint16_t)(head + 1U) & rb->mask) == tail) {
        rb->overflow++;
        return 0U;
    }

    rb->buf[head] = byte;
    __DMB();
    rb->head = (uint16_t)((uint16_t)(head + 1U) & rb->mask);
    return 1U;
}

uint8_t ringbuf_get_byte(RingBuf *rb, uint8_t *byte)
{
    uint16_t tail;
    uint16_t head;

    if ((rb == 0) || (byte == 0)) {
        return 0U;
    }

    tail = rb->tail;
    __DMB();
    head = rb->head;

    if (tail == head) {
        rb->dropped++;
        return 0U;
    }

    *byte = rb->buf[tail];
    __DMB();
    rb->tail = (uint16_t)((uint16_t)(tail + 1U) & rb->mask);
    return 1U;
}

uint8_t ringbuf_peek_byte(const RingBuf *rb, uint8_t *byte)
{
    uint16_t tail;
    uint16_t head;

    if ((rb == 0) || (byte == 0)) {
        return 0U;
    }

    tail = rb->tail;
    __DMB();
    head = rb->head;

    if (tail == head) {
        return 0U;
    }

    *byte = rb->buf[tail];
    return 1U;
}
