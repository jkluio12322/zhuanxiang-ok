/**
 * @file    ringbuf.h
 * @brief   裸机模块间通信使用的单生产者单消费者环形缓冲区。
 * @note    head 只由生产者写入，tail 只由消费者写入。
 *
 *          使用示例:
 *            RingBuf rb;
 *            uint8_t buf[256];
 *            RingBuf_Init(&rb, buf, sizeof(buf));
 *            RingBuf_Put(&rb, 0xAA);     // 阻塞写入
 *            RingBuf_PutNB(&rb, 0xAA);   // 非阻塞，满时返回 -1
 *            RingBuf_Get(&rb, &byte);    // 成功返回 0，空时返回 -1
 */

#ifndef __RINGBUF_H
#define __RINGBUF_H  /* 头文件保护宏，防止环形缓冲接口重复声明。 */

#include <stdint.h>

/* ---- 通用字节环形缓冲区 ---- */
typedef struct {
    uint8_t          *buf;
    uint16_t          capacity;    /* 缓冲区总容量，单位字节。 */
    volatile uint16_t head;        /* 写索引，只由生产者修改。 */
    volatile uint16_t tail;        /* 读索引，只由消费者修改。 */
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
 * @note   缓冲区满时忙等；禁止在中断上下文调用。
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
