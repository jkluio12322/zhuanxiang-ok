/**
 * @file    yingYongChuanGanQi.h
 * @brief   应用层传感器任务接口。
 */
#ifndef __YING_YONG_CHUAN_GAN_QI_H
#define __YING_YONG_CHUAN_GAN_QI_H  /* 头文件保护宏，防止接口重复声明。 */

/**
 * @brief  执行 10 ms 周期 MPU6050 数据处理任务。
 * @note   只有异步 I2C 读取完成后才更新原始数据和 AHRS，
 *         然后根据数据就绪标志启动下一次读取。
 */
void appSensorTask10ms(void);

#endif
