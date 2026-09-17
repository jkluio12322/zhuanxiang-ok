/**
 * @file    yingYongKongZhi.h
 * @brief   应用层电机控制与循迹控制接口。
 */
#ifndef __YING_YONG_KONG_ZHI_H
#define __YING_YONG_KONG_ZHI_H  /* 头文件保护宏，防止接口重复声明。 */

/**
 * @brief  执行 50 ms 周期控制任务。
 * @note   依次完成编码器测速、命令执行、循迹状态机计算、左右独立增量式速度环更新、
 *         低速补偿、电机输出和遥测数据填充。
 */
void appControlTask50ms(void);

/**
 * @brief Reset motion, filter, and tracking state after a mode change.
 */
void appControlResetModeState(void);

#endif
