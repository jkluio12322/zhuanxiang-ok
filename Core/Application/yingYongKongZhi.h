/**
 * @file    yingYongKongZhi.h
 * @brief   应用层周期速度控制接口。
 */
#ifndef __YING_YONG_KONG_ZHI_H
#define __YING_YONG_KONG_ZHI_H  /* 头文件保护宏，防止接口重复声明。 */

/**
 * @brief  执行 50 ms 周期控制任务。
 * @note   依次完成编码器测速、命令执行、循迹目标消费、左右独立增量式速度环
 *         更新、低速补偿、电机输出和遥测数据填充。循迹状态机由 20 ms 任务运行。
 */
void appControlTask50ms(void);

/**
 * @brief  模式切换后复位运动、滤波器和循迹状态。
 * @note   该函数不会修改 PID 参数。
 */
void appControlResetModeState(void);

#endif
