/**
 * @file    zhanShuBao.h
 * @brief   战术包模块接口。
 * @note    该模块存放可复用的战术动作。当前提供基于编码器标定值的急弯转向，
 *          查表循迹检测到急弯后直接调用本模块即可。
 */
#ifndef __ZHAN_SHU_BAO_H
#define __ZHAN_SHU_BAO_H  /* 头文件保护宏，防止接口被重复包含。 */

#include <stdint.h>

/**
 * @brief  清除急弯转向的锁存状态和编码器起点。
 * @note   重新启动循迹、急弯超时或强制停车时必须调用，防止下一次转弯
 *         复用上一次的编码器累计值。
 */
void zhanShuBaoChongZhiZhuan90Du(void);

/**
 * @brief  执行一次基于编码器标定值的急弯转向。
 * @param  zhuanXiang 转弯方向，正值为右转，负值为左转。
 * @return 0 表示仍在转向，1 表示编码器已达到标定阈值并停止输出。
 * @note   第一次调用时自动锁存左右编码器累计计数，并输出外侧轮 +50 RPM、
 *         内侧轮 -50 RPM 的反向目标转速；每个循迹周期调用一次，达到目标计数差后
 *         自动清零左右目标并返回 1，表示编码器转弯计数已达到标定阈值。
 */
uint8_t zhanShuBaoZhuan90Du(int8_t zhuanXiang);

/**
 * @brief  判断八路红外是否全部检测到黑线。
 * @return 1 表示八路 raw 全部为 0，也就是八路指示状态全灭且全部压黑线；
 *         0 表示至少有一路没有压到黑线。
 * @note   当前底层定义 raw=0 表示黑线、raw=1 表示白线。该函数将黑线全部
 *         压住的状态统一放在战术包中，循迹和后续动作都可以直接调用。
 */
uint8_t zhanShuBaoYuDaoHeiXian(void);

/**
 * @brief  判断八路红外是否全部丢失黑线。
 * @return 1 表示八路 raw 全部为 1，也就是八路都没有检测到黑线；
 *         0 表示至少有一路仍然压着黑线。
 * @note   当前底层定义 raw=1 表示白线、raw=0 表示黑线。循迹状态机可直接
 *         使用该函数进入丢线搜索，避免在多个模块重复编写判断。
 */
uint8_t zhanShuBaoDiuXian(void);

#endif
