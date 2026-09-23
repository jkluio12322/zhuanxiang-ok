/**
 * @file    yingYongXianShi.h
 * @brief   OLED 应用显示接口。
 */
#ifndef __YING_YONG_XIAN_SHI_H
#define __YING_YONG_XIAN_SHI_H  /* 头文件保护宏，防止接口重复声明。 */

/**
 * @brief  显示开机页。
 * @note   显示车辆名称、固件模块信息、通信方案和启动按键提示。
 */
void appDisplayShowBootScreen(void);

/**
 * @brief  运行开机配置页并等待用户启动。
 * @note   该函数会阻塞主流程，退出时复位运动状态并清空旧命令。
 */
void appBootConfigRun(void);

/**
 * @brief  根据当前工作模式刷新 OLED 状态页。
 * @note   显示循迹状态、PWM 测试信息或蓝牙速度状态。
 */
void appDisplayUpdate(void);

#endif
