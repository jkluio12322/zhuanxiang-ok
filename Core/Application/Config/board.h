/**
 * @file    board.h
 * @brief   板级硬件绑定：电机方向控制引脚的语义别名。
 * @note    这里只放需要 CubeMX 类型定义的宏，所以必须包含 main.h。
 *          纯数值参数不放在这里，放到 app_config.h。
 *
 * @warning 下面这八个引脚别名是从原 yingYongPeiZhi.h 逐字搬过来的，
 *          **不要改动，也不要照文档"理顺"**：
 *          - 电机 A 用 pwm1_0 / pwm1_1
 *          - 电机 B 用 pwm2_1 / pwm2_0（与 A 相反，B 轮的 forward/reverse
 *            对调就发生在这一层，不是在 motor.c 的函数体里）
 *          这套接线是实车调通的，改一个字轮子就可能反转。
 */
#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"
#include "app_config.h"   /* motor.c 还用 PWM 的 CCR 换算宏，这里一并带上 */

/* 电机 A 正转方向控制 GPIO 端口，实际端口由 CubeMX main.h 定义。 */
#define MOTOR_A_FORWARD_GPIO_PORT     pwm1_0_GPIO_Port

/* 电机 A 正转方向控制 GPIO 引脚，实际引脚由 CubeMX main.h 定义。 */
#define MOTOR_A_FORWARD_PIN           pwm1_0_Pin

/* 电机 A 反转方向控制 GPIO 端口。 */
#define MOTOR_A_REVERSE_GPIO_PORT     pwm1_1_GPIO_Port

/* 电机 A 反转方向控制 GPIO 引脚。 */
#define MOTOR_A_REVERSE_PIN           pwm1_1_Pin

/* 电机 B 正转方向控制 GPIO 端口。 */
#define MOTOR_B_FORWARD_GPIO_PORT     pwm2_1_GPIO_Port

/* 电机 B 正转方向控制 GPIO 引脚。 */
#define MOTOR_B_FORWARD_PIN           pwm2_1_Pin

/* 电机 B 反转方向控制 GPIO 端口。 */
#define MOTOR_B_REVERSE_GPIO_PORT     pwm2_0_GPIO_Port

/* 电机 B 反转方向控制 GPIO 引脚。 */
#define MOTOR_B_REVERSE_PIN           pwm2_0_Pin

#endif /* __BOARD_H */
