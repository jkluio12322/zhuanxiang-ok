/* 板级硬件绑定：电机方向控制引脚的语义别名。只放需要 CubeMX 类型定义的宏，故须包含
   main.h；纯数值参数放 config.h。下面八个引脚别名是从旧配置逐字搬来的，电机 B 的
   forward/reverse 对调就发生在这一层（pwm2_1 / pwm2_0，与 A 相反），实车调通，改一个字
   轮子就可能反转。 */
#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"
#include "config.h"   /* motor.c 还用 PWM 的 CCR 换算宏，这里一并带上 */

/* 电机 A 正转方向控制 GPIO 端口。 */
#define MOTOR_A_FORWARD_GPIO_PORT     pwm1_0_GPIO_Port

/* 电机 A 正转方向控制 GPIO 引脚。 */
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

/* IMU963RA 的软件片选，PB12 低有效。 */
#define IMU_CS_GPIO_Port              GPIOB
#define IMU_CS_Pin                    GPIO_PIN_12

#endif /* __BOARD_H */
