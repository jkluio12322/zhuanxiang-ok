#ifndef __MAIN_H
#define __MAIN_H  /* 头文件保护宏，防止 CubeMX 引脚定义被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "types.h"

/**
 * @brief  进入不可恢复错误状态。
 * @note   关闭全局中断并停留在死循环中，避免错误状态下继续驱动硬件。
 */
void Error_Handler(void);

/* TB6612 电机方向控制 GPIO。 */
#define pwm1_1_Pin GPIO_PIN_1
#define pwm1_1_GPIO_Port GPIOF
#define pwm1_0_Pin GPIO_PIN_2
#define pwm1_0_GPIO_Port GPIOF
#define pwm2_1_Pin GPIO_PIN_3
#define pwm2_1_GPIO_Port GPIOF
#define pwm2_0_Pin GPIO_PIN_4
#define pwm2_0_GPIO_Port GPIOF

/* 有源蜂鸣器，PG6 低电平开启。 */
#define buzzer_Pin GPIO_PIN_6
#define buzzer_GPIO_Port GPIOG

/* 拓展板四键：K1=PG2、K2=PG3、K3=PD14、K4=PD15，输入上拉，按下=低电平。
   全部由 key.c 每 20 ms 轮询采样，不配置 EXTI，因此不再有 keyN_EXTI_IRQn。 */
#define K1_Pin GPIO_PIN_2
#define K1_GPIO_Port GPIOG
#define K2_Pin GPIO_PIN_3
#define K2_GPIO_Port GPIOG
#define K3_Pin GPIO_PIN_14
#define K3_GPIO_Port GPIOD
#define K4_Pin GPIO_PIN_15
#define K4_GPIO_Port GPIOD

/* 八路循迹输入：IR1 在 GPIOG，IR2-IR6 在 GPIOD，IR7-IR8 在 GPIOG。 */
#define IR1_Pin GPIO_PIN_11
#define IR1_GPIO_Port GPIOG
#define IR2_Pin GPIO_PIN_3
#define IR2_GPIO_Port GPIOD
#define IR3_Pin GPIO_PIN_4
#define IR3_GPIO_Port GPIOD
#define IR4_Pin GPIO_PIN_5
#define IR4_GPIO_Port GPIOD
#define IR5_Pin GPIO_PIN_6
#define IR5_GPIO_Port GPIOD
#define IR6_Pin GPIO_PIN_7
#define IR6_GPIO_Port GPIOD
#define IR7_Pin GPIO_PIN_9
#define IR7_GPIO_Port GPIOG
#define IR8_Pin GPIO_PIN_10
#define IR8_GPIO_Port GPIOG

/* IMU963RA（LSM6DSR）SPI2 软件片选，低电平选中。 */
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB

/* IMU963RA INT 输出：PE15 输入下拉。本工程按周期轮询采样，不启用外部中断。 */
#define IMU963RA_INT_Pin GPIO_PIN_15
#define IMU963RA_INT_GPIO_Port GPIOE

#ifdef __cplusplus
}
#endif

#endif
