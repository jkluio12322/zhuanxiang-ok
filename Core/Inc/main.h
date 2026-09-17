#ifndef __MAIN_H
#define __MAIN_H  /* 头文件保护宏，防止 CubeMX 引脚定义被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "yingYongLeiXing.h"

/**
 * @brief  进入不可恢复错误状态。
 * @note   关闭全局中断并停留在死循环中，避免错误状态下继续驱动硬件。
 */
void Error_Handler(void);

/* CubeMX generated pin labels. */
#define pwm1_1_Pin GPIO_PIN_1
#define pwm1_1_GPIO_Port GPIOF
#define pwm1_0_Pin GPIO_PIN_2
#define pwm1_0_GPIO_Port GPIOF
#define pwm2_1_Pin GPIO_PIN_3
#define pwm2_1_GPIO_Port GPIOF
#define pwm2_0_Pin GPIO_PIN_4
#define pwm2_0_GPIO_Port GPIOF

#define buzzer_Pin GPIO_PIN_6
#define buzzer_GPIO_Port GPIOG

#define key3_Pin GPIO_PIN_6
#define key3_GPIO_Port GPIOC
#define key3_EXTI_IRQn EXTI9_5_IRQn
#define key2_Pin GPIO_PIN_7
#define key2_GPIO_Port GPIOC
#define key2_EXTI_IRQn EXTI9_5_IRQn
#define key1_Pin GPIO_PIN_8
#define key1_GPIO_Port GPIOC
#define key1_EXTI_IRQn EXTI9_5_IRQn
#define key4_Pin GPIO_PIN_9
#define key4_GPIO_Port GPIOC
#define key4_EXTI_IRQn EXTI9_5_IRQn

#define IR1_Pin GPIO_PIN_2
#define IR1_GPIO_Port GPIOD
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

#define MPU6050_INT_Pin GPIO_PIN_12
#define MPU6050_INT_GPIO_Port GPIOA
#define MPU6050_INT_EXTI_IRQn EXTI15_10_IRQn

#ifdef __cplusplus
}
#endif

#endif
