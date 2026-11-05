/* BlueCar 程序入口与任务编排。 */

#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"

/* USER CODE BEGIN Includes */
#include "cmd.h"
#include "config.h"
#include "imu.h"
#include "motion.h"
#include "sched.h"
#include "speed_loop.h"
#include "state.h"
#include "testmode.h"
#include "track.h"
#include "display.h"
#include "telemetry.h"
#include "buzzer.h"
#include "encoder.h"
#include "irtrack.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "console.h"
#include "serial.h"
#include "timer.h"
/* USER CODE END Includes */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* 程序入口，正常情况下不会返回。
   主循环只做延后任务（遥测发送、OLED 刷新）和串口命令行解析——
   中断里不做 snprintf 与 I2C，避免压住串口接收。 */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  /* OLED 初始化是非阻塞状态机：这里只是等它走完（约 150 ms），全程没有 HAL_Delay。
     蜂鸣器跟着初始化一起响，初始化结束就关。 */
  buzzer_init();
  buzzer_on();
  oled_init_begin();
  while (oled_init_poll() == 0U)
  {
    oled_recover_poll();
  }
  buzzer_off();

  motor_init();
  ir_init();
  imu_init();
  encoder_init();
  motion_init();
  speed_loop_Init();
  testmode_init();
  track_init();
  timer_sched_init();
  timer_ms_init();
  key_init();
  serial_init(&huart1);
  telemetry_boot_header();
  console_init();

  state_init();
  cmd_init();
  telemetry_init();

  /* 所有模块都初始化完了才放行中断任务，避免中断改到未初始化的对象。 */
  sched_init();

  /* 开机菜单非阻塞：主循环照常推进，遥测从第一帧起就是 20 Hz。 */
  display_boot_begin();
  while (display_boot_poll() == 0U)
  {
    deferred_run();       /* 菜单期间照常做 IMU 采样与遥测发送 */
    serial_process();
    console_process();
    console_flush();
    console_diag_task();
    oled_flush();
    oled_recover_poll();
  }

  oled_clear();
  oled_show_string(2, 1, " Calibrating... ");
  oled_flush();

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    deferred_run();       /* 姿态采样、遥测发送、OLED 刷新（由调度器置标志） */
    serial_process();

    console_process();    /* 串口命令行解析与响应 */
    console_flush();
    console_diag_task();

    oled_flush();            /* OLED 缓冲写入 */
    oled_recover_poll();     /* I2C 总线恢复状态机（没触发时立即返回） */
    /* USER CODE END 3 */
  }
}

/* 8 MHz 外部晶振经 PLL 九倍频得到 72 MHz 系统时钟；APB1 二分频 36 MHz，APB2 保持 72 MHz。 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* HAL 或应用初始化失败时进入：关全局中断后停在这里，便于调试器定位现场。 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/* 完整断言下的参数检查失败回调。当前不执行额外动作，便于在此设断点。 */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
