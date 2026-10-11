/* BlueCar 程序入口与任务编排。 */

#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* USER CODE BEGIN Includes */
#include "app_cmd.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_motion.h"
#include "app_scheduler.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "app_tracking.h"
#include "app_display.h"
#include "app_telemetry.h"
#include "buzzer.h"
#include "encoder.h"
#include "irtrack.h"
#include "key.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"
#include "app_console.h"
#include "app_serial.h"
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
  MX_I2C1_Init();
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
  OLED_Init();
  HAL_Delay(APP_OLED_BOOT_DELAY_MS);

  Motor_Init();
  IRTrack_Init();
  AppImu_Init();
  Encoder_Init_TIM_All();
  AppMotion_Init();
  AppSpeedLoop_Init();
  AppTestMode_Init();
  AppTracking_Init();
  Timer_Init();
  TIM5_Init();
  Key_Init();
  Serial_Init(&huart1);
  AppTelemetry_BootHeader();
  AppConsole_Init();
  Buzzer_Init();

  AppState_Init();
  AppCmd_Init();
  AppTelemetry_Init();

  /* 所有模块都初始化完了才放行中断任务，避免中断改到未初始化的对象。 */
  AppScheduler_Init();

  AppDisplay_ShowBoot();
  Buzzer_ON();
  HAL_Delay(APP_BOOT_BEEP_MS);
  Buzzer_OFF();

  /* 等待指定按键启动，避免上电后立即进入控制状态。 */
  AppDisplay_BootConfig();
  OLED_Clear();
  OLED_ShowString(2, 1, " Calibrating... ");
  OLED_Flush();

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    AppDeferred_Run();       /* 遥测发送和 OLED 刷新（由调度器置标志） */

    AppConsole_Process();    /* 串口命令行解析与响应 */
    AppConsole_Flush();

    OLED_Flush();            /* OLED 缓冲写入 */
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
