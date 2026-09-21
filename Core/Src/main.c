/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BlueCar application entry and task scheduling.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* USER CODE BEGIN Includes */
#include "yingYongMingLing.h"
#include "yingYongPeiZhi.h"
#include "yingYongXianShi.h"
#include "yingYongZhuangTai.h"
#include "yingYongYaoCe.h"
#include "blueserial.h"
#include "buzzer.h"
#include "encoder.h"
#include "irtrack.h"
#include "key.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"
#include "pid_tune.h"
#include "serial.h"
#include "timer.h"
/* USER CODE END Includes */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  程序入口。
 * @return 正常情况下不会返回。
 * @note   完成 HAL、CubeMX 外设和应用模块初始化后进入主循环，
 *         主循环负责调参响应、蓝牙、按键、OLED 和周期显示刷新。
 */
int main(void)
{
  /* HAL 和 CubeMX 外设初始化。 */
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
  /* 初始化显示、电机、传感器、编码器、通信和 PID 调参模块。 */
  OLED_Init();
  HAL_Delay(APP_OLED_BOOT_DELAY_MS);

  Motor_Init();
  IRTrack_Init(IR_MODE_DIGITAL);
  MPU6050_init(&hi2c1);
  Encoder_Init_TIM_All();
  Timer_Init();
  TIM5_Init();
  Key_Init();
  Serial_Init(&huart1);
  Serial_Printf("BOOT OK\r\n");
  Serial_Printf("#tgtR,rpmR,pwmR,tgtL,rpmL,pwmL,yaw,diff,yawA,lv,rh,run,gear,rpmFR,rpmFL\r\n");
  pidTuneInit();
  Buzzer_Init();
  BlueSerial_Init();

    /* 初始化应用层状态和命令队列。 */
  appStateInit();
  appCommandInit();
  appTelemetryInit();

  /* Start IMU sampling before the boot mode screen so yaw is live there too. */
  MPU6050_ReadAsync_Start();

  appDisplayShowBootScreen();
  Buzzer_ON();
  HAL_Delay(APP_BOOT_BEEP_MS);
  Buzzer_OFF();

  /* 等待指定按键启动，避免上电后立即进入控制状态。 */
  appBootConfigRun();
  OLED_Clear();
  OLED_ShowString(2, 1, " Calibrating... ");
  OLED_Flush();

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    /* 处理串口调参响应、蓝牙命令和按键事件。 */
    pidTuneProcessResponse();

    appCommandProcessBluetooth();
    appCommandProcessKeys();
    appCommandProcessTune();

    /* OLED 缓冲写入和状态页刷新。 */
    OLED_Flush();

    static uint32_t displayTick = 0;
    if (milliseconds - displayTick > APP_DISPLAY_PERIOD_MS) {
      displayTick = milliseconds;
      appDisplayUpdate();
    }
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/**
 * @brief  配置 STM32F103 系统时钟。
 * @note   使用 8 MHz 外部晶振，经 PLL 九倍频得到 72 MHz 系统时钟；
 *         APB1 二分频为 36 MHz，APB2 保持 72 MHz。
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 使用 8 MHz 外部晶振，经 PLL 倍频得到 72 MHz 系统时钟。 */
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

/**
 * @brief  HAL 或应用初始化失败时进入的错误处理函数。
 * @note   关闭全局中断后停留在死循环，便于调试器定位错误现场。
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  启用完整断言时的参数检查失败回调。
 * @param  file 触发断言的源文件名。
 * @param  line 触发断言的行号。
 * @note   当前实现不执行额外动作，只显式忽略参数，便于在此处设置断点。
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
