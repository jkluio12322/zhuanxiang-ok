/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BlueCar - 蓝牙遥控 + 按键双控四轮小车
  *                   硬件: STM32F103ZET6 + TB6612 + JGB37-520 + HC-05 + MPU6050 + OLED
  *
  *                   按键功能:
  *                     C7(KEY1): 单击 → 进入/退出 PWM 设置模式
  *                     C6(KEY2): PWM模式 +10% / RUN模式 按住前进
  *                     C8(KEY3): PWM模式 -10% / RUN模式 按住后退
  *
  *                   蓝牙协议 (兼容平衡车APP):
  *                     [joystick,LH,LV,RH,RV] → LV=前后, RH=转向
  *                     [key,1,up] → 启停切换
  *                     [key,2,up] → 速度档位-1
  *                     [key,3,up] → 速度档位+1
  *
  *                   编码器数据: USART3 每 50ms 发送 CSV 格式 (VOFA+)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "oled.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "mpu6050.h"
#include "key.h"
#include "buzzer.h"
#include "serial.h"
#include "timer.h"
#include "blueserial.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum { MODE_RUN, MODE_PWM_SET } Mode_t;

/* Private variables ---------------------------------------------------------*/
static Mode_t   Mode       = MODE_RUN;
static uint8_t  RunFlag    = 0;
static int16_t  PWMDuty    = 0;
static uint8_t  SpeedLevel = 3;        /* 蓝牙速度档位 1~6 */
static uint32_t bt_last_rx = 0;        /* 上次收到蓝牙数据的时间戳 */
#define RPM_PER_LEVEL  30.0f

static Encoder  ecd_left, ecd_right;
static PID      SpeedPID, TurnPID;

static int16_t  AX, AY, AZ, GX, GY, GZ;
static int16_t  GY_Offset  = 0;
static float    YawAngle   = 0.0f;

static const Parameter JGB37_520 = {
    .multiple = 4, .reduction_ratio = 30.0f, .ppr = 11, .r = 0.065f,
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Process_Keys(void);
static void Process_Bluetooth(void);
static void Process_Control(void);
static void Update_OLED(void);
static void VOFA_Send(void);
static void MPU6050_Calibrate(void);

/* Private user code ---------------------------------------------------------*/

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    OLED_Init();
    Motor_Init();
    MPU6050_Init();
    Encoder_Init_TIM_All();
    Timer_Init();
    TIM5_Init();
    Key_Init();
    Serial_Init();
    Buzzer_Init();
    BlueSerial_Init();

    initEncoder(&ecd_left,  JGB37_520);
    initEncoder(&ecd_right, JGB37_520);

    initPID(&SpeedPID, 100.0f, 200.0f);
    setPIDParam(&SpeedPID, 2.0f, 0.5f, 0.1f);   /* Kp=2.0 Ki=0.5 Kd=0.1 */

    initPID(&TurnPID,   30.0f,  60.0f);
    setPIDParam(&TurnPID,   1.0f, 0.3f, 0.05f);  /* Kp=1.0 Ki=0.3 Kd=0.05 */

    setPIDTarget(&SpeedPID, 0);
    setPIDTarget(&TurnPID,   0);

    MPU6050_Calibrate();

    OLED_Clear();
    OLED_ShowString(1, 1, " [BlueCar V2.1] ");
    OLED_ShowString(2, 1, " BT  + Encoder  ");
    OLED_ShowString(3, 1, " Keys: F/B/PWM  ");
    OLED_ShowString(4, 1, " Press C8 start ");
    Buzzer_ON();  HAL_Delay(200);  Buzzer_OFF();

    while (Key_GetNum() != 3);
    OLED_Clear();

    while (1)
    {
        Process_Bluetooth();   /* 蓝牙先跑, 防止 Process_Keys 误清目标 */
        Process_Keys();

        if (flag_10ms) {
            flag_10ms = 0;
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            float gz = (float)(GZ - GY_Offset) / 32768.0f * 2000.0f;
            YawAngle += gz * 0.01f;
        }

        if (flag_50ms) {
            flag_50ms = 0;
            updateEncoderLoopSimpleVersion(&ecd_left,  50, TIM2);
            updateEncoderLoopSimpleVersion(&ecd_right, 50, TIM3);

            if (Mode == MODE_RUN)
                Process_Control();
            else
                Motor_SetSpeedA(PWMDuty), Motor_SetSpeedB(PWMDuty);

            VOFA_Send();
        }

        static uint32_t oled_tick = 0;
        if (millis - oled_tick > 200) {
            oled_tick = millis;
            __disable_irq();
            Update_OLED();
            __enable_irq();
        }

        if (!RunFlag) {
            Motor_SetSpeedA(0); Motor_SetSpeedB(0);
        }
    }
}

/*============================================================
 * 按键处理
 *============================================================*/
static void Process_Keys(void)
{
    if (Key_Check(KEY_1, KEY_SINGLE)) {
        if (Mode == MODE_RUN) {
            Mode = MODE_PWM_SET;
            RunFlag = 0;
            Motor_SetSpeedA(0); Motor_SetSpeedB(0);
        } else {
            Mode = MODE_RUN;
            PWMDuty = 0;
        }
    }

    if (Mode == MODE_PWM_SET) {
        if (Key_Check(KEY_2, KEY_SINGLE)) { PWMDuty += 10; if (PWMDuty >  100) PWMDuty =  100; }
        if (Key_Check(KEY_3, KEY_SINGLE)) { PWMDuty -= 10; if (PWMDuty < -100) PWMDuty = -100; }
        RunFlag = (PWMDuty != 0);
    } else {
        /* RUN 模式: C6 前进 / C8 后退, 蓝牙可覆盖 */
        if (Key_Check(KEY_2, KEY_HOLD)) {
            RunFlag = 1;
            setPIDTarget(&SpeedPID, +50.0f);
            setPIDTarget(&TurnPID,   0);
        }
        else if (Key_Check(KEY_3, KEY_HOLD)) {
            RunFlag = 1;
            setPIDTarget(&SpeedPID, -50.0f);
            setPIDTarget(&TurnPID,   0);
        }
        else if (millis - bt_last_rx > 300) {
            /* 300ms 内没按键也没蓝牙 → 停车 */
            RunFlag = 0;
            setPIDTarget(&SpeedPID, 0);
            setPIDTarget(&TurnPID,   0);
        }
    }
}

/*============================================================
 * 蓝牙协议解析 (兼容平衡车APP)
 *============================================================*/
static void Process_Bluetooth(void)
{
    if (Mode == MODE_PWM_SET) return;
    if (!BlueSerial_ReceiveFlag()) return;
    BlueSerial_Receive();
    bt_last_rx = millis;   /* 记录收到蓝牙数据的时间 */

    /* 摇杆: [joystick,LH,LV,RH,RV] */
    if (strcmp(BlueSerial_StringArray[0], "joystick") == 0) {
        int8_t LV = atoi(BlueSerial_StringArray[2]);
        int8_t RH = atoi(BlueSerial_StringArray[3]);
        if (LV != 0 || RH != 0) {
            RunFlag = 1;
            setPIDTarget(&SpeedPID, LV / 100.0f * SpeedLevel * RPM_PER_LEVEL);
            setPIDTarget(&TurnPID,  RH / 100.0f * SpeedLevel * RPM_PER_LEVEL * 0.5f);
        } else {
            /* 摇杆回中 → 停车, 但不立即清 RunFlag (等 300ms 超时) */
            setPIDTarget(&SpeedPID, 0);
            setPIDTarget(&TurnPID,   0);
        }
    }
    /* 按键: [key,N,up/down] */
    else if (strcmp(BlueSerial_StringArray[0], "key") == 0) {
        if (strcmp(BlueSerial_StringArray[1], "1") == 0
         && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            RunFlag = !RunFlag;
            if (!RunFlag) { Motor_SetSpeedA(0); Motor_SetSpeedB(0); }
        }
        else if (strcmp(BlueSerial_StringArray[1], "2") == 0
              && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            if (SpeedLevel > 1) SpeedLevel--;
        }
        else if (strcmp(BlueSerial_StringArray[1], "3") == 0
              && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            if (SpeedLevel < 6) SpeedLevel++;
        }
    }
}

/*============================================================
 * PID 控制: 转速闭环
 *============================================================*/
static void Process_Control(void)
{
    float ave = (ecd_left.velocity.angular + ecd_right.velocity.angular) / 2.0f;
    float dif = ecd_left.velocity.angular - ecd_right.velocity.angular;

    if (RunFlag) {
        updatePID(&SpeedPID, ave);
        updatePID(&TurnPID,  dif);

        float base = SpeedPID.output;
        float diff = TurnPID.output;
        int16_t L = (int16_t)(base + diff);
        int16_t R = (int16_t)(base - diff);

        if (L >  100) L =  100;  if (L < -100) L = -100;
        if (R >  100) R =  100;  if (R < -100) R = -100;

        Motor_SetSpeedA(L);
        Motor_SetSpeedB(R);
    }
}

/*============================================================
 * OLED 显示
 *============================================================*/
static void Update_OLED(void)
{
    char buf[21];
    if (Mode == MODE_PWM_SET) {
        OLED_ShowString(1, 1, "[PWM SET MODE]  ");
        sprintf(buf, "Duty: %+04d %%    ", PWMDuty);
    } else {
        OLED_ShowString(1, 1, RunFlag ? "[RUN] BT+Key   " : "[STOP]          ");
        sprintf(buf, "T:%+05.0f D:%+05.0f ", SpeedPID.target, TurnPID.target);
    }
    OLED_ShowString(2, 1, buf);

    sprintf(buf, "L:%+05.0f R:%+05.0f ", ecd_left.velocity.angular, ecd_right.velocity.angular);
    OLED_ShowString(3, 1, buf);

    sprintf(buf, "Yaw:%+06.1f Lv:%1d", YawAngle, SpeedLevel);
    OLED_ShowString(4, 1, buf);
}

/*============================================================
 * VOFA+: 编码器数据通过 USART3 发送
 *============================================================*/
static void VOFA_Send(void)
{
    Serial_Printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\r\n",
        SpeedPID.target,
        ecd_left.velocity.angular,
        ecd_right.velocity.angular,
        TurnPID.target,
        ecd_left.velocity.angular - ecd_right.velocity.angular,
        YawAngle,
        RunFlag);
}

/*============================================================
 * MPU6050 零偏校准
 *============================================================*/
static void MPU6050_Calibrate(void)
{
    int32_t sum = 0;
    for (uint16_t i = 0; i < 200; i++) {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum += GZ;
        HAL_Delay(5);
    }
    GY_Offset = (int16_t)(sum / 200);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef  clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.HSIState = RCC_HSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) {}
#endif
