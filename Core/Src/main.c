/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BlueCar - 蓝牙遥控四轮小车 (主程序)
  *                   硬件: STM32F103ZET6 + TB6612 + JGB37-520 + HC-05 + MPU6050 + OLED
  *
  *                   主要功能:
  *                     - 蓝牙遥控: HC-05 接收摇杆/按键指令
  *                     - PID 闭环控制: 转速环 + 差速环 → 左右轮 PWM 合成
  *                     - 编码器测速: TIM2/TIM3 编码器模式 (4 倍频)
  *                     - OLED 显示: SSD1306 显示 RPM/偏航角/状态
  *                     - VOFA+ 调试: USART3 输出 CSV 格式数据帧
  *                     - MPU6050 姿态: 陀螺仪 Z 轴积分偏航角
  *
  *                   控制循环:
  *                     10ms: MPU6050 数据采集 + 偏航角积分
  *                     50ms: 编码器读取 + PID 计算 + VOFA 输出
  *                    200ms: OLED 刷新
  *
  *                   蓝牙协议:
  *                     [joystick,<LH>,<LV>,<RH>,<RV>] — 摇杆数据
  *                     [key,<N>,<up/down>]            — 按键事件
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/

/** 运行标志: 1=运行, 0=停止 */
static uint8_t  RunFlag    = 0;

/** 按键正在控制: 1=按键控车, 0=蓝牙可接管 */
static uint8_t  KeyActive  = 0;

/** 速度档位: 1~6, 每档 30 RPM, 最大 180 RPM */
static uint8_t  SpeedLevel = 4;
#define RPM_PER_LEVEL   30.0f   /* 每档 30 RPM, 最大 6×30=180 RPM */

/** 左右轮编码器对象 */
static Encoder  ecd_left, ecd_right;

/** PID 控制器: SpeedPID=转速环, TurnPID=差速环 */
static PID      SpeedPID, TurnPID;

/** MPU6050 六轴原始数据 */
static int16_t  AX, AY, AZ, GX, GY, GZ;

/** 陀螺仪 Z 轴零偏校准值 */
static int16_t  GY_Offset  = 0;

/** 偏航角 (陀螺仪 Z 轴积分, 单位: °) */
static float    YawAngle   = 0.0f;

/** JGB37-520 编码器参数: 4倍频, 30减速比, 11线, 半径6.5cm */
static const Parameter JGB37_520 = {
    .multiple        = 4,
    .reduction_ratio = 30.0f,
    .ppr             = 11,
    .r               = 0.065f,
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Process_Control(void);
static void Process_Bluetooth(void);
static void Update_OLED(void);
static void VOFA_Send(void);
static void BlueSerial_VOFA_Send(void);
static void Serial_OLED_Data(void);
static void MPU6050_Calibrate(void);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  主函数: 外设初始化 → 校准 → 主循环
 * @note   主循环包含:
 *          1. 蓝牙协议处理
 *          2. 按键扫描 (SET=启停, ADD=加档, DEC=减档)
 *          3. 10ms: MPU6050 + 偏航角积分
 *          4. 50ms: 编码器 + PID + VOFA
 *          5. 200ms: OLED 刷新
 *          6. 停止状态: 电机强制停转
 */
int main(void)
{
    /* --- HAL 库初始化 --- */
    HAL_Init();
    SystemClock_Config();

    /* --- 外设初始化 --- */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();    /* PWM 电机驱动 */
    MX_TIM2_Init();    /* 左轮编码器 */
    MX_TIM3_Init();    /* 右轮编码器 */
    MX_TIM5_Init();    /* millis 计数器 */
    MX_TIM6_Init();    /* 周期标志 (10ms/50ms) */
    MX_USART2_UART_Init();  /* HC-05 蓝牙 */
    MX_USART3_UART_Init();  /* VOFA+ 调试 */

    /* --- 应用层初始化 --- */
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

    /* --- 编码器对象初始化 --- */
    initEncoder(&ecd_left,  JGB37_520);
    initEncoder(&ecd_right, JGB37_520);

    /* --- PID 控制器初始化 --- */
    initPID(&SpeedPID, 100.0f, 200.0f);   /* 输出限幅±100, 积分限幅±200 */
    initPID(&TurnPID,   30.0f,  60.0f);   /* 输出限幅±30,  积分限幅±60  */
    setPIDTarget(&SpeedPID, 0);
    setPIDTarget(&TurnPID,   0);

    /* --- 陀螺仪零偏校准 (需保持静止) --- */
    MPU6050_Calibrate();

    /* --- 开机画面 --- */
    OLED_Clear();
    OLED_ShowString(1, 1, " [BlueCar V1.0] ");
    OLED_ShowString(2, 1, " BT Remote 4WD  ");
    OLED_ShowString(3, 1, " ZET6 + TB6612  ");
    OLED_ShowString(4, 1, " Press SET start");
    Buzzer_ON();  HAL_Delay(200);  Buzzer_OFF();

    /* 等待按下 SET 键启动 */
    while (Key_GetNum() != 1);

    /* 启动: 直接进入 RUN 状态 */
    RunFlag = 1;
    OLED_Clear();
    OLED_ShowString(1, 1, "[RUN] Key Control");

    /* --- 主循环 --- */
    while (1)
    {
        /* 2. 蓝牙协议处理 */
        Process_Bluetooth();

        /* 1. 10ms 周期: 按键扫描 + 方向控制 + MPU6050 */
        if (flag_10ms) {
            flag_10ms = 0;

            Key_Scan();  /* 非阻塞按键扫描 */

            /* ── 按键方向控制 (优先级高于蓝牙) ── */
            if (Key1_Held()) {           /* KEY1 按住 = 前进 */
                KeyActive = 1;
                RunFlag = 1;
                setPIDTarget(&SpeedPID, +SpeedLevel * RPM_PER_LEVEL);
                setPIDTarget(&TurnPID,   0);
            } else if (Key3_Held()) {    /* KEY3 按住 = 后退 */
                KeyActive = 1;
                RunFlag = 1;
                setPIDTarget(&SpeedPID, -SpeedLevel * RPM_PER_LEVEL);
                setPIDTarget(&TurnPID,   0);
            } else if (KeyActive) {      /* 松开按键 = 停车 */
                KeyActive = 0;
                setPIDTarget(&SpeedPID, 0);
                setPIDTarget(&TurnPID,   0);
            }
            /* KeyActive==0 时蓝牙可自由控制 PID 目标 */

            /* KEY2 短按: 切换 RUN/STOP */
            if (Key2_Pressed()) {
                RunFlag = !RunFlag;
                if (!RunFlag) { Motor_SetSpeedA(0); Motor_SetSpeedB(0); }
            }

            /* KEY2 长按(>1s): 循环切换档位 1→2→3→4→5→6→1 */
            static uint8_t key2_hold_cnt = 0;
            if (Key2_Held()) {
                if (++key2_hold_cnt > 100) {  /* 100×10ms = 1s */
                    key2_hold_cnt = 0;
                    SpeedLevel = (SpeedLevel % 6) + 1;
                }
            } else {
                key2_hold_cnt = 0;
            }

            /* MPU6050 数据采集 + 偏航角积分 */
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            float gz = (float)(GZ - GY_Offset) / 32768.0f * 2000.0f;
            YawAngle += gz * 0.01f;
        }

        /* 3. 50ms 周期: 编码器 + PID 控制 + VOFA 输出 */
        if (flag_50ms) {
            flag_50ms = 0;
            updateEncoderLoopSimpleVersion(&ecd_left,  50, TIM2);
            updateEncoderLoopSimpleVersion(&ecd_right, 50, TIM3);
            Process_Control();
            VOFA_Send();

            /* 蓝牙 VOFA: 每 200ms 发一次 (9600bps 限速) */
            static uint8_t bt_vofa_cnt = 0;
            if (++bt_vofa_cnt >= 4) {
                bt_vofa_cnt = 0;
                BlueSerial_VOFA_Send();
            }
        }

        /* 4. 200ms 周期: OLED 刷新 + 串口输出 OLED 数据 */
        static uint32_t oled_tick = 0;
        if (millis - oled_tick > 200) {
            oled_tick = millis;
            Update_OLED();
            Serial_OLED_Data();  /* 串口读取 OLED 显示的数据 */

            /* 调试: 打印按键原始电平 (0=按下, 1=松开) */
            Serial_Printf("KEY:1=%d 2=%d 3=%d\r\n",
                HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
                HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7),
                HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6));
        }

        /* 5. 停止状态: 电机强制停转 */
        if (!RunFlag) {
            Motor_SetSpeedA(0);
            Motor_SetSpeedB(0);
        }
    }
}

/*============================================================
 * PID 控制: 转速环(RPM) + 差速环(RPM) → 左右轮 PWM 合成
 *   - 转速环: 控制平均转速 = 目标前进速度
 *   - 差速环: 控制转速差   = 转向速度
 *   - 输出混控: MotorA = base + diff, MotorB = base - diff
 *============================================================*/
static void Process_Control(void)
{
    float aveRPM = (ecd_left.velocity.angular + ecd_right.velocity.angular) / 2.0f;
    float difRPM = ecd_left.velocity.angular - ecd_right.velocity.angular;

    if (RunFlag) {
        /* 两个 PID 独立运算 */
        updatePID(&SpeedPID, aveRPM);
        updatePID(&TurnPID,  difRPM);

        /* 混控: 前进 + 差速 = 左右轮分别输出 */
        float base = SpeedPID.output;
        float diff = TurnPID.output;
        int16_t L = (int16_t)(base + diff);
        int16_t R = (int16_t)(base - diff);

        /* PWM 占空比限幅 ±100 (约 ±2.4%) */
        if (L >  100) L =  100;  if (L < -100) L = -100;
        if (R >  100) R =  100;  if (R < -100) R = -100;

        Motor_SetSpeedA(L);
        Motor_SetSpeedB(R);
    }
}

/*============================================================
 * 蓝牙协议解析: 帧格式 [...]
 *   摇杆: [joystick,LH,LV,RH,RV] → 目标 RPM = LV * 档位级数
 *   按键: [key,N,up/down]       → N=1启停, 2减速, 3加速
 *============================================================*/
static void Process_Bluetooth(void)
{
    if (KeyActive) return;  /* 按键控车中, 蓝牙不干预 */
    if (!BlueSerial_ReceiveFlag()) return;
    BlueSerial_Receive();

    /* 摇杆模式 */
    if (strcmp(BlueSerial_StringArray[0], "joystick") == 0) {
        int8_t LV = atoi(BlueSerial_StringArray[2]);  /* 前后: -100~100 */
        int8_t RH = atoi(BlueSerial_StringArray[3]);  /* 左右: -100~100 */

        /* LV 映射到目标转速, RH 映射到差速 (0.5 倍系数) */
        float targetRPM = LV / 100.0f * SpeedLevel * RPM_PER_LEVEL;
        float turnRPM  = RH / 100.0f * SpeedLevel * RPM_PER_LEVEL * 0.5f;
        setPIDTarget(&SpeedPID, targetRPM);
        setPIDTarget(&TurnPID,  turnRPM);
    }
    /* 按键模式 */
    else if (strcmp(BlueSerial_StringArray[0], "key") == 0) {
        if (strcmp(BlueSerial_StringArray[1], "1") == 0
         && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            /* 按键 1: 启停切换 */
            RunFlag = !RunFlag;
            if (!RunFlag) { Motor_SetSpeedA(0); Motor_SetSpeedB(0); }
            BlueSerial_Printf("RunFlag=%d\r\n", RunFlag);
        }
        else if (strcmp(BlueSerial_StringArray[1], "2") == 0
              && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            /* 按键 2: 减速 (档位-1) */
            if (SpeedLevel > 1) SpeedLevel--;
            BlueSerial_Printf("SpeedLevel=%d\r\n", SpeedLevel);
        }
        else if (strcmp(BlueSerial_StringArray[1], "3") == 0
              && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            /* 按键 3: 加速 (档位+1) */
            if (SpeedLevel < 6) SpeedLevel++;
            BlueSerial_Printf("SpeedLevel=%d\r\n", SpeedLevel);
        }
    }
}

/*============================================================
 * OLED 显示: 4 行界面
 *   Line1: 运行状态 (RUN/STOP)
 *   Line2: 目标转速 + 目标差速
 *   Line3: 实际转速 (左/右轮)
 *   Line4: 偏航角 + 当前档位
 *============================================================*/
static void Update_OLED(void)
{
    char buf[21];
    OLED_ShowString(1, 1, RunFlag ? "[RUN] BT Remote" : "[STOP] BT Remote");

    sprintf(buf, "TargRPM:%+05.1f Diff:%+05.1f",
        SpeedPID.target, TurnPID.target);
    OLED_ShowString(2, 1, buf);

    sprintf(buf, "RealL:%+05.1f R:%+05.1f",
        ecd_left.velocity.angular, ecd_right.velocity.angular);
    OLED_ShowString(3, 1, buf);

    sprintf(buf, "Yaw:%+06.1f Lv:%1d", YawAngle, SpeedLevel);
    OLED_ShowString(4, 1, buf);
}

/*============================================================
 * VOFA+ FireWater 协议: 每 50ms 发送 CSV 数据帧
 *   字段: targetRPM, RPM_L, RPM_R, diffRPM_target, diffRPM_actual, YawAngle, RunFlag
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
 * 蓝牙 VOFA+ 数据发送: 每 200ms 通过 HC-05 发送编码器+PID 数据
 *   字段: targetRPM, RPM_L, RPM_R, diff_target, diff_actual, Yaw, RunFlag
 *   波特率 9600 → ~5Hz 发送频率, 每帧约 55 字节
 *============================================================*/
static void BlueSerial_VOFA_Send(void)
{
    BlueSerial_Printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\r\n",
        SpeedPID.target,
        ecd_left.velocity.angular,
        ecd_right.velocity.angular,
        TurnPID.target,
        ecd_left.velocity.angular - ecd_right.velocity.angular,
        YawAngle,
        RunFlag);
}

/*============================================================
 * 串口输出 OLED 数据: 每 200ms 发送, 与 OLED 显示内容同步
 *   可在 PC 端用串口助手 (115200bps) 直接读取
 *============================================================*/
static void Serial_OLED_Data(void)
{
    Serial_Printf("\r\n=== BlueCar OLED ===\r\n");
    Serial_Printf("Status : %s\r\n", RunFlag ? "RUN" : "STOP");
    Serial_Printf("Target : %+.1f RPM  Diff:%+.1f\r\n", SpeedPID.target, TurnPID.target);
    Serial_Printf("Real L : %+.1f RPM  R:%+.1f RPM\r\n",
        ecd_left.velocity.angular, ecd_right.velocity.angular);
    Serial_Printf("Yaw    : %+.1f deg  Level:%d\r\n", YawAngle, SpeedLevel);
    Serial_Printf("==================\r\n");
}

/*============================================================
 * MPU6050 陀螺仪零偏校准: 取 200 次平均, 保持静止
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

/*============================================================
 * System Clock Configuration
 *   HSE (8MHz) → PLL ×9 → SYSCLK = 72MHz
 *   AHB = 72MHz, APB1 = 36MHz, APB2 = 72MHz
 *============================================================*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** HSE 配置: 8MHz 外部晶振, PLL ×9 = 72MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    /** 系统时钟: SYSCLK=72M, AHB=72M, APB1=36M, APB2=72M, Flash=2等待 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/**
 * @brief  错误处理: 关闭全局中断, 死循环
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
