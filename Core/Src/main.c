/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BlueCar V3.0 — 协程架构 + 环形缓冲解耦
  *                   硬件: STM32F103ZET6 + TB6612 + JGB37-520 + HC-05 + MPU6050 + OLED
  *
  *                   架构 (协程 + 消息队列):
  *                     ┌──────────┐  CmdMsg   ┌──────────┐  VofaMsg  ┌──────────┐
  *                     │ BT/Key/  │───→ Q ──→│ Control  │───→ Q ──→│  VOFA    │
  *                     │ PIDTune  │           │ (50ms)   │           │  Output  │
  *                     └──────────┘           └──────────┘           └──────────┘
  *                                                  ↓
  *                                            Motor + Encoder
  *
  *                   USART3 TX: DMA 非阻塞 (serial.c), 永不阻塞主循环
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
#include "pid_tune.h"
#include "ringbuf.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* CmdType / CmdMsg / Mode_t 声明在 main.h */

/* ---- 遥测数据快照 (Control → VOFA) ---- */
typedef struct {
    float target_pct;    /* 目标 RPM */
    float rpm_r;         /* 右轮 RPM */
    float rpm_l;         /* 左轮 RPM */
    float speed_out;     /* SpeedPID PWM输出 % */
    float yaw_out;       /* TrackPID 差速 RPM */
    float turn_target;   /* 转向差速 RPM */
    float rpm_diff;      /* 左右轮 RPM 差 */
    float yaw_angle;     /* 偏航角 ° */
    int16_t bt_lv;       /* 蓝牙摇杆 LV (int16 避免 int8→int 变参问题) */
    int16_t bt_rh;       /* 蓝牙摇杆 RH */
    int16_t run_flag;    /* 0/1 */
    int16_t gear;        /* 速度档位 1~6 */
    int16_t fresh;       /* 新数据标志 */
} VofaMsg;

/* ---- 命令消息队列 (SPSC: BT/Key/Serial → Control) ---- */
#define CMD_QUEUE_SIZE  8
static CmdMsg  cmd_queue[CMD_QUEUE_SIZE];
static uint8_t cmd_head = 0;   /* 生产者写入位置 */
static uint8_t cmd_tail = 0;   /* 消费者读取位置 */

void CmdQueue_Put(CmdMsg msg)
{
    uint8_t next = (cmd_head + 1) % CMD_QUEUE_SIZE;
    if (next == cmd_tail) return;   /* 队列满则丢弃 (不应发生) */
    cmd_queue[cmd_head] = msg;
    cmd_head = next;
}

static int CmdQueue_Get(CmdMsg *msg)
{
    if (cmd_head == cmd_tail) return -1;   /* 空 */
    *msg = cmd_queue[cmd_tail];
    cmd_tail = (cmd_tail + 1) % CMD_QUEUE_SIZE;
    return 0;
}

static uint8_t CmdQueue_Available(void)
{
    return (cmd_head + CMD_QUEUE_SIZE - cmd_tail) % CMD_QUEUE_SIZE;
}

/* Shared state --------------------------------------------------------------*/
Mode_t   Mode       = MODE_RUN;
uint8_t  RunFlag    = 0;
int16_t  PWMDuty    = 0;
static uint8_t  SpeedLevel   = 3;        /* 蓝牙速度档位 1~6 */
static uint8_t  SpeedPercent = 50;       /* 按键速度百分比 0~100 */
uint32_t bt_last_rx = 0;               /* 上次收到蓝牙数据的时间戳 */
uint8_t  VOFA_Enable = 1;
static const float SpeedTable[6] = {30, 55, 80, 100, 120, 140}; /* 提速: 原极速105→140 */

float    TurnTarget = 0;
float    YawTarget  = 0;
volatile int8_t BT_LV = 0, BT_RH = 0;  /* ISR 读取, 主循环写入, 必须 volatile */

/* PID 控制器 (参数在 main() 中初始化) -----------------------------------------*/
PID_t SpeedPID;
PID_t YawPID;
PID_t TrackPID;                             /* 转弯差速 PID */

/* 编码器 --------------------------------------------------------------------*/
static Encoder  ecd_left, ecd_right;
static const Parameter JGB37_520 = {
    .multiple = 4, .reduction_ratio = 30.0f, .ppr = 11, .r = 0.065f,
};

/* MPU6050 -------------------------------------------------------------------*/
static int16_t AX, AY, AZ, GX, GY, GZ;
static int16_t GZ_Offset = 0, AX_Offset = 0, AZ_Offset = 0;
static float YawAngle   = 0.0f;
static float TiltFilter = 0.0f;       /* 倾角一阶低通滤波值 */

/* VOFA 遥测快照 ---------------------------------------------------------------*/
static VofaMsg vofa_data;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Cmd_ProcessBluetooth(void);
static void Cmd_ProcessKeys(void);
static void Cmd_ProcessPIDTune(void);
void Task_Control_50ms(void);          /* ISR 调用 */
void Task_Pulse_10ms(void);            /* ISR 调用 */
void Task_VOFA_Send(void);             /* ISR 调用 */
void Task_Sensor_10ms(void);           /* ISR 调用 */
static void Task_OLED_Update(void);
static void MPU6050_Calibrate(void);
static void MPU6050_Process(void);

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
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    OLED_Init();
    HAL_Delay(100);  /* OLED 上电稳定延时 */
    Motor_Init();
    MPU6050_Init();
    Encoder_Init_TIM_All();
    Timer_Init();
    TIM5_Init();
    Key_Init();
    Serial_Init();
    Serial_Printf("BOOT OK\r\n");   /* 启动诊断 */
    PIDTune_Init();
    Buzzer_Init();
    BlueSerial_Init();

    initEncoder(&ecd_left,  JGB37_520);
    initEncoder(&ecd_right, JGB37_520);

    PID_Init(&SpeedPID);
    /* SpeedPID: 直接控制速度 */
    PID_SetParam(&SpeedPID, 0.65f, 0.1f, 0.3f);  /* Ki 消除稳态误差 */
    SpeedPID.Kf              = 0.4f;       /* 前馈: 稳态PWM/目标RPM ≈ 0.65, 提供基线输出 */
    SpeedPID.OutMax          = 100;         /* 满占空比, Kf提供基线 + I补齐余量 */
    SpeedPID.OutMin          = -100;
    SpeedPID.OutOffset       = 0;
    SpeedPID.IntegralLimit   = 50.0f;
    SpeedPID.Deadband        = 2.0f;
    SpeedPID.OutputRateLimit = 0.0f;

    PID_Init(&YawPID);
    PID_SetParam(&YawPID, 0.0f, 0.0f, 0.0f);
    YawPID.OutMax          =  20;
    YawPID.OutMin          = -20;
    YawPID.IntegralLimit   = 40.0f;
    YawPID.Deadband        = 1.0f;
    YawPID.OutputRateLimit = 10.0f;

    /* TrackPID: 差速转弯 */
    PID_Init(&TrackPID);
    PID_SetParam(&TrackPID, 0.5f, 0.05f, 0.1f);  /* Ki>0 消除左右轮稳态速差 */
    TrackPID.OutMax          =  50;
    TrackPID.OutMin          = -50;
    TrackPID.OutOffset       = 0;
    TrackPID.IntegralLimit   = 20.0f;
    TrackPID.Deadband        = 3.0f;
    TrackPID.OutputRateLimit = 15.0f;

    OLED_Clear();
    OLED_ShowString(1, 1, " [BlueCar V3.0] ");
    OLED_ShowString(2, 1, " Co-op Tasks   ");
    OLED_ShowString(3, 1, " DMA TX + RB   ");
    OLED_ShowString(4, 1, " Press C8 start ");
    OLED_Flush();
    Buzzer_ON();  HAL_Delay(200);  Buzzer_OFF();

    while (Key_GetNum() != 3);
    OLED_Clear();
    OLED_ShowString(2, 1, " Calibrating... ");
    OLED_Flush();

    MPU6050_Calibrate();
    MPU6050_ReadAsync_Start();

    /*====================================================================
     * 主循环 — 协程式任务调度
     *
     *   1. 命令注入 (每次循环): BT / Key / PIDTune → cmd_queue
     *   2. 传感器 (10ms): MPU6050 陀螺仪积分
     *   3. 控制   (50ms): 编码器 + PID + 电机输出 → vofa_data
     *   4. VOFA   (50ms): 遥测快照格式化 + DMA TX 非阻塞发送
     *   5. OLED   (200ms): 显示更新
     *
     *   所有数据通过 cmd_queue / vofa_data 解耦
     *   Serial TX 使用 DMA, 永不阻塞主循环
     *====================================================================*/
    while (1)
    {
        /* ---- 串口命令响应 (关中断防 ISR 竞争 RingBuf) ---- */
        if (tune_rsp_pending) {
            __disable_irq();
            Serial_SendString(tune_rsp_buf);
            __enable_irq();
            tune_rsp_pending = 0;
        }

        /*---- 阶段1: 命令注入 (每次循环, 轻量) ----*/
        Cmd_ProcessBluetooth();     /* BT → cmd_queue */
        Cmd_ProcessKeys();          /* Key → cmd_queue */
        Cmd_ProcessPIDTune();       /* Serial → cmd_queue */

        /*---- 阶段2: 传感器 + 内环控制 (10ms) ----*/
        if (flag_10ms) {
            flag_10ms = 0;          /* 保留标志, 供扩展 */
        }

        /*---- OLED 非阻塞刷新: 帧缓冲 → SSD1306 (IT I2C) ----*/
        OLED_Flush();

        /*---- 阶段3: 控制 (50ms) — 已由 TIM6 ISR 硬实时驱动 ----*/

        /*---- 阶段4: OLED (200ms) ----*/
        static uint32_t oled_tick = 0;
        if (millis - oled_tick > 200) {
            oled_tick = millis;
            Task_OLED_Update();
        }

        /* 停车保护: RunFlag=0 时强制输出 0 */
        if (!RunFlag) {
            Motor_SetSpeedA(0); Motor_SetSpeedB(0);
        }
    }
}

/*====================================================================
 * 传感器任务 (10ms): MPU6050 数据解算
 *====================================================================*/
void Task_Sensor_10ms(void)
{
    MPU6050_Process();
}

/**
 * @brief  MPU6050 陀螺仪积分 + 倾角解算 (每 10ms)
 * @note   停车时自动校准 GZ 零偏 (指数滑动平均)
 *         YawAngle 由陀螺仪 Z 轴积分得到
 *         TiltFilter 由加速度计 atan2 计算并经一阶低通滤波
 */
static void MPU6050_Process(void)
{
    if (!MPU6050_ReadAsync_Done()) return;  /* 数据未就绪, 下一周期再试 */

    MPU6050_ReadAsync_Get(&AX, &AY, &AZ, &GX, &GY, &GZ);

    /* 停车时自动校准 GZ 零偏 (指数滑动平均) */
    static int32_t gz_sum = 0;
    static uint16_t gz_cnt = 0;
    if (!RunFlag) {
        gz_sum += GZ;
        if (++gz_cnt >= 100) {
            int16_t new_offset = (int16_t)(gz_sum / 100);
            GZ_Offset = (int16_t)(GZ_Offset * 0.9f + new_offset * 0.1f);
            gz_sum = 0;
            gz_cnt = 0;
        }
    } else {
        gz_sum = 0;
        gz_cnt = 0;
    }

    float gz = (float)(GZ - GZ_Offset) / 32768.0f * 2000.0f;
    YawAngle += gz * 0.01f;

    float ax_cali = (float)(AX - AX_Offset);
    float az_cali = (float)(AZ - AZ_Offset);
    float AngleAcc = -atan2f(ax_cali, az_cali) / 3.1415926535f * 180.0f;

    float Alpha0 = 0.8f;
    TiltFilter = Alpha0 * TiltFilter + (1.0f - Alpha0) * AngleAcc;

    MPU6050_ReadAsync_Start();   /* 启动下一次异步读取 */
}

/*====================================================================
 * 命令注入: 蓝牙 (每次循环)
 * @brief  解析 HC-05 蓝牙数据帧 → 生成 CmdMsg 投递到 cmd_queue
 * @note   摇杆: [joystick,LH,LV,RH,RV] → SpeedPID 目标 + TurnTarget
 *         按键: [key,N,up] → 启停/档位切换
 *         摇杆回中 (LV=RH=0) 时发停车指令
 *====================================================================*/
static void Cmd_ProcessBluetooth(void)
{
    if (Mode == MODE_PWM_SET) return;
    if (!BlueSerial_ReceiveFlag()) return;

    BlueSerial_Receive();
    bt_last_rx = millis;
    CmdMsg msg;

    if (strcmp(BlueSerial_StringArray[0], "joystick") == 0) {
        int8_t LV = atoi(BlueSerial_StringArray[2]);
        int8_t RH = atoi(BlueSerial_StringArray[3]);
        BT_LV = LV; BT_RH = RH;
        if (LV != 0 || RH != 0) {
            msg.type = CMD_SPEED;
            msg.val  = LV / 100.0f * SpeedTable[SpeedLevel-1];
            CmdQueue_Put(msg);
            msg.type = CMD_TURN;
            msg.val  = RH / 100.0f * SpeedTable[SpeedLevel-1] * 0.5f;
            CmdQueue_Put(msg);
            msg.type = CMD_START; msg.val = 0;
            CmdQueue_Put(msg);
        } else {
            msg.type = CMD_STOP; msg.val = 0; CmdQueue_Put(msg);
        }
    }
    else if (strcmp(BlueSerial_StringArray[0], "key") == 0) {
        if (strcmp(BlueSerial_StringArray[1], "1") == 0
         && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            msg.type = RunFlag ? CMD_STOP : CMD_START;
            msg.val = 0;
            CmdQueue_Put(msg);
        }
        else if (strcmp(BlueSerial_StringArray[1], "2") == 0
              && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            msg.type = CMD_GEAR_DOWN; msg.val = 0;
            CmdQueue_Put(msg);
        }
        else if (strcmp(BlueSerial_StringArray[1], "3") == 0
              && strcmp(BlueSerial_StringArray[2], "up") == 0) {
            msg.type = CMD_GEAR_UP; msg.val = 0;
            CmdQueue_Put(msg);
        }
    }
}

/*====================================================================
 * 命令注入: 按键 (每次循环)
 * @brief  非阻塞按键状态机 → 生成 CmdMsg 投递到 cmd_queue
 * @note   KEY1: 单击切换 RUN/PWM_SET 模式
 *         KEY4: 单击循环升档 (1→2→...→6→1)
 *         RUN模式: KEY2按住前进, KEY3按住后退, 松手 300ms 后自动停车
 *         PWM模式: KEY2/KEY3 调节占空比 ±10%
 *====================================================================*/
static void Cmd_ProcessKeys(void)
{
    CmdMsg msg;

    /* KEY1: 单击切换模式 */
    if (Key_Check(KEY_1, KEY_SINGLE)) {
        if (Mode == MODE_RUN) {
            Mode = MODE_PWM_SET;
            msg.type = CMD_STOP; msg.val = 0;
            CmdQueue_Put(msg);
        } else {
            Mode = MODE_RUN;
            PWMDuty = 0;
        }
    }

    /* KEY4: 单击循环升档 */
    if (Key_Check(KEY_4, KEY_SINGLE)) {
        SpeedLevel = (SpeedLevel < 6) ? SpeedLevel + 1 : 1;
        SpeedPercent = 50;
    }

    if (Mode == MODE_PWM_SET) {
        if (Key_Check(KEY_2, KEY_SINGLE)) { PWMDuty += 10; if (PWMDuty >  100) PWMDuty =  100; }
        if (Key_Check(KEY_3, KEY_SINGLE)) { PWMDuty -= 10; if (PWMDuty < -100) PWMDuty = -100; }
        RunFlag = (PWMDuty != 0);
    } else {
        /* RUN 模式: KEY2=前进 / KEY3=后退 */
        uint8_t k2 = Key_Check(KEY_2, KEY_HOLD);
        uint8_t k3 = Key_Check(KEY_3, KEY_HOLD);

        if (k2 || k3) {
            if (SpeedPercent < 100) SpeedPercent++;
            msg.type = CMD_START; msg.val = 0;
            CmdQueue_Put(msg);
            float spd = SpeedTable[SpeedLevel-1] * SpeedPercent / 100.0f;
            msg.type = CMD_SPEED;
            msg.val  = k2 ? spd : -spd;
            CmdQueue_Put(msg);
            msg.type = CMD_TURN; msg.val = 0;
            CmdQueue_Put(msg);
            bt_last_rx = millis;
        }
        else if (millis - bt_last_rx > 300) {
            SpeedPercent = 0;
            msg.type = CMD_STOP; msg.val = 0;
            CmdQueue_Put(msg);
            msg.type = CMD_TURN; msg.val = 0;
            CmdQueue_Put(msg);
        }
    }
}

/*====================================================================
 * 命令注入: PID 串口调参 (每次循环)
 * @brief  消费 USART3 串口命令行 → 直接改 PID 参数 或 投递 CmdMsg
 * @note   串口命令列表 (115200bps, 以 \r\n 结尾):
 *           skp/ski/skd/skf N → SpeedPID 参数 (即时生效)
 *           ykp/yki/ykd/ykf N → YawPID 参数 (即时生效)
 *           t N / turn N / start / stop / duty N → cmd_queue (下周期生效)
 *           quiet / log / show / help
 *====================================================================*/
static void Cmd_ProcessPIDTune(void)
{
    PIDTune_Process();
}

/*====================================================================
 * 控制任务 (50ms): 编码器读取 → 命令消费 → 级联 PID → 电机输出
 * @brief  每 50ms 执行一次, 是系统的核心控制循环
 * @note   处理流程:
 *           1. 读取左右编码器 RPM
 *           2. 消费 cmd_queue 中的所有待处理命令
 *           3. PWM_SET 模式: 直接输出占空比
 *           4. RUN 模式: SpeedPID(50ms) + YawPID(100ms) → 差速合成 →
 *              超速保护 → Motor PWM 输出
 *           5. 填充遥测快照 vofa_data 供 VOFA 输出
 *====================================================================*/
void Task_Control_50ms(void)
{
    /*---- 读取编码器 ----*/
    updateEncoderLoopSimpleVersion(&ecd_left,  50, TIM3);
    updateEncoderLoopSimpleVersion(&ecd_right, 50, TIM2);

    /* PWM 设置模式: 直接输出 ----*/
    if (Mode == MODE_PWM_SET) {
        Motor_SetSpeedA(PWMDuty);
        Motor_SetSpeedB(PWMDuty);
        return;
    }

    /*---- 处理命令队列 ----*/
    while (CmdQueue_Available()) {
        CmdMsg msg;
        if (CmdQueue_Get(&msg) < 0) break;

        switch (msg.type) {
        case CMD_SPEED:
            PID_SetTarget(&SpeedPID, msg.val);
            break;
        case CMD_TURN:
            TurnTarget = msg.val;
            break;
        case CMD_START:
            RunFlag = 1;
            break;
        case CMD_STOP:
            RunFlag = 0;
            PID_SetTarget(&SpeedPID, 0);
            TurnTarget = 0;
            SpeedPID.Out = 0; SpeedPID.IOut = 0;
            SpeedPID.POut = 0; SpeedPID.DOut = 0;
            SpeedPID.LastOutput = 0;
            TrackPID.Out = 0; TrackPID.IOut = 0;
            TrackPID.POut = 0; TrackPID.DOut = 0;
            TrackPID.LastOutput = 0;
            break;
        case CMD_GEAR_UP:
            if (SpeedLevel < 6) {
                SpeedLevel++;
                PID_SetTarget(&SpeedPID, SpeedTable[SpeedLevel-1]);
            }
            break;
        case CMD_GEAR_DOWN:
            if (SpeedLevel > 1) {
                SpeedLevel--;
                PID_SetTarget(&SpeedPID, SpeedTable[SpeedLevel-1]);
            }
            break;
        case CMD_SET_SPEED_KP: SpeedPID.Kp = msg.val; break;
        case CMD_SET_SPEED_KI: SpeedPID.Ki = msg.val; break;
        case CMD_SET_SPEED_KD: SpeedPID.Kd = msg.val; break;
        case CMD_SET_SPEED_KF: SpeedPID.Kf = msg.val; break;
        case CMD_SET_YAW_KP:   YawPID.Kp = msg.val; break;
        case CMD_SET_YAW_KI:   YawPID.Ki = msg.val; break;
        case CMD_SET_YAW_KD:   YawPID.Kd = msg.val; break;
        case CMD_SET_YAW_KF:   YawPID.Kf = msg.val; break;
        case CMD_YAW_TARGET:   YawTarget = msg.val;
                               PID_SetTarget(&YawPID, msg.val); break;
        case CMD_QUIET: VOFA_Enable = 0; break;
        case CMD_LOG:   VOFA_Enable = 1; break;
        default: break;
        }
    }

    /* 停车时: 刷新遥测但 PID 不运算 */
    float ave = (ecd_left.velocity.angular + ecd_right.velocity.angular) / 2.0f;
    float dif = ecd_left.velocity.angular - ecd_right.velocity.angular;

    if (!RunFlag) {
        float spd_max = SpeedTable[SpeedLevel-1];
        if (spd_max < 1.0f) spd_max = 1.0f;
        vofa_data.target_pct  = 0;
        vofa_data.rpm_r       = 0;
        vofa_data.rpm_l       = 0;
        vofa_data.speed_out   = 0;
        vofa_data.yaw_out     = 0;
        vofa_data.turn_target = 0;
        vofa_data.rpm_diff    = 0;
        vofa_data.yaw_angle   = YawAngle;
        vofa_data.bt_lv       = BT_LV;
        vofa_data.bt_rh       = BT_RH;
        vofa_data.run_flag    = 0;
        vofa_data.gear        = SpeedLevel;
        vofa_data.fresh       = 1;
        return;
    }

    /*---- SpeedPID (50ms): 平均转速 → PWM ----*/
    /* 低通滤波平滑编码器抖动, 减少 D 项误触发 */
    {
        static float ave_f = 0;
        static uint8_t ave_init = 0;
        if (!ave_init) { ave_f = ave; ave_init = 1; }
        else { ave_f = ave_f * 0.4f + ave * 0.6f; }   /* α=0.6, τ≈35ms, D项响应快 */
        SpeedPID.Actual = ave_f;
    }
    /* 方向反转时积分清零已由 PID_SetTarget 内部自动处理 */
    PID_Update(&SpeedPID);
    float base = SpeedPID.Out;

    /*---- TrackPID (50ms): 位置/差速 → 转弯纠偏 ----*/
    TrackPID.Actual = dif;
    PID_SetTarget(&TrackPID, TurnTarget);
    PID_Update(&TrackPID);
    float tdiff = TrackPID.Out;

    /* 合成左右轮 PWM */
    int16_t L = (int16_t)(base + tdiff);
    int16_t R = (int16_t)(base - tdiff);
    if (L >  100) L =  100;
    if (L < -100) L = -100;
    if (R >  100) R =  100;
    if (R < -100) R = -100;
    Motor_SetSpeedA(L);
    Motor_SetSpeedB(R);

    /*---- 填充遥测 (速度用 RPM 实际值) ----*/
    vofa_data.target_pct  = SpeedPID.Target;               /* 目标 RPM */
    vofa_data.rpm_r       = ecd_right.velocity.angular;    /* 右轮 RPM */
    vofa_data.rpm_l       = ecd_left.velocity.angular;     /* 左轮 RPM */
    vofa_data.speed_out   = SpeedPID.Out;                  /* PWM % */
    vofa_data.yaw_out     = TrackPID.Out;                  /* 差速 RPM */
    vofa_data.turn_target = TurnTarget;                     /* 差速目标 RPM */
    vofa_data.rpm_diff    = dif;                            /* 左右轮 RPM 差 */
    vofa_data.yaw_angle   = YawAngle;
    vofa_data.bt_lv       = BT_LV;
    vofa_data.bt_rh       = BT_RH;
    vofa_data.run_flag    = RunFlag;
    vofa_data.gear        = SpeedLevel;
    vofa_data.fresh       = 1;
}

/*====================================================================
 * VOFA 输出任务 (50ms, 在 Control 之后)
 * @brief  遥测快照格式化为 CSV → DMA TX 非阻塞发送
 *====================================================================
 * 内环控制任务 (10ms): 已删除, 由 Task_Control_50ms 统一处理
 *====================================================================*/
void Task_Pulse_10ms(void) {}

/*====================================================================
 * VOFA 输出任务 (50ms, 在 Control 之后)
 * @brief  遥测快照格式化为 CSV → DMA TX 非阻塞发送
 * @note   数据字段 (12列): TargetRPM, RPM_R, RPM_L, SpeedOut%, YawOutRPM,
 *         TurnTargetRPM, RPM_Diff, YawAngle°, BT_LV, BT_RH, RunFlag, Gear
 *         速度用 RPM 实际值, PWM 保持百分比
 *         使用 snprintf 防栈溢出, 通过 Serial_Write → RingBuf → DMA 发送
 *====================================================================*/
void Task_VOFA_Send(void)
{
    static uint8_t stop_cnt = 0;
    if (!RunFlag) stop_cnt++; else stop_cnt = 0;
    if (!VOFA_Enable || !vofa_data.fresh) return;
    if (!RunFlag && stop_cnt > 20) return;  /* 停车 1 秒后静默 */
    vofa_data.fresh = 0;

    char buf[128];
    int n;

    /* 防御: 钳位异常值 */
    int _bt_lv = (int)vofa_data.bt_lv;
    int _bt_rh = (int)vofa_data.bt_rh;
    int _run   = (int)vofa_data.run_flag;
    if (_bt_lv < -128 || _bt_lv > 127) _bt_lv = 0;
    if (_bt_rh < -128 || _bt_rh > 127) _bt_rh = 0;

    /* 拆成两次 snprintf, 避免软浮点 ABI 下 double(8B) 和 int(4B)
       混用导致变参栈偏移错误 — 这是 newlib-nano 在 Cortex-M3 上的已知问题 */
    n = snprintf(buf, sizeof(buf),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,",
        vofa_data.target_pct,
        vofa_data.rpm_r,
        vofa_data.rpm_l,
        vofa_data.speed_out,
        vofa_data.yaw_out,
        vofa_data.turn_target,
        vofa_data.rpm_diff,
        vofa_data.yaw_angle);

    if (n > 0 && n < (int)sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - n, "%d,%d,%d,%d\r\n",
            _bt_lv, _bt_rh, _run, (int)vofa_data.gear);
    }

    if (n > 0 && n < (int)sizeof(buf)) {
        Serial_Write((uint8_t *)buf, (uint16_t)n);
    }
}

/*====================================================================
 * OLED 更新任务 (200ms)
 * @brief  每 200ms 刷新 SSD1306 四行显示
 * @note   第1行: 运行模式/状态
 *         第2行: PID 目标 RPM + 转向差速
 *         第3行: 左右轮实际 RPM
 *         第4行: 偏航角 + 速度档位
 *====================================================================*/
static void Task_OLED_Update(void)
{
    OLED_Clear();
    char buf[24];

    if (Mode == MODE_PWM_SET) {
        OLED_ShowString(1, 1, "[PWM SET]       ");
        snprintf(buf, sizeof(buf), "Duty: %+04d %%   ", PWMDuty);
    } else {
        snprintf(buf, sizeof(buf), "[%s] G%d       ",
            RunFlag ? "RUN" : "STOP", SpeedLevel);
        OLED_ShowString(1, 1, buf);
        snprintf(buf, sizeof(buf), "T:%+04.0f P:%+04.0f",
            SpeedPID.Target, SpeedPID.Out);
    }
    OLED_ShowString(2, 1, buf);

    snprintf(buf, sizeof(buf), "L:%+04.0f R:%+04.0f",
        ecd_left.velocity.angular, ecd_right.velocity.angular);
    OLED_ShowString(3, 1, buf);

    snprintf(buf, sizeof(buf), "Y:%+05.1f BT:%+04d ",
        YawAngle, (int)BT_LV);
    OLED_ShowString(4, 1, buf);
}

/*====================================================================
 * MPU6050 零偏校准 (启动时调用一次)
 * @brief  采样 200 次取平均, 计算 GZ/AX/AZ 零偏值
 *====================================================================*/
static void MPU6050_Calibrate(void)
{
    int32_t sum_gz = 0, sum_ax = 0, sum_az = 0;
    for (uint16_t i = 0; i < 200; i++) {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum_gz += GZ;
        sum_ax += AX;
        sum_az += AZ;
        HAL_Delay(5);
    }
    GZ_Offset = (int16_t)(sum_gz / 200);
    AX_Offset = (int16_t)(sum_ax / 200);
    AZ_Offset = (int16_t)(sum_az / 200);
}

/**
 * @brief  系统时钟配置: HSE 8MHz → PLL×9 → SYSCLK 72MHz
 *         APB1=36MHz, APB2=72MHz, Flash Latency=2
 */
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

/** @brief HAL 错误处理: 关全局中断, 死循环等待看门狗/WDT 复位 */
void Error_Handler(void) { __disable_irq(); while (1) {} }

#ifdef USE_FULL_ASSERT
/** @brief assert 失败回调 (仅 USE_FULL_ASSERT 时编译) */
void assert_failed(uint8_t *f, uint32_t l) {}
#endif
