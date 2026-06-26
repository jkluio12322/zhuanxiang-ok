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
    float target_l;      /* 左轮目标 RPM */
    float target_r;      /* 右轮目标 RPM */
    float rpm_r;         /* 右轮 RPM */
    float rpm_l;         /* 左轮 RPM */
    float pwm_l;         /* 左轮 SpeedPID_L.Out % */
    float pwm_r;         /* 右轮 SpeedPID_R.Out % */
    float yaw_out;       /* TrackPID 差速 RSS */
    float rpm_diff;      /* 左右轮 RPM 差 */
    float yaw_angle;     /* 偏航角 ° */
    int16_t bt_lv;       /* 蓝牙摇杆 LV */
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

uint32_t bt_last_rx = 0;               /* 上次收到蓝牙数据的时间戳 */
uint8_t  VOFA_Enable = 1;
static const float SpeedTable[6] = {30, 55, 80, 100, 120, 140}; /* 提速: 原极速105→140 */

float    TurnTarget = 0;
float    YawTarget  = 0;
volatile int8_t BT_LV = 0, BT_RH = 0;  /* ISR 读取, 主循环写入, 必须 volatile */

/* PID 控制器 (参数在 main() 中初始化) -----------------------------------------*/
PID_t SpeedPID_L;      /* 左轮独立速度PID */
PID_t SpeedPID_R;      /* 右轮独立速度PID */
PID_t YawPID;
PID_t TrackPID;        /* 转弯差速 PID */

/* 编码器 --------------------------------------------------------------------*/
static Encoder  ecd_left, ecd_right;
static const Parameter JGB37_520 = {
    .multiple = 4, .reduction_ratio = 30.0f, .ppr = 11, .r = 0.065f,
};

/* MPU6050 + Madgwick AHRS ------------------------------------------------*/
static MPU6050 mpu;
static int16_t AX, AY, AZ, GX, GY, GZ;  /* 异步 I2C 原始 ADC 缓存 */

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
    MPU6050_init(&hi2c2);
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

    /* SpeedPID L/R: 左右独立速度闭环 */
    PID_Init(&SpeedPID_L);
    PID_Init(&SpeedPID_R);
    /* ── SpeedPID (PI): 编码器信号有±3RPM噪声, 不适用D项 ── */
    PID_SetParam(&SpeedPID_L, 0.35f, 0.08f, 0.0f);
    PID_SetParam(&SpeedPID_R, 0.35f, 0.08f, 0.0f);
    SpeedPID_L.Kf = 0.69f;     /* 左轮 */
    SpeedPID_R.Kf = 0.65f;     /* 右轮 */
    SpeedPID_L.OutMax = SpeedPID_L.OutMin = 0; /* 后面统一设 */
    SpeedPID_L.OutMax = 100; SpeedPID_L.OutMin = -100;
    SpeedPID_R.OutMax = 100; SpeedPID_R.OutMin = -100;
    SpeedPID_L.OutOffset       = 0;
    SpeedPID_R.OutOffset       = 0;
    SpeedPID_L.IntegralLimit   = 40.0f;
    SpeedPID_R.IntegralLimit   = 40.0f;
    SpeedPID_L.Deadband        = 1.5f;
    SpeedPID_R.Deadband        = 1.5f;
    SpeedPID_L.OutputRateLimit = 35.0f;
    SpeedPID_R.OutputRateLimit = 35.0f;

    /* ── YawPID (PID): AHRS四元数信号已滤波, 干净可开D ── */
    PID_Init(&YawPID);
    PID_SetParam(&YawPID, 2.0f, 0.02f, 0.5f);
    YawPID.OutMax          =  20;
    YawPID.OutMin          = -20;
    YawPID.IntegralLimit   = 40.0f;
    YawPID.Deadband        = 1.0f;
    YawPID.OutputRateLimit = 10.0f;

    /* ── TrackPID (PI): 差速信号来自编码器, 噪声同SpeedPID ── */
    PID_Init(&TrackPID);
    PID_SetParam(&TrackPID, 0.15f, 0.04f, 0.0f);
    TrackPID.OutMax          =  25;
    TrackPID.OutMin          = -25;
    TrackPID.OutOffset       = 0;
    TrackPID.IntegralLimit   = 10.0f;
    TrackPID.Deadband        = 3.0f;
    TrackPID.OutputRateLimit = 12.0f;

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

        /* 停车保护已由 ISR Task_Control_50ms 处理, 主循环不再干预 */
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
 * @brief  MPU6050 数据解算 — Madgwick 自适应 AHRS (每 10ms, ISR 安全)
 * @note   异步 I2C 取数 → 填入 mpu 结构体 → 纯数学 AHRS 解算
 */
static void MPU6050_Process(void)
{
    if (!MPU6050_ReadAsync_Done()) return;

    MPU6050_ReadAsync_Get(&AX, &AY, &AZ, &GX, &GY, &GZ);

    /* 填入 mpu 结构体 (已减零偏, 由 MPU6050_init 校准) */
    mpu.AccX = AX; mpu.AccY = AY; mpu.AccZ = AZ;
    mpu.GyroX = GX; mpu.GyroY = GY; mpu.GyroZ = GZ;

    /* Madgwick AHRS (dt=0.01s) */
    MPU6050_AHRS_Update(&mpu, 0.01f);

    MPU6050_ReadAsync_Start();
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

    /* KEY4: 单击循环升档 (运行中自动更新目标) */
    if (Key_Check(KEY_4, KEY_SINGLE)) {
        SpeedLevel = (SpeedLevel < 6) ? SpeedLevel + 1 : 1;
        if (RunFlag) {
            float spd = SpeedTable[SpeedLevel-1];
            float sign = (SpeedPID_L.Target >= 0) ? 1.0f : -1.0f;
            msg.type = CMD_SPEED; msg.val = spd * sign;
            CmdQueue_Put(msg);
        }
    }

    if (Mode == MODE_PWM_SET) {
        int16_t prev = PWMDuty;
        if (Key_Check(KEY_2, KEY_SINGLE)) { PWMDuty += 10; if (PWMDuty >  100) PWMDuty =  100; }
        if (Key_Check(KEY_3, KEY_SINGLE)) { PWMDuty -= 10; if (PWMDuty < -100) PWMDuty = -100; }
        if (PWMDuty != prev) {
            if (PWMDuty == 0) {
                msg.type = CMD_STOP; msg.val = 0;
            } else {
                msg.type = CMD_START; msg.val = 0;
                CmdQueue_Put(msg);
                msg.type = CMD_SPEED; msg.val = PWMDuty;
            }
            CmdQueue_Put(msg);
        }
    } else {
        /* RUN 模式: KEY2单击=前进/停, KEY3单击=后退/停 */
        uint8_t k2 = Key_Check(KEY_2, KEY_SINGLE);
        uint8_t k3 = Key_Check(KEY_3, KEY_SINGLE);

        if (k2 || k3) {
            float spd = SpeedTable[SpeedLevel-1];
            if (k2) {
                if (RunFlag && SpeedPID_L.Target > 0) {
                    msg.type = CMD_STOP; msg.val = 0;
                } else {
                    msg.type = CMD_START; msg.val = 0;
                    CmdQueue_Put(msg);
                    msg.type = CMD_SPEED; msg.val = spd;
                }
            } else {
                if (RunFlag && SpeedPID_L.Target < 0) {
                    msg.type = CMD_STOP; msg.val = 0;
                } else {
                    msg.type = CMD_START; msg.val = 0;
                    CmdQueue_Put(msg);
                    msg.type = CMD_SPEED; msg.val = -spd;
                }
            }
            CmdQueue_Put(msg);
            msg.type = CMD_TURN; msg.val = 0;
            CmdQueue_Put(msg);
            bt_last_rx = millis;
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

    /*---- 处理命令队列 ----*/
    while (CmdQueue_Available()) {
        CmdMsg msg;
        if (CmdQueue_Get(&msg) < 0) break;

        switch (msg.type) {
        case CMD_SPEED:
            PID_SetTarget(&SpeedPID_L, msg.val);
            PID_SetTarget(&SpeedPID_R, msg.val);
            break;
        case CMD_SPEED_L:
            PID_SetTarget(&SpeedPID_L, msg.val);
            break;
        case CMD_SPEED_R:
            PID_SetTarget(&SpeedPID_R, msg.val);
            break;
        case CMD_TURN:
            TurnTarget = msg.val;
            break;
        case CMD_START:
            RunFlag = 1;
            break;
        case CMD_STOP:
            RunFlag = 0;
            PID_SetTarget(&SpeedPID_L, 0);
            PID_SetTarget(&SpeedPID_R, 0);
            TurnTarget = 0;
            SpeedPID_L.Out = 0; SpeedPID_L.IOut = 0;
            SpeedPID_L.POut = 0; SpeedPID_L.DOut = 0;
            SpeedPID_L.LastOutput = 0;
            SpeedPID_R.Out = 0; SpeedPID_R.IOut = 0;
            SpeedPID_R.POut = 0; SpeedPID_R.DOut = 0;
            SpeedPID_R.LastOutput = 0;
            TrackPID.Out = 0; TrackPID.IOut = 0;
            TrackPID.POut = 0; TrackPID.DOut = 0;
            TrackPID.LastOutput = 0;
            break;
        case CMD_GEAR_UP:
            if (SpeedLevel < 6) {
                SpeedLevel++;
                PID_SetTarget(&SpeedPID_L, SpeedTable[SpeedLevel-1]);
                PID_SetTarget(&SpeedPID_R, SpeedTable[SpeedLevel-1]);
            }
            break;
        case CMD_GEAR_DOWN:
            if (SpeedLevel > 1) {
                SpeedLevel--;
                PID_SetTarget(&SpeedPID_L, SpeedTable[SpeedLevel-1]);
                PID_SetTarget(&SpeedPID_R, SpeedTable[SpeedLevel-1]);
            }
            break;
        case CMD_SET_SPEED_KP: SpeedPID_L.Kp = SpeedPID_R.Kp = msg.val; break;
        case CMD_SET_SPEED_KI: SpeedPID_L.Ki = SpeedPID_R.Ki = msg.val; break;
        case CMD_SET_SPEED_KD: SpeedPID_L.Kd = SpeedPID_R.Kd = msg.val; break;
        case CMD_SET_SPEED_KF: SpeedPID_L.Kf = SpeedPID_R.Kf = msg.val; break;
        case CMD_SET_YAW_KP:   YawPID.Kp = msg.val; break;
        case CMD_SET_YAW_KI:   YawPID.Ki = msg.val; break;
        case CMD_SET_YAW_KD:   YawPID.Kd = msg.val; break;
        case CMD_SET_YAW_KF:   YawPID.Kf = msg.val; break;
        case CMD_YAW_TARGET:   YawTarget = msg.val;
                               PID_SetTarget(&YawPID, msg.val); break;
        case CMD_QUIET: VOFA_Enable = 0; break;
        case CMD_IMU_CALIB: MPU6050_Set_Angle0(&mpu); break;
        case CMD_LOG:   VOFA_Enable = 1; break;
        default: break;
        }
    }

    /* PWM_SET 模式: duty 值作为 RPM 目标, 走 PID 闭环 */
    if (Mode == MODE_PWM_SET) {
        PID_SetTarget(&SpeedPID_L, PWMDuty);
        PID_SetTarget(&SpeedPID_R, PWMDuty);
    }

    /* 停车时: 刷新遥测但 PID 不运算 */
    float dif = ecd_left.velocity.angular - ecd_right.velocity.angular;

    if (!RunFlag) {
        float spd_max = SpeedTable[SpeedLevel-1];
        if (spd_max < 1.0f) spd_max = 1.0f;
        vofa_data.target_l   = 0;
        vofa_data.target_r   = 0;
        vofa_data.rpm_r      = ecd_right.velocity.angular;
        vofa_data.rpm_l      = ecd_left.velocity.angular;
        vofa_data.pwm_l      = 0;
        vofa_data.pwm_r      = 0;
        vofa_data.yaw_out    = 0;
        vofa_data.rpm_diff   = 0;
        vofa_data.yaw_angle  = mpu.yaw;
        vofa_data.bt_lv      = BT_LV;
        vofa_data.bt_rh      = BT_RH;
        vofa_data.run_flag   = 0;
        vofa_data.gear       = SpeedLevel;
        vofa_data.fresh      = 1;
        Motor_SetSpeedA(0);
        Motor_SetSpeedB(0);
        return;
    }

    /*---- 左右轮 RPM 独立低通滤波 ----*/
    float target_l = SpeedPID_L.Target;
    float target_r = SpeedPID_R.Target;
    {
        static float l_f = 0, r_f = 0;
        static uint8_t filt_init = 0;
        if (!filt_init) {
            l_f = ecd_left.velocity.angular;
            r_f = ecd_right.velocity.angular;
            filt_init = 1;
        } else {
            l_f = l_f * 0.6f + ecd_left.velocity.angular  * 0.4f;
            r_f = r_f * 0.6f + ecd_right.velocity.angular * 0.4f;
        }
        SpeedPID_L.Actual = l_f;
        SpeedPID_R.Actual = r_f;
    }

    /* 左右轮各自独立 SpeedPID */
    PID_SetTarget(&SpeedPID_L, target_l);
    PID_Update(&SpeedPID_L);
    PID_SetTarget(&SpeedPID_R, target_r);
    PID_Update(&SpeedPID_R);

    // /*---- TrackPID (50ms): 差速纠偏 ----*/
    // {
    //     static float dif_f = 0;
    //     static uint8_t dif_init = 0;
    //     if (!dif_init) { dif_f = dif; dif_init = 1; }
    //     else { dif_f = dif_f * 0.6f + dif * 0.4f; }
    //     TrackPID.Actual = dif_f;
    // }
    // PID_SetTarget(&TrackPID, TurnTarget);
    // PID_Update(&TrackPID);
    // float tdiff = TrackPID.Out;

    /* 左右轮各自 PID 输出 */
    int16_t L = (int16_t)SpeedPID_L.Out;
    int16_t R = (int16_t)SpeedPID_R.Out;
    if (target_l > 0 && target_l < 20.0f) {
        if (L > 0 && L < 12) L = 12;
    } else if (target_l < 0 && target_l > -20.0f) {
        if (L < 0 && L > -12) L = -12;
    }
    if (target_r > 0 && target_r < 20.0f) {
        if (R > 0 && R < 12) R = 12;
    } else if (target_r < 0 && target_r > -20.0f) {
        if (R < 0 && R > -12) R = -12;
    }
    if (L >  100) L =  100;
    if (L < -100) L = -100;
    if (R >  100) R =  100;
    if (R < -100) R = -100;
    Motor_SetSpeedA(L);
    Motor_SetSpeedB(R);

    /*---- 填充遥测 ----*/
    vofa_data.target_l   = target_l;                       /* 左轮目标 RPM */
    vofa_data.target_r   = target_r;                       /* 右轮目标 RPM */
    vofa_data.rpm_r      = ecd_right.velocity.angular;   /* 右轮 RPM */
    vofa_data.rpm_l      = ecd_left.velocity.angular;    /* 左轮 RPM */
    vofa_data.pwm_l      = SpeedPID_L.Out;               /* 左 PID PWM % */
    vofa_data.pwm_r      = SpeedPID_R.Out;               /* 右 PID PWM % */
    vofa_data.yaw_out    = TrackPID.Out;                 /* 差速 RSS */
    vofa_data.rpm_diff   = dif;                           /* 左右轮 RPM 差 */
    vofa_data.yaw_angle  = mpu.yaw;
    vofa_data.bt_lv      = BT_LV;
    vofa_data.bt_rh      = BT_RH;
    vofa_data.run_flag   = RunFlag;
    vofa_data.gear       = SpeedLevel;
    vofa_data.fresh      = 1;
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
 * @note   每周期输出 2 行, L/R 各一行:
 *           "L,<Tgt>,<RPM>,<PWM>,<YawRSS>,<RPM_Diff>,<mpu.yaw>,<BT_LV>,<BT_RH>,<Run>,<Gear>"
 *           "R,<Tgt>,<RPM>,<PWM>,<YawRSS>,<RPM_Diff>,<mpu.yaw>,<BT_LV>,<BT_RH>,<Run>,<Gear>"
 *         VOFA 中按 L/R 标签分色显示, 速度用 RPM, PWM 保持百分比
 *====================================================================*/
void Task_VOFA_Send(void)
{
    if (!VOFA_Enable || !vofa_data.fresh) return;
    /* 静止时静默: 左右轮 RPM 均 < 0.5 且未运行 */
    if (!RunFlag && fabsf(vofa_data.rpm_r) < 0.5f && fabsf(vofa_data.rpm_l) < 0.5f) return;
    vofa_data.fresh = 0;

    char buf[128];
    int n;

    /* 防御: 钳位异常值 */
    int _bt_lv = (int)vofa_data.bt_lv;
    int _bt_rh = (int)vofa_data.bt_rh;
    int _run   = (int)vofa_data.run_flag;
    int _gear  = (int)vofa_data.gear;
    if (_bt_lv < -128 || _bt_lv > 127) _bt_lv = 0;
    if (_bt_rh < -128 || _bt_rh > 127) _bt_rh = 0;

    /* 拆成两次 snprintf: float/double(8B) 和 int(4B) 混用
       会导致变参栈偏移错误 — newlib-nano Cortex-M3 已知问题 */
    n = snprintf(buf, sizeof(buf),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,",
        vofa_data.target_r, vofa_data.rpm_r, vofa_data.pwm_r,
        vofa_data.target_l, vofa_data.rpm_l, vofa_data.pwm_l,
        vofa_data.yaw_out, vofa_data.rpm_diff, vofa_data.yaw_angle);
    if (n > 0 && n < (int)sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - n, "%d,%d,%d,%d\r\n",
            _bt_lv, _bt_rh, _run, _gear);
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
            SpeedPID_L.Target, (SpeedPID_L.Out + SpeedPID_R.Out) / 2.0f);
    }
    OLED_ShowString(2, 1, buf);

    snprintf(buf, sizeof(buf), "L:%+04.0f R:%+04.0f",
        ecd_left.velocity.angular, ecd_right.velocity.angular);
    OLED_ShowString(3, 1, buf);

    snprintf(buf, sizeof(buf), "Y:%+05.1f BT:%+04d ",
        mpu.yaw, (int)BT_LV);
    OLED_ShowString(4, 1, buf);
}

/*====================================================================
 * MPU6050 零偏校准 (启动时调用一次, 阻塞)
 * @brief  采样 200 次取平均, 计算陀螺 XYZ 零偏值
 *====================================================================*/
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
