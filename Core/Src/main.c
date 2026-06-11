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
    float target_pct;    /* 目标速度% */
    float rpm_r;         /* 右轮 -RPM/2 */
    float rpm_l;         /* 左轮 -RPM/2 */
    float speed_out;     /* SpeedPID 输出 */
    float yaw_out;       /* YawPID 输出 */
    float turn_target;   /* 手动转向 RPM */
    float rpm_diff;      /* 左右轮 RPM 差 */
    float yaw_angle;     /* 偏航角 ° */
    int8_t bt_lv;        /* 蓝牙摇杆 LV */
    int8_t bt_rh;        /* 蓝牙摇杆 RH */
    uint8_t run_flag;    /* 0/1 */
    uint8_t fresh;       /* 新数据标志 */
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
static const float SpeedTable[6] = {20, 60, 120, 180, 250, 350};

float    TurnTarget = 0;
float    YawTarget  = 0;
int8_t   BT_LV = 0, BT_RH = 0;

/* PID 控制器 ----------------------------------------------------------------*/
PID_t SpeedPID = {
    .Kp = 0.8f, .Ki = 0.01f, .Kd = 0.6f, .Kf = 0.0f,
    .OutMax =  100,  .OutMin = -100,
    .OutOffset       = 15.0f,
    .IntegralLimit   = 50.0f,
    .Deadband        = 3.0f,
    .OutputRateLimit = 0.0f,
};
PID_t YawPID = {
    .Kp = 0.3f, .Ki = 0.02f, .Kd = 0.0f,
    .OutMax =  20,   .OutMin = -20,
    .OutOffset       = 0,
    .IntegralLimit   = 40.0f,
    .Deadband        = 1.0f,
    .OutputRateLimit = 10.0f,
};
PID_t PulsePID;                             /* 内环: 脉冲率 → PWM (10ms) */
static float inner_diff = 0;                /* 外环差速, 内环读取 */

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
    PIDTune_Init();
    Buzzer_Init();
    BlueSerial_Init();

    initEncoder(&ecd_left,  JGB37_520);
    initEncoder(&ecd_right, JGB37_520);

    PID_Init(&SpeedPID);
    /* SpeedPID 对标平衡车：只做微调修正，主体由外部前馈负责 */
    PID_SetParam(&SpeedPID, 0.15f, 0.05f, 0.3f);
    SpeedPID.Kf              = 0.0f;        /* 前馈移到 PID 外部 */
    SpeedPID.OutMax          =  30;         /* 只修正 ±136 RPM，对标平衡车 ±20° */
    SpeedPID.OutMin          = -30;
    SpeedPID.OutOffset       = 0;
    SpeedPID.IntegralLimit   = 20.0f;
    SpeedPID.Deadband        = 3.0f;
    SpeedPID.OutputRateLimit = 0.0f;

    PID_Init(&YawPID);
    PID_SetParam(&YawPID, 0.0f, 0.0f, 0.0f);
    YawPID.OutMax          =  20;
    YawPID.OutMin          = -20;
    YawPID.IntegralLimit   = 40.0f;
    YawPID.Deadband        = 1.0f;
    YawPID.OutputRateLimit = 10.0f;

    /* 内环 PulsePID: 脉冲率 → PWM, 10ms 高频 */
    PID_Init(&PulsePID);
    PID_SetParam(&PulsePID, 2.0f, 0.1f, 0.3f);
    PulsePID.Kf              = 2.5f;        /* 脉冲→PWM前馈 */
    PulsePID.OutMax          =  100;
    PulsePID.OutMin          = -100;
    PulsePID.OutOffset       = 15.0f;       /* TB6612 死区补偿 */
    PulsePID.IntegralLimit   = 15.0f;
    PulsePID.Deadband        = 1.0f;
    PulsePID.OutputRateLimit = 0.0f;

    MPU6050_Calibrate();
    MPU6050_ReadAsync_Start();   /* 启动非阻塞 I2C 链路, 供 ISR 消费 */

    OLED_Clear();
    OLED_ShowString(1, 1, " [BlueCar V3.0] ");
    OLED_ShowString(2, 1, " Co-op Tasks   ");
    OLED_ShowString(3, 1, " DMA TX + RB   ");
    OLED_ShowString(4, 1, " Press C8 start ");
    Buzzer_ON();  HAL_Delay(200);  Buzzer_OFF();

    while (Key_GetNum() != 3);
    OLED_Clear();

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
        /* ---- 串口命令响应 (不受 VOFA 数据流影响) ---- */
        if (tune_rsp_pending) {
            Serial_SendString(tune_rsp_buf);
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
            msg.type = CMD_SPEED; msg.val = 0; CmdQueue_Put(msg);
            msg.type = CMD_TURN;  msg.val = 0; CmdQueue_Put(msg);
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
            inner_diff = 0;
            SpeedPID.Out = 0; SpeedPID.IOut = 0;
            SpeedPID.POut = 0; SpeedPID.DOut = 0;
            SpeedPID.LastOutput = 0;
            PulsePID.Out = 0; PulsePID.IOut = 0;
            PulsePID.POut = 0; PulsePID.DOut = 0;
            PulsePID.LastOutput = 0;
            PulsePID.Target = 0;
            break;
        case CMD_GEAR_UP:
            if (SpeedLevel < 6) SpeedLevel++;
            break;
        case CMD_GEAR_DOWN:
            if (SpeedLevel > 1) SpeedLevel--;
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

    if (!RunFlag) return;

    /*---- 外环 SpeedPID (50ms): RPM误差 → 脉冲率目标 ----*/
    float ave = (ecd_left.velocity.angular + ecd_right.velocity.angular) / 2.0f;

    /* EMA 低通滤波 RPM */
    static float rpm_filtered = 0;
    float rpm_raw = -ave;
    rpm_filtered = 0.3f * rpm_raw + 0.7f * rpm_filtered;

    /* 目标斜坡: 每 50ms 最多变 8 RPM */
    static float ramp_target = 0;
    float delta = SpeedPID.Target - ramp_target;
    if      (delta >  8.0f) ramp_target += 8.0f;
    else if (delta < -8.0f) ramp_target -= 8.0f;
    else                    ramp_target = SpeedPID.Target;

    SpeedPID.Actual = rpm_filtered;
    float save_tgt = SpeedPID.Target;
    SpeedPID.Target = ramp_target;
    PID_Update(&SpeedPID);
    SpeedPID.Target = save_tgt;

    /* 前馈基线 (对标平衡车：SpeedPID 只出微调，主体由外部前馈负责) */
    float ff_pulse = SpeedPID.Target * 0.22f;   /* RPM → 脉冲/10ms */

    /* SpeedPID.Out 是纯修正量，EMA 滤波后叠加到前馈基线上 */
    static float correction_ema = 0;
    if (ramp_target <= 8.0f) correction_ema = 0;       /* 起步时清零残留 */
    correction_ema = 0.25f * SpeedPID.Out + 0.75f * correction_ema;
    PulsePID.Target = ff_pulse + correction_ema;

    /*---- 外环 YawPID (100ms) ----*/
    static uint8_t outer_cycle = 0;
    if (++outer_cycle >= 2) {
        outer_cycle = 0;
        YawPID.Actual = YawAngle;
        PID_Update(&YawPID);
    }
    float diff_yaw = 0;  // YawPID.Out (暂关)

    /* 差速 (内环 10ms 读取, 叠加到左右轮 PWM) */
    inner_diff = TurnTarget + diff_yaw;

    /*---- 填充遥测快照 (供 VOFA 输出) ----*/
    float spd_max = SpeedTable[SpeedLevel-1];
    vofa_data.target_pct  = SpeedPID.Target / spd_max * 100.0f;
    vofa_data.rpm_r       = -ecd_right.velocity.angular / 2.0f;
    vofa_data.rpm_l       = -ecd_left.velocity.angular / 2.0f;
    vofa_data.speed_out   = PulsePID.Out;     /* 内环 PWM 输出 */
    vofa_data.yaw_out     = YawPID.Out;
    vofa_data.turn_target = TurnTarget;
    vofa_data.rpm_diff    = ecd_left.velocity.angular - ecd_right.velocity.angular;
    vofa_data.yaw_angle   = YawAngle;
    vofa_data.bt_lv       = BT_LV;
    vofa_data.bt_rh       = BT_RH;
    vofa_data.run_flag    = RunFlag;
    vofa_data.fresh       = 1;
}

/*====================================================================
 * VOFA 输出任务 (50ms, 在 Control 之后)
 * @brief  遥测快照格式化为 CSV → DMA TX 非阻塞发送
/*====================================================================
 * 内环控制任务 (10ms): 脉冲率 → PWM, 高频兜底
 * @brief  读取 TIM2/TIM3 编码器原始 CNT, 差分得 10ms 脉冲增量
 *          PulsePID 把脉冲误差转为 PWM, 叠加外环差速
 * @note   外环 50ms 更新 PulsePID.Target, 内环 10ms 消费
 *====================================================================*/
void Task_Pulse_10ms(void)
{
    static uint16_t last_cnt2 = 0, last_cnt3 = 0;
    static uint8_t first = 1;

    if (!RunFlag) {
        PulsePID.Out = 0; PulsePID.IOut = 0; PulsePID.POut = 0;
        PulsePID.DOut = 0; PulsePID.LastOutput = 0;
        Motor_SetSpeedA(0);
        Motor_SetSpeedB(0);
        first = 1;
        return;
    }

    uint16_t now2 = TIM2->CNT;
    uint16_t now3 = TIM3->CNT;

    if (first) { last_cnt2 = now2; last_cnt3 = now3; first = 0; return; }

    int16_t d_r = (int16_t)(now2 - last_cnt2);         /* 右轮 (TIM2) */
    int16_t d_l = -(int16_t)(now3 - last_cnt3);         /* 左轮 (TIM3) 取反 */

    last_cnt2 = now2;
    last_cnt3 = now3;

    float pulse_actual = (d_r + d_l) / 2.0f;            /* 两轮平均脉冲/10ms */
    PulsePID.Actual = pulse_actual;
    PID_Update(&PulsePID);

    int16_t L = (int16_t)(PulsePID.Out + inner_diff);
    int16_t R = (int16_t)(PulsePID.Out - inner_diff);
    if (L >  100) L =  100;  if (L < -100) L = -100;
    if (R >  100) R =  100;  if (R < -100) R = -100;

    Motor_SetSpeedA(L);
    Motor_SetSpeedB(R);
}

/*====================================================================
 * VOFA 输出任务 (50ms, 在 Control 之后)
 * @brief  遥测快照格式化为 CSV → DMA TX 非阻塞发送
 * @note   数据字段 (11列): Target%, RPM_R, RPM_L, SpeedOut, YawOut,
 *         TurnTarget, RPM_Diff, YawAngle, BT_LV, BT_RH, RunFlag
 *         使用 snprintf 防栈溢出, 通过 Serial_Write → RingBuf → DMA 发送
 *====================================================================*/
void Task_VOFA_Send(void)
{
    if (!VOFA_Enable || !vofa_data.fresh) return;
    vofa_data.fresh = 0;

    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d\r\n",
        vofa_data.target_pct,
        vofa_data.rpm_r,
        vofa_data.rpm_l,
        vofa_data.speed_out,
        vofa_data.yaw_out,
        vofa_data.turn_target,
        vofa_data.rpm_diff,
        vofa_data.yaw_angle,
        vofa_data.bt_lv,
        vofa_data.bt_rh,
        vofa_data.run_flag);

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
    OLED_Clear();  /* 清帧缓冲, 准备重绘 */
    char buf[21];
    if (Mode == MODE_PWM_SET) {
        OLED_ShowString(1, 1, "[PWM SET MODE]  ");
        snprintf(buf, sizeof(buf), "Duty: %+04d %%    ", PWMDuty);
    } else {
        OLED_ShowString(1, 1, RunFlag ? "[RUN] V3.0     " : "[STOP]          ");
        snprintf(buf, sizeof(buf), "T:%+05.0f D:%+05.0f ", SpeedPID.Target, TurnTarget);
    }
    OLED_ShowString(2, 1, buf);

    snprintf(buf, sizeof(buf), "L:%+05.0f R:%+05.0f ", ecd_left.velocity.angular, ecd_right.velocity.angular);
    OLED_ShowString(3, 1, buf);

    snprintf(buf, sizeof(buf), "Yaw:%+06.1f Lv:%1d", YawAngle, SpeedLevel);
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
