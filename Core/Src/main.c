/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BlueCar - 蓝牙遥控 + 按键双控四轮小车 (串级 PID)
  *                   硬件: STM32F103ZET6 + TB6612 + JGB37-520 + HC-05 + MPU6050 + OLED
  *
  *                   控制架构 (参考平衡车测试程序-V1.0):
  *                     SpeedPID(内环, 50ms) → base PWM
  *                     YawPID(外环, 100ms)  → diff_yaw 纠偏
  *                     手动转向(BT/按键)     → diff_manual 直接叠加
  *                     L = base + diff, R = base - diff
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
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum { MODE_RUN, MODE_PWM_SET } Mode_t;

/* Shared with pid_tune.c ----------------------------------------------------*/
Mode_t   Mode       = MODE_RUN;
uint8_t  RunFlag    = 0;
int16_t  PWMDuty    = 0;
static uint8_t  SpeedLevel   = 3;        /* 蓝牙速度档位 1~6 */
static uint8_t  SpeedPercent = 50;       /* 按键速度百分比 0~100 */
uint32_t bt_last_rx = 0;               /* 上次收到蓝牙数据的时间戳 */
static const float SpeedTable[6] = {20, 60, 120, 180, 250, 350}; /* 档位1~6 RPM映射 */

float    TurnTarget = 0;               /* 手动差速目标 (RPM), BT/按键/串口设置 */
float    YawTarget  = 0;               /* 航向角目标 (°) */
int8_t   BT_LV = 0, BT_RH = 0;        /* 蓝牙摇杆值, VOFA 显示用 */

/* PID 控制器 ----------------------------------------------------------------
 *
 * SpeedPID: P 主导纯反馈控制 (无前馈, 无变化率限制)
 *
 *   设计策略:
 *     Kf=0 完全去掉前馈 — 纯反馈, P 项主导输出
 *     Kp=0.8 大比例增益 — 50RPM 误差 → POut=40, 加 OutOffset=15 得 55%起步PWM
 *     Kd=0.6 强微分阻尼 — 配合大Kp抑制超调震荡 (旧 Kd=0.15 时 Kp>0.5 就震荡)
 *     Ki=0.01 适中的积分 — Kf归零后 I 需承担更多稳态输出
 *     IntegralLimit=50 — 放宽积分上限, I 最多补 50% PWM
 *     OutputRateLimit=0 — 完全解除变化率限制, 响应即时, 靠 Kd 天然软限速
 *
 *   输出估算 (Target=50RPM):
 *     起步: POut(40) + IOut(0) + OutOffset(15) = 55
 *     稳态: POut(~1.6) + IOut(~45) + OutOffset(15) ≈ 61.6  (误差2RPM时)
 *
 * YawPID 输出限幅 ±20: 外环只微调, 不主导转向, 手动 diff 直接叠加不走 PID
 *----------------------------------------------------------------------------*/
PID_t SpeedPID = {
    .Kp = 0.8f, .Ki = 0.01f, .Kd = 0.6f, .Kf = 0.0f,  /* P主导, 无前馈 */
    .OutMax =  100,  .OutMin = -100,   /* PWM 物理上下限 */
    .OutOffset       = 15.0f,          /* 电机死区补偿 (不变) */
    .IntegralLimit   = 50.0f,          /* 积分限幅 30→50 (Kf归零后I需补更多) */
    .Deadband        = 3.0f,           /* 积分死区 (不变, 编码器抖动特性没变) */
    .OutputRateLimit = 0.0f,           /* 变化率限制 20→0 (完全解除, 靠Kd抑制突变) */
};
PID_t YawPID = {
    .Kp = 0.3f, .Ki = 0.02f, .Kd = 0.0f,
    .OutMax =  20,   .OutMin = -20,   /* 差速 ±20RPM */
    .OutOffset       = 0,              /* 差速不需要死区补偿 */
    .IntegralLimit   = 40.0f,          /* 比SpeedPID大, 陀螺仪漂移需更多积分余量 */
    .Deadband        = 1.0f,           /* ±1°停止积分, 防陀螺仪噪声摇摆 */
    .OutputRateLimit = 10.0f,          /* 外环100ms, 变化更平缓 */
};

/* 编码器 --------------------------------------------------------------------*/
static Encoder  ecd_left, ecd_right;
static const Parameter JGB37_520 = {
    .multiple = 4, .reduction_ratio = 30.0f, .ppr = 11, .r = 0.065f,
};

/* MPU6050 传感器数据 ---------------------------------------------------------*/
static int16_t AX, AY, AZ, GX, GY, GZ;
static int16_t GZ_Offset       = 0;   /* GZ 陀螺仪零偏 */
static int16_t AX_Offset       = 0;   /* AX 加速度计零偏 */
static int16_t AZ_Offset       = 0;   /* AZ 加速度计零偏 */
static float YawAngle    = 0.0f;      /* Z 轴积分偏航角 (°) */
static float TiltAngle   = 0.0f;      /* 倾角 (AX/AZ atan2) */
static float TiltFilter  = 0.0f;      /* 倾角一阶低通滤波值 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Process_Keys(void);
static void Process_Bluetooth(void);
static void Process_Control(void);
static void Update_OLED(void);
static void VOFA_Send(void);
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
    Buzzer_Init();
    BlueSerial_Init();

    initEncoder(&ecd_left,  JGB37_520);
    initEncoder(&ecd_right, JGB37_520);

/**
 * @brief 串级 PID 参数初始化
 *
 * ┌──────────────────────────────────────────────────────────┐
 * │ 串级控制架构 (参考平衡车测试程序-V1.0)                      │
 * │                                                          │
 * │   SpeedTarget ─→ SpeedPID(内环50ms) ─→ base PWM          │
 * │                     ↑                                    │
 * │                  平均转速                                 │
 * │                                                          │
 * │   YawTarget ─→ YawPID(外环100ms) ─→ diff_yaw             │
 * │                     ↑                  ↘                 │
 * │                  偏航角               diff ─→ L/R PWM     │
 * │                                    ↗                     │
 * │   BT摇杆/按键 ─→ TurnTarget ─→ diff_manual               │
 * │                                                          │
 * │   最终: L = base+diff, R = base-diff, 双侧钳位±100       │
 * └──────────────────────────────────────────────────────────┘
 *
 * SpeedPID: 输出是 PWM 占空比 (-100~+100)
 * YawPID:   输出是差速 RPM (-20~+20), 只做微调不主导转向
 */

/*───────────────────────────────────────────────────────────
 * SpeedPID (内环, 50ms): 平均转速 → PWM 占空比
 *
 * 设计策略: P 主导纯反馈 (Kf=0, OutputRateLimit=0)
 *   Kp=0.8  大比例增益 — 50RPM 误差 → POut=40, P项主导输出
 *   Kd=0.6  强微分阻尼 — 配合大Kp抑制超调, RPM突变100时抵消60单位输出
 *   Ki=0.01 适中积分   — Kf归零后 I 承担稳态输出, 累积速度提升 2x
 *   Kf=0    无前馈     — 纯反馈控制, 不为Target预设输出基线
 *   OutputRateLimit=0   — 完全解除变化率限制, 响应即时, Kd天然软限速
 *   IntegralLimit=50   — 积分上限 30→50, Kf归零后I需补更多
 *
 * 输出估算 (Target=50RPM):
 *   起步: POut(40) + IOut(0) + OutOffset(15) = 55
 *   稳态: POut(~1.6) + IOut(~45) + OutOffset(15) ≈ 61.6  (误差2RPM时)
 *───────────────────────────────────────────────────────────*/
PID_Init(&SpeedPID);
PID_SetParam(&SpeedPID, 0.2f, 0.06f, 0.1f);  /* Kp=0.3 Ki=0.02 Kd=0.1 */
SpeedPID.Kf = 0.30f;              /* 前馈: 0.65→0 (完全去掉前馈, 纯反馈) */
SpeedPID.OutMax =  100;           /* 正转PWM上限 (物理边界, 不变) */
SpeedPID.OutMin = -100;           /* 反转PWM下限 (物理边界, 不变) */
SpeedPID.OutOffset       = 15.0f; /* 死区补偿 (不变, TB6612硬件特性) */
SpeedPID.IntegralLimit   = 35.0f; /* 积分限幅 30→50 (Kf归零后I需承担更多稳态输出) */
SpeedPID.Deadband        = 3.0f;  /* 积分死区 (不变, 编码器抖动特性没变) */
SpeedPID.OutputRateLimit = 12.0f;  /* 变化率限制 20→0 (完全解除, 靠Kd微分先行抑制突变) */

/*───────────────────────────────────────────────────────────
 * YawPID (外环, 100ms): 航向角 → 差速 RPM 纠偏
 *
 * 设计思路:
 *   外环只做微调, 输出限幅 ±20 RPM, 不主导转向
 *   手动转向 (TurnTarget) 直接叠加, 不走PID, 响应即时
 *   航向角由 MPU6050 陀螺仪 Z 轴积分得到
 *
 * 为什么外环比内环慢一倍 (100ms vs 50ms)?
 *   - 航向变化本身是个慢过程 (转动惯量大)
 *   - 内外环频率错开 = 分离时间尺度, 互不干涉
 *   - 内环先稳定速度, 外环再在稳定速度上做航向纠偏
 *───────────────────────────────────────────────────────────*/
PID_Init(&YawPID);
PID_SetParam(&YawPID, 0.0f, 0.0f, 0.0f);//0.3 0.02 0.0
YawPID.OutMax =  20;              /* 差速上限: ±20RPM差速足以产生明显转向力矩 */
YawPID.OutMin = -20;              /* 差速下限: 对称限幅 */
/* OutOffset=0: 差速RPM不需要死区补偿 (直接叠加到PWM, 死区已在SpeedPID处理) */
YawPID.IntegralLimit   = 40.0f;   /* 积分限幅: 比SpeedPID大10, 陀螺仪漂移需更多积分余量 */
YawPID.Deadband        = 1.0f;    /* 积分死区: 航向误差≤1°停积积分, 防陀螺仪噪声引起左右摇摆 */
YawPID.OutputRateLimit = 10.0f;   /* 变化率限制: 外环100ms一次, 每周期最多变±10RPM, 防突然甩头 */

    MPU6050_Calibrate();

    OLED_Clear();
    OLED_ShowString(1, 1, " [BlueCar V2.1] ");
    OLED_ShowString(2, 1, " Cascade PID    ");
    OLED_ShowString(3, 1, " Speed+Yaw Ctrl ");
    OLED_ShowString(4, 1, " Press C8 start ");
    Buzzer_ON();  HAL_Delay(200);  Buzzer_OFF();

    while (Key_GetNum() != 3);
    OLED_Clear();

    while (1)
    {
        Process_Bluetooth();
        Process_Keys();

        /* 10ms: MPU6050 GZ积分偏航角 (参考平衡车陀螺仪解算) */
        if (flag_10ms) {
            flag_10ms = 0;
            MPU6050_Process();
        }

        /* 50ms: 编码器 + PID 控制 */
        if (flag_50ms) {
            flag_50ms = 0;
            updateEncoderLoopSimpleVersion(&ecd_left,  50, TIM3);  /* 左轮编码器 → TIM3 */
            updateEncoderLoopSimpleVersion(&ecd_right, 50, TIM2);  /* 右轮编码器 → TIM2 */

            if (Mode == MODE_RUN)
                Process_Control();
            else
                Motor_SetSpeedA(PWMDuty), Motor_SetSpeedB(PWMDuty);

            VOFA_Send();
        }

        static uint32_t oled_tick = 0;
        if (millis - oled_tick > 200) {
            oled_tick = millis;
            Update_OLED();
        }

        if (!RunFlag) {
            Motor_SetSpeedA(0); Motor_SetSpeedB(0);
        }
    }
}

/*============================================================
 * MPU6050 数据解算 (参考平衡车测试程序-V1.0)
 *   每 10ms 调用: 读取原始数据 → 零偏校准 → 积分偏航角 + 倾角互补滤波
 *   停车时自动追踪 GZ 零偏漂移 (指数滑动平均), 无需手动校准
 *============================================================*/
static void MPU6050_Process(void)
{
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

    /* ---- 停车时自动校准 GZ 零偏 (指数滑动平均, 追踪温漂) ---- */
    static int32_t gz_sum = 0;
    static uint16_t gz_cnt = 0;
    if (!RunFlag) {
        gz_sum += GZ;
        if (++gz_cnt >= 100) {                          /* 1 秒采集 100 个点 */
            int16_t new_offset = (int16_t)(gz_sum / 100);
            GZ_Offset = (int16_t)(GZ_Offset * 0.9f + new_offset * 0.1f);  /* EMA α=0.1 */
            gz_sum = 0;
            gz_cnt = 0;
        }
    } else {
        gz_sum = 0;
        gz_cnt = 0;
    }

    /* GZ 陀螺仪校准 → °/s (参考平衡车: GY_Cali / 32768.0 * 2000.0) */
    float gz = (float)(GZ - GZ_Offset) / 32768.0f * 2000.0f;

    /* 偏航角积分 (10ms 周期) */
    YawAngle += gz * 0.01f;

    /* 倾角计算 (参考平衡车: AngleAcc = -atan2(AX, AZ) / PI * 180) */
    float ax_cali = (float)(AX - AX_Offset);
    float az_cali = (float)(AZ - AZ_Offset);
    float AngleAcc = -atan2f(ax_cali, az_cali) / 3.1415926535f * 180.0f;

    /* 一阶低通滤波 (参考平衡车 Alpha0 = 0.8) */
    float Alpha0 = 0.8f;
    TiltFilter = Alpha0 * TiltFilter + (1.0f - Alpha0) * AngleAcc;
    TiltAngle = TiltFilter;
}

/*============================================================
 * 按键处理
 *============================================================*/
static void Process_Keys(void)
{
    /* KEY1: 单击切换模式 */
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

    /* KEY4: 单击循环升档 1→6, 同时复位速度百分比 */
    if (Key_Check(KEY_4, KEY_SINGLE)) {
        SpeedLevel = (SpeedLevel < 6) ? SpeedLevel + 1 : 1;
        SpeedPercent = 50;
    }

    if (Mode == MODE_PWM_SET) {
        if (Key_Check(KEY_2, KEY_SINGLE)) { PWMDuty += 10; if (PWMDuty >  100) PWMDuty =  100; }
        if (Key_Check(KEY_3, KEY_SINGLE)) { PWMDuty -= 10; if (PWMDuty < -100) PWMDuty = -100; }
        RunFlag = (PWMDuty != 0);
    } else {
        /* RUN 模式: KEY2=前进(百分比递增) / KEY3=后退(百分比递增) */
        uint8_t k2 = Key_Check(KEY_2, KEY_HOLD);
        uint8_t k3 = Key_Check(KEY_3, KEY_HOLD);

        if (k2 || k3) {
            /* 按住时百分比递增 (每50ms+1, 5秒满) */
            if (SpeedPercent < 100) SpeedPercent++;
            RunFlag = 1;
            float spd = SpeedTable[SpeedLevel-1] * SpeedPercent / 100.0f;
            PID_SetTarget(&SpeedPID, k2 ? spd : -spd);
            TurnTarget = 0;
            bt_last_rx = millis;  /* 防止超时停车 */
        }
        else if (millis - bt_last_rx > 300) {
            /* 松手 + 无蓝牙 → 停车, 百分比归零 */
            RunFlag = 0;
            PID_SetTarget(&SpeedPID, 0);
            TurnTarget = 0;
            SpeedPercent = 0;
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
    bt_last_rx = millis;

    /* 摇杆: [joystick,LH,LV,RH,RV] */
    if (strcmp(BlueSerial_StringArray[0], "joystick") == 0) {
        int8_t LV = atoi(BlueSerial_StringArray[2]);
        int8_t RH = atoi(BlueSerial_StringArray[3]);
        BT_LV = LV; BT_RH = RH;
        if (LV != 0 || RH != 0) {
            RunFlag = 1;
            PID_SetTarget(&SpeedPID, LV / 100.0f * SpeedTable[SpeedLevel-1]);
            TurnTarget = RH / 100.0f * SpeedTable[SpeedLevel-1] * 0.5f;
        } else {
            PID_SetTarget(&SpeedPID, 0);
            TurnTarget = 0;
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

/*────────────────────────────────────────────────────────────
 * PID 控制: 串级结构 (参考平衡车测试程序-V1.0)
 *
 *   调用频率: 每 50ms (由 TIM6 中断的 flag_50ms 触发)
 *
 *   内环 SpeedPID (50ms): 平均转速 → base PWM
 *     输入: (左轮RPM + 右轮RPM) / 2
 *     输出: base PWM ( -100 ~ +100 )
 *     作用: 让车按目标速度跑, 不关心航向
 *
 *   外环 YawPID (100ms): 航向角 → diff_yaw 纠偏
 *     输入: MPU6050 陀螺仪Z轴积分偏航角
 *     输出: diff_yaw RPM ( -20 ~ +20, 限幅在PID内部 )
 *     作用: 航向偏离时自动拉回, 100ms一次比内环慢一倍
 *           (航向变化慢 + 分离时间尺度, 内外环互不干涉)
 *
 *   手动转向: TurnTarget → diff_manual
 *     来源: 蓝牙摇杆RH / 按键 / 串口 turn 命令
 *     不走PID, 直接叠加, 响应即时
 *
 *   合成: diff = diff_manual + diff_yaw
 *         L = base + diff    (左轮PWM)
 *         R = base - diff    (右轮PWM)
 *         双侧钳位 ±100 (最后一道防线, 防止PWM溢出)
 *──────────────────────────────────────────────────────────*/
static void Process_Control(void)
{
    /* 平均转速 = 左右轮 RPM 均值 */
    float ave = (ecd_left.velocity.angular + ecd_right.velocity.angular) / 2.0f;

    if (!RunFlag) return;

    /* ---- 内环 SpeedPID (50ms): 平均转速闭环 ---- */
    SpeedPID.Actual = -ave;   /* 编码器与PWM输出方向相反 */
    PID_Update(&SpeedPID);
    float base = SpeedPID.Out;        /* base ∈ [-100, +100] */

    /* ---- 超速保护: |RPM| > 110 时反向拉输出, 抑制冲过头 ---- */
    if      (ave >  110.0f) base -= (ave - 110.0f) * 1.5f;
    else if (ave < -110.0f) base -= (ave + 110.0f) * 1.5f;

    /* ---- 外环 YawPID (100ms): 航向角闭环 ----
     * 外环每 2 次 (100ms) 才执行一次, 原因:
     *   1. 航向变化是慢过程 (转动惯量大), 不需要 50ms 高频
     *   2. 内外环时间尺度分离, 内环先收敛再纠航向, 互不干涉
     */
    static uint8_t outer_cycle = 0;
    if (++outer_cycle >= 2) {
        outer_cycle = 0;
        YawPID.Actual = YawAngle;
        PID_Update(&YawPID);
    }
    float diff_yaw = 0;  //YawPID.Out;    /* YawPID 暂关, 先调 SpeedPID */

    /* ---- 手动转向 (BT/按键直接叠加, 不走PID, 响应即时) ---- */
    float diff_manual = TurnTarget;

    /* 合成差速 + 左右轮 PWM 分配 */
    float diff = diff_manual + diff_yaw;
    int16_t L = (int16_t)(base + diff);   /* 左轮 = base + 差速 */
    int16_t R = (int16_t)(base - diff);   /* 右轮 = base - 差速 (差速转向原理) */

    /* 最终输出钳位 — 最后一道防线
     * 理论上 base(≤±100) + diff(≤±40) 可能超限, 必须钳位
     * 钳位后可能导致短暂转向失效 (两侧同时被限), 但优先保证PWM不溢出 */
    if (L >  100) L =  100;  if (L < -100) L = -100;
    if (R >  100) R =  100;  if (R < -100) R = -100;

    Motor_SetSpeedA(L);
    Motor_SetSpeedB(R);
}

/*============================================================
 * OLED 显示
 *============================================================*/
static void Update_OLED(void)
{
    char buf[21];
    if (Mode == MODE_PWM_SET) {
        OLED_ShowString(1, 1, "[PWM SET MODE]  ");
        snprintf(buf, sizeof(buf), "Duty: %+04d %%    ", PWMDuty);
    } else {
        OLED_ShowString(1, 1, RunFlag ? "[RUN] Cascade   " : "[STOP]          ");
        snprintf(buf, sizeof(buf), "T:%+05.0f D:%+05.0f ", SpeedPID.Target, TurnTarget);
    }
    OLED_ShowString(2, 1, buf);

    snprintf(buf, sizeof(buf), "L:%+05.0f R:%+05.0f ", ecd_left.velocity.angular, ecd_right.velocity.angular);
    OLED_ShowString(3, 1, buf);

    snprintf(buf, sizeof(buf), "Yaw:%+06.1f Lv:%1d", YawAngle, SpeedLevel);
    OLED_ShowString(4, 1, buf);
}

/*============================================================
 * VOFA+: 编码器数据通过 USART3 发送 (兼容平衡车数据格式)
 *============================================================*/
static void VOFA_Send(void)
{
    float spd_max = SpeedTable[SpeedLevel-1];
    Serial_Printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d\r\n",
        SpeedPID.Target / spd_max * 100.0f,                /* 目标%% */
        ecd_right.velocity.angular / spd_max * 100.0f,     /* 右轮%% */
        ecd_left.velocity.angular  / spd_max * 100.0f,     /* 左轮%% */
        SpeedPID.Out,                                      /* PWM% */
        YawPID.Out,
        TurnTarget,
        ecd_left.velocity.angular - ecd_right.velocity.angular,
        YawAngle,
        BT_LV, BT_RH,
        RunFlag);
}

/*============================================================
 * MPU6050 零偏校准 (参考平衡车测试程序-V1.0)
 *============================================================*/
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
