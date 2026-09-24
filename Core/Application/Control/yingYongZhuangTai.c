/**
 * @file    yingYongZhuangTai.c
 * @brief   应用层状态、速度档位和 PID 参数初始化。
 */

#include "yingYongZhuangTai.h"
#include "yingYongPeiZhi.h"
#include "motor.h"

/* ------------------------------------------------------------------------- */
/* 速度档位和 PID 常调参数                                                  */
/* ------------------------------------------------------------------------- */

/* 上电默认速度档位，2 对应 G2。 */
#define SPEED_LEVEL_DEFAULT 2U

/* G1~G6 档位对应的中心速度，单位 RPM。 */
#define SPEED_TABLE_G1_RPM 30.0f
#define SPEED_TABLE_G2_RPM 55.0f
#define SPEED_TABLE_G3_RPM 80.0f
#define SPEED_TABLE_G4_RPM 105.0f
#define SPEED_TABLE_G5_RPM 130.0f
#define SPEED_TABLE_G6_RPM 170.0f

/* 左轮增量式速度环参数。 */
#define SPEED_PID_LEFT_KP 0.25f
#define SPEED_PID_LEFT_KI 0.020f
#define SPEED_PID_LEFT_KD 0.0f
#define SPEED_PID_LEFT_KF 0.0f

/* 右轮增量式速度环参数。 */
#define SPEED_PID_RIGHT_KP 0.25f
#define SPEED_PID_RIGHT_KI 0.020f
#define SPEED_PID_RIGHT_KD 0.0f
#define SPEED_PID_RIGHT_KF 0.0f

#define SPEED_PID_AVERAGE_KP 0.25f
#define SPEED_PID_AVERAGE_KI 0.020f
#define SPEED_PID_AVERAGE_KD 0.0f
#define SPEED_PID_AVERAGE_KF 0.0f

/* 左右速度环公共输出限制、死区和单周期变化限制。 */
#define SPEED_PID_OUT_MAX 100.0f
#define SPEED_PID_OUT_MIN (-100.0f)
#define SPEED_PID_DEADBAND 1.8f
#define SPEED_PID_RATE_LIMIT 35.0f

/* 航向角环常调参数。 */
#define YAW_PID_KP 2.0f
#define YAW_PID_KI 0.02f
#define YAW_PID_KD 0.5f
#define YAW_PID_OUT_MAX 10.0f
#define YAW_PID_OUT_MIN (-10.0f)
#define YAW_PID_INTEGRAL_LIMIT 10.0f
#define YAW_PID_DEADBAND 0.5f
#define YAW_PID_RATE_LIMIT 2.0f

/* 默认进入蓝牙/手动模式，上电后不会自动驱动电机。 */
workMode_t workMode = modeRun;
uint8_t runFlag = 0;
int16_t pwmDuty = 0;
float turnTarget = 0.0f;
float yawTarget = 0.0f;
int16_t bluetoothLv = 0;
int16_t bluetoothRh = 0;
uint32_t bluetoothLastRx = 0;
uint8_t vofaEnabled = 1;

/* 左右轮独立增量速度环、目标量、航向环和循迹遥测对象。 */
PID_t speedPidLeft;
PID_t speedPidRight;
PID_t speedPidAverage;
float targetLeftRpm = 0.0f;
float targetRightRpm = 0.0f;
PID_t yawPid;

Encoder encoderLeft;
Encoder encoderRight;
MPU6050 imu;

/* 档位数组下标为 0~5，对外显示的档位为 1~6。 */
static uint8_t speedLevel = SPEED_LEVEL_DEFAULT;
static uint8_t lapTarget = TRACK_LAP_TARGET_DEFAULT;
static uint8_t completedLaps = 0U;
static const float speedTable[SPEED_LEVEL_COUNT] = {
    SPEED_TABLE_G1_RPM,
    SPEED_TABLE_G2_RPM,
    SPEED_TABLE_G3_RPM,
    SPEED_TABLE_G4_RPM,
    SPEED_TABLE_G5_RPM,
    SPEED_TABLE_G6_RPM
};

static const Parameter wheelParameter = {
    .multiple = ENCODER_MULTIPLE,
    .reduction_ratio = ENCODER_REDUCTION_RATIO,
    .ppr = ENCODER_PPR,
    .r = ENCODER_WHEEL_RADIUS_M
};

/**
 * @brief  初始化一个左右轮增量式速度 PID。
 * @param  pid 待初始化的速度 PID 对象。
 * @param  kp 该轮增量式速度环比例系数。
 * @param  ki 该轮增量式速度环积分系数。
 * @param  kd 该轮增量式速度环微分系数。
 * @param  kf 该轮增量式速度环前馈系数。
 * @note   前馈项在应用层按目标增量补入 Out，用于直接建立克服电机死区所需的
 *         基线 PWM；积分限幅和 OutOffset 仍不使用。
 */
static void appStateInitSpeedPid(PID_t *pid,
                                 float kp,
                                 float ki,
                                 float kd,
                                 float kf)
{
    PID_Init(pid);
    PID_SetParam(pid, kp, ki, kd);
    pid->Kf = kf;
    pid->OutMax = SPEED_PID_OUT_MAX;
    pid->OutMin = SPEED_PID_OUT_MIN;
    pid->OutOffset = 0.0f;
    pid->IntegralLimit = 0.0f;
    pid->Deadband = SPEED_PID_DEADBAND;
    pid->OutputRateLimit = SPEED_PID_RATE_LIMIT;
}

/**
 * @brief  清空一个增量式速度 PID 的目标、输出和误差历史。
 * @param  pid 待复位的速度 PID 对象。
 * @note   只清运行状态，不改 Kp、Ki、Kd，在线调参结果不会被停车命令覆盖。
 */
static void appStateResetSpeedPid(PID_t *pid)
{
    PID_SetTarget(pid, 0.0f);
    PID_ResetRuntime(pid);
}

/**
 * @brief  初始化编码器、左右增量速度环和航向环。
 * @note   编码器机械参数和 PID 默认参数全部来自 yingYongPeiZhi.h。
 */
void appStateInit(void)
{
    /* 轮径、减速比、每圈脉冲数统一来自 yingYongPeiZhi.h。 */
    initEncoder(&encoderLeft, wheelParameter);
    initEncoder(&encoderRight, wheelParameter);

    /* 左、右轮分别建立独立增量速度环，后续可单独在线调参。 */
    appStateInitSpeedPid(&speedPidLeft,
                         SPEED_PID_LEFT_KP,
                         SPEED_PID_LEFT_KI,
                         SPEED_PID_LEFT_KD,
                         SPEED_PID_LEFT_KF);
    appStateInitSpeedPid(&speedPidRight,
                         SPEED_PID_RIGHT_KP,
                         SPEED_PID_RIGHT_KI,
                         SPEED_PID_RIGHT_KD,
                         SPEED_PID_RIGHT_KF);
    appStateInitSpeedPid(&speedPidAverage,
                         SPEED_PID_AVERAGE_KP,
                         SPEED_PID_AVERAGE_KI,
                         SPEED_PID_AVERAGE_KD,
                         SPEED_PID_AVERAGE_KF);

    targetLeftRpm = 0.0f;
    targetRightRpm = 0.0f;

    /* 初始化航向角环。 */
    PID_Init(&yawPid);
    PID_SetParam(&yawPid, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
    PID_SetField(&yawPid, PID_FIELD_KF, 0.0f);
    yawPid.OutMax = YAW_PID_OUT_MAX;
    yawPid.OutMin = YAW_PID_OUT_MIN;
    yawPid.OutOffset = 0.0f;
    yawPid.IntegralLimit = YAW_PID_INTEGRAL_LIMIT;
    yawPid.Deadband = YAW_PID_DEADBAND;
    yawPid.OutputRateLimit = YAW_PID_RATE_LIMIT;
    appStateResetYawPid();

}

/**
 * @brief  复位车辆运动状态并立即关闭电机。
 * @note   清空启停标志、转向量、蓝牙摇杆值以及左右轮和航向环的历史量，
 *         防止停车后重新启动时出现积分残留或输出突变。
 */
void appStateResetMotion(void)
{
    /* 立即清除运行标志、左右目标和两个增量速度环历史。 */
    runFlag = 0;
    turnTarget = 0.0f;
    bluetoothLv = 0;
    bluetoothRh = 0;

    targetLeftRpm = 0.0f;
    targetRightRpm = 0.0f;
    appStateResetSpeedPid(&speedPidLeft);
    appStateResetSpeedPid(&speedPidRight);
    appStateResetSpeedPid(&speedPidAverage);
    appStateResetYawPid();

    /* 同时关闭两路电机输出。 */
    Motor_SetSpeedA(0);
    Motor_SetSpeedB(0);
}

/**
 * @brief  清除航向角环的目标、输出和误差历史。
 */
void appStateResetYawPid(void)
{
    PID_SetTarget(&yawPid, 0.0f);
    PID_ResetRuntime(&yawPid);
}

/**
 * @brief  读取当前速度档位。
 * @return 当前档位编号，范围 1~6。
 */
uint8_t appStateGetSpeedLevel(void)
{
    return speedLevel;
}

/**
 * @brief  设置速度档位。
 * @param  level 目标档位；小于 1 时回到最高档，大于 6 时回到最低档。
 */
void appStateSetSpeedLevel(uint8_t level)
{
    /* 档位按 1~6 循环，越界时回到另一端。 */
    if (level < 1U) level = SPEED_LEVEL_COUNT;
    if (level > SPEED_LEVEL_COUNT) level = 1U;
    speedLevel = level;
}

/**
 * @brief  按方向调整当前速度档位。
 */
void appStateStepSpeedLevel(int8_t direction)
{
    uint8_t level = speedLevel;

    if (workMode == modeTrack) {
        if (level > TRACK_MAX_SPEED_LEVEL) {
            level = TRACK_MAX_SPEED_LEVEL;
        }
        if (direction > 0 && level < TRACK_MAX_SPEED_LEVEL) {
            level++;
        }
        if (direction < 0 && level > 1U) {
            level--;
        }
        speedLevel = level;
        return;
    }

    int16_t next = (int16_t)level + direction;
    if (next > (int16_t)SPEED_LEVEL_COUNT) {
        next = 1;
    }
    if (next < 1) {
        next = (int16_t)SPEED_LEVEL_COUNT;
    }
    speedLevel = (uint8_t)next;
}

/**
 * @brief  获取当前档位对应的中心速度。
 * @return 当前档位速度，单位 RPM。
 */
float appStateGetSpeedRpm(void)
{
    /* 将 1~6 档转换为数组索引 0~5。 */
    return speedTable[speedLevel - 1U];
}

/**
 * @brief  获取循迹目标圈数。
 * @return 当前目标圈数，范围 1~5。
 */
uint8_t appStateGetLapTarget(void)
{
    return lapTarget;
}

/**
 * @brief  设置循迹目标圈数，限制在 1~5。
 * @param  target 目标圈数。
 */
void appStateSetLapTarget(uint8_t target)
{
    if (target < 1U) target = 1U;
    if (target > TRACK_LAP_TARGET_MAX) target = TRACK_LAP_TARGET_MAX;
    lapTarget = target;
}

/**
 * @brief  按 1 -> 2 -> ... -> 5 -> 1 循环目标圈数。
 */
void appStateCycleLapTarget(void)
{
    uint8_t next = (uint8_t)(lapTarget + 1U);

    if (next > TRACK_LAP_TARGET_MAX) {
        next = 1U;
    }
    appStateSetLapTarget(next);
}

/**
 * @brief  获取当前已完成圈数。
 * @return 当前已完成圈数。
 */
uint8_t appStateGetCompletedLaps(void)
{
    return completedLaps;
}

/**
 * @brief  完成一圈，最多记录到目标圈数。
 */
void appStateIncrementCompletedLaps(void)
{
    if (completedLaps < lapTarget) {
        completedLaps++;
    }
}

/**
 * @brief  清零已完成圈数，不清除目标圈数。
 */
void appStateResetLapProgress(void)
{
    completedLaps = 0U;
}
