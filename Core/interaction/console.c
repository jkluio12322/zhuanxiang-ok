/* USART1 命令行：台架手动转速、状态查看，以及仅停车时生效的 PID 整定。
   执行上下文：UART RX 中断只往行缓冲塞字节、收到换行置就绪；主循环解析整行并执行
   （涉及浮点，只能主循环）；主循环把响应交给发送缓冲。 */

#include "console.h"
#include "cmd.h"
#include "config.h"
#include "imu.h"
#include "key.h"
#include "spi.h"
#include "main.h"
#include "motion.h"
#include "position_loop.h"
#include "speed_loop.h"
#include "state.h"
#include "telemetry.h"
#include "testmode.h"
#include "track.h"
#include "serial.h"
#include "eight.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 响应缓冲区大小，要容得下 show 的几行输出。 */
#define CONSOLE_RESPONSE_BUFFER_SIZE 384U
/* 单行命令的最大长度（含结尾 '\0'）。 */
#define CONSOLE_LINE_BUFFER_SIZE     32U
/* 单个 token（命令名或参数）的最大长度。 */
#define CONSOLE_TOKEN_BUFFER_SIZE    16U

/* 命令表中每一项对应的动作。 */
typedef enum {
    CMD_LEFT_RPM,    /* l <rpm>   只设左轮目标转速 */
    CMD_RIGHT_RPM,   /* r <rpm>   只设右轮目标转速 */
    CMD_BOTH_RPM,    /* t <rpm>   左右同时设 */
    CMD_STOP,        /* stop      停车 */
    CMD_SHOW,        /* show      打印当前状态 */
    CMD_QUIET,       /* quiet     暂停 VOFA 输出 */
    CMD_LOG,         /* log       恢复 VOFA 输出 */
    CMD_SIGN_MOTOR,  /* mot       翻转两路电机极性 */
    CMD_SIGN_ENCODER,/* enc       翻转两路编码器极性 */
    CMD_MODE,        /* mode <0|1|2|3|4> 切模式 */
    CMD_GO,          /* go        启动（当前模式） */
    CMD_FWD,         /* fwd       PWM-SPD 正转 */
    CMD_REV,         /* rev       PWM-SPD 反转 */
    CMD_DUTY,        /* duty <pct> PWM-SPD 占空比 */
    CMD_KEYS,        /* keys       打印四个按键的原始电平 */
    CMD_IMUTEST,     /* imutest    软件模拟SPI读WHO_AM_I，区分接线/外设问题 */
    CMD_IMUDUMP,     /* imudump   读回 IMU 配置寄存器与 12 字节原始数据 */
    CMD_IMUCAL,      /* imucal     重新做零偏标定（车必须静止） */
    CMD_IMUDIAG,     /* imudiag   独立 IMU CSV 诊断流 */
    CMD_TRACK_DIAG,  /* trackdiag 独立循迹 CSV 训练流 */
    CMD_GEAR,        /* gear <1..6> 停车时设置档位 */
    CMD_TRACK_KP,    /* tkp <value> 设置循迹 Kp */
    CMD_TRACK_KD,    /* tkd <value> 设置循迹 Kd */
    CMD_TRACK_GYRO,  /* tkg <value> 设置陀螺仪阻尼 */
    CMD_POS_LKP,     /* plkp <value> 设置左轮连续位置 Kp */
    CMD_POS_LKD,     /* plkd <value> 设置左轮连续位置 Kd */
    CMD_POS_RKP,     /* prkp <value> 设置右轮连续位置 Kp */
    CMD_POS_RKD,     /* prkd <value> 设置右轮连续位置 Kd */
    CMD_SPEED_LKP,   /* slkp <value> 设置左速度环 Kp */
    CMD_SPEED_LKI,   /* slki <value> 设置左速度环 Ki */
    CMD_SPEED_LKD,   /* slkd <value> 设置左速度环 Kd */
    CMD_SPEED_RKP,   /* srkp <value> 设置右速度环 Kp */
    CMD_SPEED_RKI,   /* srki <value> 设置右速度环 Ki */
    CMD_SPEED_RKD,   /* srkd <value> 设置右速度环 Kd */
    CMD_LOOP_PROGRAM,/* lp <0..4> LOOP-TEST 程序 */
    CMD_LOOP_OPTION, /* lo <0..2> LOOP-TEST 档位/子测试 */
    CMD_HELP         /* help / ?  命令列表 */
} ConsoleAction_t;

/* 命令表的一行。 */
typedef struct {
    const char     *name;    /* 命令名。 */
    uint8_t         needsArg;/* 1 表示必须跟一个数字参数。 */
    ConsoleAction_t action;  /* 对应 ConsoleAction_t。 */
} ConsoleCommand_t;

/* 命令表。顺序无关，查找是线性扫描。 */
static const ConsoleCommand_t commandTable[] = {
    {"l",     1, CMD_LEFT_RPM},
    {"r",     1, CMD_RIGHT_RPM},
    {"t",     1, CMD_BOTH_RPM},
    {"stop",  0, CMD_STOP},
    {"show",  0, CMD_SHOW},
    {"quiet", 0, CMD_QUIET},
    {"log",   0, CMD_LOG},
    {"mot",   0, CMD_SIGN_MOTOR},
    {"enc",   0, CMD_SIGN_ENCODER},
    {"mode",  1, CMD_MODE},
    {"go",    0, CMD_GO},
    {"fwd",   0, CMD_FWD},
    {"rev",   0, CMD_REV},
    {"duty",  1, CMD_DUTY},
    {"keys",  0, CMD_KEYS},
    {"imutest",0, CMD_IMUTEST},
    {"imudump",0, CMD_IMUDUMP},
    {"imucal",0, CMD_IMUCAL},
    {"imudiag",0, CMD_IMUDIAG},
    {"trackdiag",0, CMD_TRACK_DIAG},
    {"gear",  1, CMD_GEAR},
    {"tkp",   1, CMD_TRACK_KP},
    {"tkd",   1, CMD_TRACK_KD},
    {"tkg",   1, CMD_TRACK_GYRO},
    {"plkp",  1, CMD_POS_LKP},
    {"plkd",  1, CMD_POS_LKD},
    {"prkp",  1, CMD_POS_RKP},
    {"prkd",  1, CMD_POS_RKD},
    {"slkp",  1, CMD_SPEED_LKP},
    {"slki",  1, CMD_SPEED_LKI},
    {"slkd",  1, CMD_SPEED_LKD},
    {"srkp",  1, CMD_SPEED_RKP},
    {"srki",  1, CMD_SPEED_RKI},
    {"srkd",  1, CMD_SPEED_RKD},
    {"lp",    1, CMD_LOOP_PROGRAM},
    {"lo",    1, CMD_LOOP_OPTION},
    {"help",  0, CMD_HELP},
    {"?",     0, CMD_HELP}
};

/* 有响应待发送；主循环写、主循环清。 */
static volatile uint8_t consoleResponsePending = 0;
/* 一整行命令已经收齐；由 UART 中断置位，由主循环在临界区里消费。 */
static volatile uint8_t consoleLineReady = 0;
/* 响应文本缓冲。同一时刻只允许一条响应待发，见 consoleRespond()。 */
static char consoleResponseBuffer[CONSOLE_RESPONSE_BUFFER_SIZE];
/* 行缓冲，由 UART 中断逐字节填充。 */
static char consoleLine[CONSOLE_LINE_BUFFER_SIZE];
/* 行缓冲当前的写入下标；中断写、中断清，主循环不碰。 */
static uint8_t consoleRxIndex = 0;
static uint8_t imuDiagEnabled;
static uint32_t imuDiagLastSendMs;
static uint8_t trackDiagEnabled;
static uint32_t trackDiagLastSendMs;

static void                consoleRespond(const char *format, ...);
static const ConsoleCommand_t *consoleFind(const char *name);
static const char         *consolePhaseName(void);

/* 解析并执行一行命令（主循环调用）：等就绪标志 -> 临界区拷私有副本并消费标志 -> 切两个
   token -> 查表 -> 校验参数 -> 执行。标志检查必须在临界区内；开头消费标志（结尾清会误删
   解析期间新到的一行），拷贝期间关中断。 */
void console_process(void)
{
    char line[CONSOLE_LINE_BUFFER_SIZE];
    char token[CONSOLE_TOKEN_BUFFER_SIZE];
    char argument[CONSOLE_TOKEN_BUFFER_SIZE];
    char *command;
    const ConsoleCommand_t *entry;
    int count;

    /* 标志检查与拷贝必须在同一个临界区里完成，见函数头。 */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        if (!consoleLineReady) {
            __set_PRIMASK(primask);
            return;
        }
        memcpy(line, consoleLine, sizeof(line));
        consoleLineReady = 0;
        __set_PRIMASK(primask);
    }

    token[0] = '\0';
    argument[0] = '\0';
    command = line;
    /* 跳过行首空格，否则 sscanf 会把第一个 token 解析成空。 */
    while (*command == ' ') command++;

    /* 只取前两个 token，多余的参数被忽略。 */
    count = sscanf(command, "%15s %15s", token, argument);
    if (count < 1) {
        return;
    }

    entry = consoleFind(token);
    if (entry == 0) {
        consoleRespond("#? '%s' (type 'help')\r\n", token);
        return;
    }
    if (entry->needsArg && count < 2) {
        consoleRespond("#? %s needs a value\r\n", token);
        return;
    }

    switch (entry->action) {
    /* 台架手动转速 l/r/t 只在 PWM 速度测试模式有意义，其它模式轮速目标另有来源，
       改了下一周期就被覆盖，所以在这里就拦住而非入队后静默失效。 */
    case CMD_LEFT_RPM:
    case CMD_RIGHT_RPM:
    case CMD_BOTH_RPM: {
        char *end = 0;
        float parsed;

        if (state_view()->mode != modePwmSpeed) {
            consoleRespond("#[MANUAL] only in PWM-SPEED mode (KEY1)\r\n");
            break;
        }

        /* 严格解析：整串必须就是一个数。strtof 对 "abc" 也返回 0 不报错，
           不校验会把一个手误当成 "l 0" 送进队列、把车凭空开起来。 */
        parsed = strtof(argument, &end);
        if ((end == argument) || (*end != '\0')) {
            consoleRespond("#? %s needs a number, got '%s'\r\n", token, argument);
            break;
        }

        /* 范围校验顺带挡 NaN 和 ±inf（NaN 与任何值比较都为假），下游 clamp 仍兜底。 */
        if (!((parsed >= -SPEED_TARGET_LIMIT_RPM)
              && (parsed <= SPEED_TARGET_LIMIT_RPM))) {
            consoleRespond("#? %s range +-%d rpm\r\n", token,
                           (int)SPEED_TARGET_LIMIT_RPM);
            break;
        }

        if (cmd_push_value(
                (entry->action == CMD_LEFT_RPM)  ? commandSetLeftRpm
                : (entry->action == CMD_RIGHT_RPM) ? commandSetRightRpm
                : commandSetBothRpm,
                parsed) == 0U) {
            consoleRespond("[MANUAL] queue full, dropped\r\n");
            break;
        }

        /* 回显送出的值，让操作者分清"生效了"和"被丢了"（被模式门挡下时有回显）。 */
        consoleRespond("#[MANUAL] %s %.1f rpm\r\n", token, parsed);
        break;
    }

    case CMD_STOP:
        /* 走命令队列，由控制任务统一执行停车与复位。 */
        (void)cmd_push_value(commandStop, 0.0f);
        break;

    case CMD_SHOW: {
        /* 一次性把测试模式、两组速度环、循迹环、目标和实测都打出来。 */
        SpeedPidInfo_t infoL;
        SpeedPidInfo_t infoR;
        float kp = track_get_gain(PID_FIELD_KP);
        float kd = track_get_gain(PID_FIELD_KD);
        speed_loop_GetInfo(SPEED_CH_LEFT, &infoL);
        speed_loop_GetInfo(SPEED_CH_RIGHT, &infoR);
        consoleRespond(
            "#Mode=%s Run=%d Phase=%s Gear=%d Base=%d%% Lap=%u/%u\r\n"
            "#Tgt L=%.1f R=%.1f  Act L=%.1f R=%.1f\r\n"
            "#SpeedL Kp=%.2f Ki=%.2f Kd=%.2f\r\n"
            "#SpeedR Kp=%.2f Ki=%.2f Kd=%.2f\r\n"
            "#Track Kp=%.2f Kd=%.2f Gyro=%.4f steer=%.1f err=%.2f lostY=%.1f pat=0x%02X st=%u\r\n"
            "#VoltL=%.2fV VoltR=%.2fV\r\n"
            "#MAG ok=%d ready=%d id=0x%02X raw=%d/%d/%d (unused)\r\n"
            "#IMU ok=%d cal=%u who=0x%02X err=%u yaw=%.1f gz=%.2f ax=%.2f ay=%.2f az=%.2f bz=%+.2f\r\n",
            mode_name(state_view()->mode), state_view()->running,
            consolePhaseName(),
            state_view()->gear, (int)testmode_get_base_pwm(),
            (unsigned int)state_view()->laps_done,
            (unsigned int)state_view()->lap_target,
            /* Tgt 取速度环里存的目标（夹到 ±SPEED_TARGET_LIMIT_RPM 后的实际值）；
               state_view() 里是原始请求，直接打会显示一个车永远不会执行的转速。 */
            infoL.target, infoR.target,
            telemetry_data.actualLeftRpm, telemetry_data.actualRightRpm,
            infoL.kp, infoL.ki, infoL.kd,
            infoR.kp, infoR.ki, infoR.kd,
            kp, kd, track_get_gyro_gain(), track_get_steer(),
            (double)track_output()->line_error,
            (double)track_output()->lost_yaw_deg,
            (unsigned int)track_output()->pattern,
            (unsigned int)track_output()->state,
            telemetry_data.commandVoltageLeft,
            telemetry_data.commandVoltageRight,
            (int)imu_get()->mag_ok, (int)imu_get()->mag_ready,
            (unsigned int)imu_get()->mag_id,
            (int)imu_get()->mag_raw[0], (int)imu_get()->mag_raw[1],
            (int)imu_get()->mag_raw[2],
            (int)imu_get()->ok, (unsigned int)imu_get()->cal_status,
            (unsigned int)imu_whoami(),
            (unsigned int)imu_err_count(), (double)imu_get()->yaw,
            (double)imu_get()->gyro_dps[2], (double)imu_get()->acc_g[0],
            (double)imu_get()->acc_g[1], (double)imu_get()->acc_g[2],
            (double)imu_get()->gyro_bias[2]);
        break;
    }

    case CMD_QUIET:
        (void)cmd_push_value(commandQuiet, 0.0f);
        break;

    case CMD_LOG:
        imuDiagEnabled = 0U;
        trackDiagEnabled = 0U;
        (void)cmd_push_value(commandLog, 0.0f);
        break;

    case CMD_SIGN_MOTOR:
    case CMD_SIGN_ENCODER: {
        int8_t mL, mR, eL, eR;
        if (entry->action == CMD_SIGN_MOTOR) {
            motion_flip_motors();
        } else {
            motion_flip_encoders();
        }
        /* 极性一变，旧积分就是错方向的历史包袱，清掉重来。 */
        speed_loop_ResetRuntime();
        motion_get_signs(&mL, &mR, &eL, &eR);
        consoleRespond("#[SIGN] mot L=%+d R=%+d  enc L=%+d R=%+d\r\n",
                        (int)mL, (int)mR, (int)eL, (int)eR);
        break;
    }

    case CMD_MODE: {
        char *end = 0;
        long modeIndex = strtol(argument, &end, 10);
        if ((end == argument) || (*end != '\0') || (modeIndex < 0) || (modeIndex > 4)) {
            consoleRespond("#? mode 0|1|2|3|4\r\n");
            break;
        }
        (void)cmd_push_value((modeIndex == 0) ? commandModeTrack
                             : (modeIndex == 1) ? commandModePwmSpeed
                             : (modeIndex == 2) ? commandModePwmTrack
                             : (modeIndex == 3) ? commandModeLoopTest
                             : commandModeEight, 0.0f);
        consoleRespond("#[MODE] -> %s\r\n", mode_name((Mode)modeIndex));
        break;
    }

    case CMD_GO:
        (void)cmd_push_value(commandStart, 0.0f);
        consoleRespond("#[GO] %s\r\n", mode_name(state_view()->mode));
        break;

    case CMD_FWD:
    case CMD_REV:
        (void)cmd_push_value((entry->action == CMD_FWD) ? commandPwmSpeedForward
                                                        : commandPwmSpeedReverse, 0.0f);
        consoleRespond("#[OPEN] %s dir=%s\r\n", mode_name(state_view()->mode),
                       (entry->action == CMD_FWD) ? "FWD" : "REV");
        break;

    case CMD_DUTY: {
        char *end = 0;
        float parsed = strtof(argument, &end);
        if ((end == argument) || (*end != '\0') || (parsed < -50.0f) || (parsed > 50.0f)) {
            consoleRespond("#? duty -50..50 (step %)\r\n");
            break;
        }
        (void)cmd_push_value(commandPwmSpeedDutyStep, parsed);
        consoleRespond("#[DUTY] step %+.0f%%\r\n", (double)parsed);
        break;
    }

    case CMD_KEYS:
        /* 原始电平：松开=1（上拉），按下=0。用于确认按键接线与引脚是否对上。
           num 是「此刻正被按住」的键号 1~4，0 = 没有键按下（即时读取，不等待）。 */
        consoleRespond("#[KEYS] K1(pg2)=%d K2(pg3)=%d K3(pd14)=%d K4(pd15)=%d num=%u\r\n",
                       (int)HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin),
                       (int)HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin),
                       (int)HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin),
                       (int)HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin),
                       (unsigned int)key_get_num());
        break;

    case CMD_IMUTEST: {
        /* 硬件自检：先用 SPI 外设读一次 WHO_AM_I，再把 PB12/13/14/15 拆成普通 GPIO
           用软件时序读一次。两次都 0x00 说明模块没在 MISO 上应答（接线/供电/片选）；
           只有硬件 SPI 是 0x00 说明外设配置问题。测完 MX_SPI2_Init() 恢复引脚。 */
        uint8_t hwValue = 0U;
        uint8_t bbValue = 0U;
        uint8_t misoIdle;
        GPIO_InitTypeDef gpio = {0};
        volatile uint32_t d;

        hwValue = imu_whoami();   /* 公开接口，内部就是读 0x0F */

        /* PB12=CS、PB13=SCK、PB15=MOSI 推挽输出；PB14=MISO 上拉输入（读空闲电平）。 */
        gpio.Mode = GPIO_MODE_OUTPUT_PP;   gpio.Pull = GPIO_NOPULL;   gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
        HAL_GPIO_Init(GPIOB, &gpio);
        gpio.Mode = GPIO_MODE_INPUT;       gpio.Pull = GPIO_PULLUP;   gpio.Pin = GPIO_PIN_14;
        HAL_GPIO_Init(GPIOB, &gpio);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        misoIdle = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) ? 1U : 0U;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   /* CS 选中 */
        for (d = 0U; d < 50U; d++) { }                            /* tCSS */

        {
            uint8_t out[2];
            uint8_t in[2];
            uint8_t k;
            out[0] = 0x8FU;   /* 读命令：0x0F | 0x80 */
            out[1] = 0x00U;
            for (k = 0U; k < 2U; k++) {
                uint8_t bit;
                uint8_t value = 0U;
                for (bit = 0U; bit < 8U; bit++) {
                    if ((out[k] & (uint8_t)(0x80U >> bit)) != 0U) {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
                    } else {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
                    }
                    for (d = 0U; d < 20U; d++) { }
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   /* 上升沿采样 */
                    for (d = 0U; d < 20U; d++) { }
                    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) {
                        value |= (uint8_t)(0x80U >> bit);
                    }
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                in[k] = value;
            }
            bbValue = in[1];
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        MX_SPI2_Init();   /* 恢复 SPI2 外设配置。
                             注意：HAL_SPI_Init() 对已经初始化过的句柄不会再次调用
                             HAL_SPI_MspInit()，所以手动 GPIO 测试后必须显式恢复
                             PB13/PB14/PB15 的 SPI2 复用，否则后续硬件 SPI 会失效。 */
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Pin = GPIO_PIN_13 | GPIO_PIN_15;
        HAL_GPIO_Init(GPIOB, &gpio);
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        gpio.Pin = GPIO_PIN_14;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        imu_init();       /* 重新走一遍初始化/标定 */

        consoleRespond("#[IMUTEST] spi_who=0x%02X bitbang_who=0x%02X misoIdle=%u (expect 0x6B,1)\r\n",
                       (unsigned int)hwValue, (unsigned int)bbValue, (unsigned int)misoIdle);
        break;
    }

    case CMD_IMUCAL:
        if ((state_view()->running != 0U)
            || (fabsf(motion_view()->left_pwm) > 0.01f)
            || (fabsf(motion_view()->right_pwm) > 0.01f)
            || (fabsf(motion_view()->left_rpm_raw) > IMU_CAL_ENCODER_RPM_MAX)
            || (fabsf(motion_view()->right_rpm_raw) > IMU_CAL_ENCODER_RPM_MAX)) {
            consoleRespond("#[IMUCAL] rejected: stop the car first\r\n");
        } else if (imu_recalibrate() != 0U) {
            consoleRespond("#[IMUCAL] settling 2 s, then collecting 200 still samples\r\n");
        } else {
            consoleRespond("#[IMUCAL] rejected: sensor is not ready for recalibration\r\n");
        }
        break;

    case CMD_IMUDIAG:
        if (imuDiagEnabled != 0U) {
            imuDiagEnabled = 0U;
            consoleRespond("#[IMUDIAG] stopped\r\n");
        } else if (state_view()->vofa != 0U) {
            consoleRespond("#[IMUDIAG] rejected: send quiet first\r\n");
        } else if (state_view()->running != 0U) {
            consoleRespond("#[IMUDIAG] rejected: stop the car first\r\n");
        } else {
            imuDiagEnabled = 1U;
            trackDiagEnabled = 0U;
            imuDiagLastSendMs = 0U;
            serial_send_string("#IMU_DIAG:ms,dt,rawX,rawY,rawZ,corrX,corrY,corrZ,biasX,biasY,biasZ,yaw,cal\r\n");
            consoleRespond("#[IMUDIAG] started; 10 Hz CSV, `imudiag` stops\r\n");
        }
        break;

    case CMD_TRACK_DIAG:
        if (trackDiagEnabled != 0U) {
            trackDiagEnabled = 0U;
            consoleRespond("#[TRACKDIAG] stopped\r\n");
        } else if (state_view()->vofa != 0U) {
            consoleRespond("#[TRACKDIAG] rejected: send quiet first\r\n");
        } else if (state_view()->running != 0U) {
            consoleRespond("#[TRACKDIAG] rejected: stop before starting capture\r\n");
        } else {
            trackDiagEnabled = 1U;
            imuDiagEnabled = 0U;
            trackDiagLastSendMs = 0U;
            serial_send_string(
                "#TRACK_DIAG:ms,run,lap,gear,pat,st,err,u,steer,yawRate,"
                "base,ffL,ffR,refL,actL,posErrL,tgtL,actRpmL,"
                "refR,actR,posErrR,tgtR,actRpmR,pwmL,pwmR,lostY,yaw,kp,kd,gyroKp\r\n");
            consoleRespond("#[TRACKDIAG] started; 50 Hz CSV\r\n");
        }
        break;

    case CMD_GEAR: {
        char *end = 0;
        long value = strtol(argument, &end, 10);
        if ((end == argument) || (*end != '\0')
            || (value < 1) || (value > (long)SPEED_LEVEL_COUNT)) {
            consoleRespond("#? gear 1..%u\r\n", (unsigned int)SPEED_LEVEL_COUNT);
        } else if (state_view()->running != 0U) {
            consoleRespond("#[GEAR] rejected: stop before changing gear\r\n");
        } else {
            (void)state_set_gear((uint8_t)value);
            consoleRespond("#[GEAR] %ld  %.1f rpm\r\n",
                           value, (double)state_view()->gear_rpm);
        }
        break;
    }

    case CMD_TRACK_KP:
    case CMD_TRACK_KD: {
        char *end = 0;
        float value = strtof(argument, &end);
        PidField_t field = (entry->action == CMD_TRACK_KP)
                           ? PID_FIELD_KP : PID_FIELD_KD;
        float max_value = (field == PID_FIELD_KP) ? 10.0f
                        : 3.0f;

        if ((end == argument) || (*end != '\0')
            || !((value >= 0.0f) && (value <= max_value))) {
            consoleRespond("#? %s range 0..%.2f\r\n", token, (double)max_value);
        } else if (state_view()->running != 0U) {
            consoleRespond("#[TRACK] rejected: stop before changing gain\r\n");
        } else if (track_set_gain(field, value) == 0U) {
            consoleRespond("#[TRACK] gain update failed\r\n");
        } else {
            consoleRespond("#[TRACK] %s=%.3f\r\n", token, (double)value);
        }
        break;
    }

    case CMD_TRACK_GYRO: {
        char *end = 0;
        float value = strtof(argument, &end);

        if ((end == argument) || (*end != '\0')
            || !((value >= 0.0f) && (value <= 0.02f))) {
            consoleRespond("#? tkg range 0..0.0200\r\n");
        } else if (state_view()->running != 0U) {
            consoleRespond("#[TRACK] rejected: stop before changing gain\r\n");
        } else if (track_set_gyro_gain(value) == 0U) {
            consoleRespond("#[TRACK] gyro gain update failed\r\n");
        } else {
            consoleRespond("#[TRACK] tkg=%.4f\r\n", (double)value);
        }
        break;
    }

    case CMD_POS_LKP:
    case CMD_POS_LKD:
    case CMD_POS_RKP:
    case CMD_POS_RKD: {
        char *end = 0;
        float value = strtof(argument, &end);
        PositionSide_t side =
            ((entry->action == CMD_POS_LKP) || (entry->action == CMD_POS_LKD))
                ? POSITION_LEFT : POSITION_RIGHT;
        uint8_t is_kp =
            (uint8_t)((entry->action == CMD_POS_LKP)
                      || (entry->action == CMD_POS_RKP));
        float kp = position_loop_get_track_kp(side);
        float kd = position_loop_get_track_kd(side);

        if ((end == argument) || (*end != '\0')
            || !((value >= 0.0f) && (value <= 10.0f))) {
            consoleRespond("#? %s range 0..10.0\r\n", token);
        } else if (state_view()->running != 0U) {
            consoleRespond("#[POS] rejected: stop before changing gain\r\n");
        } else {
            if (is_kp != 0U) kp = value;
            else kd = value;
            if (position_loop_set_track_gains(side, kp, kd) == 0U) {
                consoleRespond("#[POS] gain update failed\r\n");
            } else {
                consoleRespond("#[POS] %s=%.3f\r\n", token, (double)value);
            }
        }
        break;
    }

    case CMD_SPEED_LKP:
    case CMD_SPEED_LKI:
    case CMD_SPEED_LKD:
    case CMD_SPEED_RKP:
    case CMD_SPEED_RKI:
    case CMD_SPEED_RKD: {
        char *end = 0;
        float value = strtof(argument, &end);
        SpeedChannel_t channel =
            ((entry->action == CMD_SPEED_LKP)
             || (entry->action == CMD_SPEED_LKI)
             || (entry->action == CMD_SPEED_LKD)) ? SPEED_CH_LEFT : SPEED_CH_RIGHT;
        PidField_t field =
            ((entry->action == CMD_SPEED_LKP) || (entry->action == CMD_SPEED_RKP))
                ? PID_FIELD_KP
            : ((entry->action == CMD_SPEED_LKI) || (entry->action == CMD_SPEED_RKI))
                ? PID_FIELD_KI : PID_FIELD_KD;
        float max_value = (field == PID_FIELD_KP) ? 2.0f
                        : (field == PID_FIELD_KI) ? 1.0f : 0.5f;

        if ((end == argument) || (*end != '\0')
            || !((value >= 0.0f) && (value <= max_value))) {
            consoleRespond("#? %s range 0..%.2f\r\n", token, (double)max_value);
        } else if (state_view()->running != 0U) {
            consoleRespond("#[SPEED] rejected: stop before changing gain\r\n");
        } else if (speed_loop_SetGain(channel, field, value) == 0U) {
            consoleRespond("#[SPEED] gain update failed\r\n");
        } else {
            consoleRespond("#[SPEED] %s=%.3f\r\n", token, (double)value);
        }
        break;
    }

    case CMD_LOOP_PROGRAM: {
        char *end = 0;
        long value = strtol(argument, &end, 10);
        if ((end == argument) || (*end != '\0') || (value < 0) || (value > 4)) {
            consoleRespond("#? lp 0|1|2|3|4\r\n");
            break;
        }
        (void)cmd_push_value(commandLoopTestSetProgram, (float)value);
        consoleRespond("#[LOOP] program %ld\r\n", value);
        break;
    }

    case CMD_LOOP_OPTION: {
        char *end = 0;
        long value = strtol(argument, &end, 10);
        if ((end == argument) || (*end != '\0') || (value < 0) || (value > 2)) {
            consoleRespond("#? lo 0|1|2\r\n");
            break;
        }
        (void)cmd_push_value(commandLoopTestSetOption, (float)value);
        consoleRespond("#[LOOP] option %ld\r\n", value);
        break;
    }


    case CMD_IMUDUMP: {
        uint8_t sgl[12];
        uint8_t bst[12];
        uint8_t i;
        uint8_t bad = 0U;

        for (i = 0U; i < 12U; i++) { sgl[i] = 0U; bst[i] = 0U; }
        for (i = 0U; i < 12U; i++) {
            if (imu_dbg_reg((uint8_t)(0x22U + i), &sgl[i]) == 0U) { bad = (uint8_t)(bad + 1U); }
        }
        (void)imu_dbg_burst(0x22U, bst, 12u);
        consoleRespond("#[IMU] bad=%u single:\r\n"
                       "#  %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n"
                       "#  burst:\r\n"
                       "#  %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                       (unsigned int)bad,
                       (unsigned int)sgl[0], (unsigned int)sgl[1], (unsigned int)sgl[2], (unsigned int)sgl[3],
                       (unsigned int)sgl[4], (unsigned int)sgl[5], (unsigned int)sgl[6], (unsigned int)sgl[7],
                       (unsigned int)sgl[8], (unsigned int)sgl[9], (unsigned int)sgl[10], (unsigned int)sgl[11],
                       (unsigned int)bst[0], (unsigned int)bst[1], (unsigned int)bst[2], (unsigned int)bst[3],
                       (unsigned int)bst[4], (unsigned int)bst[5], (unsigned int)bst[6], (unsigned int)bst[7],
                       (unsigned int)bst[8], (unsigned int)bst[9], (unsigned int)bst[10], (unsigned int)bst[11]);
        break;
    }

    case CMD_HELP:
        consoleRespond(
            "#l <rpm>  left wheel target (neg = reverse)\r\n"
            "#r <rpm>  right wheel target\r\n"
            "#t <rpm>  both wheels\r\n"
            "#stop     stop the car\r\n"
            "#show     print state / IMU\r\n"
            "#quiet    pause VOFA   log  resume VOFA\r\n"
            "#mot      flip both motor signs\r\n"
            "#enc      flip both encoder signs\r\n"
            "#mode <0..4> 0=TRACK 1=PWM-SPD 2=PWM-TRK 3=LOOP 4=EIGHT\r\n"
            "#lp <0..4> LOOP program  lo <0..2> option\r\n"
            "#go       start current mode\r\n"
            "#fwd      PWM-SPD open-loop forward\r\n"
            "#rev      PWM-SPD open-loop reverse\r\n"
            "#duty <%> PWM-SPD duty step (+-50)\r\n"
            "#keys     raw K1..K4 levels\r\n"
            "#imutest  SPI vs bit-bang WHO_AM_I\r\n"
            "#imucal   stationary re-calibration (2 s settle + up to 6 s collect)\r\n"
            "#imudiag  toggle 10 Hz IMU CSV; requires quiet and stopped\r\n"
            "#trackdiag toggle 20 Hz tracking CSV; requires quiet and stopped\r\n"
            "#gear <1..6> set stopped tracking gear\r\n"
            "#tkp/tkd/tkg <v> set steering gains while stopped\r\n"
            "#plkp/plkd/prkp/prkd <v> set continuous position gains\r\n"
            "#slkp/slki/slkd <v> set left speed PI gains; srkp/... right\r\n"
            "#Manual rpm needs PWM-SPEED mode (cycle with KEY1).\r\n");
        break;

    default:
        break;
    }
}

/* 格式化一条待发送响应。已有响应待发时本次调用被直接丢弃（不排队不覆盖）。 */
static void consoleRespond(const char *format, ...)
{
    va_list arguments;
    if (consoleResponsePending) return;
    va_start(arguments, format);
    vsnprintf(consoleResponseBuffer, sizeof(consoleResponseBuffer),
              format, arguments);
    va_end(arguments);
    consoleResponsePending = 1;
}

/* 在命令表里按名字查一项。命中返回表项指针，未命中返回 0（线性扫描，表很小）。 */
static const ConsoleCommand_t *consoleFind(const char *name)
{
    uint8_t i;
    for (i = 0; i < sizeof(commandTable) / sizeof(commandTable[0]); i++) {
        if (strcmp(commandTable[i].name, name) == 0) return &commandTable[i];
    }
    return 0;
}

/* 取 show 里 Phase 字段要显示的文本：测试模式下是当前相位名，循迹模式返回 "-"。
   相位名统一取自 app_testmode.c 的 phase_name()。PWM 循迹下表示注入图案是否被左右
   镜像（LEFT 原始 / RIGHT 镜像），见 testmode_get_pattern()。 */
static const char *consolePhaseName(void)
{
    if (state_view()->mode == modePwmSpeed) {
        return phase_name(testmode_get_speed_phase());
    }
    if (state_view()->mode == modePwmTrack) {
        return phase_name(testmode_get_track_phase());
    }
    if (state_view()->mode == modeEight) {
        return eight_phase_name(eight_view()->phase);
    }
    return "-";
}

/* 初始化命令行模块状态：只清软件状态，UART 接收中断由 app_serial.c 的 serial_init() 启动。 */
void console_init(void)
{
    consoleRxIndex = 0;
    consoleResponsePending = 0;
    consoleLineReady = 0;
    imuDiagEnabled = 0U;
    imuDiagLastSendMs = 0U;
    trackDiagEnabled = 0U;
    trackDiagLastSendMs = 0U;
}

/* 缓存一个收到的字节，收到换行则标记整行就绪（中断上下文，只搬运不做解析/浮点/printf）。
   超长行截断（多余字节丢弃），不会越界。 */
void console_rx_byte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        /* 空行（首字符就是换行）不产生命令。 */
        if (consoleRxIndex > 0) {
            consoleLine[consoleRxIndex] = '\0';
            consoleLineReady = 1;
            consoleRxIndex = 0;
        }
    } else if (consoleRxIndex < sizeof(consoleLine) - 1U) {
        /* 预留最后一个字节给结尾 '\0'。 */
        consoleLine[consoleRxIndex++] = byte;
    }
}

/* 把已格式化的响应交给发送缓冲（主循环调用，须在 console_process() 之后）。
   无需临界区：唯一写者 consoleRespond() 也只在主循环，两者不并发；发送环形缓冲的
   保护在 serial_write() 内部。 */
void console_flush(void)
{
    if (!consoleResponsePending) return;
    serial_send_string(consoleResponseBuffer);
    consoleResponsePending = 0;
}

void console_diag_task(void)
{
    char buffer[320];
    int length;
    uint32_t now;

    if ((imuDiagEnabled == 0U) && (trackDiagEnabled == 0U)) return;

    now = HAL_GetTick();
    if (state_view()->vofa != 0U) {
        imuDiagEnabled = 0U;
        trackDiagEnabled = 0U;
        return;
    }

    if (trackDiagEnabled != 0U) {
        TrackOutput_t track_snapshot;
        MotionView_t motion_snapshot;
        PositionLoopView_t position_snapshot;
        uint32_t primask;
        const TrackOutput_t *track = &track_snapshot;
        const MotionView_t *motion = &motion_snapshot;
        const PositionLoopView_t *position = &position_snapshot;

        if ((uint32_t)(now - trackDiagLastSendMs) < 20U) return;
        trackDiagLastSendMs = now;
        primask = __get_PRIMASK();
        __disable_irq();
        track_snapshot = *track_output();
        motion_snapshot = *motion_view();
        position_snapshot = *position_loop_view();
        __set_PRIMASK(primask);
        length = snprintf(
            buffer, sizeof(buffer),
            "%lu,%u,%u,%u,%u,%u,%.3f,%.3f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.2f,"
            "%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.4f\r\n",
            (unsigned long)now,
            (unsigned int)state_view()->running,
            (unsigned int)state_view()->laps_done,
            (unsigned int)state_view()->gear,
            (unsigned int)track->pattern,
            (unsigned int)track->state,
            (double)track->line_error,
            (double)track->steer_ratio,
            (double)track->steer_rpm,
            (double)track->yaw_rate_dps,
            (double)track->base_rpm,
            (double)position->axis[POSITION_LEFT].velocity_ff_rpm,
            (double)position->axis[POSITION_RIGHT].velocity_ff_rpm,
            (double)position->axis[POSITION_LEFT].reference_deg,
            (double)motion->left_angle_deg,
            (double)position->axis[POSITION_LEFT].error_deg,
            (double)state_view()->target_left_rpm,
            (double)motion->left_rpm,
            (double)position->axis[POSITION_RIGHT].reference_deg,
            (double)motion->right_angle_deg,
            (double)position->axis[POSITION_RIGHT].error_deg,
            (double)state_view()->target_right_rpm,
            (double)motion->right_rpm,
            (double)motion->left_pwm,
            (double)motion->right_pwm,
            (double)track->lost_yaw_deg,
            (double)imu_get()->yaw,
            (double)track_get_gain(PID_FIELD_KP),
            (double)track_get_gain(PID_FIELD_KD),
            (double)track_get_gyro_gain());
        if ((length > 0) && (length < (int)sizeof(buffer))) {
            (void)serial_write((const uint8_t *)buffer, (uint16_t)length);
        }
        return;
    }

    if (state_view()->running != 0U) return;
    if ((uint32_t)(now - imuDiagLastSendMs) < 100U) return;
    imuDiagLastSendMs = now;
    {
        const imu_data_t *imu = imu_get();
        length = snprintf(
            buffer, sizeof(buffer),
            "%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,%.3f,%u\r\n",
            (unsigned long)imu->sample_ms,
            (unsigned long)imu->sample_dt_ms,
            (double)imu->gyro_raw_dps[0], (double)imu->gyro_raw_dps[1],
            (double)imu->gyro_raw_dps[2], (double)imu->gyro_dps[0],
            (double)imu->gyro_dps[1], (double)imu->gyro_dps[2],
            (double)imu->gyro_bias[0], (double)imu->gyro_bias[1],
            (double)imu->gyro_bias[2], (double)imu->yaw,
            (unsigned int)imu->cal_status);
        if ((length > 0) && (length < (int)sizeof(buffer))) {
            (void)serial_write((const uint8_t *)buffer, (uint16_t)length);
        }
    }
}
