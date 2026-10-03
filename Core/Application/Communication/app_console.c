/**
 * @file    app_console.c
 * @brief   USART1 在线命令行 — PID 调参、测试模式控制和状态查询。
 *
 * @note    执行上下文的划分（本模块同时横跨中断与主循环）：
 * @code
 *   UART RX 中断  -> AppConsole_RxByte()   只往行缓冲塞字节，收到换行置就绪标志
 *   主循环        -> AppConsole_Process()  解析整行、执行命令（注意：可能涉及浮点）
 *   主循环        -> AppConsole_Flush()    把准备好的响应交给发送环形缓冲
 * @endcode
 *
 * @note    命令分两类，分开走不同的路径：
 *          - **直接执行**：改 PID 增益、改测试模式参数，立即生效、不进队列。
 *            只在本次上电有效，重新上电恢复 app_config.h 的默认值。
 *          - **写入命令队列**：start/stop/spdgear/simlevel/base/sim/quiet/log，
 *            交给 50 ms 控制任务统一执行。
 *
 * @note    行缓冲的竞争已处理：AppConsole_RxByte() 在中断里，收到换行后会把
 *          tuneRxIndex 清零，下一个字节就开始覆盖 tuneLine[0]。
 *          AppConsole_Process() 因此在解析前先在临界区里把整行拷成私有副本，
 *          解析过程只操作副本，中断可以自由开始写下一行。
 *
 * @warning 命令解析涉及浮点（strtof、vsnprintf 里的 %f），只能放在主循环。
 *          绝不可为了降低延迟把 AppConsole_Process() 挪进中断。
 *
 * @see     app_serial.c   接收中断与发送环形缓冲的实现
 * @see     app_cmd.c      命令队列
 */

#include "app_console.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_state.h"
#include "app_tracking.h"
#include "app_speedloop.h"
#include "app_testmode.h"
#include "app_telemetry.h"
#include "app_pid.h"
#include "app_serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 响应缓冲区大小，要容得下 show 命令的五行输出。 */
#define PID_TUNE_RESPONSE_BUFFER_SIZE 384U
/* 单行命令的最大长度（含结尾 '\0'）。 */
#define PID_TUNE_LINE_BUFFER_SIZE     32U
/* 单个 token（命令名或参数）的最大长度。 */
#define PID_TUNE_TOKEN_BUFFER_SIZE    16U

/**
 * @brief  命令表中每一项对应的动作。
 * @note   命名沿用历史前缀 TUNE_；SPDGEAR 等取值与命令字符串是分开的，
 *         加新命令时要同时改 tuneTable 和这个枚举。
 */
typedef enum {
    TUNE_SKP,
    TUNE_SKI,
    TUNE_SKD,
    TUNE_LKP,
    TUNE_LKI,
    TUNE_LKD,
    TUNE_RKP,
    TUNE_RKI,
    TUNE_RKD,
    TUNE_TKP,
    TUNE_TKI,
    TUNE_TKD,
    TUNE_TKG,
    TUNE_START,
    TUNE_STOP,
    TUNE_SPDGEAR,
    TUNE_SIMLEVEL,
    TUNE_BASE,
    TUNE_SIM,
    TUNE_QUIET,
    TUNE_LOG,
    TUNE_SHOW,
    TUNE_HELP
} TuneAction_t;

/**
 * @brief  命令表的一行。
 */
typedef struct {
    const char *name;    /**< 命令名，如 "skp"。 */
    uint8_t needsArg;    /**< 1 表示必须跟一个数字参数。 */
    TuneAction_t action; /**< 对应 TuneAction_t。 */
} TuneCommand_t;

/* 命令表。顺序无关，查找是线性扫描。 */
static const TuneCommand_t tuneTable[] = {
    {"skp", 1, TUNE_SKP},
    {"ski", 1, TUNE_SKI},
    {"skd", 1, TUNE_SKD},
    {"lkp", 1, TUNE_LKP},
    {"lki", 1, TUNE_LKI},
    {"lkd", 1, TUNE_LKD},
    {"rkp", 1, TUNE_RKP},
    {"rki", 1, TUNE_RKI},
    {"rkd", 1, TUNE_RKD},
    {"tkp", 1, TUNE_TKP},
    {"tki", 1, TUNE_TKI},
    {"tkd", 1, TUNE_TKD},
    {"tkg", 1, TUNE_TKG},
    {"start", 0, TUNE_START},
    {"stop", 0, TUNE_STOP},
    {"spdgear", 1, TUNE_SPDGEAR},
    {"simlevel", 1, TUNE_SIMLEVEL},
    {"base", 1, TUNE_BASE},
    {"sim", 1, TUNE_SIM},
    {"quiet", 0, TUNE_QUIET},
    {"log", 0, TUNE_LOG},
    {"show", 0, TUNE_SHOW},
    {"help", 0, TUNE_HELP},
    {"?", 0, TUNE_HELP}
};

/* 有响应待发送；主循环写、主循环清，但被中断路径间接影响，故 volatile。 */
static volatile uint8_t tuneResponsePending = 0;
/* 一整行命令已经收齐；由 UART 中断置位，由主循环消费后清除。 */
static volatile uint8_t tuneLineReady = 0;
/* 响应文本缓冲。同一时刻只允许一条响应待发，见 consoleRespond()。 */
static char tuneResponseBuffer[PID_TUNE_RESPONSE_BUFFER_SIZE];
/* 行缓冲，由 UART 中断逐字节填充。 */
static char tuneLine[PID_TUNE_LINE_BUFFER_SIZE];
/* 行缓冲当前的写入下标；中断写、中断清，主循环不碰。 */
static uint8_t tuneRxIndex = 0;

/**
 * @brief  格式化一条待发送的响应。
 * @param  format printf 风格格式串。
 * @param  ...    格式参数。
 * @note   若已有响应待发，本次调用被**直接丢弃**（不排队、不覆盖）。
 *         这样设计是因为调用方是命令处理逻辑，宁可少一条回显，
 *         也不要让后一条响应顶掉前一条造成输出错乱。
 */
static void consoleRespond(const char *format, ...)
{
    va_list arguments;
    if (tuneResponsePending) return;
    va_start(arguments, format);
    vsnprintf(tuneResponseBuffer, sizeof(tuneResponseBuffer), format, arguments);
    va_end(arguments);
    tuneResponsePending = 1;
}

/**
 * @brief  初始化命令行模块的状态。
 * @note   只清软件状态；UART 接收中断由 app_serial.c 的 Serial_Init() 启动。
 */
void AppConsole_Init(void)
{
    tuneRxIndex = 0;
    tuneResponsePending = 0;
    tuneLineReady = 0;
    /* 接收中断由 app_serial.c 的 Serial_Init() 启动。 */
}

/**
 * @brief  缓存一个收到的字节，收到换行则标记整行就绪。
 * @param  byte 本次收到的字节。
 * @note   **中断上下文** — 只做缓存的搬运，不做解析、不调用浮点或 printf。
 *         超长行会被截断（多余字节丢弃），不会越界。
 *
 *         原注释已过时：本函数此前同时承担了启动 HAL_UART_Receive_IT 的职责，
 *         阶段 5 之后接收装填已移至 app_serial.c 的 HAL_UART_RxCpltCallback。
 */
void AppConsole_RxByte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        /* 空行（首字符就是换行）不产生命令。 */
        if (tuneRxIndex > 0) {
            tuneLine[tuneRxIndex] = '\0';
            tuneLineReady = 1;
            tuneRxIndex = 0;
        }
    } else if (tuneRxIndex < sizeof(tuneLine) - 1U) {
        /* 预留最后一个字节给结尾 '\0'。 */
        tuneLine[tuneRxIndex++] = byte;
    }
}

/**
 * @brief  在命令表里按名字查一项。
 * @param  name 命令名（不含参数）。
 * @return 命中的表项指针；未找到返回 0。
 * @note   线性扫描，表很小（24 项），不值得用哈希或二分。
 */
static const TuneCommand_t *consoleFind(const char *name)
{
    uint8_t i;
    for (i = 0; i < sizeof(tuneTable) / sizeof(tuneTable[0]); i++) {
        if (strcmp(tuneTable[i].name, name) == 0) return &tuneTable[i];
    }
    return 0;
}

/**
 * @brief  取 show 命令里 Phase 字段要显示的文本。
 * @return 测试模式下是当前相位名；循迹模式没有相位可言，返回 "-"。
 * @note   相位名统一取自 app_testmode.c 的 AppPhaseName()，
 *         避免显示层和命令行各维护一份字符串。
 */
static const char *consolePhaseName(void)
{
    if (AppState()->mode == modePwmSpeed) {
        return AppPhaseName(AppTestMode_GetSpeedPhase());
    }
    if (AppState()->mode == modePwmTrack) {
        return AppPhaseName(AppTestMode_GetTrackPhase());
    }
    return "-";
}

/**
 * @brief  解析并执行一行命令。
 * @note   在主循环里调用。整行命令的格式是 `<命令名> [数字参数]`，
 *         分隔靠 sscanf 的空白切分，因此参数与命令名之间用空格或制表符。
 *
 *         流程：等就绪标志 -> 切分两个 token -> 查表 -> 校验参数 -> 执行动作。
 *         所有出口（包括未找到命令、参数缺失）都会清掉 tuneLineReady，
 *         否则这一行会被反复解析。
 *
 * @warning 解析期间中断可能改写 tuneLine，见文件头的竞争说明。
 */
void AppConsole_Process(void)
{
    char line[PID_TUNE_LINE_BUFFER_SIZE];
    char token[PID_TUNE_TOKEN_BUFFER_SIZE];
    char argument[PID_TUNE_TOKEN_BUFFER_SIZE];
    char *command;
    const TuneCommand_t *entry;
    float value = 0.0f;
    int count;

    if (!tuneLineReady) return;

    /* 先把整行拷成私有副本再解析。UART 中断在收到换行后会把写入下标清零，
       下一个字节就开始覆写 tuneLine，直接解析它可能读到半新半旧的内容。
       拷贝期间关中断，拷完立即放行——中断随后可以自由开始写下一行。 */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(line, tuneLine, sizeof(line));
        tuneLineReady = 0;
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
        tuneLineReady = 0;
        return;
    }

    entry = consoleFind(token);
    if (entry == 0) {
        consoleRespond("? '%s' (type 'help')\r\n", token);
        tuneLineReady = 0;
        return;
    }
    if (entry->needsArg && count < 2) {
        consoleRespond("? %s needs a value\r\n", token);
        tuneLineReady = 0;
        return;
    }
    /* 只有确实给了参数才解析，避免把空串当 0 —— 那样 printf 会把垃圾写进缓冲区。 */
    if (count >= 2) value = strtof(argument, 0);

    switch (entry->action) {
    /* 左右同时设置三个增益。 */
    case TUNE_SKP:
    case TUNE_SKI:
    case TUNE_SKD: {
        PidField_t field = (entry->action == TUNE_SKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_SKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppSpeedLoop_SetGain(SPEED_CH_LEFT, field, value);
        AppSpeedLoop_SetGain(SPEED_CH_RIGHT, field, value);
        break;
    }
    /* 只设左轮。 */
    case TUNE_LKP:
    case TUNE_LKI:
    case TUNE_LKD: {
        PidField_t field = (entry->action == TUNE_LKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_LKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppSpeedLoop_SetGain(SPEED_CH_LEFT, field, value);
        break;
    }
    /* 只设右轮。 */
    case TUNE_RKP:
    case TUNE_RKI:
    case TUNE_RKD: {
        PidField_t field = (entry->action == TUNE_RKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_RKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppSpeedLoop_SetGain(SPEED_CH_RIGHT, field, value);
        break;
    }
    /* 循迹环增益。默认 Ki = 0（位置环用 PD），见 README §4.3.3。 */
    case TUNE_TKP:
    case TUNE_TKI:
    case TUNE_TKD: {
        PidField_t field = (entry->action == TUNE_TKP) ? PID_FIELD_KP
                           : (entry->action == TUNE_TKI) ? PID_FIELD_KI
                           : PID_FIELD_KD;
        AppTracking_SetGain(field, value);
        break;
    }
    /* 差速 RPM 换算成 PWM 百分比的增益，只影响 PWM 循迹测试。 */
    case TUNE_TKG:
        AppTracking_SetSteerRpmToPwmGain(value);
        break;
    case TUNE_START:
        AppCmd_PushValue(commandStart, 0.0f);
        break;
    case TUNE_STOP:
        AppCmd_PushValue(commandStop, 0.0f);
        break;
    case TUNE_SPDGEAR:
        /* 上下界与 SPEED_LEVEL_COUNT 一致；此处写字面量 6，改档位数时要同步。 */
        if (value < 1.0f || value > 6.0f) {
            consoleRespond("[PWM-SPEED] invalid gear 1..6\r\n");
        } else {
            AppCmd_PushValue(commandSetSpeedGear, value);
        }
        break;
    case TUNE_SIMLEVEL: {
        /* 合法取值必须与 app_testmode.c 里的 angleTable 完全一致。 */
        int level = (int)value;
        if ((level != 5) && (level != 10) && (level != 45) && (level != 90)) {
            consoleRespond("[PWM-TRACK] invalid simlevel\r\n");
        } else {
            AppCmd_PushValue(commandPwmTrackSetAngle, (float)level);
        }
        break;
    }
    case TUNE_BASE:
        /* base 是绝对目标值，而队列里传的是增量，所以要先减当前值。 */
        AppCmd_PushValue(commandPwmTrackBaseStep,
                            value - (float)AppTestMode_GetBasePwm());
        break;
    case TUNE_SIM:
        if (strcmp(argument, "on") == 0) {
            AppCmd_PushValue(commandPwmTrackSim, 1.0f);
        } else if (strcmp(argument, "off") == 0) {
            AppCmd_PushValue(commandPwmTrackSim, 0.0f);
        } else {
            consoleRespond("[PWM-TRACK] use sim on/off\r\n");
        }
        break;
    case TUNE_QUIET:
        AppCmd_PushValue(commandQuiet, 0.0f);
        break;
    case TUNE_LOG:
        AppCmd_PushValue(commandLog, 0.0f);
        break;
    case TUNE_SHOW: {
        /* 一次性把两组速度环、循迹环、测试模式的状态全打出来，便于对照调参。 */
        SpeedPidInfo_t infoL;
        SpeedPidInfo_t infoR;
        float kp = AppTracking_GetGain(PID_FIELD_KP);
        float ki = AppTracking_GetGain(PID_FIELD_KI);
        float kd = AppTracking_GetGain(PID_FIELD_KD);
        AppSpeedLoop_GetInfo(SPEED_CH_LEFT, &infoL);
        AppSpeedLoop_GetInfo(SPEED_CH_RIGHT, &infoR);
        consoleRespond(
            "Mode=%s Run=%d Phase=%s Gear=%d Base=%d%%\r\n"
            "SpeedL Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Out=%.1f\r\n"
            "SpeedR Kp=%.2f Ki=%.2f Kd=%.2f Tgt=%.1f Out=%.1f\r\n"
            "Track Kp=%.2f Ki=%.2f Kd=%.2f tkg=%.2f\r\n"
            "VoltL=%.2fV VoltR=%.2fV\r\n",
            AppModeName(AppState()->mode), AppState()->running, consolePhaseName(),
            AppState()->gear, (int)AppTestMode_GetBasePwm(),
            infoL.kp, infoL.ki, infoL.kd, infoL.target, infoL.out,
            infoR.kp, infoR.ki, infoR.kd, infoR.target, infoR.out,
            kp, ki, kd, AppTracking_GetSteerRpmToPwmGain(),
            telemetryData.commandVoltageLeft,
            telemetryData.commandVoltageRight);
        break;
    }
    case TUNE_HELP:
        consoleRespond(
            "Commands: start stop spdgear simlevel base sim on|off\r\n"
            "Speed: skp/ski/skd lkp/lki/lkd rkp/rki/rkd\r\n"
            "Track: tkp/tki/tkd/tkg\r\n"
            "VOFA: quiet log  Info: show help\r\n");
        break;
    }
    /* 这里不再清 tuneLineReady：标志已在开头的临界区里消费掉了。
       若在此处再清一次，会误删掉解析期间新到达的那一行。 */
}

/**
 * @brief  把已经格式化好的响应交给发送缓冲。
 * @note   在主循环里调用。Serial_SendString() 只用毫秒级以下的时间把数据
 *         拷进发送环形缓冲，但这里仍短暂关中断，保证在它拷贝期间
 *         tuneResponseBuffer 不会被新的 consoleRespond() 改写。
 * @note   必须在 AppConsole_Process() 之后调用，否则刚格式化的响应要等下一轮。
 */
void AppConsole_Flush(void)
{
    if (!tuneResponsePending) return;
    __disable_irq();
    Serial_SendString(tuneResponseBuffer);
    __enable_irq();
    tuneResponsePending = 0;
}
