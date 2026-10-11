/**
 * @file    app_console.c
 * @brief   USART1 极简命令行 — 台架手动转速 + 状态查看。
 *
 * @note    文件内排序：核心任务放最上方，辅助函数与对外接口放下面。
 *
 * @note    命令集刻意保持极小。控制参数一律在 `app_config.h` 里改、重新编译，
 *          不做在线调参——那套命令连带一串 SetGain 接口只会增加维护面。
 *          保留的只有台架测试真正要用的几条。
 *
 * @note    执行上下文的划分（本模块同时横跨中断与主循环）：
 * @code
 *   UART RX 中断  -> AppConsole_RxByte()   只往行缓冲塞字节，收到换行置就绪标志
 *   主循环        -> AppConsole_Process()  解析整行、执行命令（涉及浮点，只能在主循环）
 *   主循环        -> AppConsole_Flush()    把准备好的响应交给发送环形缓冲
 * @endcode
 *
 * @note    行缓冲的竞争已处理：AppConsole_Process() 在**同一个临界区里**
 *          先查就绪标志、再把整行拷成私有副本，解析过程只操作副本，
 *          中断可以自由写下一行。标志检查必须在临界区之内——放在外面的话，
 *          查到标志与关中断之间到达的字节会把行首覆写掉。
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
#include "app_speedloop.h"
#include "app_state.h"
#include "app_telemetry.h"
#include "app_testmode.h"
#include "app_tracking.h"
#include "app_serial.h"

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

/**
 * @brief  命令表中每一项对应的动作。
 */
typedef enum {
    CMD_LEFT_RPM,    /* l <rpm>   只设左轮目标转速 */
    CMD_RIGHT_RPM,   /* r <rpm>   只设右轮目标转速 */
    CMD_BOTH_RPM,    /* t <rpm>   左右同时设 */
    CMD_STOP,        /* stop      停车 */
    CMD_SHOW,        /* show      打印当前状态 */
    CMD_QUIET,       /* quiet     暂停 VOFA 输出 */
    CMD_LOG,         /* log       恢复 VOFA 输出 */
    CMD_HELP         /* help / ?  命令列表 */
} ConsoleAction_t;

/**
 * @brief  命令表的一行。
 */
typedef struct {
    const char     *name;    /**< 命令名。 */
    uint8_t         needsArg;/**< 1 表示必须跟一个数字参数。 */
    ConsoleAction_t action;  /**< 对应 ConsoleAction_t。 */
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

/* ── 辅助函数前置声明（定义在文件下方） ───────────────────────────── */
static void                consoleRespond(const char *format, ...);
static const ConsoleCommand_t *consoleFind(const char *name);
static const char         *consolePhaseName(void);

/* ══════════════ 核心任务 ══════════════ */

/**
 * @brief  解析并执行一行命令 —— 本文件的核心。
 * @note   在**主循环**里调用。整行格式是 `<命令名> [数字参数]`，
 *         分隔靠 sscanf 的空白切分。
 *
 * @note   流程：等就绪标志 -> 在临界区里拷一份私有副本并消费标志 ->
 *         切分两个 token -> 查表 -> 校验参数 -> 执行动作。
 *
 * @note   为什么先拷副本：UART 中断在收到换行后会把写入下标清零，
 *         下一个字节就开始覆写 consoleLine，直接解析它可能读到半新半旧的内容。
 *         拷贝期间关中断，拷完立即放行。
 *
 * @note   为什么在开头消费标志而不是结尾：若在结尾清，解析期间新到的那一行
 *         会被误删。现在的写法下，中断可以自由开始写下一行。
 */
void AppConsole_Process(void)
{
    char line[CONSOLE_LINE_BUFFER_SIZE];
    char token[CONSOLE_TOKEN_BUFFER_SIZE];
    char argument[CONSOLE_TOKEN_BUFFER_SIZE];
    char *command;
    const ConsoleCommand_t *entry;
    int count;

    /* 标志检查与拷贝必须在同一个临界区里完成，见函数头的说明。 */
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
        consoleRespond("? '%s' (type 'help')\r\n", token);
        return;
    }
    if (entry->needsArg && count < 2) {
        consoleRespond("? %s needs a value\r\n", token);
        return;
    }

    switch (entry->action) {
    /* 台架手动转速：l / r / t。
       只在 PWM 速度测试模式下有意义——其它模式下轮速目标另有来源
       （循迹环、测试相位），改了下一周期就被覆盖。
       所以在这里就拦住并给出提示，而不是入队后静默失效。 */
    case CMD_LEFT_RPM:
    case CMD_RIGHT_RPM:
    case CMD_BOTH_RPM: {
        char *end = 0;
        float parsed;

        if (AppState()->mode != modePwmSpeed) {
            consoleRespond("[MANUAL] only in PWM-SPEED mode (KEY1)\r\n");
            break;
        }

        /* 严格解析：整串必须就是一个数。strtof 对 "abc" 也会返回 0 且不报错，
           不校验的话一个手误会被当成 "l 0" 送进队列——而执行侧会顺手把车置成
           运行态，右轮还保留着上一次的旧目标，等于凭空把车开起来。 */
        parsed = strtof(argument, &end);
        if ((end == argument) || (*end != '\0')) {
            consoleRespond("? %s needs a number, got '%s'\r\n", token, argument);
            break;
        }

        /* 范围校验顺带挡住 NaN 和 ±inf：NaN 与任何值比较都为假，这一条会把
           它们一并拒绝，不用另外引 math.h。下游 clampSpeedTarget() 仍然兜着。 */
        if (!((parsed >= -SPEED_TARGET_LIMIT_RPM)
              && (parsed <= SPEED_TARGET_LIMIT_RPM))) {
            consoleRespond("? %s range +-%d rpm\r\n", token,
                           (int)SPEED_TARGET_LIMIT_RPM);
            break;
        }

        if (AppCmd_PushValue(
                (entry->action == CMD_LEFT_RPM)  ? commandSetLeftRpm
                : (entry->action == CMD_RIGHT_RPM) ? commandSetRightRpm
                : commandSetBothRpm,
                parsed) == 0U) {
            consoleRespond("[MANUAL] queue full, dropped\r\n");
            break;
        }

        /* 回显送出的值。静默成功会让操作者分不清"生效了"和"被丢了"，
           而被模式门挡下时是有回显的——两边不对称更容易误判。 */
        consoleRespond("[MANUAL] %s %.1f rpm\r\n", token, parsed);
        break;
    }

    case CMD_STOP:
        /* 走命令队列，由控制任务统一执行停车与复位。 */
        (void)AppCmd_PushValue(commandStop, 0.0f);
        break;

    case CMD_SHOW: {
        /* 一次性把测试模式、两组速度环、循迹环、目标和实测都打出来。 */
        SpeedPidInfo_t infoL;
        SpeedPidInfo_t infoR;
        float kp = AppTracking_GetGain(PID_FIELD_KP);
        float ki = AppTracking_GetGain(PID_FIELD_KI);
        float kd = AppTracking_GetGain(PID_FIELD_KD);
        AppSpeedLoop_GetInfo(SPEED_CH_LEFT, &infoL);
        AppSpeedLoop_GetInfo(SPEED_CH_RIGHT, &infoR);
        consoleRespond(
            "Mode=%s Run=%d Phase=%s Gear=%d Base=%d%%\r\n"
            "Tgt L=%.1f R=%.1f  Act L=%.1f R=%.1f\r\n"
            "SpeedL Kp=%.2f Ki=%.2f Kd=%.2f\r\n"
            "SpeedR Kp=%.2f Ki=%.2f Kd=%.2f\r\n"
            "Track Kp=%.2f Ki=%.2f Kd=%.2f tkg=%.2f\r\n"
            "VoltL=%.2fV VoltR=%.2fV\r\n",
            AppModeName(AppState()->mode), AppState()->running,
            consolePhaseName(),
            AppState()->gear, (int)AppTestMode_GetBasePwm(),
            /* Tgt 取速度环里存的目标：那是夹到 ±SPEED_TARGET_LIMIT_RPM 之后的
               实际值。AppState() 里存的是原始请求，直接打会显示一个车永远不会
               执行的转速（t 500 打出 500，而车只跑 190）。 */
            infoL.target, infoR.target,
            telemetryData.actualLeftRpm, telemetryData.actualRightRpm,
            infoL.kp, infoL.ki, infoL.kd,
            infoR.kp, infoR.ki, infoR.kd,
            kp, ki, kd, AppTracking_GetSteerRpmToPwmGain(),
            telemetryData.commandVoltageLeft,
            telemetryData.commandVoltageRight);
        break;
    }

    case CMD_QUIET:
        (void)AppCmd_PushValue(commandQuiet, 0.0f);
        break;

    case CMD_LOG:
        (void)AppCmd_PushValue(commandLog, 0.0f);
        break;

    case CMD_HELP:
        consoleRespond(
            "l <rpm>  left wheel target (neg = reverse)\r\n"
            "r <rpm>  right wheel target\r\n"
            "t <rpm>  both wheels\r\n"
            "stop     stop the car\r\n"
            "show     print state\r\n"
            "quiet    pause VOFA   log  resume VOFA\r\n"
            "Manual rpm needs PWM-SPEED mode (cycle with KEY1).\r\n");
        break;

    default:
        break;
    }
}

/* ══════════════ 辅助函数 ══════════════ */

/**
 * @brief  格式化一条待发送的响应。
 * @param  format printf 风格格式串。
 * @param  ...    格式参数。
 * @note   若已有响应待发，本次调用被**直接丢弃**（不排队、不覆盖）。
 *         宁可少一条回显，也不要让后一条响应顶掉前一条造成输出错乱。
 */
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

/**
 * @brief  在命令表里按名字查一项。
 * @param  name 命令名（不含参数）。
 * @return 命中的表项指针；未找到返回 0。
 * @note   线性扫描，表很小（9 项），不值得用哈希或二分。
 */
static const ConsoleCommand_t *consoleFind(const char *name)
{
    uint8_t i;
    for (i = 0; i < sizeof(commandTable) / sizeof(commandTable[0]); i++) {
        if (strcmp(commandTable[i].name, name) == 0) return &commandTable[i];
    }
    return 0;
}

/**
 * @brief  取 show 里 Phase 字段要显示的文本。
 * @return 测试模式下是当前相位名；循迹模式没有相位可言，返回 "-"。
 * @note   相位名统一取自 app_testmode.c 的 AppPhaseName()，
 *         避免显示层和命令行各维护一份字符串。
 * @note   PWM 循迹测试下这个相位表示注入图案是否被左右镜像
 *         （LEFT = 原始图案，RIGHT = 镜像），见 AppTestMode_GetPattern()。
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

/* ══════════════ 对外接口 ══════════════ */

/**
 * @brief  初始化命令行模块的状态。
 * @note   只清软件状态；UART 接收中断由 app_serial.c 的 Serial_Init() 启动。
 */
void AppConsole_Init(void)
{
    consoleRxIndex = 0;
    consoleResponsePending = 0;
    consoleLineReady = 0;
}

/**
 * @brief  缓存一个收到的字节，收到换行则标记整行就绪。
 * @param  byte 本次收到的字节。
 * @note   **中断上下文** — 只做缓存的搬运，不做解析、不调用浮点或 printf。
 *         超长行会被截断（多余字节丢弃），不会越界。
 */
void AppConsole_RxByte(uint8_t byte)
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

/**
 * @brief  把已经格式化好的响应交给发送缓冲。
 * @note   在主循环里调用，且必须在 AppConsole_Process() 之后。
 * @note   这里**不需要**临界区：consoleResponseBuffer 的唯一写者
 *         consoleRespond() 也只在主循环跑，两者不会并发；而发送环形缓冲
 *         的保护在 Serial_Write() 内部（它用的是保存/恢复 PRIMASK 的写法）。
 *         早先这里套了一层 __disable_irq()/__enable_irq()，既用了盲 enable
 *         （从已关中断的上下文调过来会把中断错误地打开），又没有保护到任何
 *         东西，所以删掉。
 */
void AppConsole_Flush(void)
{
    if (!consoleResponsePending) return;
    Serial_SendString(consoleResponseBuffer);
    consoleResponsePending = 0;
}
