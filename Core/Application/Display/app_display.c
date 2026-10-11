/* OLED 开机页与三个模式状态页。只读快照（AppState/Tracking/telemetry），不读不写硬件状态。
   100 ms 任务在主循环（I2C 不能放中断）；BootConfig 阻塞式只在开机跑。写屏是先写缓冲再 Flush。 */

#include "app_display.h"
#include "app_cmd.h"
#include "app_config.h"
#include "app_speedloop.h"
#include "app_state.h"
#include "app_testmode.h"
#include "app_tracking.h"
#include "app_telemetry.h"
#include "irtrack.h"
#include "key.h"
#include "oled.h"
#include "timer.h"

#include <stdio.h>

static void showVoltageLine(uint8_t row);
static void cycleBootMode(void);
static void showTrackPage(void);
static void showPwmSpeedPage(void);
static void showPwmTrackPage(void);

/* 100 ms 显示任务：按当前模式刷新一页。主循环里调（I2C 不能放中断）。
   三模式各占一个 static 函数，每页字段布局一眼看清。 */
void AppDisplay_Task100ms(void)
{
    OLED_Clear();

    if (AppState()->mode == modeTrack) {
        showTrackPage();
    } else if (AppState()->mode == modePwmSpeed) {
        showPwmSpeedPage();
    } else {
        showPwmTrackPage();
    }
}

/* 在指定行显示「命令电压 L/R」。三页共用；内容来自遥测快照，是 PWM 百分比按标称电压换算的近似值。 */
static void showVoltageLine(uint8_t row)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
             telemetryData.commandVoltageLeft,
             telemetryData.commandVoltageRight);
    OLED_ShowStringSmall(row, 1, buffer);
}

/* 切到下一个模式并复位。必须跟 ResetModeState（停电机/复位循迹/测试），否则残留输出会立刻驱电机。
   开机阶段命令队列还没消费，这里直接改状态而非发命令。 */
static void cycleBootMode(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 临界区：这里写的 mode 与 PID/循迹/测试对象，也是 TIM6 控制任务在碰的东西。开机 running 恒 0、
       两边写的是同一组复位值，本来不会出事——但那是巧合，挡住中断就不靠巧合。 */
    __disable_irq();
    AppState_SetMode(AppState_NextMode());
    AppSpeedLoop_ResetModeState();
    __set_PRIMASK(primask);
}

/* 读一致的 64 位时间戳。milliseconds 是 volatile uint64_t，直接读会被拆成两条 32 位 load 撕裂。 */
static uint64_t bootTick(void)
{
    uint32_t primask = __get_PRIMASK();
    uint64_t value;

    __disable_irq();
    value = milliseconds;
    __set_PRIMASK(primask);

    return value;
}

/* 显示循迹页。IR 那行取自循迹快照 pattern，不直接读 ir_data——传感器只允许 app_tracking.c 读，
   显示与控制看同一份采样。 */
static void showTrackPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    char irBits[IRTRACK_CHANNELS + 1U];
    uint8_t pattern = AppTracking_Output()->pattern;

    /* 保持原有的显示习惯：压黑显示 '0'、白线显示 '1'。pattern 的位是「压黑置 1」，所以取反。 */
    for (uint8_t i = 0; i < IRTRACK_CHANNELS; i++) {
        irBits[i] = ((pattern >> i) & 0x01U) ? '0' : '1';
    }
    irBits[IRTRACK_CHANNELS] = '\0';

    snprintf(buffer, sizeof(buffer), "TRK %s G%d L%u/%u",
             AppState()->running ? "RUN" : "STOP", AppState()->gear,
             (unsigned int)AppState()->laps_done,
             (unsigned int)AppState()->lap_target);
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "IR:%s", irBits);
    OLED_ShowStringSmall(2, 1, buffer);

    snprintf(buffer, sizeof(buffer), "STR:%+05.1f", AppTracking_GetSteer());
    OLED_ShowStringSmall(3, 1, buffer);

    showVoltageLine(4);
}

/* 显示 PWM 速度测试页。目标转速按相位显示正负，便于与轮向对照。手动模式下相位不变，目标仍跟命令走。 */
static void showPwmSpeedPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    float target = AppState()->gear_rpm;

    if (AppTestMode_GetSpeedPhase() == PWM_TEST_PHASE_REVERSE) {
        target = -target;
    }

    snprintf(buffer, sizeof(buffer), "[PWM-SPD] %s G%d",
             AppState()->running ? "RUN" : "STOP", AppState()->gear);
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "PH:%s TGT:%+05.0f",
             AppPhaseName(AppTestMode_GetSpeedPhase()), target);
    OLED_ShowStringSmall(2, 1, buffer);

    showVoltageLine(3);
}

/* 显示 PWM 循迹测试页。第二行把注入图案按 IR1→IR8 逐位展开（1=压黑），是本页最要看的东西。
   参数（图案、基础占空比）只能用按键改。 */
static void showPwmTrackPage(void)
{
    char    buffer[OLED_TEXT_BUFFER_SIZE];
    char    bits[PATTERN_BITS + 1U];
    uint8_t pattern = AppTestMode_GetPattern();
    uint8_t i;

    /* bit0 是最左边那一路（IR1），所以按位序从左往右展开。 */
    for (i = 0U; i < PATTERN_BITS; i++) {
        bits[i] = ((pattern & (uint8_t)(1U << i)) != 0U) ? '1' : '0';
    }
    bits[PATTERN_BITS] = '\0';

    snprintf(buffer, sizeof(buffer), "[PWM-TRK] %s",
             AppState()->running ? "RUN" : "STOP");
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "P:%s PH:%s", bits,
             AppPhaseName(AppTestMode_GetTrackPhase()));
    OLED_ShowStringSmall(2, 1, buffer);

    snprintf(buffer, sizeof(buffer), "BASE:%d%% ERR:%+05.1f",
             (int)AppTestMode_GetBasePwm(), telemetryData.trackError);
    OLED_ShowStringSmall(3, 1, buffer);

    showVoltageLine(4);
}

/* 显示开机页：模式、档位、按键提示。只写缓冲并 Flush 一次（开机阶段无主循环在跑）。 */
void AppDisplay_ShowBoot(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    OLED_Clear();
    OLED_ShowStringSmall(1, 1, "[BlueCar]");
    snprintf(buffer, sizeof(buffer), "Mode:%s G:%d",
             AppModeName(AppState()->mode), AppState()->gear);
    OLED_ShowStringSmall(2, 1, buffer);
    OLED_ShowStringSmall(4, 1, "KEY1 Mode");
    OLED_ShowStringSmall(5, 1, "KEY2/3 Exit");
    OLED_Flush();
}

/* 开机配置：阻塞式，只在 main() 初始化阶段调一次。KEY1 循环切模式，KEY2/KEY3 退出。
   每 100 ms 无条件重绘开机页（写死 100ms，非 APP_DISPLAY_PERIOD_MS，此时调度器还没规律跑显示）。
   退出前复位模式状态并清空命令队列，避免开机期间按下的命令带到运行阶段。 */
void AppDisplay_BootConfig(void)
{
    uint8_t exitRequested = 0;
    uint64_t lastTick = bootTick();

    AppSpeedLoop_ResetMotion();
    AppDisplay_ShowBoot();

    while (!exitRequested) {
        if (Key_CheckSingleOrLong(KEY_1)) {
            cycleBootMode();
            AppDisplay_ShowBoot();
            lastTick = bootTick();
        }
        if (Key_Check(KEY_2, KEY_SINGLE) || Key_Check(KEY_3, KEY_SINGLE)) {
            exitRequested = 1;
        }
        if ((bootTick() - lastTick) >= 100U) {
            lastTick = bootTick();
            AppDisplay_ShowBoot();
        }
        OLED_Flush();
    }

    /* 退出时的复位同样要挡住中断，理由见 cycleBootMode()。 */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        AppSpeedLoop_ResetModeState();
        AppCmd_Init();
        Key_Clear();
        __set_PRIMASK(primask);
    }
}
