/**
 * @file    app_display.c
 * @brief   OLED 开机页与三个模式状态页。
 *
 * @note    本模块只有两类工作：开机时等待用户选模式，以及运行中每 100 ms
 *          刷新一页状态。所有内容都来自只读快照（AppState() == 、
 *          AppTracking_Output()、telemetryData），不读也不写任何硬件状态。
 *
 * @note    执行上下文：AppDisplay_Task100ms() 由 AppDeferred_Run() 在主循环里
 *          调用——OLED 走 I2C，可能耗时上 ms，绝不能放进中断。
 *          而 AppDisplay_BootConfig() 是阻塞式的，只在开机上电阶段运行一次。
 *
 * @note    I2C 写屏是「先写缓冲、再 Flush」两段式：本模块只往 OLED 的帧缓冲
 *          里写字符，真正的总线传输由 main.c 每轮调 OLED_Flush() 完成。
 */

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

/**
 * @brief  在指定行显示「命令电压 L/R」。
 * @param  row 显示行号（OLED 的行，从 1 开始）。
 * @note   三个模式页都要显示这一行，抽出来避免同一段 snprintf 抄三遍。
 *         内容来自遥测快照，是 PWM 百分比按标称电压换算出来的近似值。
 */
static void showVoltageLine(uint8_t row)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    snprintf(buffer, sizeof(buffer), "V L:%+.2f R:%+.2f",
             telemetryData.commandVoltageLeft,
             telemetryData.commandVoltageRight);
    OLED_ShowStringSmall(row, 1, buffer);
}

/**
 * @brief  切到模式循环里的下一个模式并复位。
 * @note   模式顺序由 AppState_NextMode() 统一给出，与按键路径共用一份逻辑。
 * @note   必须跟着调 AppSpeedLoop_ResetModeState()：切换模式要停电机、
 *         复位循迹状态机和两个测试模式，否则切换后残留的输出会立刻驱动电机。
 * @note   开机阶段调度器已经放行，但命令队列还没有被消费
 *         （控制任务要等下一次 50 ms 时隙），所以这里直接改状态而不是发命令。
 */
static void cycleBootMode(void)
{
    AppState_SetMode(AppState_NextMode());
    AppSpeedLoop_ResetModeState();
}

/**
 * @brief  显示开机页：当前模式、档位和按键提示。
 * @note   只写帧缓冲并调用一次 OLED_Flush()，因为开机阶段没有主循环在跑。
 */
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

/**
 * @brief  开机配置阶段：等待用户选好模式后退出。
 * @note   **阻塞式**，只在 main() 的初始化阶段调用一次。
 * @code
 *   KEY1      循环切换 循迹 / PWM 速度 / PWM 循迹
 *   KEY2、KEY3 退出，进入正常运行
 * @endcode
 * @note   每 100 ms 无条件重绘一次开机页，保证按键改变模式后显示能跟上；
 *         这个 100 ms 是写死的（不是 APP_DISPLAY_PERIOD_MS），
 *         因为此时调度器还没有规律地跑显示任务。
 * @note   退出前会复位模式状态并重新清空命令队列：开机期间用户可能已经按下
 *         若干按键，那些命令不该带到运行阶段执行。
 */
void AppDisplay_BootConfig(void)
{
    uint8_t exitRequested = 0;
    uint64_t lastTick = milliseconds;

    AppSpeedLoop_ResetMotion();
    AppDisplay_ShowBoot();

    while (!exitRequested) {
        if (Key_CheckSingleOrLong(KEY_1)) {
            cycleBootMode();
            AppDisplay_ShowBoot();
            lastTick = milliseconds;
        }
        if (Key_Check(KEY_2, KEY_SINGLE) || Key_Check(KEY_3, KEY_SINGLE)) {
            exitRequested = 1;
        }
        if ((milliseconds - lastTick) >= 100U) {
            lastTick = milliseconds;
            AppDisplay_ShowBoot();
        }
        OLED_Flush();
    }

    AppSpeedLoop_ResetModeState();
    AppCmd_Init();
    Key_Clear();
}

/**
 * @brief  显示循迹页。
 * @note   四行依次是：运行状态与圈数进度、八路红外状态位、循迹差速、命令电压。
 *         IR 那一行把八路开关量铺成 0/1 串，方便现场确认哪几路压线。
 * @note   八路状态取自循迹快照的 pattern 字段，**不直接读 ir_data**——
 *         驱动层全局只允许 app_tracking.c 读，显示要用就从快照转发，
 *         这样传感器只有一处解读，显示和控制看到的必然是同一份采样。
 */
static void showTrackPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    char irBits[IRTRACK_CHANNELS + 1U];
    uint8_t pattern = AppTracking_Output()->pattern;

    /* 保持原有的显示习惯：压黑显示 '0'、白线显示 '1'。
       pattern 的位是「压黑置 1」，所以这里取反。 */
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

/**
 * @brief  显示 PWM 速度测试页。
 * @note   目标转速按当前相位显示正负：反转相位显示成负数，
 *         便于和实际轮向对照。
 */
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

/**
 * @brief  显示 PWM 循迹测试页。
 * @note   第三行的 SRC 字段说明当前用的是模拟角度（SIM）还是真实红外（IR），
 *         调参时容易忘记这一项，显示出来避免误判。
 */
static void showPwmTrackPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    snprintf(buffer, sizeof(buffer), "[PWM-TRK] %s A:%u",
             AppState()->running ? "RUN" : "STOP",
             (unsigned int)AppTestMode_GetAngle());
    OLED_ShowStringSmall(1, 1, buffer);

    snprintf(buffer, sizeof(buffer), "PH:%s SRC:%s",
             AppPhaseName(AppTestMode_GetTrackPhase()),
             AppTestMode_GetSimEnabled() ? "SIM" : "IR");
    OLED_ShowStringSmall(2, 1, buffer);

    snprintf(buffer, sizeof(buffer), "BASE:%d%% ERR:%+05.1f",
             (int)AppTestMode_GetBasePwm(), telemetryData.trackError);
    OLED_ShowStringSmall(3, 1, buffer);

    showVoltageLine(4);
}

/**
 * @brief  100 ms 显示任务：按当前模式刷新对应的一页。
 * @note   由 AppDeferred_Run() 在主循环调用。整屏重绘前先 OLED_Clear()，
 *         不存在残影问题。
 * @note   三个模式各占一个 static 函数，而不是塞进一个长函数里用 if-else
 *         包住整段绘制代码——这样每页的字段布局能一眼看清。
 */
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
