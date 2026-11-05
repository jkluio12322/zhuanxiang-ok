/* OLED 开机页与五个模式状态页。只读快照，不直接读写硬件状态。
   100 ms 任务在主循环（I2C 不能放中断）；BootConfig 阻塞式只在开机跑。写屏是先写缓冲再 Flush。

   三页共用同一版面骨架：
     第 1 行  8x16 大字号：模式名 + RUN/STOP
     第 3 行  6x8 小字号：档位/占空比 + 圈数
     第 4 行  传感器（循迹图案、模拟图案）
     第 5 行  轮速 实际/目标 或 转向量
     第 6 行  PWM 输出 或 环路说明
     第 7 行  电压 / 航向
     第 8 行  按键提示
   8x16 大字号占页 0~1（= 小字号第 1、2 行），所以小字号从第 3 行开始写。 */

#include "display.h"
#include "imu.h"
#include "cmd.h"
#include "config.h"
#include "motion.h"
#include "speed_loop.h"
#include "state.h"
#include "testmode.h"
#include "track.h"
#include "telemetry.h"
#include "loop_test.h"
#include "eight.h"
#include "irtrack.h"
#include "key.h"
#include "oled.h"
#include "timer.h"

#include <stdio.h>

/* 开机页最长停留时间：这段时间里还能用 KEY1 选模式，到点自动进运行页，
   否则停在菜单里就永远看不到各模式的数据页。 */
#define BOOT_MENU_HOLD_MS 1500U

static void showHeader(const char *title, uint8_t running);
static void showPowerLine(uint8_t row);
static void cycleBootMode(void);
static void showTrackPage(void);
static void showPwmSpeedPage(void);
static void showPwmTrackPage(void);
static void showLoopTestPage(void);
static void showEightPage(void);

/* 100 ms 显示任务：按当前模式刷新一页。主循环里调（I2C 不能放中断）。
   三模式各占一个 static 函数，每页字段布局一眼看清。 */
void display_task(void)
{
    oled_clear();

    if (state_view()->mode == modeTrack) {
        showTrackPage();
    } else if (state_view()->mode == modePwmSpeed) {
        showPwmSpeedPage();
    } else if (state_view()->mode == modePwmTrack) {
        showPwmTrackPage();
    } else if (state_view()->mode == modeLoopTest) {
        showLoopTestPage();
    } else {
        showEightPage();
    }
}

/* 页头：8x16 大字号，左边模式名、右边 RUN/STOP，整行 16 字符对齐。 */
static void showHeader(const char *title, uint8_t running)
{
    char buffer[17];

    snprintf(buffer, sizeof(buffer), "%-8s%8s", title, (running != 0U) ? "RUN" : "STOP");
    oled_show_string(1, 1, buffer);
}

/* 电源行：命令电压 L/R。三页共用；电压是 PWM 百分比按标称电压换算的近似值。 */
static void showPowerLine(uint8_t row)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];

    snprintf(buffer, sizeof(buffer), "V %.2f/%.2f",
             (double)telemetry_data.commandVoltageLeft,
             (double)telemetry_data.commandVoltageRight);
    oled_show_string_small(row, 1, buffer);
}

/* 姿态行：横滚 R / 俯仰 P / 航向 Y，单位度。roll/pitch 来自加速度重力方向（静止准），
   yaw 是陀螺 Z 积分（短时准、会慢漂）。*/
static void showAngleLine(uint8_t row)
{
    char             buffer[OLED_TEXT_BUFFER_SIZE];
    const imu_data_t *imu = imu_get();

    snprintf(buffer, sizeof(buffer), "R%+04.0f P%+04.0f Y%+04.0f",
             (double)imu->roll_deg, (double)imu->pitch_deg, (double)imu->yaw);
    oled_show_string_small(row, 1, buffer);
}

/* 切到下一个模式并复位。必须跟 ResetModeState（停电机/复位循迹/测试），否则残留输出会立刻驱电机。
   开机阶段命令队列还没消费，这里直接改状态而非发命令。 */
static void cycleBootMode(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 临界区：这里写的 mode 与 PID/循迹/测试对象，也是 TIM6 控制任务在碰的东西。开机 running 恒 0、
       两边写的是同一组复位值，本来不会出事——但那是巧合，挡住中断就不靠巧合。 */
    __disable_irq();
    state_set_mode(state_next_mode());
    speed_loop_ResetModeState();
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

/* 循迹页：档位/圈数、八路图案与偏差、左右轮实际/目标、PWM、电压航向。
   IR 行取自循迹快照 pattern，不直接读传感器——显示与控制看同一份采样。 */
static void showTrackPage(void)
{
    char        buffer[OLED_TEXT_BUFFER_SIZE];
    char        irBits[IRTRACK_CHANNELS + 1U];
    uint8_t     pattern = track_output()->pattern;
    const StateView *state = state_view();
    uint8_t     i;

    /* 保持原有显示习惯：压黑显示 '0'、白线显示 '1'。pattern 的位是「压黑置 1」，所以取反。 */
    for (i = 0U; i < IRTRACK_CHANNELS; i++) {
        irBits[i] = ((pattern >> i) & 0x01U) ? '0' : '1';
    }
    irBits[IRTRACK_CHANNELS] = '\0';

    showHeader("TRACK", state->running);

    snprintf(buffer, sizeof(buffer), "G%u %3.0fRPM  LAP %u/%u",
             (unsigned int)state->gear, (double)state->gear_rpm,
             (unsigned int)state->laps_done, (unsigned int)state->lap_target);
    oled_show_string_small(3, 1, buffer);

    snprintf(buffer, sizeof(buffer), "IR%s E %+.2f",
             irBits, (double)telemetry_data.trackError);
    oled_show_string_small(4, 1, buffer);

    snprintf(buffer, sizeof(buffer), "L%3.0f/%3.0f R%3.0f/%3.0f",
             (double)motion_view()->left_rpm, (double)state->target_left_rpm,
             (double)motion_view()->right_rpm, (double)state->target_right_rpm);
    oled_show_string_small(5, 1, buffer);

    snprintf(buffer, sizeof(buffer), "P%4d/%4d %4.1fV",
             (int)motion_view()->left_pwm, (int)motion_view()->right_pwm,
             (double)telemetry_data.commandVoltageLeft);
    oled_show_string_small(6, 1, buffer);

    showAngleLine(7);
    oled_show_string_small(8, 1, "K2 SPD K3 GO K4 LAP");
}

/* PWM 速度页：手动开环，只显示占空比/方向，不显示编码器闭环量。 */
static void showPwmSpeedPage(void)
{
    char        buffer[OLED_TEXT_BUFFER_SIZE];
    int8_t      dir = testmode_get_pwm_speed_dir();
    int16_t     duty = testmode_get_pwm_speed_duty();
    const char *dirText = (dir > 0) ? "FWD" : ((dir < 0) ? "REV" : "--");

    showHeader("PWM-SPD", state_view()->running);

    snprintf(buffer, sizeof(buffer), "DUTY %2d%%   DIR %s", (int)duty, dirText);
    oled_show_string_small(3, 1, buffer);

    snprintf(buffer, sizeof(buffer), "L%3d%% R%3d%%  OPEN", (int)duty, (int)duty);
    oled_show_string_small(4, 1, buffer);

    /* 第 5 行给极性判定用：开环正转时这里应为正数。负数 = 编码器 A/B 反接（改 ENCODER_x_SIGN），
       轮子方向不对则改 MOTOR_x_SIGN。 */
    snprintf(buffer, sizeof(buffer), "ENC L%+4.0f R%+4.0f",
             (double)motion_view()->left_rpm, (double)motion_view()->right_rpm);
    oled_show_string_small(5, 1, buffer);

    showAngleLine(6);
    showPowerLine(7);
    oled_show_string_small(8, 1, "K2 DUTY K3 FWD K4 REV");
}

/* PWM 循迹页：模拟图案 + 相位 + 基础占空比 + 循迹环输出。 */
static void showPwmTrackPage(void)
{
    char    buffer[OLED_TEXT_BUFFER_SIZE];
    char    bits[PATTERN_BITS + 1U];
    uint8_t pattern = testmode_get_pattern();
    uint8_t i;

    /* bit0 是最左边那一路（IR1），所以按位序从左往右展开。 */
    for (i = 0U; i < PATTERN_BITS; i++) {
        bits[i] = ((pattern & (uint8_t)(1U << i)) != 0U) ? '1' : '0';
    }
    bits[PATTERN_BITS] = '\0';

    showHeader("PWM-TRK", state_view()->running);

    snprintf(buffer, sizeof(buffer), "PAT %s", bits);
    oled_show_string_small(3, 1, buffer);

    snprintf(buffer, sizeof(buffer), "PH %-5s BASE %3d%%",
             phase_name(testmode_get_track_phase()), (int)testmode_get_base_pwm());
    oled_show_string_small(4, 1, buffer);

    snprintf(buffer, sizeof(buffer), "E %+.2f S %+.1f",
             (double)telemetry_data.trackError, (double)track_get_steer());
    oled_show_string_small(5, 1, buffer);

    showAngleLine(6);
    showPowerLine(7);
    oled_show_string_small(8, 1, "K2 PAT K3 GO K4 BASE");
}

static void showLoopTestPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    const LoopTestView_t *test = loop_test_view();

    showHeader("LOOP", test->running);

    snprintf(buffer, sizeof(buffer), "%s %s",
             loop_test_program_name(test->program),
             loop_test_option_name(test->program, test->option));
    oled_show_string_small(3, 1, buffer);

    snprintf(buffer, sizeof(buffer), "%s %lus",
             loop_test_phase_name(test),
             (unsigned long)(test->elapsed_ms / 1000U));
    oled_show_string_small(4, 1, buffer);

    snprintf(buffer, sizeof(buffer), "A %+5.1f/%+5.1f",
             (double)test->actual_angle_deg[0],
             (double)test->actual_angle_deg[1]);
    oled_show_string_small(5, 1, buffer);

    snprintf(buffer, sizeof(buffer), "R%+3.0f/%+3.0f %s",
             (double)test->target_rpm[0],
             (double)test->target_rpm[1],
             (test->fake != 0U) ? "FAKE" : "REAL");
    oled_show_string_small(6, 1, buffer);

    snprintf(buffer, sizeof(buffer), "P %+4.1f/%+4.1f",
             (double)test->pwm[0], (double)test->pwm[1]);
    oled_show_string_small(7, 1, buffer);

    oled_show_string_small(8, 1, "K2 PRG K3 RUN K4 SEL");
}

static void showEightPage(void)
{
    char buffer[OLED_TEXT_BUFFER_SIZE];
    const EightView_t *route = eight_view();

    showHeader("EIGHT", state_view()->running);
    if (route->phase == EIGHT_FAULT) {
        snprintf(buffer, sizeof(buffer), "FAULT %s", eight_fault_name(route->fault));
    } else {
        snprintf(buffer, sizeof(buffer), "%s %2.0fRPM",
                 eight_phase_name(route->phase), (double)route->speed_rpm);
    }
    oled_show_string_small(3, 1, buffer);

    snprintf(buffer, sizeof(buffer), "D%4.0f/%4.0fcm",
             (double)route->progress_cm, (double)route->target_distance_cm);
    oled_show_string_small(4, 1, buffer);

    snprintf(buffer, sizeof(buffer), "Y%+5.1f T%+5.1f",
             (double)route->yaw_deg, (double)route->target_yaw_deg);
    oled_show_string_small(5, 1, buffer);

    snprintf(buffer, sizeof(buffer), "IR%02X E%+.1f",
             (unsigned int)route->ir_pattern, (double)route->line_error);
    oled_show_string_small(6, 1, buffer);

    snprintf(buffer, sizeof(buffer), "L%3.0f R%3.0f %lus",
             (double)motion_view()->left_rpm, (double)motion_view()->right_rpm,
             (unsigned long)(route->elapsed_ms / 1000U));
    oled_show_string_small(7, 1, buffer);
    oled_show_string_small(8, 1, "K2 SPD K3 GO K1 MODE");
}

/* 开机页：模式、档位、占空比、圈数、按键提示。只写缓冲并 Flush 一次（开机阶段无主循环在跑）。 */
void display_show_boot(void)
{
    char            buffer[OLED_TEXT_BUFFER_SIZE];
    const StateView *state = state_view();

    oled_clear();
    oled_show_string(1, 1, "BlueCar    V1.0");

    snprintf(buffer, sizeof(buffer), "MODE  %s", mode_name(state->mode));
    oled_show_string_small(3, 1, buffer);

    snprintf(buffer, sizeof(buffer), "SPEED G%u %3.0fRPM",
             (unsigned int)state->gear, (double)state->gear_rpm);
    oled_show_string_small(4, 1, buffer);

    snprintf(buffer, sizeof(buffer), "DUTY  %2d%%   LAP %u/%u",
             (int)testmode_get_pwm_speed_duty(),
             (unsigned int)state->laps_done, (unsigned int)state->lap_target);
    oled_show_string_small(5, 1, buffer);

    oled_show_string_small(7, 1, "K1  SWITCH MODE");
    oled_show_string_small(8, 1, "K2/K3  START");
    oled_flush();
}

/* 开机配置菜单：begin 画首页并记时，poll 每轮调用一次、立即返回（1 = 菜单已结束）。
   原实现是一个 while 死循环，整个菜单期间（最长 1.5 s）主循环不转 —— 遥测、命令行、
   OLED 刷屏全部停摆。拆开之后由 main() 的主循环驱动，这些任务照常推进。 */
#define BOOT_REDRAW_MS      100U   /* 开机页重绘周期 */
#define BOOT_VOFA_MS        500U   /* 菜单期间补发遥测的周期 */
#define BOOT_MENU_HARD_MS   3000U  /* HAL_GetTick 兜底：TIM5 停摆时菜单也能退出 */

static uint8_t  bootActive;      /* 1 = 菜单进行中 */
static uint64_t bootLastDraw;    /* 上次重绘时刻 */
static uint64_t bootLastVofa;    /* 上次补发遥测时刻 */
static uint64_t bootStart;       /* 自动退出计时起点，KEY1 切模式时重置 */
static uint32_t bootHardStart;   /* 兜底计时起点（HAL_GetTick，不依赖 TIM5） */

void display_boot_begin(void)
{
    speed_loop_ResetMotion();
    display_show_boot();

    bootLastDraw  = bootTick();
    bootLastVofa  = bootLastDraw;
    bootStart     = bootLastDraw;
    bootHardStart = HAL_GetTick();
    bootActive    = 1U;
}

uint8_t display_boot_poll(void)
{
    if (bootActive == 0U) {
        return 1U;
    }

    if (key_check_single_or_long(KEY_1)) {
        cycleBootMode();
        display_show_boot();
        bootLastDraw = bootTick();
        bootStart    = bootLastDraw;
    }
    if (key_check(KEY_2, KEY_SINGLE) || key_check(KEY_3, KEY_SINGLE)) {
        bootActive = 0U;
    }
    if ((bootTick() - bootLastDraw) >= BOOT_REDRAW_MS) {
        bootLastDraw = bootTick();
        display_show_boot();
    }

    /* 菜单期间主循环还没进延后任务，这里每 500 ms 补发一帧遥测。 */
    if ((bootTick() - bootLastVofa) >= BOOT_VOFA_MS) {
        bootLastVofa = bootTick();
        telemetry_send();
    }

    /* 自动退出：正常走 TIM5 时间戳；TIM5 若停摆，HAL_GetTick 兜底，避免永远停在菜单。 */
    if (((bootTick() - bootStart) >= (uint64_t)BOOT_MENU_HOLD_MS)
        || ((uint32_t)(HAL_GetTick() - bootHardStart) >= BOOT_MENU_HARD_MS)) {
        bootActive = 0U;
    }

    if (bootActive != 0U) {
        return 0U;
    }

    /* 退出时的复位同样要挡住中断，理由见 cycleBootMode()。 */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        speed_loop_ResetModeState();
        cmd_init();
        key_clear();
        __set_PRIMASK(primask);
    }
    return 1U;
}
