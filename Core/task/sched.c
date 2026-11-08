/* BlueCar 任务编排：TIM4 出 2 ms 采样拍，TIM7 出 5 ms 控制拍。
 *
 * 为什么用两个硬件定时器而不是一个 1 kHz 再取模：
 *   取模出来的节拍会把「中断响应延迟」直接叠进周期里，两档还会互相排队；
 *   两个硬件时基各自独立触发，采样拍永远能插进控制拍之间，互不顶替。
 *   NVIC 上 TIM4 抢占 0、TIM7 抢占 1：控制拍正在算的时候采样拍可以插队，
 *   反过来的情况不存在。
 *
 * 所有 dt 一律取 DWT->CYCCNT 换算的真实微秒，不用「假设的 5 ms」：
 *   中断延迟、HAL 开销、被串口/I2C 抢占的时间都会被实测吸收掉。
 */

#include "sched.h"
#include "main.h"
#include "config.h"
#include "state.h"
#include "motion.h"
#include "irtrack.h"
#include "imu.h"
#include "track.h"
#include "key.h"
#include "tactic.h"
#include "control.h"
#include "mission.h"
#include "turntest.h"   /* 转角自检（测试代码，见 Core/selftest） */

#define SCHED_DT_FALLBACK_US  ((uint32_t)TASK_CONTROL_MS * 1000UL)
#define SCHED_SAMPLE_JUMP_US  ((uint32_t)TASK_SAMPLE_MS * 2000UL)

static uint16_t s_sample_div = 0U;      /* 采样拍计数，用于再分频出 IMU */
static uint16_t s_control_div = 0U;     /* 控制拍计数，用于再分频出遥测与显示 */
static uint32_t s_cycles_per_us = 72U;
static uint32_t s_last_sample_us = 0U;
static uint32_t s_last_control_us = 0U;
static uint8_t s_control_first = 1U;   /* 首拍丢掉：从 sched_init 到第一次溢出没有测量意义 */
static volatile uint8_t s_flags = 0U;
static volatile uint32_t s_imu_mark_us = 0U;   /* IMU 标志置位时刻，主循环读 */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = (uint32_t)(SystemCoreClock / 1000000UL);
    if (s_cycles_per_us == 0U) {
        s_cycles_per_us = 1U;
    }
}

/* 微秒时间戳。CYCCNT 32 位、72 MHz 下 59.6 s 回绕；
   调用方一律用无符号减法求差，回绕不影响结果。 */
uint32_t sched_now_us(void)
{
    return (uint32_t)(DWT->CYCCNT / s_cycles_per_us);
}

uint32_t sched_imu_mark_us(void)
{
    return s_imu_mark_us;
}

/* 2 ms 采样拍：只读编码器、推进测速窗口，不做任何决策。
   它比控制拍优先级高，控制拍算到一半时也能按时进来。 */
void sched_sample_isr(void)
{
    AppState *st = state();
    uint32_t latency;
    uint32_t now_us;
    uint32_t dt_us;
    uint32_t t0;
    uint32_t elapsed;

    /* TIM4 也是 1 MHz 计数、ARR=1999，CNT 就是「本次溢出之后过了多少微秒」，
       即硬件中断进入本函数之前的全部延迟（含 HAL 回调链路）。 */
    latency = (uint32_t)TIM4->CNT;
    st->sample_isr_latency_us = latency;
    if (latency > st->sample_isr_latency_max_us) {
        st->sample_isr_latency_max_us = latency;
    }

    t0 = DWT->CYCCNT;
    now_us = sched_now_us();

    dt_us = now_us - s_last_sample_us;
    if ((dt_us == 0U) || (dt_us > SCHED_SAMPLE_JUMP_US)) {
        dt_us = (uint32_t)TASK_SAMPLE_MS * 1000UL;   /* 首拍或被长时间打断，退回标称值 */
    }
    s_last_sample_us = now_us;

    motion_update(now_us, dt_us);

#if (SPEED_LOOP_IN_SAMPLE_ISR != 0U)
    /* 速度环跟采样拍同频：测速窗就是按 2 ms 一节推进的，这里读到的永远是最新一段，
       不需要等 5 ms 控制拍。它跑在抢占 0 的中断里，执行时间由 speed_exec_us 单独打点。 */
    control_speed_task(dt_us);
#endif

    elapsed = (uint32_t)(DWT->CYCCNT - t0) / s_cycles_per_us;
    st->sample_exec_us = elapsed;
    if (elapsed > st->sample_exec_max_us) {
        st->sample_exec_max_us = elapsed;
    }

    /* IMU 的 SPI 读在主循环，这里只置标志：每 TASK_IMU_MS 置一次 */
    s_sample_div++;
    if ((s_sample_div % (TASK_IMU_MS / TASK_SAMPLE_MS)) == 0U) {
        s_imu_mark_us = now_us;
        s_flags |= SCHED_FLAG_IMU;
    }
}

/* 5 ms 控制拍：红外 → 循迹决策 → 速度环 → 电机。
   顺序不可调换：速度环必须用本拍刚更新的编码器值与刚算出的目标。 */
void sched_control_isr(void)
{
    AppState *st = state();
    uint32_t latency;
    uint32_t now_us;
    uint32_t dt_us;
    uint32_t t0;
    uint32_t elapsed;

    /* TIM7 同样 1 MHz 计数、ARR=4999，CNT 即进入本函数前的延迟（0..4999 µs） */
    latency = (uint32_t)TIM7->CNT;
    st->control_isr_latency_us = latency;
    if (latency > st->control_isr_latency_max_us) {
        st->control_isr_latency_max_us = latency;
    }

    t0 = DWT->CYCCNT;
    now_us = sched_now_us();

    dt_us = now_us - s_last_control_us;
    if (s_control_first != 0U) {
        s_control_first = 0U;           /* 首拍只建立时间基准，不参与 dt/dev 统计 */
        dt_us = SCHED_DT_FALLBACK_US;
    } else if ((dt_us == 0U) || (dt_us > (SCHED_DT_FALLBACK_US * 10UL))) {
        dt_us = SCHED_DT_FALLBACK_US;
    }
    s_last_control_us = now_us;

    /* 控制拍的真实间隔与标称 5 ms 的偏差，是时间轴抖动的直接度量：
       它一大，速度环的积分项就跟着偏，和「假设的 5 ms」不是一回事。 */
    st->control_dt_us = dt_us;
    {
        uint32_t deviation = (dt_us > SCHED_DT_FALLBACK_US)
                           ? (dt_us - SCHED_DT_FALLBACK_US)
                           : (SCHED_DT_FALLBACK_US - dt_us);
        if ((s_control_first == 0U) && (deviation > st->control_dt_dev_max_us)) {
            st->control_dt_dev_max_us = deviation;
        }
    }

    ir_read();
    ir_compute();
    key_tick();   /* 4 路按键扫描 + 15 ms 去抖，按下沿事件留给主循环消费 */
    /* 任务包统一出口：MISSION_TRACK 内部调 track_task，MISSION_SPIN 走陀螺转角，
       两者都通过 control_apply_targets 写目标，速度环在采样拍里读。 */
    mission_tick(dt_us);
    /* 战术包在 mission 之后跑：它要看到「上一段转角是否 DONE」再决定起下一段，
       放在前面会晚一拍，步进链大约慢 5 ms，可忽略但没必要。 */
    tactic_tick(dt_us);
    /* 转角自检排在最后：它要读到本拍 mission/tactic 的最终状态。
       不跑自检时这个函数第一句就返回，代价是一次比较。 */
    turntest_tick(dt_us);
#if (SPEED_LOOP_IN_SAMPLE_ISR == 0U)
    control_task(dt_us);
#endif

    elapsed = (uint32_t)(DWT->CYCCNT - t0) / s_cycles_per_us;
    st->control_exec_us = elapsed;
    if (elapsed > st->control_exec_max_us) {
        st->control_exec_max_us = elapsed;
    }
    if (elapsed > (uint32_t)SCHED_OVERRUN_US) {
        st->control_overrun++;
    }
    st->tick_count++;

    /* 再分频：每 4 拍（20 ms）遥测，每 20 拍（100 ms）刷屏 */
    s_control_div++;
    if ((s_control_div % (TASK_VOFA_MS / TASK_CONTROL_MS)) == 0U) {
        s_flags |= SCHED_FLAG_TELEMETRY;
    }
    if ((s_control_div % (TASK_OLED_MS / TASK_CONTROL_MS)) == 0U) {
        s_flags |= SCHED_FLAG_DISPLAY;
    }
}

void sched_init(void)
{
    dwt_init();
    s_sample_div = 0U;
    s_control_div = 0U;
    s_flags = 0U;
    s_last_sample_us = sched_now_us();
    s_last_control_us = s_last_sample_us;
}

uint8_t sched_take_flags(void)
{
    uint8_t flags;

    __disable_irq();
    flags = s_flags;
    s_flags = 0U;
    __enable_irq();
    return flags;
}
