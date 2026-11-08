/* BlueCar 程序入口。
 *
 * 任务分配（没有 RTOS，中断 + 主循环两段式）：
 *   TIM4 2 ms 中断 → sched_sample_isr()：只读编码器、推进测速窗
 *   TIM7 5 ms 中断 → sched_control_isr()：红外 → 循迹 → 速度环 → 电机
 *   主循环         → IMU 采样（SPI 忙等）、VOFA 发送、OLED 刷新、串口命令行
 * 两个时基由硬件各自触发，采样拍（抢占 0）随时可以插进控制拍（抢占 1）之间，
 * 不再靠「一个 1 kHz 取模」把两档节拍绑死在一起。
 * 中断里绝不做 snprintf 和 I2C：那会把控制周期拖长，也会压住串口接收。
 */

#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "board.h"
#include "config.h"
#include "state.h"
#include "sched.h"
#include "timer.h"
#include "serial.h"
#include "console.h"
#include "telemetry.h"
#include "pidtrace.h"
#include "display.h"
#include "mission.h"
#include "tactic.h"
#include "turntest.h"
#include "motion.h"
#include "encoder.h"
#include "irtrack.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "buzzer.h"
#include "track.h"
#include "control.h"
#include "key.h"

/* 独立看门狗：LSI 40 kHz 经 64 分频 = 625 Hz，625 计数约 1.0 s。
   直接写寄存器而不用 HAL：F1 的 HAL_IWDG 模块在本工程未启用（hal_conf 里被注释），
   为了一个只有 5 行的功能去改 CubeMX 配置头不划算。 */
#define IWDG_KEY_RELOAD   0x0000AAAAU
#define IWDG_KEY_ENABLE   0x0000CCCCU
#define IWDG_KEY_WRITE    0x00005555U

static void watchdog_init(void);
static void watchdog_feed(void);
static void key_handle(void);

void SystemClock_Config(void);

int main(void)
{
    AppState *st;
    uint8_t flags;

    HAL_Init();
    SystemClock_Config();

    /* 时钟安全系统：HSE 一失效立刻切 HSI 并进 NMI，NMI 里停车。
       F1 的 HAL 没有 __HAL_RCC_CSS_ENABLE 宏，只能动 CR 位。 */
    SET_BIT(RCC->CR, RCC_CR_CSSON);

    MX_GPIO_Init();
    MX_DMA_Init();   /* DMA1 时钟必须先开：串口 RX（Ch5）、OLED TX（Ch4）全靠它，漏调则 DMA 寄存器恒为 0 */
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM7_Init();
    MX_TIM5_Init();
    MX_I2C2_Init();
    oled_dma_init();  /* I2C2_TX 绑到 DMA1_Channel4，必须在 I2C2 初始化之后 */
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();

    /* OLED 初始化是非阻塞状态机，这里等它走完（约 150 ms），不占用 HAL_Delay */
    buzzer_init();
    buzzer_on();
    oled_init_begin();
    while (oled_init_poll() == 0U) {
        oled_recover_poll();
    }
    buzzer_off();

    motor_init();
    encoder_init();
    motion_init();
    ir_init();
    imu_init();

    state_init();
    track_init();
    control_init();
    mission_init();
    key_init();

    serial_init(&huart1);
    console_init();
    telemetry_banner();
    pidtrace_init();

    /* 顺序要求：sched_init 先装好 DWT，再启动两个时基，否则第一拍就进未初始化状态 */
    sched_init();
    timer_sched_init();
    timer_ms_init();

    display_boot();

    /* 看门狗放在最后启动：前面 OLED 初始化要等约 150 ms，不该被它盯着 */
    watchdog_init();

#if (MISSION_SELFTEST_ENABLE != 0U)
    /* 上电自检：不跑赛道，直接原地转一个已知角度再停。
       用来验证「陀螺闭环转向」这条链路——如果转出来的角度明显不是设定值，
       问题一定在 IMU 轴向/符号标定，而不是循迹参数。 */
    mission_start_spin(MISSION_ANGLE_DEG_DEFAULT);
#endif

    st = state();

    static uint8_t s_loop_seen = 0U;   /* 首圈从复位时刻算起，必然是几秒，不能计入最大 */

    while (1) {
        uint32_t loop_start_us = sched_now_us();
        uint32_t loop_us;

        flags = sched_take_flags();

        if ((flags & SCHED_FLAG_IMU) != 0U) {
            /* 从标志置位到现在多久：这个延迟才是「IMU 采样周期抖动」的真凶，
               它由主循环里的 OLED / 串口 / 命令解析决定，不是定时器决定的 */
            uint32_t imu_lat = sched_now_us() - sched_imu_mark_us();
            st->imu_latency_us = imu_lat;
            if (imu_lat > st->imu_latency_max_us) {
                st->imu_latency_max_us = imu_lat;
            }
            imu_task();          /* SPI 忙等约 25 us，放在主循环 */
        }
        if ((flags & SCHED_FLAG_TELEMETRY) != 0U) {
            telemetry_task();    /* PID 目标/实际 + yaw → VOFA */
        }
        if ((flags & SCHED_FLAG_DISPLAY) != 0U) {
            display_task();      /* 8 路红外 / 三角度 / 圈数 / PID */
        }

        pidtrace_task();         /* 转向环采集：按控制拍分频到 50 Hz，整帧入队 */
        turntest_task();         /* 转角自检结果：中断只攒队列，格式化与发送都在这里 */
        key_handle();            /* 按键事件：K1 切模式 / K2 停车 / K3 丢线策略 / K4 yaw 清零 */
        serial_task();           /* 推进发送队列 */
        console_task();          /* 串口命令解析 */
        oled_flush();            /* I2C 非阻塞状态机，只推进不等待 */
        oled_recover_poll();

        /* 主循环单圈耗时：从取标志到本圈收尾。它一大，IMU 标志从置位到执行的
           延迟就跟着大，所以超过 1 ms 的圈数单独计数，不取平均。 */
        loop_us = sched_now_us() - loop_start_us;
        st->loop_period_us = loop_us;
        if (s_loop_seen == 0U) {
            s_loop_seen = 1U;
        } else if (loop_us > st->loop_period_max_us) {
            st->loop_period_max_us = loop_us;
        }
        if (loop_us > 1000U) {
            st->loop_over_1ms++;
        }

        watchdog_feed();
    }
}

/* 按键动作都在主循环里做：中断只负责生成「按下沿」事件，避免在 5 ms 中断里
   调用 mission / track 这些会改大量状态、还会写电机目标的模块。 */
static void key_handle(void)
{
    uint8_t k = key_take_press();

    if (k == 0U) {
        return;
    }

    if ((k & KEY_BIT(KEY_1)) != 0U) {
        /* K1 开始/停止：正在跑就停（等效串口 x），停着就按当前战术起跑（等效 s）。 */
        if (tactic_running() != 0U) {
            tactic_stop();
        } else {
            tactic_start();
        }
    }

    if ((k & KEY_BIT(KEY_2)) != 0U) {
        /* K2 切战术包：循迹 ↔ 步进转角，切的同时把正在跑的动作停掉 */
        tactic_next();
    }

    if ((k & KEY_BIT(KEY_3)) != 0U) {
        /* K3 加量：战术 1 目标圈数 +1；战术 2 每步角度 +45° */
        tactic_bump_primary();
    }

    if ((k & KEY_BIT(KEY_4)) != 0U) {
        /* K4 姿态清零：停车 + 圈数清零 + 航向归零 + 步进计数清零 */
        tactic_reset_pose();
    }
}

static void watchdog_init(void)
{
    IWDG->KR = IWDG_KEY_WRITE;    /* 允许改 PR/RLR */
    IWDG->PR = 0x04U;             /* 64 分频 */
    IWDG->RLR = 625U;             /* 625 / 625 Hz = 1.0 s */
    IWDG->KR = IWDG_KEY_RELOAD;
    IWDG->KR = IWDG_KEY_ENABLE;   /* 启动后不可关闭：仿真器断点会触发复位 */
}

static void watchdog_feed(void)
{
    IWDG->KR = IWDG_KEY_RELOAD;
}

/* 8 MHz 外部晶振经 PLL 九倍频得到 72 MHz；APB1 二分频 36 MHz，APB2 保持 72 MHz。 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
