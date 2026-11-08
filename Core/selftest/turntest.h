/* 转角自检（测试代码，独立目录，不属于固件功能）。
 *
 * 做一件事：按「战术包 2」的时序（转一步 → 停 TACTIC_STEP_GAP_MS → 再转）
 * 连续走 N 圈，在每一段的停车窗口里读八路红外，判定车是否正好停在黑线上。
 *
 * 判据（实地标定过，不要改）：
 *   - 累计角度是 90° 的整数倍 → 车头正对十字臂 → 图案必须是 00011000（只有中间两路黑）
 *   - 累计角度不是 90° 的整数倍（如 45°）→ 车头对着空白 → 图案必须是 00000000（全白）
 *   图样字符串索引 0 = 物理最左，'1' = 压黑线，与 OLED 第一段显示同一套约定。
 *
 * 为什么判定放在固件里：停车窗口只有 2 s，人工逐步发命令对不上窗口；
 * 而且判定要读的 raw[8] 不进采集帧，放上位机就得改帧格式。
 *
 * 输出走串口（UTF-8，115200），每步一行 + 跑完一行汇总；
 * 中断里只打点进队列，snprintf 全部在主循环里做。
 */
#ifndef __TURNTEST_H
#define __TURNTEST_H

#include <stdint.h>

/* 开始自检：step_deg = 每步转的角度，laps = 圈数。已在跑则先停掉再重开。 */
void turntest_start(float step_deg, uint16_t laps);

/* 中止自检（停车，不再起下一步）。 */
void turntest_abort(void);

/* 5 ms 控制拍里调：推进「起转 → 等到位 → 停车窗口判定 → 起下一步」。 */
void turntest_tick(uint32_t dt_us);

/* 主循环里调：把中断里攒下的结果行格式化后发到串口。 */
void turntest_task(void);

/* 是否在跑（OLED/命令回显用来判断要不要忽略别的动作）。 */
uint8_t turntest_active(void);

#endif /* __TURNTEST_H */