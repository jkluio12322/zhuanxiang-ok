/* 战术包：把「循迹」和「固定动作」都做成任务，主循环/串口只负责下发。
 *
 * 目前两个任务：
 *   MISSION_TRACK —— 常规红外循迹（内部调 track_task）
 *   MISSION_SPIN  —— 只看陀螺原地顺时针转固定角度，完全不看红外
 *
 * 为什么单独一层：循迹的 entry/exit、圈数、模式切换原本散在 console 和 track 里，
 * 加一个固定动作就得再改一次 track_task。抽出来后 track_task 只负责「在线时的转向」，
 * 「什么时候跑什么」由 mission 决定。
 */
#ifndef __MISSION_H
#define __MISSION_H

#include <stdint.h>

typedef enum {
    MISSION_IDLE = 0,
    MISSION_TRACK = 1,
    MISSION_SPIN = 2
} MissionId;

typedef enum {
    MISSION_ST_IDLE = 0,
    MISSION_ST_RUNNING = 1,
    MISSION_ST_DONE = 2,
    MISSION_ST_TIMEOUT = 3,
    MISSION_ST_ABORTED = 4
} MissionStatus;

void mission_init(void);

/* 开始循迹：清圈数、清循迹状态、切 TRACK 模式。 */
void mission_start_track(void);

/* 开始顺时针原地转角（度）。只依赖陀螺 yaw，红外在此任务里不参与决策。
   陀螺未标定完成时直接置 ABORTED，不空转。 */
void mission_start_spin(float deg_cw);

/* 立即中止：停车 + 状态置 ABORTED。 */
void mission_abort(void);

/* 每 5 ms 控制拍调用一次（sched 里唯一入口）。 */
void mission_tick(uint32_t dt_us);

MissionId mission_id(void);
MissionStatus mission_status(void);
float mission_spin_err_deg(void);
uint32_t mission_spin_ms(void);
uint8_t mission_spin_count(void);

/* 状态枚举转文本，供串口与 OLED 显示。 */
const char *mission_status_text(MissionStatus status);

#endif /* __MISSION_H */
