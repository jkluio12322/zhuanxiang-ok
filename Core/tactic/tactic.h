/* 战术包（tactic）层：决定「这一轮打什么战术、参数是多少」。
 *
 * 与 mission 的分工是刻意的：
 *   mission = 单个动作（循迹 / 原地转一个角度），只管执行和状态机；
 *   tactic  = 一串动作 + 可调参数（圈数目标、每步角度、步间等待），
 *             以及「哪个战术现在选着」。
 * 这样换战术只动这一个目录，mission / track / control 都不用碰。
 *
 * 当前两个战术：
 *   TACTIC_TRACK     —— 战术 1：常规循迹，K3 加一圈；
 *   TACTIC_STEP_SPIN —— 战术 2：转到目标角 → 停车等 TACTIC_STEP_GAP_MS → 再转一步，
 *                       步长由 tactic_step_deg() 给，默认 45°，K3 每次 +45°。
 */
#ifndef __TACTIC_H
#define __TACTIC_H

#include <stdint.h>

typedef enum {
    TACTIC_TRACK = 0,
    TACTIC_STEP_SPIN = 1,
    TACTIC_COUNT
} TacticId;

void tactic_init(void);

/* K2：切下一个战术包，同时把正在跑的动作停掉。 */
void tactic_next(void);

TacticId tactic_id(void);
const char *tactic_name(TacticId id);   /* "TRACK" / "STEP"，给 OLED 的 ASCII 行 */
uint8_t tactic_running(void);

/* K1：开始（战术 1 起循迹 / 战术 2 转第一步），重复调用先停再起。 */
void tactic_start(void);
/* K1：停止；任何战术都回待机。 */
void tactic_stop(void);

/* 5 ms 控制拍调用（sched 里在 mission_tick 之后）：负责战术 2 的步进链。 */
void tactic_tick(uint32_t dt_us);

/* K3：战术 1 目标圈数 +1；战术 2 每步角度 +TACTIC_STEP_DEG_BUMP。 */
void tactic_bump_primary(void);

/* K4：姿态清零 —— 圈数、圈内弯数、航向、步进计数全归零，并停车。 */
void tactic_reset_pose(void);

void tactic_set_step_deg(float deg);
float tactic_step_deg(void);
uint8_t tactic_steps_done(void);

#endif /* __TACTIC_H */