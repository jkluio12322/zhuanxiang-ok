#include "tactic.h"
#include "config.h"
#include "state.h"
#include "mission.h"
#include "laps.h"
#include "imu.h"
#include "track.h"
#include "motion.h"

static TacticId s_id = TACTIC_TRACK;
static uint8_t s_running = 0U;
static float s_step_deg = TACTIC_STEP_DEG_DEFAULT;
static uint32_t s_gap_us = 0U;      /* 上一段转角完成后已等待的微秒数 */
static uint8_t s_steps = 0U;        /* 已完成步数（显示用） */

void tactic_init(void)
{
    s_id = TACTIC_TRACK;
    s_running = 0U;
    s_step_deg = TACTIC_STEP_DEG_DEFAULT;
    s_gap_us = 0U;
    s_steps = 0U;
}

const char *tactic_name(TacticId id)
{
    return (id == TACTIC_STEP_SPIN) ? "STEP" : "TRACK";
}

TacticId tactic_id(void)
{
    return s_id;
}

uint8_t tactic_running(void)
{
    return s_running;
}

void tactic_next(void)
{
    tactic_stop();
    s_id = (TacticId)(((uint16_t)s_id + 1U) % (uint16_t)TACTIC_COUNT);
    s_gap_us = 0U;
    s_steps = 0U;
}

void tactic_start(void)
{
    s_gap_us = 0U;
    s_steps = 0U;
    if (s_id == TACTIC_STEP_SPIN) {
        mission_start_spin(s_step_deg);
        /* 陀螺没标定好时 mission 会直接置 ABORTED，这时不要把 running 当成功 */
        s_running = (mission_status() == MISSION_ST_RUNNING) ? 1U : 0U;
    } else {
        mission_start_track();
        s_running = 1U;
    }
}

void tactic_stop(void)
{
    if (s_running != 0U) {
        mission_abort();
    }
    s_running = 0U;
    s_gap_us = 0U;
}

/* 战术 2 的步进链：一步转完（mission DONE）→ 等 TACTIC_STEP_GAP_MS → 再起一步。
   之所以不在 mission 内部循环，是因为「等多久、转多少度」属于战术参数，
   以后加第三个战术时 mission 不用改。 */
void tactic_tick(uint32_t dt_us)
{
    AppState *st = state();
    MissionStatus ms = mission_status();

    if (s_running != 0U) {
        if (s_id == TACTIC_TRACK) {
            /* 丢线超时、用户停车、圈数到（若开）都会让 mission 离开 TRACK */
            if (mission_id() != MISSION_TRACK) {
                s_running = 0U;
            }
        } else {
            if ((ms == MISSION_ST_TIMEOUT) || (ms == MISSION_ST_ABORTED)) {
                s_running = 0U;          /* 转角失败/被中止，整条链停下，不空转 */
            } else if ((mission_id() == MISSION_SPIN) && (ms == MISSION_ST_DONE)) {
                s_gap_us += dt_us;
                if (s_gap_us >= ((uint32_t)TACTIC_STEP_GAP_MS * 1000UL)) {
                    s_gap_us = 0U;
                    if (s_steps < 255U) {
                        s_steps++;
                    }
                    mission_start_spin(s_step_deg);
                }
            }
        }
    }

    st->tactic_id = (uint8_t)s_id;
    st->tactic_running = s_running;
    st->tactic_step_deg = s_step_deg;
    st->tactic_steps = s_steps;
}

void tactic_bump_primary(void)
{
    if (s_id == TACTIC_STEP_SPIN) {
        float d = s_step_deg + TACTIC_STEP_DEG_BUMP;

        /* 到顶就停在顶，不绕回 45：用户按出来的数只能被用户改。
           想回到小角度就按 K4（步长回 TACTIC_STEP_DEG_DEFAULT）。 */
        if (d > TACTIC_STEP_MAX_DEG) {
            d = TACTIC_STEP_MAX_DEG;
        }
        s_step_deg = d;
    } else {
        uint8_t t = (uint8_t)(laps_target() + TACTIC_LAP_STEP);
        laps_set_target(t);                /* 内部已经限幅并写 state->lap_target */
    }
}

/* 姿态清零 = 把「这一轮是怎么算出来的」全部推回上电状态：
   位姿（yaw）、计数（圈数/弯数）、以及两个 K3 能改的量（目标圈数、步长）。
   不动的只有里程以外的硬件状态：编码器累计与里程也在下面一起清，
   否则屏幕上 D 还挂着上一轮的数，看着像没清干净。 */
void tactic_reset_pose(void)
{
    tactic_stop();
    laps_reset();
    laps_set_target(LAP_DEFAULT_TARGET);   /* K3 在战术 1 加的是目标圈数，这里回默认 */
    state()->lap_count = 0U;
    imu_yaw_reset();
    s_step_deg = TACTIC_STEP_DEG_DEFAULT;  /* K3 在战术 2 加的是步长，这里回 45° */
    s_gap_us = 0U;
    s_steps = 0U;
    motion_reset_position();               /* 里程与编码器累计归零 */
}

void tactic_set_step_deg(float deg)
{
    if (deg < TACTIC_STEP_DEG_BUMP) {
        deg = TACTIC_STEP_DEG_BUMP;
    }
    if (deg > TACTIC_STEP_MAX_DEG) {
        deg = TACTIC_STEP_MAX_DEG;
    }
    s_step_deg = deg;
}

float tactic_step_deg(void)
{
    return s_step_deg;
}

uint8_t tactic_steps_done(void)
{
    return s_steps;
}