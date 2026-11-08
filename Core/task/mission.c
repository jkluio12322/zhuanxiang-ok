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
#include "mission.h"
#include "config.h"
#include "state.h"
#include "track.h"
#include "laps.h"
#include "imu.h"
#include "control.h"
#include "types.h"
#include <math.h>

static MissionId s_id = MISSION_IDLE;
static MissionStatus s_status = MISSION_ST_IDLE;
static float s_spin_target = 0.0f;
static uint32_t s_spin_ms = 0U;      /* 本步已耗时（只做诊断显示，不做判据） */
static uint8_t s_spin_count = 0U;
static float s_spin_best_err = 0.0f;  /* 本步到达过的最小 |误差| */
static uint32_t s_spin_stall_ms = 0U; /* 距上次「更靠近目标」已过去多久 */
/* V2 原样的跨步补偿：把上一步的残差并进本步的提前量（lead），
   限 ±5 度，防止某一步被外力推歪后连带放大。 */
static uint8_t s_spin_lead_valid = 0U;

void mission_init(void)
{
    s_id = MISSION_IDLE;
    s_status = MISSION_ST_IDLE;
    s_spin_target = 0.0f;
    s_spin_ms = 0U;
    s_spin_count = 0U;
    s_spin_lead_valid = 0U;
    s_spin_best_err = 0.0f;
    s_spin_stall_ms = 0U;
}

void mission_start_track(void)
{
    laps_reset();
    track_reset();
    state_start_run();
    state()->lap_target = laps_target();   /* state_start_run 会把 lap_count 清零，目标保留 */
    s_id = MISSION_TRACK;
    s_status = MISSION_ST_RUNNING;
}

void mission_start_spin(float deg_cw)
{
    const imu_data_t *imu = imu_get();

    if ((imu == 0) || (imu->ok == 0U)) {
        s_id = MISSION_SPIN;
        s_status = MISSION_ST_ABORTED;
        return;
    }

    /* 绝对累计：以上一步的「目标角」为基准继续往前推，实测残差一律不并进下一步。
       以前的写法把残差累进 lead（限幅 ±5°）当作「上一步少转了」，结果每步都比
       指令多转一点、限幅吃满后每步固定多转 5°，16 步累积成 40~50°。
       这里残差不参与，步距永远是设定值；车若某步差 0.4°，那 0.4° 就留在原地
       不叠加。若 yaw 被清零过（串口 y 或姿态复位），基准角与当前角会跳开很远，
       那时自动重新以当前角为基准。 */
    {
        float base = imu->yaw;

        if ((s_spin_lead_valid != 0U) &&
            (fabsf(track_wrap180(s_spin_target - imu->yaw)) < 90.0f)) {
            base = s_spin_target;
        }
        s_spin_lead_valid = 1U;
        s_spin_target = track_wrap180(base - deg_cw);
    }

    track_turn_begin(s_spin_target);
    control_set_manual(0.0f, 0.0f);   /* 切 MANUAL：track_task 首句就 return，红外失效 */
    s_spin_ms = 0U;
    s_spin_best_err = 999.0f;
    s_spin_stall_ms = 0U;
    s_status = MISSION_ST_RUNNING;
    s_id = MISSION_SPIN;
}

void mission_abort(void)
{
    control_apply_targets(0.0f, 0.0f);
    state_stop(STOP_USER);
    s_status = MISSION_ST_ABORTED;
    s_id = MISSION_IDLE;
    s_spin_lead_valid = 0U;
}

/* 控制拍里唯一入口：先同步状态到 AppState（显示/遥测只读），再按任务推进。 */
void mission_tick(uint32_t dt_us)
{
    AppState *st = state();
    uint32_t dt_ms = dt_us / 1000U;

    if (dt_ms == 0U) {
        dt_ms = 1U;
    }

    st->mission_id = (uint8_t)s_id;

    switch (s_id) {
    case MISSION_TRACK:
        if (s_status == MISSION_ST_RUNNING) {
            track_task(dt_us);
            if (st->stop_reason != STOP_NONE) {
                /* 圈数到 = 正常完成，其它停车原因 = 被中止 */
                s_status = (st->stop_reason == STOP_LAP_DONE)
                         ? MISSION_ST_DONE : MISSION_ST_ABORTED;
            }
        }
        break;

    case MISSION_SPIN:
        if (s_status == MISSION_ST_RUNNING) {
            s_spin_ms += dt_ms;
            if (track_turn_step(dt_us) != 0U) {
                control_apply_targets(0.0f, 0.0f);
                if (s_spin_count < 255U) {
                    s_spin_count++;
                }
                s_status = MISSION_ST_DONE;
            } else {
                /* 放弃判据是物理量：连续 MISSION_SPIN_STALL_MS 没有更靠近目标
                   （堵转、陀螺没数据）。没有到点判死的时钟。 */
                float err_abs = fabsf(track_turn_err_deg());

                if (err_abs < (s_spin_best_err - MISSION_SPIN_PROGRESS_DEG)) {
                    s_spin_best_err = err_abs;
                    s_spin_stall_ms = 0U;
                } else if (s_spin_stall_ms < 0xFFFFFFFFU) {
                    s_spin_stall_ms += dt_ms;
                }

                if (s_spin_stall_ms >= MISSION_SPIN_STALL_MS) {
                    control_apply_targets(0.0f, 0.0f);
                    s_status = MISSION_ST_TIMEOUT;
                }
            }
        }
        break;

    default:
        break;
    }

    st->mission_status = (uint8_t)s_status;
    st->spin_ms = s_spin_ms;
    st->spin_count = s_spin_count;
    st->spin_stall_ms = s_spin_stall_ms;
    if (s_id == MISSION_SPIN) {
        st->spin_err_deg = track_turn_err_deg();
    }
}

MissionId mission_id(void)
{
    return s_id;
}

MissionStatus mission_status(void)
{
    return s_status;
}

float mission_spin_err_deg(void)
{
    return track_turn_err_deg();
}

uint32_t mission_spin_ms(void)
{
    return s_spin_ms;
}

uint8_t mission_spin_count(void)
{
    return s_spin_count;
}

const char *mission_status_text(MissionStatus status)
{
    switch (status) {
    case MISSION_ST_IDLE:    return "IDLE";
    case MISSION_ST_RUNNING: return "RUN";
    case MISSION_ST_DONE:    return "DONE";
    case MISSION_ST_TIMEOUT: return "FAIL";
    case MISSION_ST_ABORTED: return "STOP";
    default:                 return "?";
    }
}