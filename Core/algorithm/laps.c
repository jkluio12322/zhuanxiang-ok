#include "laps.h"
#include "config.h"
#include "state.h"

/* 直角弯个数 → 圈：赛道是矩形，4 个弯算一圈。只在本文件私有，不进 config.h。 */
#define LAPS_CORNERS_PER_LAP  4U

static uint8_t s_count = 0U;      /* 已完成圈数 */
static uint8_t s_corners = 0U;    /* 当前圈内已过的弯数 */
static uint8_t s_target = LAP_DEFAULT_TARGET;
static uint8_t s_confirm = 0U;
static uint8_t s_in_corner = 0U;
static uint16_t s_gap_ticks = 0U;  /* 距上一个弯的拍数（不是毫秒） */

void laps_init(uint8_t target)
{
    s_count = 0U;
    s_corners = 0U;
    s_target = (target > LAP_MAX_TARGET) ? LAP_MAX_TARGET : target;
    s_confirm = 0U;
    s_in_corner = 0U;
    s_gap_ticks = 0U;
}

void laps_reset(void)
{
    s_count = 0U;
    s_corners = 0U;
    s_confirm = 0U;
    s_in_corner = 0U;
    s_gap_ticks = 0U;
}

void laps_update(uint8_t corner_now)
{
    AppState *st = state();

    if (s_gap_ticks < TICKS_OF_MS(LAP_CORNER_GAP_MS)) {
        s_gap_ticks++;
    }

    if (corner_now != 0U) {
        if (s_in_corner == 0U) {
            s_confirm++;
            if ((s_confirm >= LAP_CORNER_CONFIRM) && (s_gap_ticks >= TICKS_OF_MS(LAP_CORNER_GAP_MS))) {
                s_in_corner = 1U;
                s_confirm = 0U;
                s_gap_ticks = 0U;
                s_corners++;
                if (s_corners >= LAPS_CORNERS_PER_LAP) {
                    s_corners = 0U;
                    if (s_count < LAP_MAX_TARGET) {
                        s_count++;
                    }
                }
                st->lap_count = s_count;
#if (LAP_STOP_ENABLE != 0U)
                /* 跑够目标圈数自动停车。默认关闭：圈数只做计数与显示，
                   免得弯道识别漏一拍就把车停在不该停的地方。 */
                if (s_count >= s_target) {
                    state_stop(STOP_LAP_DONE);
                }
#endif
            }
        }
    } else {
        s_confirm = 0U;
        s_in_corner = 0U;
    }

    st->lap_count = s_count;
}

uint8_t laps_count(void)
{
    return s_count;
}

uint8_t laps_target(void)
{
    return s_target;
}

void laps_set_target(uint8_t target)
{
    if (target == 0U) {
        target = 1U;
    }
    s_target = (target > LAP_MAX_TARGET) ? LAP_MAX_TARGET : target;
    state()->lap_target = s_target;
}
