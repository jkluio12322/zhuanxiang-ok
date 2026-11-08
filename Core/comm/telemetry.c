/* VOFA+ 遥测：FireWater 协议，一行 name:value,name:value,...\n。
 * 通道顺序固定（顺序一变，已保存的 VOFA 布局会错位）：
 *   lt = 左轮目标 RPM   la = 左轮实际 RPM
 *   rt = 右轮目标 RPM   ra = 右轮实际 RPM
 *   yaw = 航向角 deg
 *   st  = 转向量 -1..1（整形前的 steer，正=右转）
 *   gy  = 车体 +Z 角速度 deg/s（右转为负）
 * 频率 TASK_VOFA_MS = 20 ms（50 Hz）。
 *
 * 为什么不用 snprintf("%.1f")：Keil ARMCC 的浮点格式化会把 printf 库的
 * 完整浮点分支链进固件（约 +6 KB Flash），而且单个 %f 在 72 MHz 上要
 * 100~300 µs；这段代码在主循环被触发，必须短。这里改定点手拼：
 * 先按 10 倍/100 倍取整成 int32，再逐位输出数字，单帧约 20 µs。 */

#include "telemetry.h"
#include "config.h"
#include "serial.h"
#include "imu.h"
#include "control.h"
#include "state.h"

/* 追加无符号十进制，返回新的写指针 */
static char *put_uint(char *p, uint32_t value)
{
    char tmp[10];
    uint8_t n = 0U;

    do {
        tmp[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (n > 0U) {
        *p++ = tmp[--n];
    }
    return p;
}

/* 定点输出：scale=10 保留 1 位小数，scale=100 保留 2 位，四舍五入 */
static char *put_fixed(char *p, float value, uint32_t scale)
{
    int32_t scaled;
    uint32_t frac;

    scaled = (int32_t)((value >= 0.0f) ? (value * (float)scale + 0.5f)
                                       : (value * (float)scale - 0.5f));
    if (scaled < 0) {
        *p++ = '-';
        scaled = -scaled;
    }

    p = put_uint(p, (uint32_t)(scaled / (int32_t)scale));
    *p++ = '.';
    frac = (uint32_t)(scaled % (int32_t)scale);
    if ((scale >= 100U) && (frac < 10U)) {
        *p++ = '0';
    }
    return put_uint(p, frac);
}

static char *put_name(char *p, const char *name, uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++) {
        *p++ = name[i];
    }
    *p++ = ':';
    return p;
}

void telemetry_task(void)
{
    const imu_data_t *imu = imu_get();
    const AppState *st = state();
    const Pid *pl = control_pid_left();
    const Pid *pr = control_pid_right();
    char line[TELEMETRY_TEXT_SIZE];
    char *p = line;
    float lt = 0.0f;
    float la = 0.0f;
    float rt = 0.0f;
    float ra = 0.0f;
    float yaw = 0.0f;

    if (imu != 0) {
        yaw = imu->yaw;
    }
    if (pl != 0) {
        lt = pl->target;
        la = pl->actual;
    }
    if (pr != 0) {
        rt = pr->target;
        ra = pr->actual;
    }

    p = put_name(p, "lt", 2U);
    p = put_fixed(p, lt, 10U);
    *p++ = ',';
    p = put_name(p, "la", 2U);
    p = put_fixed(p, la, 10U);
    *p++ = ',';
    p = put_name(p, "rt", 2U);
    p = put_fixed(p, rt, 10U);
    *p++ = ',';
    p = put_name(p, "ra", 2U);
    p = put_fixed(p, ra, 10U);
    *p++ = ',';
    p = put_name(p, "yaw", 3U);
    p = put_fixed(p, yaw, 100U);
    *p++ = ',';
    p = put_name(p, "st", 2U);
    p = put_fixed(p, st->steer, 100U);
    *p++ = ',';
    p = put_name(p, "gy", 2U);
    p = put_fixed(p, st->gyro_z_dps, 10U);
    *p++ = '\n';

    serial_write_bytes((const uint8_t *)line, (uint16_t)(p - line));
}

void telemetry_banner(void)
{
    serial_write("BlueCar ready. '?' for commands.\n");
}
