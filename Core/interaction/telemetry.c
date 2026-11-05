/* VOFA FireWater CSV telemetry: the 5 ms control ISR publishes a snapshot; the main loop
   telemetry_send() 临界区拷副本+清 fresh 再 snprintf 发送（格式化 8 个字段要
   100~300 µs，放进中断会压住串口丢字节）。一致性靠临界区整体拷贝，fresh 只回答"有无
   新数据"。读 motion_view()/imu_get() 只读视图是只读跨层，避免填遥测传 10 个参数。 */

#include "telemetry.h"
#include "config.h"
#include "imu.h"
#include "motion.h"
#include "serial.h"
#include "state.h"
#include "loop_test.h"

#include <stdio.h>
#include <string.h>

/* 遥测快照本体。写它的只有本文件的 Publish()，读它的有 Send() 和显示模块。 */
telemetryData_t telemetry_data;

/* Format the latest snapshot as one numeric CSV row for the FireWater parser. */
void telemetry_send(void)
{
    telemetryData_t snapshot;

    if (!state_view()->vofa) return;

    if (!telemetry_data.fresh) return;

    if (state_view()->mode == modeLoopTest) {
        LoopTestView_t test;
        char test_buffer[TELEMETRY_BUFFER_SIZE];
        int test_length;
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        test = *loop_test_view();
        telemetry_data.fresh = 0;
        __set_PRIMASK(primask);

        /* LOOP-TEST columns:
           angleTargetL,angleL,angleTargetR,angleR,tgtRpmL,rpmL,tgtRpmR,rpmR,
           pwmL,pwmR,posErrL,posErrR. */
        test_length = snprintf(
            test_buffer, sizeof(test_buffer),
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f,%.2f\r\n",
            (double)test.target_angle_deg[0],
            (double)test.actual_angle_deg[0],
            (double)test.target_angle_deg[1],
            (double)test.actual_angle_deg[1],
            (double)test.target_rpm[0],
            (double)test.actual_rpm[0],
            (double)test.target_rpm[1],
            (double)test.actual_rpm[1],
            (double)test.pwm[0],
            (double)test.pwm[1],
            (double)test.position_error_deg[0],
            (double)test.position_error_deg[1]);
        if ((test_length > 0) && (test_length < (int)sizeof(test_buffer))) {
            serial_write((const uint8_t *)test_buffer, (uint16_t)test_length);
        }
        return;
    }

    /* 整体拷贝 + 清标志放在同一个临界区里，之后一律从副本格式化。
       顺序很关键：先清标志再逐个读字段的话，标志根本保护不了快照。 */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        snapshot = telemetry_data;
        telemetry_data.fresh = 0;
        __set_PRIMASK(primask);
    }

    char buffer[TELEMETRY_BUFFER_SIZE];
    int length;

    /* 8-channel CSV: target left/right, filtered left/right, steering,
       raw speed difference, yaw, track phase. */
    length = snprintf(buffer, sizeof(buffer),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\r\n",
        snapshot.targetLeftRpm,
        snapshot.filteredLeftRpm,
        snapshot.targetRightRpm,
        snapshot.filteredRightRpm,
        snapshot.steerRpm,
        snapshot.actualDiffRpm,
        snapshot.yawAngleDeg,
        (int)snapshot.turnPhase);

    if (length > 0 && length < (int)sizeof(buffer)) {
        serial_write((const uint8_t *)buffer, (uint16_t)length);
    }
}

/* 用本周期控制结果填充遥测快照（中断上下文，5 ms 控制任务调用，只赋值不格式化）。
   参数单位 RPM/百分比；速度与姿态字段直接读只读视图，其余由调用方传入。最后置 fresh=1
   是通知主循环"有新快照可发"的唯一入口。commandVoltage 是 PWM 百分比按标称电压换算的
   近似值，仅供观察趋势（本工程无母线电压采样）。 */
void telemetry_publish(float targetLeft, float targetRight,
                          int16_t leftPwm, int16_t rightPwm,
                          float steerRpm, float trackError,
                          int16_t turnPhase, float rpmDifference)
{
    telemetry_data.targetLeftRpm = targetLeft;
    telemetry_data.targetRightRpm = targetRight;
    telemetry_data.actualLeftRpm = motion_view()->left_rpm_raw;
    telemetry_data.actualRightRpm = motion_view()->right_rpm_raw;
    telemetry_data.filteredLeftRpm = motion_view()->left_rpm;
    telemetry_data.filteredRightRpm = motion_view()->right_rpm;
    telemetry_data.steerRpm = steerRpm;
    telemetry_data.actualDiffRpm = rpmDifference;
    telemetry_data.yawAngleDeg = imu_get()->yaw;
    telemetry_data.trackError = trackError;
    telemetry_data.mode = (int16_t)state_view()->mode;
    telemetry_data.runFlag = state_view()->running;
    telemetry_data.gear = (int16_t)state_view()->gear;
    telemetry_data.turnPhase = turnPhase;
    telemetry_data.commandVoltageLeft =
        (float)leftPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetry_data.commandVoltageRight =
        (float)rightPwm * MOTOR_SUPPLY_VOLTAGE / 100.0f;
    telemetry_data.fresh = 1;
}

/* Keep the startup line numeric so it is also a valid FireWater frame. */
void telemetry_boot_header(void)
{
    serial_printf("0,0,0,0,0,0,0,0\r\n");
}

/* 初始化遥测快照：全部字段清零，保证首次发送前数据有效。 */
void telemetry_init(void)
{
    memset(&telemetry_data, 0, sizeof(telemetry_data));
}
