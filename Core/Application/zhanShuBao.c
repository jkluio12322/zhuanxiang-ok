/**
 * @file    zhanShuBao.c
 * @brief   战术包实现，提供可复用的九十度转向动作。
 */

#include "zhanShuBao.h"
#include "yingYongPeiZhi.h"
#include "yingYongZhuangTai.h"
#include "irtrack.h"

/* ------------------------------------------------------------------------- */
/* 九十度转向常调参数                                                        */
/* ------------------------------------------------------------------------- */

/* 编码器累计计数差达到该值，认为车体已经转过约 90 度。
 * 1016 为理论值，当前使用 1100，仅保留少量打滑和安装误差余量。
 * 转不够就增大，转过头就减小。 */
#define TRACK_CORNER_TURN_90_DIFF_COUNTS 2000

/* 九十度转向时外侧轮目标转速，单位 RPM，正值表示前进。
 * 右转时外侧轮是左轮，左转时外侧轮是右轮。 */
#define TRACK_CORNER_PIVOT_OUTER_RPM 30.0f

/* 九十度转向时内侧轮目标转速，单位 RPM；当前为 0，表示内侧轮停止。
 * 右转时内侧轮是右轮，左转时内侧轮是左轮。 */
#define TRACK_CORNER_PIVOT_INNER_RPM (-30.0f)

/* 八路红外全部压到黑线时，每一路 raw 的物理状态都为 0。 */
#define ZHAN_SHU_BAO_HEI_XIAN_RAW_MIE 0U

/* 八路红外全部丢线时，每一路 raw 的物理状态都为 1。 */
#define ZHAN_SHU_BAO_DIU_XIAN_RAW_BAI 1U

/* 当前正在执行的九十度转向方向，0 表示当前没有转向动作。 */
static int8_t zhanShuBaoZhuanXiang = 0;

/* 九十度转向开始瞬间锁存的左右编码器累计计数。 */
static int64_t zhanShuBaoQiShiZuoJiShu = 0;
static int64_t zhanShuBaoQiShiYouJiShu = 0;

/* 九十度转向已经运行的循迹周期数，用于避开启动瞬间的无效计数。 */
static uint16_t zhanShuBaoYunXingZhouQi = 0U;

/**
 * @brief  将浮点数限制在指定范围内。
 * @param  shuZhi 待限制的数值。
 * @param  zuiXiaoZhi 允许的最小值。
 * @param  zuiDaZhi 允许的最大值。
 * @return 限制后的数值。
 */
static float zhanShuBaoXianZhi(float shuZhi,
                               float zuiXiaoZhi,
                               float zuiDaZhi)
{
    if (shuZhi > zuiDaZhi) {
        return zuiDaZhi;
    }
    if (shuZhi < zuiXiaoZhi) {
        return zuiXiaoZhi;
    }
    return shuZhi;
}

/**
 * @brief  清除九十度转向的内部状态。
 * @note   仅清除战术包内部的锁存值，不修改 PID 和编码器硬件计数。
 */
void zhanShuBaoChongZhiZhuan90Du(void)
{
    zhanShuBaoZhuanXiang = 0;
    zhanShuBaoQiShiZuoJiShu = 0;
    zhanShuBaoQiShiYouJiShu = 0;
    zhanShuBaoYunXingZhouQi = 0U;
}

/**
 * @brief  执行一次基于编码器的九十度转向。
 * @param  zhuanXiang 转弯方向，正值为右转，负值为左转。
 * @return 0 表示仍在转向，1 表示已经完成并停止。
 * @note   右转时左轮是外侧并正转，右轮是内侧；左转时方向相反。
 *         当前内侧轮参数为 0，表示内侧轮停止。该函数绕过普通循迹的
 *         正转下限，因此参数改为负值时也可以让内侧轮真正反转。
 */
uint8_t zhanShuBaoZhuan90Du(int8_t zhuanXiang)
{
    int64_t jiShuCha;
    float muBiaoZuo;    /* 左轮目标转速。 */
    float muBiaoYou;    /* 右轮目标转速。 */

    if (zhuanXiang == 0) {
        zhanShuBaoChongZhiZhuan90Du();
        targetLeftRpm = 0.0f;
        targetRightRpm = 0.0f;
        turnTarget = 0.0f;
        return 1U;
    }

    if (zhanShuBaoZhuanXiang != zhuanXiang) {
        zhanShuBaoZhuanXiang = zhuanXiang;
        zhanShuBaoQiShiZuoJiShu = encoderLeft.counter.count_total;
        zhanShuBaoQiShiYouJiShu = encoderRight.counter.count_total;
        zhanShuBaoYunXingZhouQi = 0U;
    }

    zhanShuBaoYunXingZhouQi++;

    jiShuCha =
        (encoderLeft.counter.count_total - zhanShuBaoQiShiZuoJiShu)
        - (encoderRight.counter.count_total - zhanShuBaoQiShiYouJiShu);

    if (jiShuCha < 0) {
        jiShuCha = -jiShuCha;
    }

    if ((zhanShuBaoYunXingZhouQi >= TRACK_CORNER_MIN_TICKS)
        && (jiShuCha >= (int64_t)TRACK_CORNER_TURN_90_DIFF_COUNTS)) {
        zhanShuBaoChongZhiZhuan90Du();
        targetLeftRpm = 0.0f;
        targetRightRpm = 0.0f;
        turnTarget = 0.0f;
        return 1U;
    }

    if (zhuanXiang > 0) {
        muBiaoZuo = TRACK_CORNER_PIVOT_OUTER_RPM; /* 右转：左轮前进。 */
        muBiaoYou = TRACK_CORNER_PIVOT_INNER_RPM; /* 右转：右轮停止。 */
    } else {
        muBiaoZuo = TRACK_CORNER_PIVOT_INNER_RPM; /* 左转：左轮停止。 */
        muBiaoYou = TRACK_CORNER_PIVOT_OUTER_RPM; /* 左转：右轮前进。 */
    }

    targetLeftRpm = zhanShuBaoXianZhi(muBiaoZuo, /* 逻辑左轮输出。 */
                                      -SPEED_TARGET_LIMIT_RPM,
                                      SPEED_TARGET_LIMIT_RPM);
    targetRightRpm = zhanShuBaoXianZhi(muBiaoYou, /* 逻辑右轮输出。 */
                                       -SPEED_TARGET_LIMIT_RPM,
                                       SPEED_TARGET_LIMIT_RPM);
    turnTarget = (targetLeftRpm - targetRightRpm) * 0.5f;

    return 0U;
}

/**
 * @brief  判断八路红外是否全部检测到黑线。
 * @return 1 表示八路 raw 全部为 0，0 表示尚未全部压线。
 * @note   该判断对应“遇到黑线时八路灭”的状态，调用前需要先执行
 *         IRTrack_Read() 更新 raw 数组。
 */
uint8_t zhanShuBaoYuDaoHeiXian(void)
{
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i] != ZHAN_SHU_BAO_HEI_XIAN_RAW_MIE) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  判断八路红外是否全部丢失黑线。
 * @return 1 表示八路 raw 全部为 1，0 表示仍有通道检测到黑线。
 * @note   调用前需要先执行 IRTrack_Read() 更新 raw 数组。
 */
uint8_t zhanShuBaoDiuXian(void)
{
    for (uint8_t i = 0U; i < IRTRACK_CHANNELS; i++) {
        if (ir_data.raw[i] != ZHAN_SHU_BAO_DIU_XIAN_RAW_BAI) {
            return 0U;
        }
    }

    return 1U;
}
