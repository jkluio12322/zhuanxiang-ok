#!/usr/bin/env python3
"""Host-side checks for PWM-SPEED and PWM-TRACK arithmetic.

PWM-TRACK 的模拟输入是一张八路图案表（app_testmode.c 的 patternTable[]）：
固件把那 8 位写进驱动层的 ir_data.raw[]，再当作传感器读数算归一化质心。
所以这里做四件事：

1. 从 irtrack.c 解析通道权重，用与固件相同的稳定分母公式复算误差；
2. 核对固件里那张表**逐项**等于本文件期望的表（表被改动了这里就会失败）；
3. 核对无效图案（全黑 / 全白 / 中间断开）确实判无效、且理由正确；
4. 核对表对「左右镜像」封闭——PWM-TRACK 的自动左右对调依赖这个性质。
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "Core/system/config.h"
IRTRACK = ROOT / "Core/hardware/irtrack.c"
TESTMODE = ROOT / "Core/logic/testmode.c"

CHANNELS = 8
# 循迹模块判无效的压黑通道上限，对应 TRACK_LINE_VALID_MAX_BLACK。
LINE_VALID_MAX_BLACK = 7

# 固件里 patternTable[] 期望的内容：(图案, 期望归一化误差或 None=无效, 期望 all_white)。
EXPECTED_TABLE = [
    (0x03, -1.00, False),   # 1100 0000  直线偏移·大（左）
    (0x0F, -0.72, False),   # 1111 0000  左转弯口
    (0x06, -0.62, False),   # 0110 0000  直线偏移·中（左）
    (0x0C, -0.29, False),   # 0011 0000  直线偏移·小（左）
    (0x18, 0.00, False),    # 0001 1000  居中
    (0x30, 0.29, False),    # 0000 1100  直线偏移·小（右）
    (0x60, 0.62, False),    # 0000 0110  直线偏移·中（右）
    (0xF0, 0.72, False),    # 0000 1111  右转弯口
    (0xC0, 1.00, False),    # 0000 0011  直线偏移·大（右）
    (0xFF, None, False),    # 1111 1111  全黑 = 丢线
    (0x00, None, True),     # 0000 0000  全白 = 到终点线
    (0x81, None, False),    # 1000 0001  岔路口（中间断开）
]


def read_macros(path: Path):
    macros = {}
    pattern = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)\s+([^\s/]+)")
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if not match:
            continue
        name, value = match.groups()
        value = re.sub(r"[uUlLfF]+$", "", value).strip("()")
        try:
            macros[name] = float(value)
        except ValueError:
            continue
    return macros


def read_pattern_table(path: Path):
    """把固件里 patternTable[] 的图案值抠出来。"""
    text = path.read_text(encoding="utf-8")
    match = re.search(r"static const uint8_t patternTable\[\]\s*=\s*\{(.*?)\};",
                      text, re.S)
    if not match:
        raise SystemExit("patternTable[] not found in app_testmode.c")
    return [int(value, 16)
            for value in re.findall(r"0x([0-9A-Fa-f]{2})U", match.group(1))]


def mirror(pattern: int) -> int:
    """左右镜像：bit0（IR1）换到 bit7（IR8），与固件的 mirror8() 同义。"""
    return sum(((pattern >> i) & 1) << (CHANNELS - 1 - i) for i in range(CHANNELS))


def classify(pattern: int, weights, bias):
    """按 app_tracking.c + irtrack.c 的规则判定一个图案。"""
    black = [i for i in range(CHANNELS) if (pattern >> i) & 1]
    count = len(black)

    contiguous = (count > 0
                  and all((pattern >> i) & 1 for i in range(black[0], black[-1] + 1)))

    total = sum(weights[i] for i in black)
    max_weight = max(abs(value) for value in weights)
    error = (total * (1.0 + bias) / (max_weight * (count + bias))
             if count else 0.0)
    error = max(-1.0, min(1.0, error))

    valid = 1 <= count <= LINE_VALID_MAX_BLACK and contiguous
    return count, contiguous, valid, count == 0, error


def main() -> int:
    macros = read_macros(CONFIG)
    # 权重与固件一样来自 irtrack.c，不在本文件里另抄一份。
    macros.update(read_macros(IRTRACK))

    phase_ms = int(macros["PWM_SPEED_TEST_PHASE_MS"])
    control_ms = int(macros["CONTROL_PERIOD_MS"])
    voltage = macros["MOTOR_SUPPLY_VOLTAGE"]
    weights = [macros[f"IR_WEIGHT_CH{i}"] for i in range(1, CHANNELS + 1)]
    bias = macros["TRACK_ERROR_DENOM_BIAS"]
    failures = []

    def expect(condition, message):
        if not condition:
            failures.append(message)

    # ---- PWM-SPEED 的相位翻转 ----
    speed_phase = "FORWARD"
    elapsed = 0
    for _ in range(2 * (phase_ms // control_ms)):
        elapsed += control_ms
        if elapsed >= phase_ms:
            elapsed = 0
            speed_phase = "REVERSE" if speed_phase == "FORWARD" else "FORWARD"
    expect(speed_phase == "FORWARD", "PWM-SPEED must return to forward after two phases")

    # ---- PWM-TRACK 的图案表 ----
    table = read_pattern_table(TESTMODE)
    expect(table == [pattern for pattern, _, _ in EXPECTED_TABLE],
           f"patternTable[] is {[hex(p) for p in table]}, expected "
           f"{[hex(p) for p, _, _ in EXPECTED_TABLE]}")

    for pattern, expected_error, expect_all_white in EXPECTED_TABLE:
        count, contiguous, valid, all_white, error = classify(pattern, weights, bias)

        if expected_error is None:
            expect(not valid, f"pattern {pattern:#04x} ({pattern:08b}) must be invalid")
        else:
            expect(valid, f"pattern {pattern:#04x} ({pattern:08b}) must be valid")
            expect(abs(error - expected_error) < 0.006,
                   f"pattern {pattern:#04x} ({pattern:08b}) gives e {error:+.2f}, "
                   f"expected {expected_error:+.2f}")

        expect(all_white == expect_all_white,
               f"pattern {pattern:#04x} all_white flag mismatch")

    # ---- 三种无效图案各自的理由要对 ----
    expect(classify(0xFF, weights, bias)[0] == 8 > LINE_VALID_MAX_BLACK,
           "all-black must exceed the black-channel limit (treated as a cross line)")
    expect(classify(0x00, weights, bias)[3] and not classify(0x00, weights, bias)[2],
           "all-white must be invalid and flagged all_white")
    expect(not classify(0x81, weights, bias)[1],
           "IR1+IR8 with a white gap must be non-contiguous")

    # ---- 表对镜像封闭：自动左右对调依赖这个性质 ----
    for pattern, _, _ in EXPECTED_TABLE:
        expect(mirror(pattern) in table,
               f"mirror of {pattern:#04x} ({mirror(pattern):08b}) is missing from the table")

    # ---- 开机默认图案：必须在表内且居中 ----
    default_match = re.search(r"#\s*define\s+PATTERN_DEFAULT\s+0x([0-9A-Fa-f]{2})U",
                              TESTMODE.read_text(encoding="utf-8"))
    expect(default_match is not None, "PATTERN_DEFAULT not found in app_testmode.c")
    if default_match:
        default = int(default_match.group(1), 16)
        expect(default in table, f"PATTERN_DEFAULT {default:#04x} must be a table entry")
        expect(abs(classify(default, weights, bias)[4]) < 1e-6,
               "PATTERN_DEFAULT must be centered (error == 0)")

    # ---- 输出级：基础占空比 ± 差速的钳位，以及占空比到电压 ----
    def clamp_pwm(base, steer):
        left = base + steer
        right = base - steer
        return max(0, min(100, left)), max(0, min(100, right))

    expect(clamp_pwm(50, 40) == (90, 10), "track clamp positive steer failed")
    expect(clamp_pwm(50, 80) == (100, 0), "track clamp upper/lower bound failed")
    expect(abs(50.0 * voltage / 100.0 - 6.0) < 1e-6, "50 percent must map to 6.0V")

    if failures:
        print("\n".join(failures))
        return 1
    print("PWM test arithmetic passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
