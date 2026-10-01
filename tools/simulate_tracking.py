#!/usr/bin/env python3
"""八路循迹的离线仿真。

模型对齐 app_tracking.c：
  - e_y 取归一化质心 (sum / count) * 0.28，居中被控量为 0
  - 单个位置式 PD 输出差速 RPM
  - 计圈用「|e_y| 持续超过阈值」的事件计数，起点横线因 e_y≈0 天然不计

参数全部从 Core/Application/Config/app_config.h 解析，改宏后直接重跑，
不需要重新编译固件。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "Core/Application/Config/app_config.h"
IRTRACK_PATH = ROOT / "Core/Hardware/Peripherals/Sensors/irtrack.c"

DEFINE_RE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)\s+([^\s/]+)")


def parse_macros(*paths: Path) -> dict[str, float]:
    macros: dict[str, float] = {}
    for path in paths:
        for line in path.read_text(encoding="utf-8").splitlines():
            match = DEFINE_RE.match(line)
            if not match:
                continue
            name, raw_value = match.groups()
            value = re.sub(r"[uUlLfF]+$", "", raw_value.strip()).strip("()")
            try:
                macros[name] = float(value)
            except ValueError:
                continue
    return macros


MACROS = parse_macros(CONFIG_PATH, IRTRACK_PATH)


def require(name: str) -> float:
    if name not in MACROS:
        raise RuntimeError(f"missing numeric macro: {name}")
    return MACROS[name]


CHANNELS = 8

# 与 irtrack.c 的权重表一致：负值偏左，正值偏右。
WEIGHTS = [require(f"IR_WEIGHT_CH{i}") for i in range(1, CHANNELS + 1)]

# 归一化系数：position = (sum / count) * 0.28。
POSITION_SCALE = 0.28

PID_KP = require("TRACK_PID_KP")
PID_KI = require("TRACK_PID_KI")
PID_KD = require("TRACK_PID_KD")
PID_DT = require("TRACK_PID_PERIOD_SECONDS")
DIFF_LIMIT = require("TRACK_FOLLOW_DIFF_LIMIT_RPM")

LINE_VALID_MAX_BLACK = int(require("TRACK_LINE_VALID_MAX_BLACK"))
INVALID_CONFIRM_TICKS = int(require("TRACK_LINE_INVALID_CONFIRM_TICKS"))
ALL_WHITE_STOP_TICKS = int(require("TRACK_ALL_WHITE_STOP_TICKS"))
LOST_STOP_TICKS = int(require("TRACK_LINE_LOST_STOP_TICKS"))
START_FORWARD_TICKS = int(require("TRACK_START_FORWARD_TICKS"))

CORNER_E_Y_MIN = require("TRACK_CORNER_E_Y_MIN")
CORNER_CONFIRM_TICKS = int(require("TRACK_CORNER_CONFIRM_TICKS"))
CORNER_GAP_TICKS = int(require("TRACK_CORNER_GAP_TICKS"))
CORNERS_PER_LAP = int(require("TRACK_CORNERS_PER_LAP"))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def close(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(a - b) <= tol


@dataclass
class LineSample:
    pattern: int
    black_count: int
    contiguous: bool
    valid: bool
    all_white: bool
    e_y: float


def classify(pattern: int) -> LineSample:
    """按 app_tracking.c 的规则判定线路。"""
    black = [i for i in range(CHANNELS) if (pattern >> i) & 1]
    count = len(black)

    if count == 0:
        contiguous = False
    else:
        contiguous = all(((pattern >> i) & 1) for i in range(black[0], black[-1] + 1))

    total = sum(WEIGHTS[i] for i in black)
    e_y = (total / count) * POSITION_SCALE if count else 0.0

    valid = 1 <= count <= LINE_VALID_MAX_BLACK and contiguous

    return LineSample(pattern, count, contiguous, valid, count == 0, e_y)


class Confirm:
    """对应 app_scheduler.h 的 ConfirmCounter。"""

    def __init__(self, need: int):
        self.need = need
        self.count = 0

    def reset(self, need: int | None = None) -> None:
        if need is not None:
            self.need = need
        self.count = 0

    def update(self, condition: bool) -> bool:
        if condition:
            if self.count < self.need:
                self.count += 1
        else:
            self.count = 0
        return self.count >= self.need


class Tracker:
    """对应 app_tracking.c 的静态状态。"""

    def __init__(self):
        self.state = "ACQUIRE"
        self.e0 = self.e1 = 0.0
        self.integral = 0.0
        self.steer = 0.0
        self.invalid = Confirm(INVALID_CONFIRM_TICKS)
        self.all_white = Confirm(ALL_WHITE_STOP_TICKS)
        self.lost_stop = Confirm(LOST_STOP_TICKS)
        self.corner = Confirm(CORNER_CONFIRM_TICKS)
        self.corner_gap = 0
        self.lap_corners = 0
        self.laps = 0
        self.startup = 0
        self.stop_reason: str | None = None

    def pid(self, error: float) -> float:
        self.integral += PID_KI * error * PID_DT
        self.integral = clamp(self.integral, -DIFF_LIMIT, DIFF_LIMIT)
        out = PID_KP * error + self.integral + PID_KD * (error - self.e1)
        self.e1 = self.e0
        self.e0 = error
        return clamp(out, -DIFF_LIMIT, DIFF_LIMIT)

    def step(self, pattern: int) -> float:
        s = classify(pattern)

        if s.valid:
            self.invalid.reset(INVALID_CONFIRM_TICKS)
            self.lost_stop.reset(LOST_STOP_TICKS)
            if self.state == "LOST":
                self.state = "FOLLOW"
        elif self.invalid.update(True):
            self.state = "LOST"

        if self.all_white.update(s.all_white):
            self.stop_reason = "ALL_WHITE"
            return self.steer

        if self.state == "LOST" and self.lost_stop.update(True):
            self.stop_reason = "LINE_LOST_TIMEOUT"
            return self.steer

        if self.state == "ACQUIRE":
            self.startup += 1
            if self.startup >= START_FORWARD_TICKS:
                self.state = "FOLLOW"
        elif self.state == "FOLLOW" and s.valid:
            self.steer = self.pid(s.e_y)

            if self.corner_gap > 0:
                self.corner_gap -= 1
            elif self.corner.update(abs(s.e_y) >= CORNER_E_Y_MIN):
                self.corner.reset(CORNER_CONFIRM_TICKS)
                self.corner_gap = CORNER_GAP_TICKS
                self.lap_corners += 1
                if self.lap_corners >= CORNERS_PER_LAP:
                    self.lap_corners = 0
                    self.laps += 1

        return self.steer


def pattern_of(*channels: int) -> int:
    """通道号从 1 开始，返回对应的压黑掩码。"""
    mask = 0
    for ch in channels:
        mask |= 1 << (ch - 1)
    return mask


FAILURES: list[str] = []


def expect(condition: bool, label: str) -> None:
    if condition:
        print(f"PASS {label}")
    else:
        print(f"FAIL {label}")
        FAILURES.append(label)


def run_tracking() -> None:
    print(f"Kp={PID_KP:.3f} Ki={PID_KI:.3f} Kd={PID_KD:.3f} "
          f"diff_limit={DIFF_LIMIT:.1f} corner_e_y={CORNER_E_Y_MIN:.1f}")
    print(f"weights: {WEIGHTS}")
    print()

    # ---- 线路判定 ----
    s = classify(pattern_of(4, 5))
    expect(s.valid and close(s.e_y, 0.0), "centered pair is valid and centered")

    s = classify(pattern_of(1, 8))
    expect(not s.valid, "IR1+IR8 with a white gap is invalid")

    s = classify(pattern_of(1, 2, 3, 4, 5, 6, 7, 8))
    expect(not s.valid, "all eight black is invalid")

    s = classify(0)
    expect(s.all_white and not s.valid, "all white is invalid")

    # ---- 单路压黑的量化取值与方向 ----
    for ch, expected in ((1, -7.0), (4, -0.84), (5, 0.84), (8, 7.0)):
        s = classify(pattern_of(ch))
        expect(close(s.e_y, expected, 0.01),
               f"single channel IR{ch} gives e_y {expected:+.2f}")

    # ---- 转向方向：e_y 为正（线在右）时输出正差速 ----
    t = Tracker()
    t.state = "FOLLOW"
    left_bias = t.step(pattern_of(6, 7))
    expect(left_bias > 0.0, "line to the right steers positive")

    t = Tracker()
    t.state = "FOLLOW"
    right_bias = t.step(pattern_of(2, 3))
    expect(right_bias < 0.0, "line to the left steers negative")

    # ---- 最外侧单路压黑时，输出稳定在 Kp * e_y_max ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(5):
        out = t.step(pattern_of(8))
    expect(close(out, PID_KP * 7.0, 1.0),
           "extreme error settles at Kp * e_y_max")
    expect(out < DIFF_LIMIT,
           "normal operation stays below the diff limit (it is only a safety clamp)")

    # ---- 丢线确认与保持 ----
    t = Tracker()
    t.state = "FOLLOW"
    t.step(pattern_of(4, 5))
    held = t.steer
    t.step(0)          # 1 次全白
    expect(t.state == "FOLLOW", "a single blank tick does not drop to LOST")
    expect(close(t.steer, held), "short blank holds the previous steer")

    # ---- 全白一秒停车 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(ALL_WHITE_STOP_TICKS):
        t.step(0)
    expect(t.stop_reason == "ALL_WHITE", "all white for one second stops the car")

    # ---- 丢线超时停车 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(LOST_STOP_TICKS + 5):
        t.step(pattern_of(1, 8))   # 断续图案 → 持续无效
    expect(t.stop_reason == "LINE_LOST_TIMEOUT", "sustained invalid line times out")

    # ---- 计圈：正对起点横线不计 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(20):
        t.step(pattern_of(1, 2, 3, 4, 5, 6, 7, 8))   # 全压，e_y≈0，但被判无效
    expect(t.lap_corners == 0, "a straight-on start bar never counts a corner")

    # ---- 计圈：斜穿起点横线（单侧 4 路）也不计 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(CORNER_CONFIRM_TICKS - 1):
        t.step(pattern_of(1, 2, 3, 4))   # e_y = -3.78，超过阈值
    expect(t.lap_corners == 0,
           "a slanted start bar shorter than the confirm window never counts")

    # ---- 计圈：持续偏在一侧才算一个弯 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(CORNER_CONFIRM_TICKS):
        t.step(pattern_of(1, 2, 3))
    expect(t.lap_corners == 1, "a sustained one-sided excursion counts one corner")

    # ---- 一个弯只计一次（间隔生效） ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(CORNER_CONFIRM_TICKS + CORNER_GAP_TICKS):
        t.step(pattern_of(1, 2, 3))
    expect(t.lap_corners == 1, "the gap prevents one corner counting twice")

    # ---- 四个弯计一圈 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(CORNERS_PER_LAP):
        for _ in range(CORNER_CONFIRM_TICKS):
            t.step(pattern_of(1, 2, 3))
        for _ in range(CORNER_GAP_TICKS + 1):
            t.step(pattern_of(4, 5))
    expect(t.laps == 1, f"{CORNERS_PER_LAP} corners count one lap")

    # ---- 连续四圈报四圈，不是五圈 ----
    t = Tracker()
    t.state = "FOLLOW"
    for _ in range(4):
        for _ in range(CORNERS_PER_LAP):
            for _ in range(CORNER_CONFIRM_TICKS):
                t.step(pattern_of(1, 2, 3))
            for _ in range(CORNER_GAP_TICKS + 1):
                t.step(pattern_of(4, 5))
        # 每圈末尾正对一次起点横线
        for _ in range(5):
            t.step(pattern_of(1, 2, 3, 4, 5, 6, 7, 8))
    expect(t.laps == 4, "four laps report four laps, not five")


def main() -> int:
    run_tracking()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} scenario(s) failed:")
        for name in FAILURES:
            print(f"  - {name}")
        return 1
    print("All tracking scenarios passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
