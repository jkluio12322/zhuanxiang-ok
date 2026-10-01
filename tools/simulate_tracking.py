#!/usr/bin/env python3
"""Deterministic host-side simulation for the eight-channel line tracker."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "Core/Application/Config/yingYongPeiZhi.h"
TRACKING_PATH = ROOT / "Core/Application/Control/yingYongXunJi.c"
STATE_PATH = ROOT / "Core/Application/Control/yingYongZhuangTai.c"
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
            value = raw_value.strip()
            value = re.sub(r"[uUlLfF]+$", "", value)
            value = value.strip("()")
            try:
                macros[name] = float(value)
            except ValueError:
                continue
    return macros


def require(macros: dict[str, float], name: str) -> float:
    if name not in macros:
        raise RuntimeError(f"missing numeric macro: {name}")
    return macros[name]


MACROS = parse_macros(CONFIG_PATH, TRACKING_PATH, STATE_PATH, IRTRACK_PATH)

PID_KP = require(MACROS, "TRACK_PID_KP")
PID_KI = require(MACROS, "TRACK_PID_KI")
PID_KD = require(MACROS, "TRACK_PID_KD")
PID_PERIOD = require(MACROS, "TRACK_PID_PERIOD_SECONDS")
INTEGRAL_LIMIT = require(MACROS, "TRACK_PID_INTEGRAL_LIMIT_RPM")
ERROR_LIMIT = require(MACROS, "TRACK_WEIGHTED_ERROR_LIMIT")
DIFF_LIMIT = require(MACROS, "TRACK_FOLLOW_DIFF_LIMIT_RPM")
STEERING_SIGN = require(MACROS, "TRACK_STEERING_SIGN")
ALL_WHITE_STOP_TICKS = int(require(MACROS, "TRACK_ALL_WHITE_STOP_TICKS"))
INVALID_LINE_CONFIRM_TICKS = int(
    require(MACROS, "TRACK_LINE_INVALID_CONFIRM_TICKS")
)
CURVE_SPEED_ERROR_START = require(MACROS, "TRACK_CURVE_SPEED_ERROR_START")
CURVE_SPEED_ERROR_FULL = require(MACROS, "TRACK_CURVE_SPEED_ERROR_FULL")
CURVE_SPEED_MIN_RPM = require(MACROS, "TRACK_CURVE_SPEED_MIN_RPM")
CURVE_SPEED_MIN_SCALE = require(MACROS, "TRACK_CURVE_SPEED_MIN_SCALE")
SPEED_RECOVERY_STEP_RPM = require(MACROS, "TRACK_SPEED_RECOVERY_STEP_RPM")
LOST_LINE_MAX_BASE_RPM = require(MACROS, "TRACK_LOST_LINE_MAX_BASE_RPM")
CORNER_LOST_LINE_MIN_DIFF_RPM = require(
    MACROS,
    "TRACK_CORNER_LOST_LINE_MIN_DIFF_RPM",
)
CORNER_LOST_LINE_MAX_BASE_RPM = require(
    MACROS,
    "TRACK_CORNER_LOST_LINE_MAX_BASE_RPM",
)
CORNER_LOST_LINE_STOP_TICKS = int(
    require(MACROS, "TRACK_CORNER_LOST_LINE_STOP_TICKS")
)
CORNER_MIN_TICKS = int(require(MACROS, "TRACK_CORNER_MIN_TICKS"))
CLEAR_TICKS = int(require(MACROS, "TRACK_CORNER_CENTER_CLEAR_TICKS"))
CLEAR_POSITION_MIN = require(MACROS, "TRACK_CORNER_CENTER_CLEAR_POSITION_MIN")
REACQUIRE_TICKS = int(require(MACROS, "TRACK_CORNER_REACQUIRE_CENTER_TICKS"))
REACQUIRE_POSITION_MAX = require(MACROS, "TRACK_CORNER_REACQUIRE_POSITION_MAX")
FINISH_BLACK_COUNT_MIN = int(require(MACROS, "TRACK_FINISH_BLACK_COUNT_MIN"))
G1_RPM = require(MACROS, "SPEED_TABLE_G1_RPM")
G2_RPM = require(MACROS, "SPEED_TABLE_G2_RPM")
G3_RPM = require(MACROS, "SPEED_TABLE_G3_RPM")
G6_RPM = require(MACROS, "SPEED_TABLE_G6_RPM")

IR_WEIGHTS = tuple(
    int(require(MACROS, f"IR_WEIGHT_CH{channel}"))
    for channel in range(1, 9)
)


def pattern_is_contiguous(pattern: int) -> bool:
    black_bits = [bit for bit in range(8) if pattern & (1 << bit)]
    if not black_bits:
        return False

    first_black = black_bits[0]
    last_black = black_bits[-1]
    return all(pattern & (1 << bit) for bit in range(first_black, last_black + 1))


@dataclass(frozen=True)
class SensorSample:
    pattern: int
    weight_sum: int
    position: float
    black_count: int

    @property
    def valid(self) -> bool:
        return 1 <= self.black_count <= 7 and pattern_is_contiguous(self.pattern)

    @property
    def left_count(self) -> int:
        return sum(1 for bit in range(4) if self.pattern & (1 << bit))

    @property
    def right_count(self) -> int:
        return sum(1 for bit in range(4, 8) if self.pattern & (1 << bit))


def sensor_sample(pattern_text: str) -> SensorSample:
    if len(pattern_text) != 8 or any(char not in "01" for char in pattern_text):
        raise ValueError(f"invalid pattern: {pattern_text!r}")

    pattern = 0
    weight_sum = 0
    black_count = 0
    for index, char in enumerate(pattern_text):
        if char == "1":
            pattern |= 1 << index
            weight_sum += IR_WEIGHTS[index]
            black_count += 1

    position = 0.0
    if black_count:
        position = int(weight_sum * 28 / black_count) / 100.0

    return SensorSample(pattern, weight_sum, position, black_count)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class TrackingOutput:
    error: float
    integral: float
    steer: float
    average_target: float
    target_left: float
    target_right: float


class TrackingPid:
    def __init__(self) -> None:
        self.weighted_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.average_target = 0.0
        self.steer = 0.0
        self.scheduled_base_rpm = 0.0
        self.invalid_line_ticks = 0

    def reset(self) -> None:
        self.weighted_error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.average_target = 0.0
        self.steer = 0.0
        self.scheduled_base_rpm = 0.0
        self.invalid_line_ticks = 0

    def schedule_curve_base(
        self,
        selected_base_rpm: float,
        weight_sum: int,
    ) -> float:
        absolute_error = abs(float(weight_sum))
        if absolute_error <= CURVE_SPEED_ERROR_START:
            speed_scale = 1.0
        elif absolute_error >= CURVE_SPEED_ERROR_FULL:
            speed_scale = CURVE_SPEED_MIN_SCALE
        else:
            speed_scale = 1.0 - (
                (1.0 - CURVE_SPEED_MIN_SCALE)
                * (
                    (absolute_error - CURVE_SPEED_ERROR_START)
                    / (CURVE_SPEED_ERROR_FULL - CURVE_SPEED_ERROR_START)
                )
            )

        minimum_base_rpm = min(selected_base_rpm, CURVE_SPEED_MIN_RPM)
        curve_base_rpm = max(
            minimum_base_rpm,
            selected_base_rpm * speed_scale,
        )

        if self.scheduled_base_rpm <= 0.0:
            next_base_rpm = curve_base_rpm
        elif curve_base_rpm < self.scheduled_base_rpm:
            next_base_rpm = curve_base_rpm
        else:
            next_base_rpm = min(
                curve_base_rpm,
                self.scheduled_base_rpm + SPEED_RECOVERY_STEP_RPM,
            )

        self.scheduled_base_rpm = next_base_rpm
        return next_base_rpm

    def schedule_lost_line_base(
        self,
        selected_base_rpm: float,
        max_base_rpm: float = LOST_LINE_MAX_BASE_RPM,
    ) -> float:
        limited_base_rpm = min(selected_base_rpm, max_base_rpm)

        if self.scheduled_base_rpm <= 0.0:
            next_base_rpm = limited_base_rpm
        elif limited_base_rpm < self.scheduled_base_rpm:
            next_base_rpm = limited_base_rpm
        else:
            next_base_rpm = self.scheduled_base_rpm

        self.scheduled_base_rpm = next_base_rpm
        return next_base_rpm

    def apply(
        self,
        sample: SensorSample,
        base_speed: float,
        turn_direction: int = 0,
    ) -> TrackingOutput:
        if sample.valid:
            self.invalid_line_ticks = 0
            self.average_target = self.schedule_curve_base(
                base_speed,
                sample.weight_sum,
            )
            error = clamp(float(sample.weight_sum), -ERROR_LIMIT, ERROR_LIMIT)
            derivative = error - self.previous_error
            self.integral = clamp(
                self.integral + PID_KI * error * PID_PERIOD,
                -INTEGRAL_LIMIT,
                INTEGRAL_LIMIT,
            )
            self.weighted_error = error
            self.previous_error = error
            self.steer = clamp(
                PID_KP * error + self.integral + PID_KD * derivative,
                -DIFF_LIMIT,
                DIFF_LIMIT,
            )
        elif turn_direction != 0:
            self.average_target = self.schedule_lost_line_base(
                base_speed,
                CORNER_LOST_LINE_MAX_BASE_RPM,
            )
            locked_direction = float(turn_direction) * STEERING_SIGN
            minimum_steer = (
                locked_direction * CORNER_LOST_LINE_MIN_DIFF_RPM
            )
            if not (
                self.steer * locked_direction > 0.0
                and abs(self.steer) > abs(minimum_steer)
            ):
                self.steer = minimum_steer
        else:
            if self.invalid_line_ticks < INVALID_LINE_CONFIRM_TICKS:
                self.invalid_line_ticks += 1

            if self.invalid_line_ticks >= INVALID_LINE_CONFIRM_TICKS:
                self.average_target = self.schedule_lost_line_base(base_speed)

        return TrackingOutput(
            error=self.weighted_error,
            integral=self.integral,
            steer=self.steer,
            average_target=self.average_target,
            target_left=self.average_target + self.steer,
            target_right=self.average_target - self.steer,
        )


class CornerMarker:
    def __init__(self) -> None:
        self.active = False
        self.direction = 0
        self.ticks = 0
        self.clear_ticks = 0
        self.cleared = False
        self.complete_ticks = 0
        self.completed = False

    def start(self, direction: int) -> None:
        self.active = True
        self.direction = direction
        self.ticks = 0
        self.clear_ticks = 0
        self.cleared = False
        self.complete_ticks = 0
        self.completed = False

    def update(self, sample: SensorSample) -> bool:
        if not self.active:
            return False

        self.ticks += 1
        recentered = (
            sample.valid and abs(sample.position) <= REACQUIRE_POSITION_MAX
        )

        if not self.cleared:
            if sample.valid and abs(sample.position) > CLEAR_POSITION_MIN:
                self.clear_ticks += 1
                if self.clear_ticks >= CLEAR_TICKS:
                    self.cleared = True
            elif recentered:
                self.clear_ticks = 0

        if self.ticks >= CORNER_MIN_TICKS and self.cleared and recentered:
            self.complete_ticks += 1
            if self.complete_ticks >= REACQUIRE_TICKS:
                self.active = False
                self.completed = True
                return True
        else:
            self.complete_ticks = 0

        return False


def direct_corner_direction(sample: SensorSample) -> int:
    if not pattern_is_contiguous(sample.pattern):
        return 0
    if sample.black_count < 3 or sample.black_count > 7:
        return 0
    if sample.left_count >= 3 and sample.left_count >= sample.right_count + 2:
        return -1
    if sample.right_count >= 3 and sample.right_count >= sample.left_count + 2:
        return 1
    return 0


def edge_candidate_direction(sample: SensorSample) -> int:
    if not pattern_is_contiguous(sample.pattern):
        return 0
    if sample.black_count == 0 or sample.black_count > 7:
        return 0
    if sample.left_count >= 2 and sample.left_count >= sample.right_count + 2:
        return -1
    if sample.right_count >= 2 and sample.right_count >= sample.left_count + 2:
        return 1
    if abs(sample.position) >= require(MACROS, "TRACK_CORNER_EDGE_POSITION_MIN"):
        return 1 if sample.position > 0.0 else -1
    return 0


def can_start_corner(sample: SensorSample) -> bool:
    if sample.black_count >= int(require(MACROS, "TRACK_FINISH_BLACK_COUNT_MIN")):
        return False
    return direct_corner_direction(sample) != 0


def limit_pwm(pwm: float, turn_marker_active: bool) -> float:
    """循迹模式下所有负 PWM 都夹到 0（对应 yingYongKongZhi.c:434-435）。

    当前代码没有「急弯期间允许 -20% 反转」这个特例，参数保留只为兼容调用点。
    """
    return max(pwm, 0.0)


class LineLossMonitor:
    def __init__(self) -> None:
        self.all_white_ticks = 0
        self.corner_lost_line_ticks = 0

    def update(
        self,
        valid_line: bool,
        all_white: bool,
        corner_turn_active: bool,
    ) -> str:
        if corner_turn_active:
            if valid_line:
                self.corner_lost_line_ticks = 0
                self.all_white_ticks = 0
                return ""

            if self.corner_lost_line_ticks < CORNER_LOST_LINE_STOP_TICKS:
                self.corner_lost_line_ticks += 1
            if self.corner_lost_line_ticks >= CORNER_LOST_LINE_STOP_TICKS:
                return "CORNER_TIMEOUT"
            return ""

        self.corner_lost_line_ticks = 0
        if all_white:
            if self.all_white_ticks < ALL_WHITE_STOP_TICKS:
                self.all_white_ticks += 1
            if self.all_white_ticks >= ALL_WHITE_STOP_TICKS:
                return "ALL_WHITE"
        else:
            self.all_white_ticks = 0
        return ""


def close(left: float, right: float, tolerance: float = 1e-6) -> bool:
    return abs(left - right) <= tolerance


def main() -> int:
    failures: list[str] = []

    def expect(condition: bool, message: str) -> None:
        if not condition:
            failures.append(message)

    print("Eight-channel tracking simulation")
    print(
        "config: "
        f"Kp={PID_KP:.3f} Ki={PID_KI:.3f} Kd={PID_KD:.3f} "
        f"error_limit={ERROR_LIMIT:.1f} diff_limit={DIFF_LIMIT:.1f}"
    )
    print(f"weights: {IR_WEIGHTS}")
    print()

    centered = sensor_sample("00011000")
    pid = TrackingPid()
    output = pid.apply(centered, G2_RPM)
    expect(close(output.error, 0.0), "centered line must produce zero error")
    expect(close(output.steer, 0.0), "centered line must produce zero steer")
    expect(close(output.target_left, G2_RPM), "left target must stay at G2")
    expect(close(output.target_right, G2_RPM), "right target must stay at G2")
    print("PASS centered straight")

    pid = TrackingPid()
    output = pid.apply(centered, G6_RPM)
    expect(
        close(output.average_target, G6_RPM),
        "centered G6 straight must keep the selected 170 RPM base",
    )
    print("PASS centered G6 straight keeps full speed")

    strong_left_edge = sensor_sample("11110000")
    pid = TrackingPid()
    output = pid.apply(strong_left_edge, G6_RPM)
    expected_g6_curve_rpm = max(
        CURVE_SPEED_MIN_RPM,
        G6_RPM * CURVE_SPEED_MIN_SCALE,
    )
    expect(
        close(output.average_target, expected_g6_curve_rpm),
        "G6 strong error must reduce the base to about 60 RPM",
    )
    print("PASS G6 strong curve error pre-slows to about 60 RPM")

    pid = TrackingPid()
    output = pid.apply(strong_left_edge, G3_RPM)
    expect(
        close(output.average_target, CURVE_SPEED_MIN_RPM),
        "G3 strong error must reduce the base to the 45 RPM floor",
    )
    print("PASS G3 strong curve error pre-slows to 45 RPM")

    pid = TrackingPid()
    output = pid.apply(strong_left_edge, G1_RPM)
    expect(
        close(output.average_target, G1_RPM),
        "G1 strong error must not be accelerated by the 45 RPM floor",
    )
    print("PASS G1 curve slowdown never raises the selected base")

    for pattern in (
        "11100000",
        "00011100",
        "00011000",
        "10000000",
    ):
        sample = sensor_sample(pattern)
        expect(sample.valid, f"contiguous pattern {pattern} must be valid")

    for pattern in (
        "10000001",
        "10010000",
        "00100100",
        "11100111",
    ):
        sample = sensor_sample(pattern)
        expect(not sample.valid, f"non-contiguous pattern {pattern} must be invalid")
    print("PASS only one contiguous black segment is accepted as a valid line")

    left_of_line = sensor_sample("00001100")
    pid = TrackingPid()
    output = pid.apply(left_of_line, G2_RPM)
    expect(output.error > 0.0, "car left of line must create positive error")
    expect(output.steer > 0.0, "car left of line must steer right")
    expect(
        output.target_left > output.target_right,
        "positive correction must speed up the left wheel",
    )
    print("PASS straight with car offset left")

    pid = TrackingPid()
    output = pid.apply(sensor_sample("11000000"), G2_RPM)
    expect(output.error < 0.0, "left outer line must create negative error")
    expect(output.steer < 0.0, "left outer line must keep steering left")
    expect(
        abs(output.steer) > 0.0,
        "IR4/IR5 being white must not zero outer-channel steering",
    )
    print("PASS center pair white, left outer line active")

    pid = TrackingPid()
    output = pid.apply(sensor_sample("00000011"), G2_RPM)
    expect(output.error > 0.0, "right outer line must create positive error")
    expect(output.steer > 0.0, "right outer line must keep steering right")
    expect(
        abs(output.steer) > 0.0,
        "IR4/IR5 being white must not zero outer-channel steering",
    )
    print("PASS center pair white, right outer line active")

    for pattern, expected_sign in (("00010000", -1), ("00001000", 1)):
        pid = TrackingPid()
        output = pid.apply(sensor_sample(pattern), G2_RPM)
        expect(
            output.steer * expected_sign > 0.0,
            f"single center channel {pattern} must keep PID steering active",
        )
    print("PASS one center channel white while the other remains active")

    pid = TrackingPid()
    output = pid.apply(sensor_sample("11000000"), G2_RPM)
    held_steer = output.steer
    all_white_ticks = 0
    stopped = False
    for _ in range(10):
        all_white_ticks += 1
        output = pid.apply(sensor_sample("00000000"), G2_RPM)
    expect(close(output.steer, held_steer), "short all-white must hold steer")
    expect(not stopped, "short all-white must not stop the tracker")
    print("PASS short all-white holds the previous steering target")

    all_white = sensor_sample("00000000")
    pid = TrackingPid()
    output = pid.apply(strong_left_edge, G6_RPM)
    held_steer = output.steer
    for _ in range(INVALID_LINE_CONFIRM_TICKS):
        output = pid.apply(all_white, G6_RPM)
    expect(
        close(output.average_target, LOST_LINE_MAX_BASE_RPM),
        "confirmed lost line must cap the base at 45 RPM",
    )
    expect(
        close(output.steer, held_steer),
        "confirmed lost line must hold the last steering difference",
    )
    print("PASS confirmed line loss holds steer and caps speed at 45 RPM")

    output = pid.apply(strong_left_edge, G6_RPM)
    expect(
        close(
            output.average_target,
            LOST_LINE_MAX_BASE_RPM + SPEED_RECOVERY_STEP_RPM,
        ),
        "the first recovered line tick may increase speed by at most 10 RPM",
    )
    output = pid.apply(strong_left_edge, G6_RPM)
    expect(
        close(output.average_target, expected_g6_curve_rpm),
        "speed recovery must continue until the curve target is reached",
    )
    print("PASS recovered speed is limited to 10 RPM per 20 ms tick")

    non_contiguous = sensor_sample("10000001")
    pid = TrackingPid()
    output = pid.apply(strong_left_edge, G6_RPM)
    held_steer = output.steer
    for _ in range(INVALID_LINE_CONFIRM_TICKS):
        output = pid.apply(non_contiguous, G6_RPM)
    expect(
        close(output.average_target, LOST_LINE_MAX_BASE_RPM),
        "non-contiguous line must use the confirmed-loss speed cap",
    )
    expect(
        close(output.steer, held_steer),
        "non-contiguous line must not update or clear the steering difference",
    )
    print("PASS non-contiguous line holds steer and enters loss protection")

    pid = TrackingPid()
    output = pid.apply(centered, G6_RPM)
    expect(close(output.steer, 0.0), "corner fallback setup must start centered")
    output = pid.apply(all_white, G6_RPM, -1)
    expect(
        close(output.average_target, CORNER_LOST_LINE_MAX_BASE_RPM),
        "corner line loss must cap the base at 45 RPM immediately",
    )
    expect(
        close(output.steer, -CORNER_LOST_LINE_MIN_DIFF_RPM),
        "corner line loss with zero history must apply minimum left steer",
    )
    expect(
        close(output.target_left, 5.0) and close(output.target_right, 85.0),
        "minimum corner fallback must produce the expected wheel targets",
    )
    print("PASS corner line loss applies its minimum steer immediately")

    pid = TrackingPid()
    output = pid.apply(strong_left_edge, G6_RPM)
    held_corner_steer = output.steer
    expect(
        held_corner_steer < -CORNER_LOST_LINE_MIN_DIFF_RPM,
        "test setup must produce a larger same-direction corner steer",
    )
    output = pid.apply(all_white, G6_RPM, -1)
    expect(
        close(output.steer, held_corner_steer),
        "corner fallback must preserve a larger same-direction steer",
    )
    print("PASS corner line loss preserves a larger same-direction steer")

    pid = TrackingPid()
    pid.apply(sensor_sample("00000011"), G6_RPM)
    output = pid.apply(all_white, G6_RPM, -1)
    expect(
        close(output.steer, -CORNER_LOST_LINE_MIN_DIFF_RPM),
        "corner fallback must reverse a wrong-direction steering history",
    )
    print("PASS corner line loss corrects an opposite steering history")

    pid = TrackingPid()
    pid.apply(centered, G1_RPM)
    output = pid.apply(all_white, G1_RPM, -1)
    expect(
        close(output.average_target, G1_RPM),
        "corner line loss must not raise a low speed level to the 45 RPM cap",
    )
    print("PASS corner line loss never raises a lower base speed")

    pid = TrackingPid()
    pid.apply(centered, G6_RPM)
    pid.apply(all_white, G6_RPM, -1)
    output = pid.apply(strong_left_edge, G6_RPM, -1)
    expect(
        close(
            output.average_target,
            CORNER_LOST_LINE_MAX_BASE_RPM + SPEED_RECOVERY_STEP_RPM,
        ),
        "corner PID recovery must still limit speed to 10 RPM per tick",
    )
    print("PASS corner line recovery resumes PID with limited acceleration")

    line_loss_monitor = LineLossMonitor()
    stop_reason = ""
    for _ in range(ALL_WHITE_STOP_TICKS):
        stop_reason = line_loss_monitor.update(False, True, False)
    expect(stop_reason == "ALL_WHITE", "50 all-white ticks must stop")
    print("PASS all-white stop after one second")

    line_loss_monitor = LineLossMonitor()
    stop_reason = ""
    for _ in range(CORNER_LOST_LINE_STOP_TICKS - 1):
        stop_reason = line_loss_monitor.update(False, True, True)
    expect(
        stop_reason == "",
        "corner loss must not stop before the two-second timeout",
    )
    stop_reason = line_loss_monitor.update(False, True, True)
    expect(
        stop_reason == "CORNER_TIMEOUT",
        "corner loss must stop at the two-second timeout",
    )
    print("PASS corner line loss times out at two seconds")

    marker = CornerMarker()
    pid = TrackingPid()
    marker.start(-1)
    for _ in range(2):
        sample = sensor_sample("11000000")
        pid.apply(sample, G2_RPM)
        marker.update(sample)

    center_line = sensor_sample("00011000")
    expect(center_line.valid, "corner completion pattern must be a contiguous line")
    completed = False
    for _ in range(REACQUIRE_TICKS + CORNER_MIN_TICKS):
        pid.apply(center_line, G2_RPM)
        completed = marker.update(center_line)
        if completed:
            break
    expect(completed, "corner must complete using a contiguous centered line")
    print("PASS corner completion with a contiguous centered line")

    marker = CornerMarker()
    marker.start(-1)
    for _ in range(2):
        marker.update(sensor_sample("11000000"))

    non_contiguous_center = sensor_sample("00100100")
    completed = False
    for _ in range(REACQUIRE_TICKS + CORNER_MIN_TICKS):
        completed = marker.update(non_contiguous_center)
        if completed:
            break
    expect(
        not completed,
        "non-contiguous center pattern must not complete corner recentering",
    )
    print("PASS non-contiguous center pattern cannot complete a corner")

    seven_black = sensor_sample("01111111")
    slanted_seven_black = sensor_sample("10111111")
    eight_black = sensor_sample("11111111")
    expect(not can_start_corner(seven_black), "seven black channels must not start")
    expect(
        not can_start_corner(slanted_seven_black),
        "non-contiguous seven black channels must not start",
    )
    expect(not can_start_corner(eight_black), "eight black channels must not start")
    expect(not eight_black.valid, "eight black channels must be invalid for PID")
    expect(
        slanted_seven_black.black_count >= FINISH_BLACK_COUNT_MIN,
        "slanted seven-channel finish line must remain finish-eligible",
    )
    print("PASS endpoint line patterns do not start a corner marker")

    expect(close(limit_pwm(-35.0, False), 0.0), "normal phase must clamp negative")
    expect(
        close(limit_pwm(-35.0, True), 0.0),
        "corner phase also clamps reverse PWM to 0",
    )
    expect(close(limit_pwm(35.0, True), 35.0), "positive PWM must be unchanged")
    print("PASS phase-specific negative PWM limits")

    sample = sensor_sample("11110000")
    candidate = edge_candidate_direction(sample)
    expect(candidate == -1, "left edge pattern must create a left candidate")
    sample = sensor_sample("00001110")
    candidate = edge_candidate_direction(sample)
    expect(candidate == 1, "right edge pattern must create a right candidate")

    sample = sensor_sample("11100001")
    expect(not sample.valid, "non-contiguous left-heavy pattern must be invalid")
    expect(
        direct_corner_direction(sample) == 0,
        "non-contiguous left-heavy pattern must not start a corner",
    )
    expect(
        edge_candidate_direction(sample) == 0,
        "non-contiguous left-heavy pattern must not refresh a candidate",
    )

    sample = sensor_sample("10000111")
    expect(not sample.valid, "non-contiguous right-heavy pattern must be invalid")
    expect(
        direct_corner_direction(sample) == 0,
        "non-contiguous right-heavy pattern must not start a corner",
    )
    expect(
        edge_candidate_direction(sample) == 0,
        "non-contiguous right-heavy pattern must not refresh a candidate",
    )
    print("PASS edge candidate directions")

    print()
    if failures:
        print(f"FAILED: {len(failures)} scenario(s)")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("All tracking scenarios passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
