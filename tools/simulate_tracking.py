#!/usr/bin/env python3
"""Host checks for the normalized infrared steering planner.

This mirrors the current firmware chain:
  weighted IR error -> steering ratio -> asymmetric wheel-speed targets.
It does not model motor dynamics; ``simulate_control_loops.py`` covers the
speed/position execution layers.
"""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "Core/system/config.h"
IRTRACK = ROOT / "Core/hardware/irtrack.c"

DEFINE_RE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)\s+([^\s/]+)")


def macros(*paths: Path) -> dict[str, float]:
    result: dict[str, float] = {}
    for path in paths:
        for line in path.read_text(encoding="utf-8").splitlines():
            match = DEFINE_RE.match(line)
            if not match:
                continue
            name, raw = match.groups()
            raw = re.sub(r"[uUlLfF]+$", "", raw.strip("()"))
            try:
                result[name] = float(raw)
            except ValueError:
                pass
    return result


CFG = macros(CONFIG, IRTRACK)
WEIGHTS = [CFG[f"IR_WEIGHT_CH{i}"] for i in range(1, 9)]
MAX_WEIGHT = max(abs(value) for value in WEIGHTS)
BIAS = CFG["TRACK_ERROR_DENOM_BIAS"]
RATIO_LIMIT = CFG["TRACK_STEER_RATIO_LIMIT"]
OUTER_GAIN = CFG["TRACK_OUTER_WHEEL_GAIN"]
MAX_BLACK = int(CFG["TRACK_LINE_VALID_MAX_BLACK"])
MAX_RPM = CFG["SPEED_TARGET_LIMIT_RPM"]
KP = CFG["TRACK_PID_KP"]

FAILURES: list[str] = []


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def contiguous(pattern: int) -> bool:
    active = [i for i in range(8) if (pattern >> i) & 1]
    if not active:
        return False
    return all((pattern >> i) & 1 for i in range(active[0], active[-1] + 1))


def classify(pattern: int) -> tuple[bool, float, int]:
    active = [i for i in range(8) if (pattern >> i) & 1]
    count = len(active)
    if count == 0:
        return False, 0.0, 0

    total = sum(WEIGHTS[i] for i in active)
    error = total * (1.0 + BIAS) / (MAX_WEIGHT * (count + BIAS))
    valid = 1 <= count <= MAX_BLACK and contiguous(pattern)
    return valid, clamp(error, -1.0, 1.0), count


def wheel_targets(base_rpm: float, ratio: float) -> tuple[float, float]:
    ratio = clamp(ratio * KP, -RATIO_LIMIT, RATIO_LIMIT)
    magnitude = abs(ratio)
    if ratio >= 0.0:
        left = base_rpm * (1.0 - magnitude)
        right = base_rpm * (1.0 + OUTER_GAIN * magnitude)
    else:
        left = base_rpm * (1.0 + OUTER_GAIN * magnitude)
        right = base_rpm * (1.0 - magnitude)
    return clamp(left, 0.0, MAX_RPM), clamp(right, 0.0, MAX_RPM)


def pattern_of(*channels: int) -> int:
    value = 0
    for channel in channels:
        value |= 1 << (channel - 1)
    return value


def expect(condition: bool, label: str) -> None:
    print(f"{'PASS' if condition else 'FAIL'} {label}")
    if not condition:
        FAILURES.append(label)


def main() -> int:
    valid, center_error, _ = classify(pattern_of(4, 5))
    expect(valid and center_error == 0.0, "center pair is valid and centered")

    valid, left_error, _ = classify(pattern_of(1))
    expect(valid and abs(left_error + 1.0) < 1e-6,
           "IR1 maps to normalized -1")

    valid, right_error, _ = classify(pattern_of(8))
    expect(valid and abs(right_error - 1.0) < 1e-6,
           "IR8 maps to normalized +1")

    valid, _, _ = classify(pattern_of(1, 8))
    expect(not valid, "disconnected edge sensors are invalid")

    valid, _, _ = classify(0xFF)
    expect(not valid, "all-black pattern is invalid")

    valid, _, _ = classify(0x00)
    expect(not valid, "all-white pattern is invalid")

    left, right = wheel_targets(60.0, 1.0)
    expect(abs(left - 21.0) < 1e-6 and abs(right - 67.8) < 1e-6,
           "positive steering lowers inner wheel and only mildly raises outer")

    left, right = wheel_targets(60.0, -1.0)
    expect(abs(left - 67.8) < 1e-6 and abs(right - 21.0) < 1e-6,
           "negative steering mirrors the asymmetric differential")

    left, right = wheel_targets(170.0, 1.0)
    expect(left >= 0.0 and right <= MAX_RPM and right <= 190.0,
           "targets stay non-negative and inside the global RPM limit")

    print()
    print(f"weights={WEIGHTS} bias={BIAS} limit={RATIO_LIMIT}")
    if FAILURES:
        print(f"{len(FAILURES)} tracking scenario(s) failed")
        return 1
    print("All tracking scenarios passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
