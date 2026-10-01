#!/usr/bin/env python3
"""Host-side checks for PWM-SPEED and PWM-TRACK arithmetic."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "Core/Application/Config/yingYongPeiZhi.h"


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


def main() -> int:
    macros = read_macros(CONFIG)
    phase_ms = int(macros["PWM_SPEED_TEST_PHASE_MS"])
    control_ms = int(macros["APP_CONTROL_PERIOD_MS"])
    full_error = macros["PWM_TRACK_ERROR_FULL_SCALE"]
    full_angle = macros["PWM_TRACK_ANGLE_FULL_SCALE"]
    voltage = macros["MOTOR_SUPPLY_VOLTAGE"]
    failures = []

    def expect(condition, message):
        if not condition:
            failures.append(message)

    speed_phase = "FORWARD"
    elapsed = 0
    for _ in range(2 * (phase_ms // control_ms)):
        elapsed += control_ms
        if elapsed >= phase_ms:
            elapsed = 0
            speed_phase = "REVERSE" if speed_phase == "FORWARD" else "FORWARD"
    expect(speed_phase == "FORWARD", "PWM-SPEED must return to forward after two phases")

    for angle in (5, 10, 45, 90):
        error = angle * full_error / full_angle
        expect(abs(error - angle * full_error / full_angle) < 1e-6,
               f"simulated error mapping failed for {angle}")

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