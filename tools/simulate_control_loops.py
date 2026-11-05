#!/usr/bin/env python3
"""Deterministic host simulation for the speed PI and wheel-position PD chain."""

import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "Core/system/config.h"


def macros():
    result = {}
    pattern = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)\s+([^\s/]+)")
    for line in CONFIG.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if not match:
            continue
        name, value = match.groups()
        value = re.sub(r"[uUlLfF]+$", "", value.strip("()"))
        try:
            result[name] = float(value)
        except ValueError:
            pass
    return result


CFG = macros()
DT = CFG["CONTROL_PERIOD_MS"] / 1000.0
KP = CFG["SPEED_PID_KP"]
KI = CFG["SPEED_PID_KI"]
POS_KP = CFG["POSITION_LEFT_KP"]
POS_KD = CFG["POSITION_LEFT_KD"]
SPEED_KI_SCALE = DT / (CFG["SPEED_PID_REFERENCE_PERIOD_MS"] / 1000.0)
POS_KP_BY_SIDE = [CFG["POSITION_TRACK_LEFT_KP"], CFG["POSITION_TRACK_RIGHT_KP"]]
POS_KD_BY_SIDE = [CFG["POSITION_TRACK_LEFT_KD"], CFG["POSITION_TRACK_RIGHT_KD"]]


class Plant:
    def __init__(self):
        self.rpm = [0.0, 0.0]
        self.angle = [0.0, 0.0]
        self.deadzone = [[22.0, 21.0], [21.0, 22.0]]
        self.gain = [[2.18, 2.12], [2.10, 2.16]]

    def target_rpm(self, pwm, side):
        direction = 1 if pwm >= 0 else -1
        row = 0 if direction > 0 else 1
        magnitude = abs(pwm)
        usable = max(0.0, magnitude - self.deadzone[row][side])
        return direction * usable * self.gain[row][side]

    def step(self, pwm, side):
        target = self.target_rpm(pwm, side)
        alpha = DT / (CFG["SPEED_FAKE_TAU_S"] + DT)
        self.rpm[side] += alpha * (target - self.rpm[side])
        self.angle[side] += self.rpm[side] * 6.0 * DT


class SpeedPi:
    def __init__(self):
        self.out = [0.0, 0.0]
        self.e0 = [0.0, 0.0]
        self.e1 = [0.0, 0.0]

    def reset(self):
        self.out = [0.0, 0.0]
        self.e0 = [0.0, 0.0]
        self.e1 = [0.0, 0.0]

    def step(self, target, actual, side):
        error = target - actual
        self.out[side] += (KP * (error - self.e1[side])
                           + KI * error * SPEED_KI_SCALE)
        self.out[side] = max(-100.0, min(100.0, self.out[side]))
        self.e1[side] = self.e0[side]
        self.e0[side] = error
        return self.out[side]


def speed_case(target):
    plant = Plant()
    pi = SpeedPi()
    samples = []
    for _ in range(int(10.0 / DT)):
        for side in range(2):
            pwm = pi.step(target, plant.rpm[side], side)
            plant.step(pwm, side)
        samples.append(tuple(plant.rpm))
    steady = samples[-100:]
    mean_left = sum(v[0] for v in steady) / len(steady)
    mean_right = sum(v[1] for v in steady) / len(steady)
    return abs(mean_left - target), abs(mean_right - target), abs(mean_left - mean_right)


def feedforward(target, side):
    magnitude = abs(target)
    if magnitude == 0.0:
        return 0.0
    deadzone = 22.0 if side == 0 else 21.0
    gain = 2.18 if side == 0 else 2.10
    duty = deadzone + magnitude / gain
    duty *= min(1.0, magnitude / CFG["SPEED_CAL_FF_TAPER_RPM"])
    return math.copysign(min(100.0, duty), target)


def position_case(target_deg):
    plant = Plant()
    pi = SpeedPi()
    reference = 0.0
    reference_velocity = 0.0
    max_rpm = CFG["POSITION_LEVEL_LOW_RPM"]
    accel = CFG["POSITION_LEVEL_LOW_ACCEL"]
    holding = [False, False]
    settling = [False, False]
    first_cross = None
    settle = None
    for index in range(int(8.0 / DT)):
        remaining = target_deg - reference
        # RPM -> deg/s adds a factor of 6; solve v^2 = 2*a*s in degree units.
        desired = math.sqrt(max(0.0, accel * abs(remaining) / 3.0))
        if abs(reference_velocity) > desired:
            desired = math.sqrt(max(
                0.0,
                CFG["POSITION_LEVEL_LOW_DECEL_SCALE"] * accel * abs(remaining) / 3.0,
            ))
        desired = min(max_rpm, desired)
        desired *= 1.0 if remaining >= 0 else -1.0
        delta = desired - reference_velocity
        reference_velocity += max(-accel * DT, min(accel * DT, delta))
        if abs(reference - target_deg) <= abs(reference_velocity) * 6.0 * DT:
            reference = target_deg
            reference_velocity = 0.0
        else:
            reference += reference_velocity * 6.0 * DT

        speeds = []
        for side in range(2):
            error = target_deg - plant.angle[side]
            magnitude = abs(error)
            if not holding[side] and magnitude <= CFG["POSITION_LEFT_ENTER_DEG"]:
                holding[side] = True
                settling[side] = True
            elif holding[side] and magnitude >= CFG["POSITION_LEFT_RELEASE_DEG"]:
                holding[side] = False

            if settling[side]:
                command = 0.0
                if abs(plant.rpm[side]) <= CFG["POSITION_SETTLE_RPM_MAX"]:
                    if magnitude < CFG["POSITION_LEFT_RELEASE_DEG"]:
                        holding[side] = True
                    settling[side] = False
            elif holding[side]:
                command = 0.0
            else:
                limited_reference_velocity = reference_velocity
                if limited_reference_velocity * error > 0.0:
                    braking_speed = math.sqrt(
                        CFG["POSITION_LEVEL_LOW_ACCEL"]
                        * CFG["POSITION_LEVEL_LOW_DECEL_SCALE"]
                        * magnitude / 3.0
                    )
                    limited_reference_velocity = math.copysign(
                        min(abs(limited_reference_velocity), braking_speed),
                        limited_reference_velocity,
                    )
                if abs(reference_velocity) < 0.01:
                    approach = math.sqrt(max(
                        0.0, CFG["POSITION_LEVEL_LOW_APPROACH_SCALE"]
                        * accel * magnitude / 3.0
                    ))
                    approach = min(max_rpm, approach)
                    command = math.copysign(approach, error) - POS_KD * plant.rpm[side]
                else:
                    command = limited_reference_velocity + POS_KP * error \
                              - POS_KD * plant.rpm[side]
                if abs(command) < CFG["POSITION_LEFT_CREEP_RPM"] and error != 0.0:
                    command = math.copysign(CFG["POSITION_LEFT_CREEP_RPM"], error)
            command = max(-max_rpm, min(max_rpm, command))
            speeds.append(command)
            pwm = feedforward(command, side) + pi.step(command, plant.rpm[side], side)
            plant.step(pwm, side)

        errors = [abs(plant.angle[s] - target_deg) for s in range(2)]
        if first_cross is None and all(a >= target_deg for a in plant.angle):
            first_cross = index * DT
        if all(holding[s] and abs(plant.rpm[s]) <= CFG["POSITION_SETTLE_RPM_MAX"]
               for s in range(2)):
            settle = index * DT
    return max(abs(plant.angle[s] - target_deg) for s in range(2)), settle


def continuous_position_case(desired_rpm):
    plant = Plant()
    pi = SpeedPi()
    reference = [0.0, 0.0]
    errors = [[], []]

    for _ in range(int(10.0 / DT)):
        for side in range(2):
            position_error = reference[side] - plant.angle[side]
            command = (desired_rpm
                       + POS_KP_BY_SIDE[side] * position_error
                       + POS_KD_BY_SIDE[side] * (desired_rpm - plant.rpm[side]))
            command = max(-CFG["SPEED_TARGET_LIMIT_RPM"],
                          min(CFG["SPEED_TARGET_LIMIT_RPM"], command))
            pwm = feedforward(command, side) + pi.step(command, plant.rpm[side], side)
            plant.step(pwm, side)
            reference[side] += desired_rpm * 6.0 * DT
            errors[side].append(abs(reference[side] - plant.angle[side]))

    steady = []
    for side in range(2):
        tail = errors[side][-int(5.0 / DT):]
        steady.append(percentile(tail, 0.95))
    return steady


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def main():
    failures = []
    for target in (-140.0, -80.0, -30.0, 30.0, 80.0, 140.0):
        errors = speed_case(target)
        ok = max(errors) <= 0.5
        print(f"SPEED {target:+.0f}: errors={tuple(round(v, 3) for v in errors)} "
              f"{'PASS' if ok else 'FAIL'}")
        if not ok:
            failures.append(f"speed {target}")

    for target in (-60.0, -45.0, -30.0, 30.0, 45.0, 60.0):
        error, settle = position_case(target)
        ok = error <= CFG["POSITION_LEFT_RELEASE_DEG"] and settle is not None
        print(f"POS {target:+.0f}: final_error={error:.3f} "
              f"settle={settle} {'PASS' if ok else 'FAIL'}")
        if not ok:
            failures.append(f"position {target}")

    for desired in (30.0, 80.0, 140.0):
        errors = continuous_position_case(desired)
        ok = max(errors) <= 1.5
        print(f"CONTINUOUS {desired:+.0f}: position_error_p95="
              f"{tuple(round(v, 3) for v in errors)} {'PASS' if ok else 'FAIL'}")
        if not ok:
            failures.append(f"continuous {desired}")

    if failures:
        print("FAILED:", ", ".join(failures))
        return 1
    print("All control-loop simulations passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
