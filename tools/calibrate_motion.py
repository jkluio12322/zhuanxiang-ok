#!/usr/bin/env python3
"""Bench calibration for wheel deadzones and open-loop RPM LUTs."""

import argparse
import json
import statistics
import time
from pathlib import Path

import serial

from tune_speed_loop import Controller, ROOT, start_openocd, symbol_addresses


DUTY_POINTS = [float(value) for value in range(0, 101, 10)]


def read_rows(ser, duration, min_rows=8):
    rows = []
    deadline = time.monotonic() + duration + 1.5
    while time.monotonic() < deadline and len(rows) < min_rows:
        raw = ser.readline()
        if not raw:
            continue
        parts = raw.decode("ascii", errors="replace").strip().split(",")
        if len(parts) != 8:
            continue
        try:
            rows.append([float(value) for value in parts])
        except ValueError:
            pass
    return rows


def measure(controller, ser, direction, duty, settle=1.2, sample=0.6):
    controller.stop_open_loop()
    time.sleep(0.45)
    ser.reset_input_buffer()
    controller.set_open_loop(direction, duty, raw=True)
    time.sleep(settle)
    rows = read_rows(ser, sample, min_rows=8)
    controller.stop_open_loop()
    if not rows:
        return 0.0, 0.0
    return (statistics.mean(row[1] for row in rows),
            statistics.mean(row[3] for row in rows))


def side_value(values, side):
    return abs(values[0 if side == "left" else 1])


def measure_repeated(controller, ser, direction, duty, repeats):
    samples = [measure(controller, ser, direction, duty) for _ in range(repeats)]
    return (statistics.median(item[0] for item in samples),
            statistics.median(item[1] for item in samples))


def find_start_deadzone(controller, ser, direction, side, step=2.0,
                        limit=40.0, repeats=3):
    previous = 0.0
    coarse_high = None
    measurements = {}
    duty = step
    while duty <= limit:
        values = measure_repeated(controller, ser, direction, duty, repeats)
        measurements[duty] = values
        if side_value(values, side) >= 2.0:
            coarse_high = duty
            break
        previous = duty
        duty += step
    if coarse_high is None:
        return None, measurements

    low = previous
    high = coarse_high
    while (high - low) > 0.25:
        middle = round((low + high) / 2.0 * 4.0) / 4.0
        values = measure_repeated(controller, ser, direction, middle, repeats)
        measurements[middle] = values
        if side_value(values, side) >= 2.0:
            high = middle
        else:
            low = middle
    return high, measurements


def find_sustain_deadzone(controller, ser, direction, side, start_duty):
    duty = min(100.0, start_duty + 5.0)
    controller.stop_open_loop()
    time.sleep(0.4)
    ser.reset_input_buffer()
    controller.set_open_loop(direction, duty, raw=True)
    time.sleep(1.5)

    previous = duty
    stopped_count = 0
    while duty >= 0.0:
        rows = read_rows(ser, 0.65, min_rows=8)
        value = abs(statistics.mean(row[1 if side == "left" else 3] for row in rows)) if rows else 0.0
        if value < 1.0:
            stopped_count += 1
            if stopped_count >= 2:
                controller.stop_open_loop()
                return previous
        else:
            stopped_count = 0
        previous = duty
        duty = round((duty - 0.25) * 4.0) / 4.0
        controller.set_open_loop(direction, max(0.0, duty), raw=True)
        time.sleep(0.65)

    controller.stop_open_loop()
    return 0.0


def calibrate_lut(controller, ser, repeats, duration):
    result = {"fwd": {"left": [], "right": []}, "rev": {"left": [], "right": []}}
    raw = []
    for direction_name, direction in (("fwd", 1), ("rev", -1)):
        for duty in DUTY_POINTS:
            samples = []
            for _ in range(repeats):
                values = measure(controller, ser, direction, duty,
                                 settle=1.2, sample=max(0.6, duration - 0.6))
                samples.append(values)
                time.sleep(0.2)
            left = statistics.median(item[0] for item in samples)
            right = statistics.median(item[1] for item in samples)
            result[direction_name]["left"].append(round(left, 3))
            result[direction_name]["right"].append(round(right, 3))
            raw.append({
                "direction": direction_name,
                "duty": duty,
                "samples": [{"left": round(item[0], 3), "right": round(item[1], 3)}
                            for item in samples],
            })
            print(f"{direction_name} {duty:5.1f}%  L={left:7.2f}  R={right:7.2f}",
                  flush=True)
    return result, raw


def c_initializer(name, values):
    lines = [f"static const float {name}[SPEED_CAL_POINT_COUNT] = {{"]
    chunks = [f"{value:.3f}f" for value in values]
    for index in range(0, len(chunks), 6):
        lines.append("    " + ", ".join(chunks[index:index + 6]) + ",")
    lines.append("};")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--mode", choices=("encoder", "deadzone", "lut", "all"),
                        default="all")
    parser.add_argument("--encoder-duty", type=float, default=20.0)
    parser.add_argument("--deadzone-step", type=float, default=2.0)
    parser.add_argument("--deadzone-repeats", type=int, default=3)
    parser.add_argument("--lut-repeats", type=int, default=3)
    parser.add_argument("--lut-duration", type=float, default=5.0)
    parser.add_argument(
        "--out",
        default=str(ROOT / "MDK-ARM" / "codex-logs" / "motion_calibration"),
    )
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    addresses = symbol_addresses()
    ocd = start_openocd()
    controller = Controller(ocd, addresses)
    ser = None
    output = {}

    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=0.03)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ocd.command("reset run")
        time.sleep(2.5)

        if args.mode in ("encoder", "all"):
            encoder = {}
            controller.stop_open_loop()
            time.sleep(0.3)
            before = controller.read_motion()
            for direction_name, direction in (("fwd", 1), ("rev", -1)):
                controller.set_open_loop(direction, args.encoder_duty, raw=True)
                time.sleep(3.0)
                controller.stop_open_loop()
                time.sleep(0.6)
                after = controller.read_motion()
                encoder[direction_name] = {
                    "delta_left_deg": round(after[4] - before[4], 3),
                    "delta_right_deg": round(after[5] - before[5], 3),
                    "left_rpm": round(after[0], 3),
                    "right_rpm": round(after[1], 3),
                }
                before = after
            output["encoder"] = encoder
            print(f"encoder: {json.dumps(encoder, ensure_ascii=False)}", flush=True)

        if args.mode in ("deadzone", "all"):
            deadzone = {}
            for direction_name, direction in (("fwd", 1), ("rev", -1)):
                deadzone[direction_name] = {}
                for side in ("left", "right"):
                    start, measurements = find_start_deadzone(
                        controller, ser, direction, side, args.deadzone_step,
                        repeats=args.deadzone_repeats)
                    sustain = (find_sustain_deadzone(
                        controller, ser, direction, side, start)
                        if start is not None else None)
                    deadzone[direction_name][side] = {
                        "start": start,
                        "sustain": sustain,
                        "measurements": {
                            str(duty): {"left": round(value[0], 3),
                                        "right": round(value[1], 3)}
                            for duty, value in measurements.items()
                        },
                    }
                    print(f"{direction_name} {side}: start={start} sustain={sustain}",
                          flush=True)
            output["deadzone"] = deadzone

        if args.mode in ("lut", "all"):
            lut, raw = calibrate_lut(controller, ser, args.lut_repeats,
                                     args.lut_duration)
            output["lut"] = lut
            output["raw_lut_samples"] = raw

    finally:
        try:
            controller.stop_open_loop()
        except Exception:
            pass
        if ser is not None:
            ser.close()
        ocd.close()

    json_path = out_dir / "calibration.json"
    json_path.write_text(json.dumps(output, ensure_ascii=False, indent=2),
                         encoding="utf-8")

    if "lut" in output:
        snippet = "\n\n".join([
            c_initializer("rpmFwdLeft", output["lut"]["fwd"]["left"]),
            c_initializer("rpmFwdRight", output["lut"]["fwd"]["right"]),
            c_initializer("rpmRevLeft", output["lut"]["rev"]["left"]),
            c_initializer("rpmRevRight", output["lut"]["rev"]["right"]),
        ])
        (out_dir / "speed_calibration_tables.c.txt").write_text(
            snippet + "\n", encoding="utf-8")
    print(f"output: {json_path}")


if __name__ == "__main__":
    main()
