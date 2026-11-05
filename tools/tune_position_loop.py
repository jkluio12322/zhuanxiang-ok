#!/usr/bin/env python3
"""Sweep wheel-position Kp/Kd using LOOP-TEST POS-CL telemetry."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import serial

from tune_speed_loop import Controller, ROOT, start_openocd, symbol_addresses


def collect(ser, duration):
    rows = []
    started = None
    deadline = time.monotonic() + duration + 5.0
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        parts = raw.decode("ascii", errors="replace").strip().split(",")
        if len(parts) != 12:
            continue
        try:
            row = [float(value) for value in parts]
        except ValueError:
            continue
        now = time.monotonic()
        if started is None:
            started = now
        rows.append((now - started, row))
        if now - started >= duration:
            break
    return rows


def analyze(rows, enter_deadband, side="left"):
    if not rows:
        return {"samples": 0}

    target_index = 0 if side == "left" else 2
    actual_index = 1 if side == "left" else 3
    rpm_index = 4 if side == "left" else 6
    error_index = 10 if side == "left" else 11
    transitions = []
    previous_target = rows[0][1][target_index]
    transitions.append((0.0, previous_target))
    for timestamp, row in rows[1:]:
        if abs(row[target_index] - previous_target) > 0.05:
            transitions.append((timestamp, row[target_index]))
            previous_target = row[target_index]

    overshoots = []
    settle_times = []
    final_errors = []
    stable_rms_errors = []
    settled_by_sequence = {}
    reversals = []
    tail_start = transitions[-1][0] if transitions else 0.0
    truncated_tail = ((len(transitions) > 1)
                      and ((rows[-1][0] - tail_start) < 1.0))
    for index, (start_time, target) in enumerate(transitions):
        complete = (index + 1 < len(transitions)
                    and not (truncated_tail and index + 2 == len(transitions)))
        end_time = transitions[index + 1][0] if index + 1 < len(transitions) else rows[-1][0]
        segment = [item for item in rows if start_time <= item[0] < end_time]
        if len(segment) < 5:
            continue
        initial_actual = segment[0][1][actual_index]
        direction = 1.0 if target >= initial_actual else -1.0
        max_overshoot = 0.0
        for _, row in segment:
            overshoot = (row[actual_index] - target) * direction
            if overshoot > 0.0:
                max_overshoot = max(max_overshoot, overshoot)
        overshoots.append(max_overshoot)

        if complete:
            outside = [timestamp for timestamp, row in segment
                       if abs(row[error_index]) > enter_deadband]
            settle = (outside[-1] + 0.05 - start_time) if outside else 0.0
            settle_times.append(settle)
            final_errors.append(abs(segment[-1][1][error_index]))
            tail = segment[-20:]
            tail_errors = [abs(row[error_index]) for _, row in tail]
            stable_rms_errors.append(
                math.sqrt(statistics.mean(e * e for e in tail_errors)))
            settled_by_sequence.setdefault(index % 6, []).append(tail_errors[-1])

        signs = []
        for _, row in segment:
            if abs(row[rpm_index]) > 2.0:
                signs.append(1 if row[rpm_index] > 0 else -1)
        if complete:
            reversals.append(sum(1 for a, b in zip(signs, signs[1:]) if a != b))

    errors = [abs(row[1][error_index]) for row in rows]
    return {
        "samples": len(rows),
        "duration_s": round(rows[-1][0], 2),
        "completed_segments": len(final_errors),
        "position_mae_deg": round(statistics.mean(errors), 3),
        "position_rms_deg": round(math.sqrt(statistics.mean(e * e for e in errors)), 3),
        "max_overshoot_deg": round(max(overshoots) if overshoots else 0.0, 3),
        "max_settle_s": round(max(settle_times) if settle_times else 0.0, 3),
        "max_final_error_deg": round(max(final_errors) if final_errors else 0.0, 3),
        "max_stable_rms_deg": round(max(stable_rms_errors) if stable_rms_errors else 0.0, 3),
        "settled_final_by_sequence_deg": {
            str(key): [round(value, 3) for value in values]
            for key, values in settled_by_sequence.items()
        },
        "max_direction_reversals": max(reversals) if reversals else 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--option", type=int, choices=(0, 1, 2), default=0)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--enter-deadband", type=float, default=0.5)
    parser.add_argument("--release-deadband", type=float, default=1.0)
    parser.add_argument("--creep-rpm", type=float, default=12.0)
    parser.add_argument(
        "--cases",
        default="0.5:0,0.8:0,1.2:0,1.6:0,0.8:0.1,1.2:0.1,1.2:0.2,1.2:0.35",
        help="Comma-separated Kp:Kd pairs",
    )
    parser.add_argument(
        "--out",
        default=str(ROOT / "MDK-ARM" / "codex-logs" / "position_tuning"),
    )
    args = parser.parse_args()

    cases = []
    for item in args.cases.split(","):
        kp, kd = item.split(":", 1)
        cases.append((float(kp), float(kd)))

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    addresses = symbol_addresses()
    ocd = start_openocd()
    controller = Controller(ocd, addresses)
    ser = None
    summaries = []

    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=0.03)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ocd.command("reset run")
        time.sleep(2.5)

        for kp, kd in cases:
            controller.loop_debug(4)
            time.sleep(2.0)
            controller.set_position_gains(
                args.side, kp, kd, args.enter_deadband,
                args.release_deadband, args.creep_rpm)
            controller.loop_debug(1, 3)
            controller.loop_debug(2, args.option)
            controller.loop_debug(5, 1 if args.side == "left" else 2)
            ser.reset_input_buffer()
            controller.loop_debug(3)
            time.sleep(0.2)
            rows = collect(ser, args.duration)
            controller.loop_debug(4)
            summary = analyze(rows, args.enter_deadband, args.side)
            summary.update({
                "kp": kp,
                "kd": kd,
                "option": args.option,
                "side": args.side,
                "enter_deadband": args.enter_deadband,
                "release_deadband": args.release_deadband,
            })
            summaries.append(summary)
            name = f"kp{kp:.3f}_kd{kd:.3f}_opt{args.option}"
            with (out_dir / f"{name}.csv").open("w", encoding="utf-8") as handle:
                handle.write(
                    "time_s,tgt_angle_l,angle_l,tgt_angle_r,angle_r,"
                    "tgt_rpm_l,rpm_l,tgt_rpm_r,rpm_r,pwm_l,pwm_r,err_l,err_r\n")
                for timestamp, row in rows:
                    handle.write(f"{timestamp:.3f}," +
                                 ",".join(f"{value:.3f}" for value in row) + "\n")
            print(
                f"kp={kp:.3f} kd={kd:.3f} n={summary['samples']} "
                f"mae={summary.get('position_mae_deg')} "
                f"os={summary.get('max_overshoot_deg')} "
                f"settle={summary.get('max_settle_s')} "
                f"stable_rms={summary.get('max_stable_rms_deg')} "
                f"trend={summary.get('settled_final_by_sequence_deg')}",
                flush=True,
            )
    finally:
        try:
            controller.loop_debug(4)
        except Exception:
            pass
        if ser is not None:
            ser.close()
        ocd.close()

    path = out_dir / "summary.json"
    path.write_text(json.dumps(summaries, ensure_ascii=False, indent=2),
                    encoding="utf-8")
    print(f"summary: {path}")


if __name__ == "__main__":
    main()
