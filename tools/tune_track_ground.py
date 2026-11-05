#!/usr/bin/env python3
"""Collect one on-ground TRACK-PID run over USART1 and summarize it."""

import argparse
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "MDK-ARM" / "codex-logs"
HEADER = "#TRACK_DIAG:"


def send(ser, command, delay=0.18):
    ser.write((command + "\r").encode("ascii"))
    ser.flush()
    time.sleep(delay)


def optional_gain(ser, command, value):
    if value is None:
        return
    send(ser, f"{command} {value:.4f}")


def parse_rows(lines):
    rows = []
    for line in lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) != 30:
            continue
        try:
            rows.append([float(value) for value in parts])
        except ValueError:
            continue
    return rows


def rms(values):
    return math.sqrt(statistics.mean(value * value for value in values)) if values else 0.0


def summarize(rows, final_show):
    if not rows:
        return {"samples": 0, "final_show": final_show}

    running_rows = [row for row in rows if row[1] >= 1.0]
    valid_rows = [row for row in running_rows if row[4] != 0.0 and row[5] == 1.0]
    speed_rows = [
        row for row in running_rows
        if abs(row[16]) > 0.5 and abs(row[21]) > 0.5
    ]

    lost_segments = 0
    previous_state = None
    steering_rate = []
    for previous, current in zip(rows, rows[1:]):
        state = int(current[5])
        if state == 2 and previous_state != 2:
            lost_segments += 1
        previous_state = state
        dt = (current[0] - previous[0]) / 1000.0
        if dt > 0.0:
            steering_rate.append(abs(current[7] - previous[7]) / dt)

    left_speed_error = [abs(row[16] - row[17]) for row in speed_rows]
    right_speed_error = [abs(row[21] - row[22]) for row in speed_rows]
    e_y = [row[6] for row in valid_rows]

    return {
        "samples": len(rows),
        "duration_s": round((rows[-1][0] - rows[0][0]) / 1000.0, 3),
        "final_lap": int(max(row[2] for row in rows)),
        "gear": int(rows[-1][3]),
        "valid_ratio": round(len(valid_rows) / len(running_rows), 4) if running_rows else 0.0,
        "lost_segments": lost_segments,
        "max_lost_yaw_deg": round(max(abs(row[25]) for row in rows), 3),
        "line_error_rms": round(rms(e_y), 4),
        "line_error_abs_mean": round(statistics.mean(abs(value) for value in e_y), 4) if e_y else 0.0,
        "line_error_abs_p95": round(sorted(abs(value) for value in e_y)[int(0.95 * (len(e_y) - 1))], 4) if e_y else 0.0,
        "steer_ratio_abs_rms": round(rms([row[7] for row in valid_rows]), 4),
        "steer_rate_abs_p95": round(sorted(steering_rate)[int(0.95 * (len(steering_rate) - 1))], 4) if steering_rate else 0.0,
        "speed_error_left_mae_rpm": round(statistics.mean(left_speed_error), 4) if left_speed_error else 0.0,
        "speed_error_right_mae_rpm": round(statistics.mean(right_speed_error), 4) if right_speed_error else 0.0,
        "speed_error_left_p95_rpm": round(sorted(left_speed_error)[int(0.95 * (len(left_speed_error) - 1))], 4) if left_speed_error else 0.0,
        "speed_error_right_p95_rpm": round(sorted(right_speed_error)[int(0.95 * (len(right_speed_error) - 1))], 4) if right_speed_error else 0.0,
        "position_error_left_p95_deg": round(sorted(abs(row[15]) for row in valid_rows)[int(0.95 * (len(valid_rows) - 1))], 4) if valid_rows else 0.0,
        "position_error_right_p95_deg": round(sorted(abs(row[20]) for row in valid_rows)[int(0.95 * (len(valid_rows) - 1))], 4) if valid_rows else 0.0,
        "kp": rows[-1][27],
        "kd": rows[-1][28],
        "gyro_kp": rows[-1][29],
        "final_show": final_show,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--gear", type=int, default=1, choices=range(1, 7))
    parser.add_argument("--kp", type=float)
    parser.add_argument("--kd", type=float)
    parser.add_argument("--tkg", type=float)
    parser.add_argument("--plkp", type=float)
    parser.add_argument("--plkd", type=float)
    parser.add_argument("--prkp", type=float)
    parser.add_argument("--prkd", type=float)
    parser.add_argument("--slkp", type=float)
    parser.add_argument("--slki", type=float)
    parser.add_argument("--srkp", type=float)
    parser.add_argument("--srki", type=float)
    parser.add_argument("--label", default="track")
    args = parser.parse_args()

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    stem = f"{stamp}-{args.label}"
    csv_path = LOG_DIR / f"{stem}.csv"
    json_path = LOG_DIR / f"{stem}.json"

    lines = []
    final_show = []
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.1
    ser.rtscts = False
    ser.dsrdtr = False
    ser.rts = False
    ser.dtr = False
    ser.open()
    try:
        send(ser, "stop", 0.45)
        send(ser, "log")
        send(ser, "quiet")
        send(ser, "mode 0")
        send(ser, f"gear {args.gear}")
        optional_gain(ser, "tkp", args.kp)
        optional_gain(ser, "tkd", args.kd)
        optional_gain(ser, "tkg", args.tkg)
        optional_gain(ser, "plkp", args.plkp)
        optional_gain(ser, "plkd", args.plkd)
        optional_gain(ser, "prkp", args.prkp)
        optional_gain(ser, "prkd", args.prkd)
        optional_gain(ser, "slkp", args.slkp)
        optional_gain(ser, "slki", args.slki)
        optional_gain(ser, "srkp", args.srkp)
        optional_gain(ser, "srki", args.srki)
        ser.reset_input_buffer()
        send(ser, "trackdiag")
        send(ser, "go", 0.15)

        deadline = time.monotonic() + args.duration
        seen_running = False
        stopped_rows = 0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if not text:
                continue
            lines.append(text)
            if text.startswith(HEADER):
                continue
            parts = text.split(",")
            if len(parts) == 30:
                try:
                    values = [float(part) for part in parts]
                except ValueError:
                    continue
                running = values[1] >= 1.0
                stopped = (
                    not running
                    and abs(values[9]) < 0.5
                    and abs(values[11]) < 0.5
                )
                if running:
                    seen_running = True
                    stopped_rows = 0
                elif seen_running and stopped:
                    stopped_rows += 1
                    if stopped_rows >= 10:
                        break
                else:
                    stopped_rows = 0
    except KeyboardInterrupt:
        pass
    finally:
        try:
            send(ser, "stop", 0.25)
            send(ser, "stop", 0.45)
            send(ser, "trackdiag", 0.15)
            send(ser, "show", 0.45)
            final_show = [
                line.strip() for line in ser.read_all().decode(
                    "ascii", errors="replace"
                ).splitlines() if line.strip()
            ]
            send(ser, "log", 0.15)
        finally:
            ser.close()

    rows = parse_rows(lines)
    summary = summarize(rows, final_show)
    summary["label"] = args.label
    summary["requested_duration_s"] = args.duration
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    for key, value in summary.items():
        if key == "final_show":
            continue
        print(f"{key}: {value}")
    print(f"raw: {csv_path}")
    print(f"summary: {json_path}")


if __name__ == "__main__":
    main()
