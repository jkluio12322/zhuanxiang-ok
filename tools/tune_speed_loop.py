#!/usr/bin/env python3
"""Run repeatable PWM-SPEED closed-loop tests over SWD and USART.

The car must be lifted. The script changes RAM only, records raw telemetry, and
always returns the controller to the stopped state.
"""

import argparse
import json
import math
import re
import socket
import statistics
import struct
import subprocess
import sys
import time
from pathlib import Path

import serial


ROOT = Path(__file__).resolve().parents[1]
AXF = ROOT / "MDK-ARM" / "build" / "bluecar" / "bluecar.axf"
OPENOCD = Path.home() / ".eide" / "tools" / "openocd_7a1adfbec_mingw32" / "bin" / "openocd.exe"
OPENOCD_SCRIPTS = OPENOCD.parents[1] / "share" / "openocd" / "scripts"

STATE_MODE_OFFSET = 0
STATE_RUNNING_OFFSET = 1
STATE_VOFA_OFFSET = 2
STATE_TARGET_LEFT_OFFSET = 12
STATE_TARGET_RIGHT_OFFSET = 16
PID_KP_OFFSET = 0
PID_KI_OFFSET = 4
MOTION_PWM_OFFSET = 24


def float_word(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def symbol_addresses():
    names = {
        "state",
        "pidLeft",
        "pidRight",
        "motionView",
        "autoPhaseEnabled",
        "speedDir",
        "speedDuty",
        "speedRawEnabled",
        "loop_test_debug_request",
        "loop_test_debug_argument",
        "positionKp",
        "positionKd",
        "positionTrackKp",
        "positionTrackKd",
        "positionDeadbandEnterDeg",
        "positionDeadbandReleaseDeg",
        "positionCreepRpm",
    }
    output = subprocess.check_output(
        ["arm-none-eabi-nm", "-C", str(AXF)],
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    addresses = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] in names:
            addresses[parts[-1]] = int(parts[0], 16)
    missing = names - addresses.keys()
    if missing:
        raise RuntimeError(f"missing symbols: {sorted(missing)}")
    return addresses


class OpenOcdTelnet:
    def __init__(self, process, sock):
        self.process = process
        self.sock = sock
        self.sock.settimeout(8.0)
        self._read_prompt()

    def _read_prompt(self):
        data = bytearray()
        while not data.endswith(b"> "):
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            data.extend(chunk)
        return data.decode("utf-8", errors="ignore")

    def command(self, command):
        self.sock.sendall((command + "\n").encode("ascii"))
        return self._read_prompt()

    def close(self):
        try:
            self.sock.close()
        finally:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()


def start_openocd():
    process = subprocess.Popen(
        [
            str(OPENOCD),
            "-s",
            str(OPENOCD_SCRIPTS),
            "-f",
            "interface/cmsis-dap.cfg",
            "-f",
            "target/stm32f1x.cfg",
        ],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("OpenOCD exited before the Telnet interface was ready")
        try:
            sock = socket.create_connection(("127.0.0.1", 4444), timeout=0.5)
            return OpenOcdTelnet(process, sock)
        except OSError:
            time.sleep(0.1)
    process.terminate()
    raise RuntimeError("OpenOCD Telnet interface did not become ready")


class Controller:
    def __init__(self, ocd, addresses):
        self.ocd = ocd
        self.state = addresses["state"]
        self.pid_left = addresses["pidLeft"]
        self.pid_right = addresses["pidRight"]
        self.motion = addresses["motionView"]
        self.auto_phase = addresses["autoPhaseEnabled"]
        self.speed_dir = addresses["speedDir"]
        self.speed_duty = addresses["speedDuty"]
        self.speed_raw = addresses["speedRawEnabled"]
        self.loop_request = addresses["loop_test_debug_request"]
        self.loop_argument = addresses["loop_test_debug_argument"]
        self.position_kp = addresses["positionKp"]
        self.position_kd = addresses["positionKd"]
        self.position_track_kp = addresses["positionTrackKp"]
        self.position_track_kd = addresses["positionTrackKd"]
        self.position_enter = addresses["positionDeadbandEnterDeg"]
        self.position_release = addresses["positionDeadbandReleaseDeg"]
        self.position_creep = addresses["positionCreepRpm"]

    def _write_byte(self, address, value):
        self.ocd.command(f"mwb 0x{address:08X} {value}")

    def _write_word(self, address, value):
        self.ocd.command(f"mww 0x{address:08X} 0x{value:08X}")

    def set_gains(self, kp, ki, side="both"):
        if side in ("left", "both"):
            self._write_word(self.pid_left + PID_KP_OFFSET, float_word(kp))
            self._write_word(self.pid_left + PID_KI_OFFSET, float_word(ki))
        if side in ("right", "both"):
            self._write_word(self.pid_right + PID_KP_OFFSET, float_word(kp))
            self._write_word(self.pid_right + PID_KI_OFFSET, float_word(ki))

    def stop(self):
        self.ocd.command("halt")
        self._write_byte(self.state + STATE_RUNNING_OFFSET, 0)
        self._write_word(self.state + STATE_TARGET_LEFT_OFFSET, 0)
        self._write_word(self.state + STATE_TARGET_RIGHT_OFFSET, 0)
        self._write_byte(self.speed_dir, 0)
        self.ocd.command("resume")

    def start(self, target, drive_side="both"):
        self.ocd.command("halt")
        self._write_byte(self.state + STATE_MODE_OFFSET, 1)
        self._write_byte(self.state + STATE_VOFA_OFFSET, 1)
        left_target = target if drive_side in ("left", "both") else 0.0
        right_target = target if drive_side in ("right", "both") else 0.0
        self._write_word(self.state + STATE_TARGET_LEFT_OFFSET,
                         float_word(left_target))
        self._write_word(self.state + STATE_TARGET_RIGHT_OFFSET,
                         float_word(right_target))
        self._write_byte(self.state + STATE_RUNNING_OFFSET, 1)
        self._write_byte(self.auto_phase, 0)
        self._write_byte(self.speed_dir, 0)
        self.ocd.command("resume")

    def set_open_loop(self, direction, duty, raw=True):
        self.ocd.command("halt")
        self._write_byte(self.state + STATE_MODE_OFFSET, 1)
        self._write_byte(self.state + STATE_RUNNING_OFFSET, 1)
        self._write_byte(self.state + STATE_VOFA_OFFSET, 1)
        self._write_byte(self.auto_phase, 0)
        self._write_byte(self.speed_dir, 1 if direction > 0 else 0xFF)
        self._write_word(self.speed_duty, float_word(duty))
        self._write_byte(self.speed_raw, 1 if raw else 0)
        self.ocd.command("resume")

    def stop_open_loop(self):
        self.ocd.command("halt")
        self._write_byte(self.speed_dir, 0)
        self._write_byte(self.state + STATE_RUNNING_OFFSET, 0)
        self.ocd.command("resume")

    def loop_debug(self, request, argument=0):
        self.ocd.command("halt")
        self._write_byte(self.loop_request, request)
        self._write_byte(self.loop_argument, argument)
        self.ocd.command("resume")
        time.sleep(0.08)

    def set_position_gains(self, side, kp, kd, enter_deadband,
                           release_deadband, creep_rpm):
        self.ocd.command("halt")
        sides = (0, 1) if side == "both" else ((0,) if side == "left" else (1,))
        for index in sides:
            offset = index * 4
            self._write_word(self.position_kp + offset, float_word(kp))
            self._write_word(self.position_kd + offset, float_word(kd))
            self._write_word(self.position_enter + offset, float_word(enter_deadband))
            self._write_word(self.position_release + offset, float_word(release_deadband))
            self._write_word(self.position_creep + offset, float_word(creep_rpm))
        self.ocd.command("resume")

    def set_continuous_position_gains(self, side, kp, kd):
        self.ocd.command("halt")
        sides = (0, 1) if side == "both" else ((0,) if side == "left" else (1,))
        for index in sides:
            offset = index * 4
            self._write_word(self.position_track_kp + offset, float_word(kp))
            self._write_word(self.position_track_kd + offset, float_word(kd))
        self.ocd.command("resume")

    def read_float(self, address):
        response = self.ocd.command(f"mdw 0x{address:08X} 1")
        match = re.search(r"0x[0-9a-fA-F]+:\s+([0-9a-fA-F]{8})", response)
        if not match:
            raise RuntimeError(f"cannot read float at 0x{address:08X}")
        return struct.unpack("<f", struct.pack("<I", int(match.group(1), 16)))[0]

    def read_motion(self):
        addresses = [
            self.motion + 0,
            self.motion + 4,
            self.motion + 8,
            self.motion + 12,
            self.motion + 16,
            self.motion + 20,
            self.motion + 24,
            self.motion + 28,
        ]
        return [self.read_float(address) for address in addresses]

    def read_pwm(self):
        self.ocd.command("halt")
        response = self.ocd.command(f"mdw 0x{self.motion + MOTION_PWM_OFFSET:08X} 2")
        self.ocd.command("resume")
        match = re.search(
            r"0x[0-9a-fA-F]+:\s+([0-9a-fA-F]{8})\s+([0-9a-fA-F]{8})",
            response,
        )
        if not match:
            return None, None
        left = struct.unpack("<f", struct.pack("<I", int(match.group(1), 16)))[0]
        right = struct.unpack("<f", struct.pack("<I", int(match.group(2), 16)))[0]
        return left, right


def collect_rows(ser, target, duration, drive_side="both"):
    rows = []
    started = None
    hard_deadline = time.monotonic() + duration + 10.0
    while time.monotonic() < hard_deadline:
        raw = ser.readline()
        if not raw:
            continue
        parts = raw.decode("ascii", errors="replace").strip().split(",")
        if len(parts) != 8:
            continue
        try:
            row = [float(value) for value in parts]
        except ValueError:
            continue
        if drive_side == "left":
            if abs(row[0] - target) > 0.1 or abs(row[2]) > 0.1:
                continue
        elif drive_side == "right":
            if abs(row[2] - target) > 0.1 or abs(row[0]) > 0.1:
                continue
        elif abs(row[0] - target) > 0.1 or abs(row[2] - target) > 0.1:
            continue
        now = time.monotonic()
        if started is None:
            started = now
        rows.append((now - started, row))
        if now - started >= duration:
            break
    return rows


def summarize(rows, target, pwm, drive_side="both"):
    if not rows:
        return {"samples": 0}
    settled = rows[40:] if len(rows) > 40 else rows
    left = [item[1][1] for item in settled]
    right = [item[1][3] for item in settled]
    diff = [item[1][5] for item in settled]
    abs_diff = [abs(value) for value in diff]
    actual = left + right if drive_side == "both" else (
        left if drive_side == "left" else right
    )
    abs_error = [abs(value - target) for value in actual]

    def percentile(values, fraction):
        ordered = sorted(values)
        return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]

    summary = {
        "samples": len(rows),
        "steady_samples": len(settled),
        "duration_s": round(rows[-1][0], 2),
        "mean_left_rpm": round(statistics.mean(left), 3),
        "mean_right_rpm": round(statistics.mean(right), 3),
        "mean_error_rpm": round(statistics.mean(abs_error), 3),
        "rms_error_rpm": round(
            math.sqrt(statistics.mean(value * value for value in abs_error)), 3
        ),
        "left_std_rpm": round(statistics.pstdev(left), 3),
        "right_std_rpm": round(statistics.pstdev(right), 3),
        "pwm_left": round(pwm[0], 2) if pwm[0] is not None else None,
        "pwm_right": round(pwm[1], 2) if pwm[1] is not None else None,
    }
    if drive_side == "both":
        summary.update({
            "mean_abs_diff_rpm": round(statistics.mean(abs_diff), 3),
            "p95_abs_diff_rpm": round(percentile(abs_diff, 0.95), 3),
            "max_abs_diff_rpm": round(max(abs_diff), 3),
            "diff_over_3rpm_pct": round(
                100.0 * sum(value > 3.0 for value in abs_diff) / len(abs_diff), 2
            ),
        })
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--target", type=float, default=-50.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--drive-side", choices=("left", "right", "both"), default="both")
    parser.add_argument(
        "--cases",
        default="0.40:0.08,0.40:0.12,0.50:0.08,0.50:0.10,0.50:0.14,0.65:0.08,0.65:0.12,0.65:0.16",
        help="Comma-separated Kp:Ki pairs",
    )
    parser.add_argument("--out", default=str(ROOT / "MDK-ARM" / "codex-logs" / "speed_tuning"))
    args = parser.parse_args()

    cases = []
    for item in args.cases.split(","):
        kp_text, ki_text = item.split(":", 1)
        cases.append((float(kp_text), float(ki_text)))

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

        for repeat_index in range(args.repeat):
            for kp, ki in cases:
                controller.stop()
                time.sleep(0.8)
                controller.set_gains(kp, ki, args.side)
                ser.reset_input_buffer()
                controller.start(args.target, args.drive_side)
                rows = collect_rows(ser, args.target, args.duration, args.drive_side)
                pwm = controller.read_pwm()
                summary = summarize(rows, args.target, pwm, args.drive_side)
                summary.update(
                    {
                        "repeat": repeat_index + 1,
                        "kp": kp,
                        "ki": ki,
                        "target_rpm": args.target,
                        "drive_side": args.drive_side,
                    }
                )
                summaries.append(summary)
                name = f"kp{kp:.2f}_ki{ki:.2f}_r{repeat_index + 1}"
                with (out_dir / f"{name}.csv").open("w", encoding="utf-8") as handle:
                    handle.write(
                        "time_s,target_left,left_rpm,target_right,right_rpm,"
                        "steer_rpm,diff_rpm,yaw_deg,track_phase\n"
                    )
                    for sample_time, row in rows:
                        handle.write(
                            f"{sample_time:.3f}," + ",".join(f"{value:.3f}" for value in row) + "\n"
                        )
                print(
                    f"kp={kp:.2f} ki={ki:.2f} repeat={repeat_index + 1} "
                    f"samples={summary['samples']} "
                    f"mae={summary.get('mean_error_rpm')} "
                    f"diff={summary.get('mean_abs_diff_rpm')} "
                    f"p95={summary.get('p95_abs_diff_rpm')}",
                    flush=True,
                )
    finally:
        try:
            controller.stop()
        except Exception as exc:
            print(f"STOP_ERROR: {exc}", file=sys.stderr)
        if ser is not None:
            ser.close()
        ocd.close()

    summary_path = out_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"summary: {summary_path}")


if __name__ == "__main__":
    main()
