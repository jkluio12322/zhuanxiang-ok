# -*- coding: utf-8 -*-
"""转角自检上位机驱动（测试脚本）。

    python turntest_run.py 45 3 [--port COM19] [--secs 240]

流程：Z 关采集帧 → q 关诊断推送 → B deg laps → 打印每步结果 → 结束补 q。
"""
import argparse
import sys
import time

import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("deg", type=float)
    ap.add_argument("laps", type=int)
    ap.add_argument("--port", default="COM19")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=240.0)
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    # 板子复位后 console 要几百毫秒才起来，早期发的命令会丢：发完等回显，没回显就重发
    time.sleep(1.0)
    for cmd, ack in ((b"Z", "trace off"), (b"q", "\u81ea\u52a8\u8bca\u65ad")):
        for _ in range(5):
            ser.reset_input_buffer()
            ser.write(cmd + b"\r\n")
            t = time.time()
            got = b""
            while time.time() - t < 0.6:
                got += ser.read(128)
                if ack.encode("utf-8") in got:
                    break
            if ack.encode("utf-8") in got:
                break
        else:
            print("警告: %s 无回显" % cmd.decode())
    ser.reset_input_buffer()

    # 上电后陀螺要沉降 2 s 再标定，太早发 B 会回 ERR gyro not ready：等到能起为止
    cmd_b = ("B %g %d\r\n" % (a.deg, a.laps)).encode()
    for attempt in range(12):
        ser.reset_input_buffer()
        ser.write(cmd_b)
        print("发送: B %g %d" % (a.deg, a.laps), flush=True)
        t = time.time()
        got = b""
        while time.time() - t < 1.0:
            got += ser.read(128)
            if b"OK selftest" in got or b"not ready" in got:
                break
        if b"OK selftest" in got:
            break
        time.sleep(1.0)
    else:
        print("陀螺一直没就绪，退出", flush=True)
        return

    t0 = time.time()
    buf = b""
    steps = 0
    done = False
    noise = 0
    while time.time() - t0 < a.secs:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", "replace").strip()
            if not text:
                continue
            if "[自检]" in text:
                print("%6.1fs %s" % (time.time() - t0, text), flush=True)
                if "汇总" in text or "中止" in text:
                    done = True
            elif text.startswith("OK") or text.startswith("ERR"):
                print("%6.1fs %s" % (time.time() - t0, text), flush=True)
            else:
                noise += 1
        if done:
            break

    ser.write(b"q\r\n")
    time.sleep(0.2)
    ser.close()
    print("结束: %s，非自检噪声行 %d" % ("收到汇总" if done else "超时", noise))
    return 0 if done else 1

if __name__ == "__main__":
    sys.exit(main())
