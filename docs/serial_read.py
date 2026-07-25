#!/usr/bin/env python3
"""按端口读取串口若干秒，用于快速验证固件输出 / ESP-NOW 链路。

用法：
    python serial_read.py <PORT> [seconds] [baud]
例：
    python serial_read.py COM10 7 115200
"""
import sys
import time

# 锁定控制台为 UTF-8，避免中文在 Windows 终端乱码
try:
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.kernel32.SetConsoleOutputCP(65001)
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import serial  # pyserial


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: python serial_read.py <PORT> [seconds] [baud]")
        return 2
    port = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 7.0
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    with serial.Serial(port, baud, timeout=0.5) as s:
        print(f"[serial_read] {port} @ {baud}, reading {seconds}s ...")
        end = time.time() + seconds
        count = 0
        while time.time() < end:
            line = s.readline().decode(errors="replace").strip()
            if line:
                count += 1
                print(line)
        print(f"[serial_read] done, {count} line(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
