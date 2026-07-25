#!/usr/bin/env python3
"""双板串口实时监视：自动识别 A/B 板（按固定 MAC），带颜色前缀同时显示。

用法：
    python monitor.py                 # 自动发现所有 ESP32-S3(303A) 串口并监视
    python monitor.py COM7 COM10      # 只监视指定端口（板名仍按 MAC 自动判定）
    python monitor.py --seconds 8     # 只监视 8 秒后自动退出（用于快速验收/取样）

按 Ctrl+C 退出。
"""
import sys
import time
import threading

# 锁定控制台为 UTF-8，避免中文标签在 Windows 终端乱码
try:
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.kernel32.SetConsoleOutputCP(65001)
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import serial  # pyserial
import serial.tools.list_ports as list_ports

# 固定硬件特征：MAC -> 板名（新板到位后在这里补一行即可）
BOARD_BY_MAC = {
    "E8:3D:C1:F2:C7:B8": "A",
    "E8:3D:C1:FA:7A:0C": "B",
}
COLOR = {"A": "\033[92m", "B": "\033[96m", "?": "\033[93m"}  # 绿/青/黄
RESET = "\033[0m"
DIM = "\033[90m"       # 灰：IMU 遥测（高频，弱化显示不喧宾夺主）
IMU_COLOR = "\033[95m"  # 品红：IMU 数值高亮
BAUD = 115200

STOP = False


def format_line(line: str) -> str:
    """把 IMU 遥测行解析成对齐易读的形式；其余行原样返回。
    输入形如：IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ms>（加速度 g / 角速度 °/s）。
    """
    if not line.startswith("IMU,"):
        return line
    parts = line.split(",")
    if len(parts) != 8:
        return line  # 字段数不符，保底原样输出
    try:
        ax, ay, az, gx, gy, gz = (float(x) for x in parts[1:7])
        ms = parts[7]
    except ValueError:
        return line
    return (
        f"{DIM}IMU{RESET} "
        f"a[{IMU_COLOR}{ax:+.2f} {ay:+.2f} {az:+.2f}{RESET}]g "
        f"g[{IMU_COLOR}{gx:+6.1f} {gy:+6.1f} {gz:+6.1f}{RESET}]°/s "
        f"{DIM}@{ms}ms{RESET}"
    )



def mac_from_hwid(hwid: str) -> str:
    # 下载态/USB-JTAG 的 hwid 里含 SER=<MAC>
    for tok in hwid.split():
        if tok.upper().startswith("SER=") and ":" in tok:
            return tok[4:].upper()
    return ""


def discover():
    """返回 [(port, letter, mac), ...]，只挑乐鑫 303A 的口。"""
    found = []
    for p in list_ports.comports():
        if "303A" in p.hwid.upper():
            mac = mac_from_hwid(p.hwid)
            found.append((p.device, BOARD_BY_MAC.get(mac, "?"), mac))
    return found


def reader(port: str, letter: str):
    tag = f"{COLOR.get(letter, COLOR['?'])}[{letter}板/{port}]{RESET}"
    try:
        s = serial.Serial(port, BAUD, timeout=0.5)
    except Exception as e:  # 端口被占用/不存在
        print(f"{tag} 打开失败：{e}")
        return
    with s:
        while not STOP:
            line = s.readline().decode(errors="replace").strip()
            if line:
                print(f"{tag} {format_line(line)}", flush=True)


def main() -> int:
    global STOP
    args = sys.argv[1:]
    seconds = None
    ports_arg = []
    i = 0
    while i < len(args):
        if args[i] == "--seconds" and i + 1 < len(args):
            seconds = float(args[i + 1])
            i += 2
        else:
            ports_arg.append(args[i])
            i += 1

    disc = {d: (l, m) for d, l, m in discover()}
    if ports_arg:
        targets = [(d, disc.get(d, ("?", ""))[0], disc.get(d, ("?", ""))[1]) for d in ports_arg]
    else:
        targets = discover()

    if not targets:
        print("未发现 ESP32-S3(303A) 串口。请检查连线，或先运行 pio device list")
        return 1

    print("== 双板串口监视（Ctrl+C 退出）==")
    for d, l, m in targets:
        print(f"  {d} -> {l}板   MAC={m or 'N/A'}")
    print("-" * 40)

    threads = []
    for d, l, _ in targets:
        t = threading.Thread(target=reader, args=(d, l), daemon=True)
        t.start()
        threads.append(t)

    end = time.time() + seconds if seconds else None
    try:
        while True:
            if end and time.time() >= end:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        STOP = True
        time.sleep(0.6)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
