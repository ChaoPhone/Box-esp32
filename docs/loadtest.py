#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP-NOW 工况压测 / 丢包率测量（一键验收）

原理：
  命令 B 板（发送端）按指定频率连续发 <count> 个带序号的 TEST 帧，
  A 板（接收端）按序号统计收到/丢失，压测结束时打印精确丢包率。

用法：
  python docs\\loadtest.py                 # 默认 100 帧/秒 × 1000 帧
  python docs\\loadtest.py --hz 200 --count 2000
  python docs\\loadtest.py --hz 50  --count 500

注意：
  - 运行前请先 Ctrl+C 关闭 monitor.py 等占用串口的程序。
  - 板子靠固定 MAC 识别 A/B，不依赖 COM 号。
"""
import sys
import time
import argparse

# --- Windows 控制台锁定 UTF-8，避免中文乱码 ---
try:
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.kernel32.SetConsoleOutputCP(65001)
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("缺少 pyserial，请先安装：pip install pyserial")
    raise SystemExit(1)

MAC_A = "E8:3D:C1:F2:C7:B8"   # A 板 = 接收端
MAC_B = "E8:3D:C1:FA:7A:0C"   # B 板 = 发送端
BAUD = 115200


def find_ports():
    """按固定 MAC 从 hwid 中定位 A/B 两块板的 COM 口。"""
    a = b = None
    for p in list_ports.comports():
        h = (p.hwid or "").upper()
        if MAC_A in h:
            a = p.device
        if MAC_B in h:
            b = p.device
    return a, b


def open_port(port):
    """打开串口且不触发板子复位（dtr/rts 保持 False）。"""
    s = serial.Serial()
    s.port = port
    s.baudrate = BAUD
    s.timeout = 0.3
    s.dtr = False
    s.rts = False
    s.open()
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=int, default=100, help="发送频率（帧/秒）")
    ap.add_argument("--count", type=int, default=1000, help="发送总帧数")
    args = ap.parse_args()

    a, b = find_ports()
    if not a or not b:
        print(f"未找到板子：A(接收)={a}  B(发送)={b}")
        print("请检查两块板是否都已连接，并关闭占用串口的程序（如 monitor.py）。")
        return 1

    dur = args.count / args.hz
    print("== ESP-NOW 工况压测 ==")
    print(f"  接收端 A = {a}   发送端 B = {b}")
    print(f"  参数：{args.hz} 帧/秒 × {args.count} 帧  ≈ {dur:.1f}s")
    print("-" * 40)

    try:
        ra = open_port(a)
        rb = open_port(b)
    except Exception as e:
        print(f"打开串口失败：{e}")
        print("端口可能被 monitor.py 占用，请先 Ctrl+C 关闭它。")
        return 1

    time.sleep(0.3)
    ra.reset_input_buffer()
    rb.reset_input_buffer()
    rb.write(f"LOAD,{args.hz},{args.count}\n".encode())
    rb.flush()

    deadline = time.time() + dur + 5.0
    result = None
    while time.time() < deadline:
        try:
            line = ra.readline().decode(errors="replace").strip()
        except Exception:
            break
        if not line:
            continue
        if "TEST" in line:
            print("[A]", line)
        if "TEST RESULT" in line:
            result = line
            break

    try:
        rb.write(b"LOAD,0\n")
        rb.flush()
    except Exception:
        pass
    rb.close()
    ra.close()

    print("-" * 40)
    if result:
        print("==== 丢包率结果 ====")
        print(result)
    else:
        print("未收到最终结果：发送端可能未启动，或端口被占用/MAC 不匹配。")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
