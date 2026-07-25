#!/usr/bin/env python3
"""
B Box ESP32 实体箱子桥接程序。

数据链路：
    ESP32-S3 + MPU6050
        -> USB 串口高层事件
        -> 本程序
        -> UDP 文本指令
        -> Unity

ESP32 上报示例：
    EVT,12,MOVE_RIGHT,203.5,153210
    EVT,13,ROTATE_CW_45,47.2,154008
    HEARTBEAT,155000,READY
    BOOT,1.0.0

字段：
    EVT,<sequence>,<action>,<value>,<timestamp_ms>

其中：
    - MOVE_* 的 value 建议为估计位移，单位 mm；
    - ROTATE_* 的 value 建议为估计转角，单位 degree；
    - sequence 是递增事件编号，用于 ACK、重传和去重。

Unity 默认监听：
    127.0.0.1:47800

Unity 可选向本程序反馈：
    127.0.0.1:47801

Unity -> Bridge -> ESP32 示例：
    HAPTIC,WALL,180,120
    HAPTIC,SUCCESS,255,600
    CALIBRATE
    DEBUG,1

依赖：
    pip install pyserial
"""

from __future__ import annotations

import argparse
import asyncio
import socket
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import serial
from serial import SerialException
from serial.tools import list_ports


DEFAULT_BAUD = 115200
DEFAULT_UNITY_ADDR = ("127.0.0.1", 47800)
DEFAULT_FEEDBACK_PORT = 47801
SERIAL_TIMEOUT_S = 0.20
HEARTBEAT_WARN_S = 2.0

ALLOWED_ACTIONS = {
    # 推动（v0.6.0 固件已实现）
    "MOVE_FORWARD",
    "MOVE_BACKWARD",
    "MOVE_RIGHT",
    "MOVE_LEFT",
    # 旋转（预留）
    "ROTATE_CW",
    "ROTATE_CCW",
    # 倾斜（预留）
    "TILT_UP",
    "TILT_DOWN",
    "TILT_LEFT",
    "TILT_RIGHT",
    # 拿起/放置（预留）
    "LIFT",
    "PLACE",
    # 摇晃/敲击（预留）
    "SHAKE",
    "TAP",
    # 控制（预留，戒指）
    "CONFIRM",
    "UNDO",
    "RESET",
}


@dataclass(frozen=True)
class Event:
    sequence: int
    action: str
    value: float
    timestamp_ms: int


class UdpSender:
    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_action(self, event: Event) -> None:
        """
        默认只把动作名发给 Unity，便于 Unity 直接 switch(message)。

        详细 value、sequence 和 timestamp 保留在桥接程序日志中。
        Unity 若需要这些字段，可将 payload 改为：
            f"{event.action},{event.value},{event.sequence},{event.timestamp_ms}"
        """
        payload = event.action
        self.sock.sendto(payload.encode("ascii"), self.addr)
        print(
            f"[bridge] -> Unity {payload} "
            f"(seq={event.sequence}, value={event.value:.2f})"
        )

    def close(self) -> None:
        self.sock.close()


class SerialLink:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.write_lock = asyncio.Lock()

    def open(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=SERIAL_TIMEOUT_S,
            write_timeout=1.0,
        )
        # 部分 ESP32 板卡打开串口时会复位，留出启动时间。
        time.sleep(1.2)
        self.ser.reset_input_buffer()
        print(f"[bridge] 已连接串口 {self.port} @ {self.baud}")

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None

    async def readline(self) -> str:
        if self.ser is None:
            raise SerialException("串口尚未打开")

        raw = await asyncio.to_thread(self.ser.readline)
        if not raw:
            return ""

        return raw.decode("utf-8", errors="replace").strip()

    async def write_line(self, line: str) -> None:
        if self.ser is None or not self.ser.is_open:
            raise SerialException("串口未连接")

        data = (line.strip() + "\n").encode("ascii", errors="strict")
        async with self.write_lock:
            await asyncio.to_thread(self.ser.write, data)
            await asyncio.to_thread(self.ser.flush)


def discover_port() -> str:
    """
    自动寻找最可能的 ESP32 串口。

    如果发现多个候选设备，为避免误连，要求用户显式传入 --port。
    """
    keywords = (
        "esp32",
        "usb jtag",
        "usb serial",
        "cp210",
        "ch340",
        "wch",
        "silicon labs",
    )

    ports = list(list_ports.comports())
    candidates = []

    for p in ports:
        text = " ".join(
            filter(
                None,
                [
                    p.device,
                    p.description,
                    p.manufacturer,
                    p.product,
                    p.hwid,
                ],
            )
        ).lower()

        if any(keyword in text for keyword in keywords):
            candidates.append(p.device)

    candidates = sorted(set(candidates))

    if len(candidates) == 1:
        return candidates[0]

    if not candidates:
        available = ", ".join(p.device for p in ports) or "无"
        raise RuntimeError(
            "没有自动找到 ESP32 串口。"
            f"当前串口：{available}。请使用 --port COMx 指定。"
        )

    raise RuntimeError(
        "发现多个可能的 ESP32 串口："
        + ", ".join(candidates)
        + "。请使用 --port 明确指定。"
    )


def parse_event(line: str) -> Event:
    parts = [part.strip() for part in line.split(",")]
    if len(parts) != 5 or parts[0] != "EVT":
        raise ValueError("事件格式应为 EVT,sequence,action,value,timestamp_ms")

    sequence = int(parts[1])
    action = parts[2].upper()
    value = float(parts[3])
    timestamp_ms = int(parts[4])

    if sequence < 0:
        raise ValueError("sequence 不能为负数")
    if action not in ALLOWED_ACTIONS:
        raise ValueError(f"未知动作：{action}")

    return Event(
        sequence=sequence,
        action=action,
        value=value,
        timestamp_ms=timestamp_ms,
    )


class FeedbackProtocol(asyncio.DatagramProtocol):
    """
    接收 Unity 的反馈命令，再转发到 ESP32。

    Unity 向 127.0.0.1:<feedback-port> 发送 ASCII 文本即可。
    """

    def __init__(self, command_queue: asyncio.Queue[str]):
        self.command_queue = command_queue

    def datagram_received(self, data: bytes, addr) -> None:
        try:
            text = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            print(f"[bridge] 丢弃非 UTF-8 Unity 反馈，来源 {addr}")
            return

        if not text:
            return

        try:
            self.command_queue.put_nowait(text)
        except asyncio.QueueFull:
            print("[bridge] Unity 反馈队列已满，丢弃命令")


async def feedback_forward_loop(
    serial_link: SerialLink,
    command_queue: asyncio.Queue[str],
) -> None:
    while True:
        command = await command_queue.get()
        try:
            await serial_link.write_line(command)
            print(f"[bridge] Unity -> ESP32: {command}")
        finally:
            command_queue.task_done()


async def heartbeat_watch_loop(last_rx_ref: list[float]) -> None:
    warned = False

    while True:
        await asyncio.sleep(0.5)
        elapsed = time.monotonic() - last_rx_ref[0]

        if elapsed >= HEARTBEAT_WARN_S and not warned:
            warned = True
            print(f"[bridge] 警告：{elapsed:.1f} 秒未收到 ESP32 数据")
        elif elapsed < HEARTBEAT_WARN_S:
            warned = False


async def serial_receive_loop(
    serial_link: SerialLink,
    udp: UdpSender,
) -> None:
    recent_sequences: deque[int] = deque(maxlen=256)
    recent_set: set[int] = set()
    last_rx_ref = [time.monotonic()]

    heartbeat_task = asyncio.create_task(heartbeat_watch_loop(last_rx_ref))

    try:
        while True:
            line = await serial_link.readline()
            if not line:
                continue

            last_rx_ref[0] = time.monotonic()

            if line.startswith("EVT,"):
                try:
                    event = parse_event(line)
                except (ValueError, TypeError) as exc:
                    print(f"[bridge] 丢弃非法事件：{line!r}，原因：{exc}")
                    continue

                if event.sequence in recent_set:
                    # ESP32 可能因未及时收到 ACK 而重传。
                    await serial_link.write_line(f"ACK,{event.sequence}")
                    print(f"[bridge] 重复事件 seq={event.sequence}，已 ACK，不再转发")
                    continue

                udp.send_action(event)
                await serial_link.write_line(f"ACK,{event.sequence}")

                if len(recent_sequences) == recent_sequences.maxlen:
                    oldest = recent_sequences[0]
                    recent_set.discard(oldest)

                recent_sequences.append(event.sequence)
                recent_set.add(event.sequence)
                continue

            if line.startswith("HEARTBEAT,"):
                print(f"[bridge] {line}")
                continue

            if line.startswith("BOOT,"):
                recent_sequences.clear()
                recent_set.clear()
                print(f"[bridge] ESP32 启动：{line}")
                continue

            if line.startswith("IMU,"):
                # 正式运行应关闭高频 IMU 输出。这里仅打印，便于标定。
                print(f"[imu] {line}")
                continue

            if line.startswith("ACK,"):
                print(f"[bridge] ESP32 确认：{line}")
                continue

            if line.startswith("LOG,"):
                print(f"[esp32] {line[4:]}")
                continue

            print(f"[bridge] 未识别串口消息：{line}")
    finally:
        heartbeat_task.cancel()
        await asyncio.gather(heartbeat_task, return_exceptions=True)


async def run_connection(
    port: str,
    baud: int,
    udp: UdpSender,
    command_queue: asyncio.Queue[str],
) -> None:
    serial_link = SerialLink(port=port, baud=baud)
    serial_link.open()

    receive_task = asyncio.create_task(serial_receive_loop(serial_link, udp))
    feedback_task = asyncio.create_task(
        feedback_forward_loop(serial_link, command_queue)
    )

    try:
        done, pending = await asyncio.wait(
            [receive_task, feedback_task],
            return_when=asyncio.FIRST_EXCEPTION,
        )

        for task in done:
            exc = task.exception()
            if exc is not None:
                raise exc

        raise RuntimeError("串口任务意外结束")
    finally:
        for task in (receive_task, feedback_task):
            task.cancel()
        await asyncio.gather(
            receive_task,
            feedback_task,
            return_exceptions=True,
        )
        serial_link.close()


async def async_main(args: argparse.Namespace) -> None:
    udp = UdpSender(args.unity_host, args.unity_port)
    command_queue: asyncio.Queue[str] = asyncio.Queue(maxsize=64)

    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        lambda: FeedbackProtocol(command_queue),
        local_addr=(args.feedback_host, args.feedback_port),
    )

    print(
        f"[bridge] Unity 输入目标："
        f"{args.unity_host}:{args.unity_port}"
    )
    print(
        f"[bridge] Unity 反馈监听："
        f"{args.feedback_host}:{args.feedback_port}"
    )

    backoff = 1.0

    try:
        while True:
            try:
                port = args.port or discover_port()
                await run_connection(
                    port=port,
                    baud=args.baud,
                    udp=udp,
                    command_queue=command_queue,
                )
                backoff = 1.0
            except asyncio.CancelledError:
                raise
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(
                    f"[bridge] 连接中断：{type(exc).__name__}: {exc}；"
                    f"{backoff:.1f} 秒后重试"
                )
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, 10.0)
    finally:
        transport.close()
        udp.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="B Box ESP32 实体箱子到 Unity 的串口/UDP 桥接程序"
    )
    parser.add_argument(
        "--port",
        help="ESP32 串口，例如 COM5；不填写时尝试自动发现",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"串口波特率，默认 {DEFAULT_BAUD}",
    )
    parser.add_argument(
        "--unity-host",
        default=DEFAULT_UNITY_ADDR[0],
        help="Unity 所在地址，默认 127.0.0.1",
    )
    parser.add_argument(
        "--unity-port",
        type=int,
        default=DEFAULT_UNITY_ADDR[1],
        help="Unity 接收实体动作的 UDP 端口，默认 47800",
    )
    parser.add_argument(
        "--feedback-host",
        default="127.0.0.1",
        help="监听 Unity 反馈的地址，默认 127.0.0.1",
    )
    parser.add_argument(
        "--feedback-port",
        type=int,
        default=DEFAULT_FEEDBACK_PORT,
        help="监听 Unity 反馈的 UDP 端口，默认 47801",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("\n[bridge] 已退出")
        return 0
    except Exception as exc:
        print(f"[bridge] 致命错误：{type(exc).__name__}: {exc}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
