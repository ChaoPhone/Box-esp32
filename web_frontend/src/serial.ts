// ============================================================
// serial.ts — Web Serial 连接 + B-Box 协议收发 + 推动检测复现
// 参考 imu-viewer/src/serial.ts，但处理完整协议（EVT/OK/NAK/IMU/IMUQ...）
// ============================================================

import {
  parseLine, createPushState, pushDynFirmware,
  quatRotate, deYawHorizontal, PUSH_THRESH,
  type UpMessage, type ImuSample, type JudgeMode, type PushJudge,
} from './protocol';

export type { JudgeMode, PushJudge };

export interface Callbacks {
  onMessage: (m: UpMessage) => void;
  onImu: (s: ImuSample) => void;
  onImuQ: (qw: number, qx: number, qy: number, qz: number, ms: number) => void;
  onEvt: (seq: number, action: string, value: number, ms: number) => void;
  onPushJudge: (j: PushJudge) => void;
  onStatus: (connected: boolean, text: string) => void;
  onError: (msg: string) => void;
}

// ---- Web Serial 状态 ----
let port: SerialPort | null = null;
let reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
let writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
let readAbort: AbortController | null = null;
let connected = false;

// ---- 推动检测复现状态 ----
const fwPush = createPushState();           // 固件模式高通滤波状态
const worldPush = createPushState();        // 世界系模式高通滤波状态
let judgeMode: JudgeMode = 'firmware';
let lastQuat: { qw: number; qx: number; qy: number; qz: number } = { qw: 1, qx: 0, qy: 0, qz: 0 };
let pushDebounceUntil = 0;                  // 去抖截止（ms 时间戳，用 performance.now）
const PUSH_DEBOUNCE_MS = 600;               // 与固件一致

export function serialSupported(): boolean {
  return 'serial' in navigator;
}

export function setJudgeMode(mode: JudgeMode): void {
  judgeMode = mode;
  // 切换模式时重置滤波状态，避免残留 DC 污染
  fwPush.maX = fwPush.maY = 0;
  worldPush.maX = worldPush.maY = 0;
}

export function isConnected(): boolean {
  return connected;
}

export async function connect(cbs: Callbacks): Promise<boolean> {
  if (!serialSupported()) {
    cbs.onError('此浏览器不支持 Web Serial API。请使用 Chrome/Edge。');
    return false;
  }
  try {
    cbs.onStatus(false, '正在打开串口…');
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: 'none' });
    reader = port.readable!.getReader();
    writer = port.writable!.getWriter();
    readAbort = new AbortController();

    const info = port.getInfo();
    const label = (info as any).usbProductName || 'ESP32';
    connected = true;
    cbs.onStatus(true, `已连接 ${label}`);

    startReadLoop(cbs);

    // ESP32-S3 原生 USB 开串口可能触发复位，补发 DEBUG,1 开启 20Hz IMU（推动复现需要）
    for (const delay of [150, 600, 1400]) {
      setTimeout(() => { void send('DEBUG,1'); }, delay);
    }
    return true;
  } catch (err: any) {
    if (err.name === 'NotFoundError' || err.name === 'AbortError') return false;
    const msg = err.message || String(err);
    if (msg.includes('already open')) {
      cbs.onError('串口已被其他程序占用。\n请关闭串口监视器 / esp32_bridge.py 后重试。');
    } else if (msg.includes('No port selected')) {
      return false;
    } else {
      cbs.onError(`串口连接失败: ${msg}`);
    }
    return false;
  }
}

export async function disconnect(cbs?: Callbacks): Promise<void> {
  if (readAbort) { readAbort.abort(); readAbort = null; }
  try { if (reader) { await reader.cancel(); reader.releaseLock(); reader = null; } } catch (_) { /* ok */ }
  try { if (writer) { writer.releaseLock(); writer = null; } } catch (_) { /* ok */ }
  try { if (port) { await port.close(); port = null; } } catch (_) { /* ok */ }
  connected = false;
  if (cbs) cbs.onStatus(false, '串口已断开');
}

/** 发送下行命令（自动补 \n） */
export function send(cmd: string): void {
  if (!writer) return;
  const data = new TextEncoder().encode(cmd.trim() + '\n');
  writer.write(data).catch(() => {});
}

async function startReadLoop(cbs: Callbacks): Promise<void> {
  const decoder = new TextDecoder();
  let buf = '';

  while (readAbort && reader && !readAbort.signal.aborted) {
    try {
      const { value, done } = await reader.read();
      if (done) break;
      if (!value) continue;

      buf += decoder.decode(value, { stream: true });
      const lines = buf.split('\n');
      buf = lines.pop() || '';

      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed) continue;
        const msg = parseLine(trimmed);
        cbs.onMessage(msg);
        dispatch(cbs, msg);
      }
    } catch (err: any) {
      if (err.name === 'AbortError') break;
      console.error('串口读取错误:', err);
      break;
    }
  }

  await disconnect(cbs);
}

/** 按消息类型分发到具体回调 + 维护推动检测复现 */
function dispatch(cbs: Callbacks, msg: UpMessage): void {
  switch (msg.type) {
    case 'EVT':
      cbs.onEvt(msg.seq, msg.action, msg.value, msg.ms);
      // 自动回复 ACK 停止板子重传
      send('ACK,' + msg.seq);
      break;
    case 'IMU':
      cbs.onImu({ ax: msg.ax, ay: msg.ay, az: msg.az, gx: msg.gx, gy: msg.gy, gz: msg.gz, ms: msg.ms });
      // 推动检测复现（每次 IMU 帧都算，给罗盘实时反馈）
      runPushJudge(cbs, msg.ax, msg.ay, msg.az);
      break;
    case 'IMUQ':
      lastQuat = { qw: msg.qw, qx: msg.qx, qy: msg.qy, qz: msg.qz };
      cbs.onImuQ(msg.qw, msg.qx, msg.qy, msg.qz, msg.ms);
      break;
    default:
      break;
  }
}

/** 推动检测复现：固件模式 vs 世界系模式 */
function runPushJudge(cbs: Callbacks, ax: number, ay: number, az: number): void {
  let dynX: number, dynY: number, mag: number;
  let judged: string | null = null;

  if (judgeMode === 'firmware') {
    // 忠实复现固件 pushDetect：仅用传感器帧 ax/ay
    const d = pushDynFirmware(fwPush, ax, ay);
    dynX = d.dynX; dynY = d.dynY; mag = d.mag;
  } else {
    // 世界系（倾斜补偿+去yaw）：旋到世界帧后再旋回 -yaw，方向锚定盒体自身轴
    const [wx, wy] = quatRotate(lastQuat.qw, lastQuat.qx, lastQuat.qy, lastQuat.qz, ax, ay, az);
    const [bx, by] = deYawHorizontal(lastQuat.qw, lastQuat.qx, lastQuat.qy, lastQuat.qz, wx, wy);
    const d = pushDynFirmware(worldPush, bx, by);
    dynX = d.dynX; dynY = d.dynY; mag = d.mag;
  }

  // 去抖 + 方向判定（与固件 PUSH_DEBOUNCE_MS / 分量最大方向一致）
  if (mag >= PUSH_THRESH) {
    const now = performance.now();
    if (now >= pushDebounceUntil) {
      pushDebounceUntil = now + PUSH_DEBOUNCE_MS;
      judged = Math.abs(dynX) > Math.abs(dynY)
        ? (dynX > 0 ? 'MOVE_RIGHT' : 'MOVE_LEFT')
        : (dynY > 0 ? 'MOVE_FORWARD' : 'MOVE_BACKWARD');
    }
  }

  cbs.onPushJudge({
    mode: judgeMode, dynX, dynY, mag, threshold: PUSH_THRESH, judged,
  });
}
