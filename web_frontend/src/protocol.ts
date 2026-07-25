// ============================================================
// protocol.ts — B-Box Serial Protocol v1 类型定义 + 行解析
// 规范见 esp32_shot/docs/PROTOCOL.md，固件见 src/main_wired.cpp
// ============================================================

/** 上行消息类型（板 -> PC） */
export type UpType =
  | 'BOOT' | 'HEARTBEAT' | 'EVT' | 'OK' | 'NAK' | 'PONG'
  | 'IMU' | 'IMUQ' | 'TILTS' | 'LOG' | 'UNKNOWN';

/** 解析后的上行消息（判别联合） */
export type UpMessage =
  | { type: 'BOOT'; fw: string; proto: number; raw: string }
  | { type: 'HEARTBEAT'; ms: number; state: string; raw: string }
  | { type: 'EVT'; seq: number; action: string; value: number; ms: number; raw: string }
  | { type: 'OK'; detail: string; raw: string }
  | { type: 'NAK'; detail: string; raw: string }
  | { type: 'PONG'; token?: string; raw: string }
  | { type: 'IMU'; ax: number; ay: number; az: number; gx: number; gy: number; gz: number; ms: number; raw: string }
  | { type: 'IMUQ'; qw: number; qx: number; qy: number; qz: number; ms: number; raw: string }
  | { type: 'LOG'; text: string; raw: string }
  | { type: 'TILTS'; pitch: number; roll: number; ms: number; raw: string }
  | { type: 'UNKNOWN'; raw: string };

/** IMU 原始采样（与 imu-viewer 保持兼容的形状） */
export interface ImuSample {
  ax: number; ay: number; az: number;
  gx: number; gy: number; gz: number;
  ms: number;
}

const num = (s: string | undefined): number => (s === undefined ? NaN : parseFloat(s));

/**
 * 解析一行上行消息。无法识别的返回 UNKNOWN，绝不抛异常。
 * 与固件 handleCommand / 上行格式严格对齐。
 */
export function parseLine(line: string): UpMessage {
  const raw = line;
  const trim = line.trim();
  if (!trim) return { type: 'UNKNOWN', raw };

  const i = trim.indexOf(',');
  const head = i > 0 ? trim.slice(0, i) : trim;
  const rest = i > 0 ? trim.slice(i + 1) : '';
  const p = rest.split(',');

  switch (head) {
    case 'BOOT': {
      // BOOT,<fw>,proto=<n>
      const fw = p[0] ?? '';
      let proto = NaN;
      for (const f of p) {
        const m = /^proto=(\d+)$/.exec(f);
        if (m) proto = parseInt(m[1], 10);
      }
      return { type: 'BOOT', fw, proto, raw };
    }
    case 'HEARTBEAT': {
      // HEARTBEAT,<ms>,<state>
      return { type: 'HEARTBEAT', ms: num(p[0]), state: p[1] ?? '', raw };
    }
    case 'EVT': {
      // EVT,<seq>,<action>,<value>,<ms>
      return {
        type: 'EVT', seq: num(p[0]), action: (p[1] ?? '').toUpperCase(),
        value: num(p[2]), ms: num(p[3]), raw,
      };
    }
    case 'OK': {
      // OK,<TYPE>[,<detail...>]
      return { type: 'OK', detail: rest, raw };
    }
    case 'NAK': {
      // NAK,<TYPE>,<reason>
      return { type: 'NAK', detail: rest, raw };
    }
    case 'PONG': {
      // PONG[,<token>]
      return { type: 'PONG', token: rest || undefined, raw };
    }
    case 'IMU': {
      // IMU,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<ms>
      return {
        type: 'IMU',
        ax: num(p[0]), ay: num(p[1]), az: num(p[2]),
        gx: num(p[3]), gy: num(p[4]), gz: num(p[5]),
        ms: num(p[6]), raw,
      };
    }
    case 'IMUQ': {
      // IMUQ,<qw>,<qx>,<qy>,<qz>,<ms>
      return {
        type: 'IMUQ',
        qw: num(p[0]), qx: num(p[1]), qy: num(p[2]), qz: num(p[3]),
        ms: num(p[4]), raw,
      };
    }
    case 'TILTS': {
      // TILTS,<pitchDeg>,<rollDeg>,<ms>
      return { type: 'TILTS', pitch: num(p[0]), roll: num(p[1]), ms: num(p[2]), raw };
    }
    case 'LOG': {
      return { type: 'LOG', text: rest, raw };
    }
    default:
      return { type: 'UNKNOWN', raw };
  }
}

/** 把 EVT 的 action 归类到统计桶 */
export type StatBucket = 'MOVE_FORWARD' | 'MOVE_BACKWARD' | 'MOVE_LEFT' | 'MOVE_RIGHT' | 'ROTATE' | 'OTHER';

export function bucketOf(action: string): StatBucket {
  switch (action) {
    case 'MOVE_FORWARD':
    case 'MOVE_BACKWARD':
    case 'MOVE_LEFT':
    case 'MOVE_RIGHT':
      return action;
    case 'ROTATE_CW':
    case 'ROTATE_CCW':
      return 'ROTATE';
    default:
      return 'OTHER';
  }
}

// ============================================================
// 推动检测复现 —— 忠实移植 main_wired.cpp pushDetect()
// ============================================================

/** 推动判定模式：firmware = 复现固件原逻辑；world = 世界系(四元数)对照 */
export type JudgeMode = 'firmware' | 'world';

/** 一次推动判定的结果（罗盘可视化 + 文字显示用） */
export interface PushJudge {
  mode: JudgeMode;
  dynX: number;      // 用于罗盘绘制的动态分量
  dynY: number;
  mag: number;
  threshold: number;
  judged: string | null;  // 本次判定方向（过阈值且去抖后非空）
}

/** 固件原逻辑：传感器帧 ax/ay 高通去 DC → 阈值 → 分量最大方向
 *  对应 main_wired.cpp 334-352 行 */
export interface PushState {
  maX: number; // pushAccMA[0]
  maY: number; // pushAccMA[1]
}

export const PUSH_THRESH = 0.30;       // 固件 PUSH_THRESH（0.8.0-tiltpush）
// 固件 alpha=0.02 @200Hz；前端收到的是 50Hz 上报流，等效 alpha = 1-(1-0.02)^4
export const PUSH_ALPHA = 0.0776;

export function createPushState(): PushState {
  return { maX: 0, maY: 0 };
}

/** 复现固件 pushDetect：返回判定方向或 null */
export function pushDetectFirmware(state: PushState, ax: number, ay: number): string | null {
  state.maX = state.maX * (1 - PUSH_ALPHA) + ax * PUSH_ALPHA;
  state.maY = state.maY * (1 - PUSH_ALPHA) + ay * PUSH_ALPHA;
  const dynX = ax - state.maX;
  const dynY = ay - state.maY;
  const mag = Math.hypot(dynX, dynY);
  if (mag < PUSH_THRESH) return null;
  if (Math.abs(dynX) > Math.abs(dynY)) {
    return dynX > 0 ? 'MOVE_RIGHT' : 'MOVE_LEFT';
  }
  return dynY > 0 ? 'MOVE_FORWARD' : 'MOVE_BACKWARD';
}

/** 返回动态分量（罗盘可视化用），无论是否过阈值 */
export function pushDynFirmware(state: PushState, ax: number, ay: number): { dynX: number; dynY: number; mag: number } {
  state.maX = state.maX * (1 - PUSH_ALPHA) + ax * PUSH_ALPHA;
  state.maY = state.maY * (1 - PUSH_ALPHA) + ay * PUSH_ALPHA;
  const dynX = ax - state.maX;
  const dynY = ay - state.maY;
  return { dynX, dynY, mag: Math.hypot(dynX, dynY) };
}

// ---- 四元数工具（世界系推动检测用） ----

/** 把世界系水平分量旋回 -yaw，得到机体航向系分量（与固件去 yaw 逻辑一致）。
 *  方向锚定盒体自身轴，yaw 漂移不影响判向。 */
export function deYawHorizontal(
  qw: number, qx: number, qy: number, qz: number,
  wx: number, wy: number,
): [number, number] {
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [c * wx + s * wy, -s * wx + c * wy]; // Rz(-yaw)
}

/** 用四元数把传感器帧加速度旋转到世界帧。
 *  q = (qw, qx, qy, qz)，v' = q ⊗ v ⊗ q*  */
export function quatRotate(
  qw: number, qx: number, qy: number, qz: number,
  vx: number, vy: number, vz: number,
): [number, number, number] {
  // 标准四元数旋转 v' = q v q*
  const qvx = qx, qvy = qy, qvz = qz;
  // t = 2 * (q_vec × v)
  const tx = 2 * (qvy * vz - qvz * vy);
  const ty = 2 * (qvz * vx - qvx * vz);
  const tz = 2 * (qvx * vy - qvy * vx);
  // v' = v + qw * t + q_vec × t
  const rx = vx + qw * tx + (qvy * tz - qvz * ty);
  const ry = vy + qw * ty + (qvz * tx - qvx * tz);
  const rz = vz + qw * tz + (qvx * ty - qvy * tx);
  return [rx, ry, rz];
}

/** 四元数 -> Roll/Pitch（度），用于显示 */
export function quatToEuler(qw: number, qx: number, qy: number, qz: number): { roll: number; pitch: number; yaw: number } {
  // roll (x-axis rotation)
  const sinr_cosp = 2 * (qw * qx + qy * qz);
  const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  const sinp = 2 * (qw * qy - qz * qx);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
  // yaw (z-axis rotation)
  const siny_cosp = 2 * (qw * qz + qx * qy);
  const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  return { roll: roll * 180 / Math.PI, pitch: pitch * 180 / Math.PI, yaw: yaw * 180 / Math.PI };
}
