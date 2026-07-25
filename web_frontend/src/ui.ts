// ============================================================
// ui.ts — UI 更新：消息流、事件统计、IMU 面板、方向罗盘、姿态立方体
// ============================================================

import { bucketOf, quatToEuler, type UpMessage, type PushJudge } from './protocol';

// ---- DOM 引用 ----
const $ = <T extends HTMLElement = HTMLElement>(id: string): T => document.getElementById(id) as T;
const stream = $('msg-stream');
const dot = $('dot');
const statusText = $('status-text');

// ---- 状态 ----
const stats: Record<string, number> = {
  MOVE_FORWARD: 0, MOVE_BACKWARD: 0, MOVE_LEFT: 0, MOVE_RIGHT: 0, ROTATE: 0, OTHER: 0,
};

// ---- 消息流过滤 ----
const filters: Record<string, boolean> = {
  EVT: true, TILTS: false, IMU: false, IMUQ: false, HEARTBEAT: false, OK: false, NAK: false, LOG: false,
  UNKNOWN: false,
};

function filterKey(m: UpMessage): string | null {
  switch (m.type) {
    case 'EVT': return 'EVT';
    case 'IMU': return 'IMU';
    case 'IMUQ': return 'IMUQ';
    case 'TILTS': return 'TILTS';
    case 'HEARTBEAT': return 'HEARTBEAT';
    case 'OK': return 'OK';
    case 'NAK': return 'NAK';
    case 'LOG': return 'LOG';
    case 'UNKNOWN': return 'UNKNOWN';
    default: return null;
  }
}

const MAX_STREAM_LINES = 400;

export function appendMessage(m: UpMessage): void {
  const key = filterKey(m);
  if (key && !filters[key]) return;

  const ts = new Date();
  const tstr = `${String(ts.getHours()).padStart(2, '0')}:${String(ts.getMinutes()).padStart(2, '0')}:${String(ts.getSeconds()).padStart(2, '0')}.${String(ts.getMilliseconds()).padStart(3, '0')}`;
  const ty = m.type;
  const div = document.createElement('div');
  div.className = 'msg';
  div.innerHTML = `<span class="ts">${tstr}</span> <span class="ty ty-${ty}">${ty}</span> ${escapeHtml(m.raw.slice(ty.length + 1))}`;
  stream.appendChild(div);

  while (stream.childElementCount > MAX_STREAM_LINES) stream.removeChild(stream.firstChild!);
  if (($('autoscroll') as HTMLInputElement).checked) stream.scrollTop = stream.scrollHeight;
}

function escapeHtml(s: string): string {
  return s.replace(/[&<>]/g, c => c === '&' ? '&amp;' : c === '<' ? '&lt;' : '&gt;');
}

export function clearStream(): void {
  stream.innerHTML = '';
}

// ---- 事件统计 ----
export function onEvt(_seq: number, action: string, value: number, ms: number): void {
  const b = bucketOf(action);
  stats[b]++;
  const el = $(`cnt-${b}`);
  if (el) el.textContent = String(stats[b]);
  $('last-event').innerHTML = `最近事件：<b>${action}</b> value=${value.toFixed(1)} ms=${ms} @${new Date().toLocaleTimeString()}`;
}

export function resetStats(): void {
  for (const k of Object.keys(stats)) {
    stats[k] = 0;
    const el = $(`cnt-${k}`);
    if (el) el.textContent = '0';
  }
  $('last-event').textContent = '最近事件：--';
}

// ---- 状态指示 ----
export function setStatus(connected: boolean, text: string): void {
  dot.className = 'dot ' + (connected ? 'on' : 'off');
  statusText.textContent = text;
  $('btn-connect').textContent = connected ? '⛔ 断开' : '🔌 连接串口';
  $('btn-connect').classList.toggle('danger', connected);
  $('btn-connect').classList.toggle('accent', !connected);
}

export function showError(msg: string): void {
  const div = document.createElement('div');
  div.className = 'msg';
  div.innerHTML = `<span class="ty ty-NAK">ERR</span> ${escapeHtml(msg)}`;
  stream.appendChild(div);
  stream.scrollTop = stream.scrollHeight;
}

// ---- IMU 数据面板 ----
const fmt = (v: number, d = 3) => (isNaN(v) ? '--' : v.toFixed(d));

export function updateImu(s: { ax: number; ay: number; az: number; gx: number; gy: number; gz: number; ms: number }): void {
  $('v-ax').textContent = fmt(s.ax);
  $('v-ay').textContent = fmt(s.ay);
  $('v-az').textContent = fmt(s.az);
  $('v-amag').textContent = fmt(Math.hypot(s.ax, s.ay, s.az));
  $('v-gx').textContent = fmt(s.gx, 1);
  $('v-gy').textContent = fmt(s.gy, 1);
  $('v-gz').textContent = fmt(s.gz, 1);
  $('v-ms').textContent = String(s.ms);
}

export function updateQuat(qw: number, qx: number, qy: number, qz: number): void {
  $('v-qw').textContent = fmt(qw, 4);
  $('v-qx').textContent = fmt(qx, 4);
  $('v-qy').textContent = fmt(qy, 4);
  $('v-qz').textContent = fmt(qz, 4);
  const { roll, pitch, yaw } = quatToEuler(qw, qx, qy, qz);
  $('v-roll').textContent = fmt(roll, 1);
  $('v-pitch').textContent = fmt(pitch, 1);
  $('v-yaw').textContent = fmt(yaw, 1);
  updateCube(qw, qx, qy, qz);
}

// ---- 姿态立方体（CSS 3D，用四元数转矩阵） ----
function updateCube(qw: number, qx: number, qy: number, qz: number): void {
  // 四元数 → 轴角，再把世界系轴换算到 CSS 屏幕系
  const x = qx, y = qy, z = qz, w = qw;
  const angle = 2 * Math.acos(Math.max(-1, Math.min(1, w)));
  const s = Math.sqrt(1 - w * w);
  let ax = 1, ay = 0, az = 0;
  if (s > 1e-6) { ax = x / s; ay = y / s; az = z / s; }
  const deg = angle * 180 / Math.PI;
  // 世界系(X右/Y前/Z上) → CSS 屏幕系(X右/Y下/Z朝观察者)，观察者位于盒体正前方：
  // 轴映射 (x,y,z)→(x, z, -y)。绕上下=rotY、绕前后=rotZ、绕左右=rotX
  $('cube').style.transform = `rotate3d(${ax}, ${az}, ${-ay}, ${deg}deg)`;
}

// ---- 方向罗盘（Canvas 2D） ----
const compass = $('compass') as HTMLCanvasElement;
const cctx = compass.getContext('2d')!;

function resizeCompass(): void {
  const dpr = window.devicePixelRatio || 1;
  const r = compass.getBoundingClientRect();
  compass.width = r.width * dpr;
  compass.height = r.height * dpr;
  cctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

let lastJudge: PushJudge | null = null;

export function drawCompass(j: PushJudge): void {
  lastJudge = j;
  const r = compass.getBoundingClientRect();
  const w = r.width, h = r.height;
  const cx = w / 2, cy = h / 2;
  const R = Math.min(w, h) / 2 - 18;
  cctx.clearRect(0, 0, w, h);

  // 背景圆
  cctx.strokeStyle = '#30363d';
  cctx.lineWidth = 1;
  cctx.beginPath(); cctx.arc(cx, cy, R, 0, Math.PI * 2); cctx.stroke();
  // 阈值圈（用当前判定阈值绘制）
  const scale = R / 0.8; // 0.8g 满量程
  cctx.strokeStyle = '#d2991d44';
  cctx.setLineDash([4, 4]);
  cctx.beginPath(); cctx.arc(cx, cy, j.threshold * scale, 0, Math.PI * 2); cctx.stroke();
  cctx.setLineDash([]);

  // 方向标签（屏幕坐标：上=前/北，右=右/东）
  cctx.fillStyle = '#8b949e';
  cctx.font = '11px system-ui';
  cctx.textAlign = 'center'; cctx.textBaseline = 'middle';
  cctx.fillText('前 F', cx, cy - R - 6);
  cctx.fillText('后 B', cx, cy + R + 8);
  cctx.fillText('右 R', cx + R + 10, cy);
  cctx.fillText('左 L', cx - R - 10, cy);

  // 十字轴
  cctx.strokeStyle = '#30363d';
  cctx.beginPath();
  cctx.moveTo(cx - R, cy); cctx.lineTo(cx + R, cy);
  cctx.moveTo(cx, cy - R); cctx.lineTo(cx, cy + R);
  cctx.stroke();

  // 动态矢量（dynX → 右, dynY → 前/上）
  // 注意：固件逻辑 dynY>0 = FORWARD(前)，屏幕"前"在上 → y 轴向上为正
  const vx = j.dynX * scale;
  const vy = -j.dynY * scale; // 屏幕Y向下，前为上故取负
  const over = j.mag >= j.threshold;
  cctx.strokeStyle = over ? '#3fb950' : '#58a6ff88';
  cctx.lineWidth = 2.5;
  cctx.beginPath();
  cctx.moveTo(cx, cy);
  cctx.lineTo(cx + vx, cy + vy);
  cctx.stroke();
  // 箭头点
  cctx.fillStyle = over ? '#3fb950' : '#58a6ff';
  cctx.beginPath();
  cctx.arc(cx + vx, cy + vy, 4, 0, Math.PI * 2);
  cctx.fill();

  // 判定文字
  const dirLabel: Record<string, string> = {
    MOVE_FORWARD: '前 FORWARD', MOVE_BACKWARD: '后 BACKWARD',
    MOVE_LEFT: '左 LEFT', MOVE_RIGHT: '右 RIGHT',
  };
  $('judge-dir').textContent = j.judged ? dirLabel[j.judged] ?? j.judged : '--';
  $('judge-dir').style.color = j.judged ? '#3fb950' : '#8b949e';
  $('judge-dyn').textContent = `${fmt(j.dynX, 3)}, ${fmt(j.dynY, 3)}`;
  $('judge-mag').textContent = `${fmt(j.mag, 3)} / ${j.threshold.toFixed(2)}`;
}

export function setFilter(key: string, on: boolean): void {
  filters[key] = on;
}

window.addEventListener('resize', resizeCompass);

export const ui = {
  appendMessage, clearStream, onEvt, resetStats, setStatus, showError,
  updateImu, updateQuat, drawCompass, setFilter,
  resizeCompass,
};
