// ============================================================
// recorder.ts — 指令引导采集：按脚本提示用户平移设备，
// 记录 IMU/IMUQ/EVT 并打指令标签，结束后导出 JSON 供离线调参
// ============================================================

import { quatRotate, deYawHorizontal, type ImuSample } from './protocol';
import { send } from './serial';

// ---- 采集脚本 ----
type Phase = 'CALIB' | 'IDLE' | 'PREP' | 'PUSH';

interface Step {
  label: string;   // 数据标签，如 PUSH_FORWARD#3
  phase: Phase;
  dir?: string;    // FORWARD/BACKWARD/LEFT/RIGHT
  text: string;    // 大字指令
  arrow: string;   // 大箭头
  color: string;   // 方向配色
  dur: number;     // 持续 ms
  beep?: 'prep' | 'go';
  cmd?: string;    // 步骤开始时下发的串口命令
}

const DIRS = [
  { d: 'FORWARD',  cn: '前', arrow: '↑', color: '#3fb950' },
  { d: 'BACKWARD', cn: '后', arrow: '↓', color: '#f85149' },
  { d: 'LEFT',     cn: '左', arrow: '←', color: '#58a6ff' },
  { d: 'RIGHT',    cn: '右', arrow: '→', color: '#d2991d' },
];

function buildScript(reps: number): Step[] {
  const steps: Step[] = [
    // 上电/开始先做零偏校准：固件 imuCalibrateGyro(200) 阻塞约 1s，留足余量
    {
      label: 'CALIB', phase: 'CALIB', text: '正在校准零偏，保持设备完全静止…',
      arrow: '⊙', color: '#d2991d', dur: 2500, cmd: 'CALIBRATE',
    },
    // 固件校准后进入 PUSH_WARMUP_MS=1500 静默期（基线/姿态收敛），显式等它结束
    {
      label: 'WARMUP', phase: 'CALIB', text: '静默期：等待检测基线收敛，请继续保持静止…',
      arrow: '◌', color: '#8b949e', dur: 2000,
    },
    {
      label: 'IDLE_HEAD', phase: 'IDLE', text: '静置设备，请勿触碰（采集本底噪声）',
      arrow: '·', color: '#8b949e', dur: 5000,
    },
  ];
  for (const { d, cn, arrow, color } of DIRS) {
    for (let r = 1; r <= reps; r++) {
      steps.push({
        label: `PREP_${d}#${r}`, phase: 'PREP', dir: d,
        text: `准备：即将向【${cn}】推 (${r}/${reps})`, arrow: '…', color: '#8b949e',
        dur: 1500, beep: 'prep',
      });
      steps.push({
        label: `PUSH_${d}#${r}`, phase: 'PUSH', dir: d,
        text: `向【${cn}】平移推一下，然后回位`, arrow, color,
        dur: 1800, beep: 'go',
      });
    }
  }
  steps.push({
    label: 'IDLE_TAIL', phase: 'IDLE', text: '静置设备，即将完成…',
    arrow: '·', color: '#8b949e', dur: 5000,
  });
  return steps;
}

// ---- 采集数据 ----
// imu 行: [tHost, msBoard, ax,ay,az, gx,gy,gz, wx,wy,wz, label]
// imuq 行: [tHost, msBoard, qw,qx,qy,qz]
// evt 行: [tHost, msBoard, action, label]
interface Capture {
  meta: Record<string, unknown>;
  segments: { label: string; tStart: number; tEnd: number }[];
  imu: (number | string)[][];
  imuq: number[][];
  evt: (number | string)[][];
}

let running = false;
let capture: Capture | null = null;
let lastCapture: Capture | null = null;
let t0 = 0;                       // 采集起点 performance.now()
let currentLabel = 'NONE';
let stepTimer: number | null = null;
let lastQuat = { qw: 1, qx: 0, qy: 0, qz: 0 };
let imuCount = 0, evtCount = 0;

// ---- 蜂鸣提示 ----
let audioCtx: AudioContext | null = null;
function beep(kind: 'prep' | 'go'): void {
  try {
    if (!audioCtx) audioCtx = new AudioContext();
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.frequency.value = kind === 'go' ? 1200 : 500;
    gain.gain.value = 0.12;
    osc.connect(gain).connect(audioCtx.destination);
    osc.start();
    osc.stop(audioCtx.currentTime + (kind === 'go' ? 0.18 : 0.07));
  } catch (_) { /* 音频不可用不影响采集 */ }
}

// ---- 遮罩层 DOM ----
const $ = (id: string): HTMLElement => document.getElementById(id)!;
const overlay = () => $('rec-overlay');

function showOverlay(show: boolean): void {
  overlay().hidden = !show;
}

function renderStep(step: Step, idx: number, total: number): void {
  $('rec-progress').textContent = `步骤 ${idx + 1}/${total}`;
  $('rec-arrow').textContent = step.arrow;
  $('rec-arrow').style.color = step.color;
  $('rec-instruction').textContent = step.text;
  $('rec-instruction').style.color = step.phase === 'PUSH' ? step.color : '#c9d1d9';
  const fill = $('rec-bar-fill');
  // 重置进度条动画：从满到空表示剩余时间
  fill.style.transition = 'none';
  fill.style.width = '100%';
  fill.style.background = step.color;
  // 强制 reflow 后启动动画
  void fill.offsetWidth;
  fill.style.transition = `width ${step.dur}ms linear`;
  fill.style.width = '0%';
}

function renderStats(): void {
  $('rec-stats-line').textContent = `IMU 帧: ${imuCount} · EVT: ${evtCount}`;
}

// ---- 状态机 ----
export function isRecording(): boolean {
  return running;
}

export function startGuided(reps = 5): void {
  if (running) return;
  const script = buildScript(reps);
  running = true;
  imuCount = 0; evtCount = 0;
  t0 = performance.now();
  capture = {
    meta: {
      createdAt: new Date().toISOString(),
      firmware: '0.8.0-tiltpush',
      fwParams: {
        PUSH_THRESH: 0.30, alpha: 0.02, PUSH_DEBOUNCE_MS: 600,
        PUSH_PEAK_WIN_MS: 120, PUSH_RETURN_SUPPRESS_MS: 900,
        PUSH_WARMUP_MS: 1500, sampleHz: 200, reportHzDebug: 50,
      },
      reps,
      note: 'imu行=[tHost,msBoard,ax,ay,az,gx,gy,gz,wx,wy,wz,bx,by,label]；wx/wy/wz=世界系加速度；bx/by=去yaw机体航向系水平分量(固件判向输入)；label=当次指令；开头CALIB=下发CALIBRATE校准段、WARMUP=固件静默期段',
    },
    segments: [], imu: [], imuq: [], evt: [],
  };
  // 确保 DEBUG 模式开着（50Hz IMU 流）
  send('DEBUG,1');
  showOverlay(true);
  renderStats();
  runStep(script, 0);
}

function runStep(script: Step[], idx: number): void {
  if (!running || !capture) return;
  if (idx >= script.length) {
    finish();
    return;
  }
  const step = script[idx];
  const now = performance.now() - t0;
  currentLabel = step.label;
  capture.segments.push({ label: step.label, tStart: Math.round(now), tEnd: Math.round(now + step.dur) });
  if (step.cmd) send(step.cmd);
  if (step.beep) beep(step.beep);
  renderStep(step, idx, script.length);
  stepTimer = window.setTimeout(() => runStep(script, idx + 1), step.dur);
}

export function cancelGuided(): void {
  if (!running) return;
  if (stepTimer !== null) { clearTimeout(stepTimer); stepTimer = null; }
  running = false;
  currentLabel = 'NONE';
  capture = null;
  showOverlay(false);
}

function finish(): void {
  running = false;
  currentLabel = 'NONE';
  if (stepTimer !== null) { clearTimeout(stepTimer); stepTimer = null; }
  lastCapture = capture;
  capture = null;
  exportLast();
  $('rec-instruction').textContent = '✅ 采集完成，JSON 已下载';
  $('rec-arrow').textContent = '✓';
  $('rec-arrow').style.color = '#3fb950';
  window.setTimeout(() => showOverlay(false), 2000);
}

/** 导出最近一次采集（结束时自动调用，也可手动重导） */
export function exportLast(): boolean {
  if (!lastCapture) return false;
  const blob = new Blob([JSON.stringify(lastCapture)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  a.href = url;
  a.download = `bbox_push_capture_${ts}.json`;
  a.click();
  URL.revokeObjectURL(url);
  return true;
}

// ---- 数据接入（由 main.ts 在串口回调里喂入） ----
export function feedImu(s: ImuSample): void {
  if (!running || !capture) return;
  const t = Math.round(performance.now() - t0);
  const [wx, wy, wz] = quatRotate(lastQuat.qw, lastQuat.qx, lastQuat.qy, lastQuat.qz, s.ax, s.ay, s.az);
  const [bx, by] = deYawHorizontal(lastQuat.qw, lastQuat.qx, lastQuat.qy, lastQuat.qz, wx, wy);
  capture.imu.push([
    t, s.ms,
    +s.ax.toFixed(4), +s.ay.toFixed(4), +s.az.toFixed(4),
    +s.gx.toFixed(2), +s.gy.toFixed(2), +s.gz.toFixed(2),
    +wx.toFixed(4), +wy.toFixed(4), +wz.toFixed(4),
    +bx.toFixed(4), +by.toFixed(4),
    currentLabel,
  ]);
  imuCount++;
  if (imuCount % 10 === 0) renderStats();
}

export function feedImuQ(qw: number, qx: number, qy: number, qz: number, ms: number): void {
  lastQuat = { qw, qx, qy, qz };
  if (!running || !capture) return;
  const t = Math.round(performance.now() - t0);
  capture.imuq.push([t, ms, +qw.toFixed(4), +qx.toFixed(4), +qy.toFixed(4), +qz.toFixed(4)]);
}

export function feedEvt(action: string, ms: number): void {
  if (!running || !capture) return;
  const t = Math.round(performance.now() - t0);
  capture.evt.push([t, ms, action, currentLabel]);
  evtCount++;
  renderStats();
}
