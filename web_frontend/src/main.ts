// ============================================================
// main.ts — B-Box 移动 Action 测试面板入口
// 组装 serial 收发 + ui 显示，绑定所有交互
// ============================================================

import {
  connect, disconnect, send, isConnected, setJudgeMode, serialSupported,
} from './serial';
import { ui } from './ui';
import { startGuided, cancelGuided, exportLast, isRecording, feedImu, feedImuQ, feedEvt } from './recorder';
import { PUSH_THRESH } from './protocol';

// ---- 连接 ----
const btnConnect = document.getElementById('btn-connect') as HTMLButtonElement;

async function toggleConnect(): Promise<void> {
  if (isConnected()) {
    await disconnect({ onMessage: () => {}, onImu: () => {}, onImuQ: () => {}, onEvt: () => {}, onPushJudge: () => {}, onStatus: ui.setStatus, onError: ui.showError });
    return;
  }
  await connect({
    onMessage: ui.appendMessage,
    onImu: (s) => { ui.updateImu(s); feedImu(s); },
    onImuQ: (qw, qx, qy, qz, ms) => { ui.updateQuat(qw, qx, qy, qz); feedImuQ(qw, qx, qy, qz, ms); },
    onEvt: (seq, action, value, ms) => { ui.onEvt(seq, action, value, ms); feedEvt(action, ms); },
    onPushJudge: ui.drawCompass,
    onStatus: ui.setStatus,
    onError: ui.showError,
  });
}
btnConnect.addEventListener('click', toggleConnect);

// ---- 校准（姿态全局参考卡片） ----
document.getElementById('btn-calibrate')!.addEventListener('click', () => send('CALIBRATE'));

// ---- 重置统计 / 清空流 ----
document.getElementById('btn-reset-stats')!.addEventListener('click', ui.resetStats);
document.getElementById('btn-clear-stream')!.addEventListener('click', ui.clearStream);

// ---- 消息流过滤 ----
const filterMap: Record<string, string> = {
  'filt-evt': 'EVT', 'filt-tilts': 'TILTS', 'filt-imu': 'IMU', 'filt-imuq': 'IMUQ',
  'filt-hb': 'HEARTBEAT', 'filt-ok': 'OK', 'filt-log': 'LOG',
  'filt-unknown': 'UNKNOWN',
};
for (const [id, key] of Object.entries(filterMap)) {
  const el = document.getElementById(id) as HTMLInputElement;
  el.addEventListener('change', () => ui.setFilter(key, el.checked));
}

// ---- 推动判定：固定为去 yaw 世界系逻辑（与固件 0.8.x 一致） ----
setJudgeMode('world');

// ---- 引导采集 ----
document.getElementById('btn-rec-start')!.addEventListener('click', () => {
  if (!isConnected()) {
    ui.showError('请先连接串口，再开始引导采集。');
    return;
  }
  const reps = parseInt((document.getElementById('rec-reps') as HTMLSelectElement).value, 10);
  startGuided(reps);
});
document.getElementById('btn-rec-cancel')!.addEventListener('click', cancelGuided);
document.getElementById('btn-rec-export')!.addEventListener('click', () => {
  if (!exportLast()) ui.showError('还没有采集过数据，先跑一轮引导采集。');
});
window.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape' && isRecording()) cancelGuided();
});

// ---- 初始化 ----
ui.resizeCompass();
// 画一帧空罗盘
ui.drawCompass({ mode: 'world', dynX: 0, dynY: 0, mag: 0, threshold: PUSH_THRESH, judged: null });

if (!serialSupported()) {
  ui.showError('此浏览器不支持 Web Serial API。请使用 Chrome / Edge（桌面版）。');
}

// Web Serial 要求用户手势触发，连接按钮已就绪
console.log('🎮 B-Box 移动 Action 测试面板已就绪 — 点击"连接串口"开始');
