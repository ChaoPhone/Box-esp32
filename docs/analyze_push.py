# -*- coding: utf-8 -*-
"""
analyze_push.py — 分析 web_frontend 引导采集导出的 bbox_push_capture_*.json
用法: python analyze_push.py <capture.json>

输出:
  1. 各段统计: IDLE 噪声底 / PUSH 段世界系&传感器系动态峰值与主方向
  2. EVT 混淆矩阵 (指令方向 vs 固件判定)
  3. yaw 漂移估计 (IMUQ)
"""
import json
import math
import sys
from collections import defaultdict

# Windows GBK 控制台兜底
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# 固件高通: 200Hz alpha=0.02; 采集数据为 50Hz 上报
# 等效 50Hz alpha: 1-(1-0.02)^4 ≈ 0.0776
ALPHA_50 = 1 - (1 - 0.02) ** 4


def highpass(series, alpha):
    """一阶移动平均高通(与固件 pushDetect 相同), 首帧用真实值初始化基线"""
    out = []
    ma = None
    for v in series:
        if ma is None:
            ma = v
            out.append(0.0)
            continue
        ma = ma * (1 - alpha) + v * alpha
        out.append(v - ma)
    return out


def yaw_of(qw, qx, qy, qz):
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


def main(path):
    with open(path, 'r', encoding='utf-8') as f:
        cap = json.load(f)

    imu = cap['imu']      # [t,ms,ax,ay,az,gx,gy,gz,wx,wy,wz,label]
    evt = cap['evt']      # [t,ms,action,label]
    imuq = cap['imuq']    # [t,ms,qw,qx,qy,qz]
    print(f"meta: {cap['meta'].get('firmware')} reps={cap['meta'].get('reps')}")
    print(f"imu帧: {len(imu)}  evt: {len(evt)}  imuq: {len(imuq)}\n")

    # ---- 全程高通(模拟固件, 跨段连续) ----
    # v0.8.0+ 采集含 bx/by(去yaw机体航向系, 固件判向输入)列, 优先用它；旧格式退回世界系 wx/wy
    has_body = len(imu[0]) >= 14
    px_col, py_col = (11, 12) if has_body else (8, 9)
    print(f"主判定坐标系: {'机体航向系 bx/by (去yaw)' if has_body else '世界系 wx/wy'}")
    wx_dyn = highpass([r[px_col] for r in imu], ALPHA_50)
    wy_dyn = highpass([r[py_col] for r in imu], ALPHA_50)
    ax_dyn = highpass([r[2] for r in imu], ALPHA_50)
    ay_dyn = highpass([r[3] for r in imu], ALPHA_50)
    labels = [r[-1] for r in imu]

    # ---- 按段聚合 ----
    seg_world = defaultdict(list)   # label -> [(mag, dynX, dynY)]
    seg_sensor = defaultdict(list)
    for i, lb in enumerate(labels):
        mw = math.hypot(wx_dyn[i], wy_dyn[i])
        ms_ = math.hypot(ax_dyn[i], ay_dyn[i])
        seg_world[lb].append((mw, wx_dyn[i], wy_dyn[i]))
        seg_sensor[lb].append((ms_, ax_dyn[i], ay_dyn[i]))

    def dir_of(dx, dy):
        if abs(dx) > abs(dy):
            return 'RIGHT' if dx > 0 else 'LEFT'
        return 'FORWARD' if dy > 0 else 'BACKWARD'

    # ---- 噪声底(IDLE/PREP 段) ----
    print("== 噪声底 (世界系水平动态模长, g) ==")
    for kind in ('IDLE_HEAD', 'IDLE_TAIL'):
        vals = [m for m, _, _ in seg_world.get(kind, [])]
        if vals:
            vals.sort()
            print(f"  {kind:10s} n={len(vals):4d} p50={vals[len(vals)//2]:.4f} "
                  f"p95={vals[int(len(vals)*0.95)]:.4f} max={vals[-1]:.4f}")
    prep_vals = sorted(m for lb, rows in seg_world.items() if lb.startswith('PREP_') for m, _, _ in rows)
    if prep_vals:
        print(f"  PREP(all)  n={len(prep_vals):4d} p50={prep_vals[len(prep_vals)//2]:.4f} "
              f"p95={prep_vals[int(len(prep_vals)*0.95)]:.4f} max={prep_vals[-1]:.4f}")

    # ---- PUSH 段峰值 ----
    print("\n== PUSH 段峰值 (世界系 | 传感器系) 与主方向 ==")
    push_peaks = defaultdict(list)
    for lb in sorted(seg_world):
        if not lb.startswith('PUSH_'):
            continue
        rows_w = seg_world[lb]
        rows_s = seg_sensor[lb]
        pw = max(rows_w, key=lambda r: r[0])
        ps = max(rows_s, key=lambda r: r[0])
        expect = lb.split('_')[1].split('#')[0]
        dw, dss = dir_of(pw[1], pw[2]), dir_of(ps[1], ps[2])
        okw = '✓' if dw == expect else '✗'
        oks = '✓' if dss == expect else '✗'
        push_peaks[expect].append(pw[0])
        print(f"  {lb:18s} 世界系 peak={pw[0]:.3f} dir={dw:8s}{okw} | 传感器系 peak={ps[0]:.3f} dir={dss:8s}{oks}")

    print("\n== 各方向世界系峰值分布 (g) ==")
    for d, ps in push_peaks.items():
        ps.sort()
        print(f"  {d:8s} n={len(ps)} min={ps[0]:.3f} p50={ps[len(ps)//2]:.3f} max={ps[-1]:.3f}")

    # ---- EVT 混淆矩阵 ----
    print("\n== EVT 混淆矩阵 (行=指令段, 列=固件判定) ==")
    conf = defaultdict(lambda: defaultdict(int))
    for _, _, action, lb in evt:
        if lb.startswith('PUSH_') or lb.startswith('PREP_'):
            expect = lb.split('_', 1)[1].split('#')[0] if lb.startswith('PUSH_') else 'PREP:' + lb.split('_', 1)[1].split('#')[0]
        else:
            expect = lb
        conf[expect][action] += 1
    acts = sorted({a for row in conf.values() for a in row})
    if acts:
        head = 'expect\\evt'.ljust(16) + ''.join(a.replace('MOVE_', '')[:8].ljust(10) for a in acts)
        print('  ' + head)
        for exp in sorted(conf):
            line = exp.ljust(16) + ''.join(str(conf[exp].get(a, 0)).ljust(10) for a in acts)
            print('  ' + line)
    else:
        print('  (无 EVT)')
    # 漏检统计
    print("\n== 漏检/命中率 ==")
    reps = cap['meta'].get('reps', 0)
    for d in ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT'):
        hit = conf.get(d, {}).get(f'MOVE_{d}', 0)
        total_evt = sum(conf.get(d, {}).values())
        print(f"  {d:8s} 指令{reps}次 -> EVT共{total_evt}条, 其中判对{hit}条")

    # ---- yaw 漂移 ----
    if len(imuq) > 10:
        y0 = yaw_of(*imuq[0][2:6])
        y1 = yaw_of(*imuq[-1][2:6])
        dt_s = (imuq[-1][0] - imuq[0][0]) / 1000
        drift = (y1 - y0 + 540) % 360 - 180
        print(f"\n== yaw 漂移 == 起={y0:.1f}° 终={y1:.1f}° Δ={drift:+.1f}° / {dt_s:.0f}s "
              f"({drift/dt_s*60:+.1f}°/min)")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'bbox_push_capture_2026-07-25T04-40-06.json')
