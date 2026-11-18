#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""组5 三轮：0.5s 细粒度，定位「开度坍缩 / eff 均值阶跃」的准确时刻。

脱开（盖脱离夹爪）的硬证据按可靠性排序：
  1) grip_ang_mm 突然变小（手指合拢，失去盖的支撑）—— 物理量，最硬
  2) eff 均值阶跃下降 + eff sd 尖峰
本机运行：python g5_timeline.py
"""
import csv, os, statistics as st

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
FILES = ['5.全量预松1_未打开.csv', '5.全量预松2_打开.csv', '5.全量预松3_已打开空转.csv']
WIN = 0.5  # 秒


def load(fn):
    with open(os.path.join(D, fn), newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


for fn in FILES:
    rows = load(fn)
    t = [float(x['t_s']) for x in rows]
    t0 = t[0]
    print('=' * 88)
    print(fn)
    # 找「开度跳变」：相邻帧差 > 0.02 mm
    jumps = []
    for k in range(1, len(rows)):
        a0 = float(rows[k - 1]['grip_ang_mm'])
        a1 = float(rows[k]['grip_ang_mm'])
        if abs(a1 - a0) > 0.02:
            jumps.append((t[k] - t0, float(rows[k]['q6']), a0, a1))
    if jumps:
        print(f'  开度跳变（>0.02mm）共 {len(jumps)} 处，首次 {len(jumps) and jumps[0][0]:.2f}s：')
        for j in jumps[:8]:
            print(f'    t={j[0]:6.2f}s  q6={j[1]:6.1f}  开度 {j[2]:.2f} -> {j[3]:.2f}')
    else:
        print('  开度全程无 >0.02mm 跳变（无脱开迹象）')

    # 0.5s 窗口
    print(f'  {"t(s)":>6} {"q6":>7} {"|i6|":>6} {"i6sd":>6} {"eff均":>7} {"effsd":>7} {"开度":>7}')
    k0 = 0
    while k0 < len(rows):
        tt = t[k0] - t0
        seg = [x for x in rows[k0:] if float(x['t_s']) - t0 < tt + WIN]
        if not seg:
            break
        i6 = [abs(float(x['i6'])) for x in seg]
        ef = [abs(float(x['grip_eff_nm'])) for x in seg]
        an = [float(x['grip_ang_mm']) for x in seg]
        print(f'  {tt:6.2f} {st.mean([float(x["q6"]) for x in seg]):>7.1f} '
              f'{st.mean(i6):>6.3f} {st.pstdev(i6):>6.3f} '
              f'{st.mean(ef):>7.3f} {st.pstdev(ef):>7.4f} {st.mean(an):>7.2f}')
        k0 += len(seg)
