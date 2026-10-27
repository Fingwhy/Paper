#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""组5 三轮：逐轮看 q6 推进 / I6 / grip_eff / grip_ang，判断哪一轮真正拧开。

判据思路（不追求绝对，只看相对突变）：
  - q6 是实测关节角，若瓶盖跟着转，负载存在 -> |i6| 或 grip_eff 有持续水平
  - 瓶盖脱开后：q6 继续转但负载骤降、grip_eff 的 std 上升（已在组1/组4验证）
本机运行：cd LEFS/experiments/20261021_stage6 && python3 或 python 均可（纯标准库）
"""
import csv, os, statistics as st

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
FILES = ['5.全量预松1_未打开.csv', '5.全量预松2_打开.csv', '5.全量预松3_已打开空转.csv']

def load(fn):
    with open(os.path.join(D, fn), newline='', encoding='utf-8') as f:
        r = csv.DictReader(f)
        rows = list(r)
    return rows

def seg_stats(rows, lo, hi):
    seg = rows[lo:hi]
    if not seg:
        return None
    i6   = [abs(float(x['i6'])) for x in seg]
    eff  = [abs(float(x['grip_eff_nm'])) for x in seg]
    ang  = [float(x['grip_ang_mm']) for x in seg]
    q6   = [float(x['q6']) for x in seg]
    dq6  = [abs(float(x['dq6'])) for x in seg]
    return dict(n=len(seg),
                i6_m=st.mean(i6), i6_sd=st.pstdev(i6),
                eff_m=st.mean(eff), eff_sd=st.pstdev(eff),
                ang_m=st.mean(ang), ang_sd=st.pstdev(ang),
                q6_0=q6[0], q6_1=q6[-1], dq6_m=st.mean(dq6))

for fn in FILES:
    rows = load(fn)
    n = len(rows)
    print('=' * 78)
    print(fn, f'  帧数={n}')
    # 时间跨度
    t0 = float(rows[0]['t_s']); t1 = float(rows[-1]['t_s'])
    print(f'  时长 {t1-t0:.1f}s   q6 {float(rows[0]["q6"]):.1f} -> {float(rows[-1]["q6"]):.1f} deg')
    # 分 10 段
    K = 10
    print(f'  {"段":>3} {"q6首":>8} {"q6末":>8} {"Δq6":>7} {"|i6|均":>7} {"i6 sd":>7} '
          f'{"eff均":>7} {"eff sd":>7} {"开度":>7} {"开度sd":>7}')
    for k in range(K):
        lo = n * k // K; hi = n * (k + 1) // K
        s = seg_stats(rows, lo, hi)
        print(f'  {k+1:>3} {s["q6_0"]:>8.1f} {s["q6_1"]:>8.1f} {s["q6_1"]-s["q6_0"]:>7.1f} '
              f'{s["i6_m"]:>7.3f} {s["i6_sd"]:>7.3f} {s["eff_m"]:>7.3f} {s["eff_sd"]:>7.4f} '
              f'{s["ang_m"]:>7.2f} {s["ang_sd"]:>7.3f}')
