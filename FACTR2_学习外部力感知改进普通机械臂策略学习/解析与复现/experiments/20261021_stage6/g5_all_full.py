#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""汇总全部「序号.中文名.csv」全量记录的关键指标，用于生成速查表。

输出列：
  I6均/I6sd        |i6| 全程均值/标准差
  eff均/effsd      |grip_eff_nm| 全程均值/标准差  <-- 本次核心判据
  ang0->ang1       夹爪开度首末（mm），变小=手指合拢=盖脱离
  ang_dr           开度总坍缩量（mm）
  t_open           首次开度跳变时刻（s），"-"=全程无跳变
  open_deg         首次跳变时的 q6（deg），即「脱开角度」
本机运行：python g5_all_full.py
"""
import csv, os, statistics as st

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
FULL = [f for f in sorted(os.listdir(D))
        if f.endswith('.csv') and f[0].isdigit()]


def analyze(fn):
    with open(os.path.join(D, fn), newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows or 'grip_eff_nm' not in rows[0]:
        return None
    t0 = float(rows[0]['t_s'])
    i6 = [abs(float(x['i6'])) for x in rows]
    ef = [abs(float(x['grip_eff_nm'])) for x in rows]
    an = [float(x['grip_ang_mm']) for x in rows]
    # 首次开度跳变
    t_open, open_deg = '-', '-'
    for k in range(1, len(rows)):
        if abs(an[k] - an[k - 1]) > 0.02:
            t_open = f"{float(rows[k]['t_s'])-t0:.2f}"
            open_deg = f"{float(rows[k]['q6']):.1f}"
            break
    return dict(n=len(rows), i6m=st.mean(i6), i6sd=st.pstdev(i6),
                efm=st.mean(ef), efsd=st.pstdev(ef),
                an0=an[0], an1=an[-1], dr=an[0] - an[-1],
                t_open=t_open, open_deg=open_deg)


print(f'{"文件":<30}{"帧":>6}{"I6均":>7}{"I6sd":>7}{"eff均":>7}{"effsd":>8}'
      f'{"开度0":>7}{"开度1":>7}{"坍缩":>7}{"跳变t":>7}{"脱开角":>8}')
print('-' * 100)
for fn in FULL:
    r = analyze(fn)
    if not r:
        continue
    print(f'{fn[:-4]:<30}{r["n"]:>6}{r["i6m"]:>7.3f}{r["i6sd"]:>7.3f}'
          f'{r["efm"]:>7.3f}{r["efsd"]:>8.4f}{r["an0"]:>7.2f}{r["an1"]:>7.2f}'
          f'{r["dr"]:>7.3f}{str(r["t_open"]):>7}{str(r["open_deg"]):>8}')
