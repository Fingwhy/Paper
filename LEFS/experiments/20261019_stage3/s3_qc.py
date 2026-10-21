#!/usr/bin/env python3
# s3_qc.py —— 阶段 3 采集的逐段体检（不用连机器人，只读产物）
#
# 为什么要有这一步
# ----------------
# 2026-10-20 13:5x 的教训：S1_6_lo 的 J6 被指令 ±15°，实测只走了 0.27°，
# 电流冲到 −12.8 A，把驱动打成「过流 + 堵转」锁死；而 run_s3.sh 只检查
# 「脚本退出码 0 + 样本齐备」，于是这一段被当成成功收下，直到下一段
# 使能 J6 失败才暴露 —— 已经浪费了一整段（77 s 录制 + 一次使能）。
# 所以：每段采完都要核对「**实测幅度 vs 设计幅度**」与「**峰值电流**」。
#
# 判据（三条，任一不过即 FAIL）
#   1) 设计有激励的轴：实测峰峰 ≥ 设计峰峰的 60%（跟得上）
#   2) 设计为 0 的轴：实测峰峰 ≤ 2°（没被别的轴带跑）
#   3) 任何轴峰值电流 ≤ 3.5 A（留 0.5 A 余量到 4 A 守卫阈值）
#
# 用法（VM，~/stage0/20261019_stage3 下）：
#   python3 s3_qc.py                 # 扫 episodes/ 下全部段
#   python3 s3_qc.py S1_6_lo         # 只查一段
#   python3 s3_qc.py --min-ratio 0.8 # 改幅度判据

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import s3_design as D  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
CUR_LIMIT_A = 3.5
TRACK_LIMIT_DEG = 2.0
FOLLOW_RATIO = 0.60
# samples.jsonl 里 q / dq 是 **弧度**（0x251 原始口径），cur 是 A。
# 2026-10-20 13:5x 第一次跑本脚本时按「度」读，把 12° 的 J1 当成 0.21° 判了 16 段全 FAIL。
RAD2DEG = 57.29577951308232


def read_samples(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def stats_axis(rows, idx):
    q = [r["q"][idx] for r in rows if r.get("q") and r["q"][idx] is not None]
    c = [r["cur"][idx] for r in rows if r.get("cur") and r["cur"][idx] is not None]
    qpp = (max(q) - min(q)) * RAD2DEG if len(q) >= 2 else 0.0
    cmax = max((abs(v) for v in c), default=0.0)
    return len(q), qpp, cmax


def over_limit_s(rows, idx, limit):
    """该轴电流超过 limit 的累计时长（秒）——看是「瞬间毛刺」还是「持续顶着」。"""
    ts = [r["t"] for r in rows if r.get("cur") and r["cur"][idx] is not None]
    if len(ts) < 2:
        return 0.0
    dt = (ts[-1] - ts[0]) / max(1, len(ts) - 1)
    n = sum(1 for r in rows if r.get("cur") and r["cur"][idx] is not None
            and abs(r["cur"][idx]) > limit)
    return n * dt


def check(name, min_ratio):
    t = D.BY_NAME.get(name)
    if t is None:
        return None
    ep = os.path.join(HERE, "episodes", name, "samples.jsonl")
    if not os.path.exists(ep):
        return None
    rows_all = read_samples(ep)
    # 只统计「有指令」的窗口：每段录制都从 pre-pose 之前 3 s 开始，
    # 那几秒臂在从上一条轨迹的中心挪到本条的中心，静止轴当然会被带跑
    # （2026-10-20 14:0x 第一次判据没排除它，S1_2_lo/S1_3_lo/S1_4_lo/S1_5_lo 全被误判）。
    rows = [r for r in rows_all if r.get("q_cmd")] or rows_all
    design_pp = [2.0 * a for a in t["amp"]]
    got = [stats_axis(rows, i) for i in range(6)]

    fails = []
    lines = []
    for i, jn in enumerate(JN):
        n, qpp, cmax = got[i]
        d = design_pp[i]
        if d > 0:
            ratio = qpp / d if d else 0.0
            ok_amp = ratio >= min_ratio
            amp_txt = f"设计 {d:6.2f}°  实测 {qpp:6.2f}°  跟随 {ratio * 100:5.1f}%"
        else:
            ratio = None
            ok_amp = qpp <= TRACK_LIMIT_DEG
            amp_txt = f"设计 静止    实测 {qpp:6.2f}°"
        ok_cur = cmax <= CUR_LIMIT_A
        if not ok_amp:
            fails.append(f"{jn} 幅度{'跟不上' if d > 0 else '被带跑'}")
        if not ok_cur:
            over = over_limit_s(rows, i, CUR_LIMIT_A)
            fails.append(f"{jn} 电流 {cmax:.2f} A 超 {CUR_LIMIT_A}（超限累计 {over:.1f} s）")
        lines.append(
            f"    {jn}  {amp_txt}   峰值电流 {cmax:6.2f} A"
            f"  {'✓' if ok_amp and ok_cur else '✗'}"
        )
    return dict(name=name, n=len(rows), lines=lines, fails=fails, design_pp=design_pp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--min-ratio", type=float, default=FOLLOW_RATIO)
    args = ap.parse_args()

    eps = os.path.join(HERE, "episodes")
    if args.names:
        names = args.names
    elif os.path.isdir(eps):
        names = sorted(d for d in os.listdir(eps) if os.path.isdir(os.path.join(eps, d)))
    else:
        names = []

    if not names:
        raise SystemExit("episodes/ 下没有段")

    nfail = 0
    for nm in names:
        r = check(nm, args.min_ratio)
        if r is None:
            print(f"[跳过] {nm}（不在 TRACKS 里，或缺 samples.jsonl）")
            continue
        head = "✓ PASS" if not r["fails"] else "✗ FAIL"
        print(f"[{head}] {nm}  样本 {r['n']}")
        for ln in r["lines"]:
            print(ln)
        if r["fails"]:
            nfail += 1
            print("    问题：" + "；".join(r["fails"]))
        print()

    print(f"=== 合计 {len(names)} 段，{nfail} 段不合格 ===")
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
