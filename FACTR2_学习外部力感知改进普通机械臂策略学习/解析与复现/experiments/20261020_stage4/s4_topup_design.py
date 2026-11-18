#!/usr/bin/env python3
"""阶段 4 补采：专补 q2 ∈ [60°, 105°] 的运动段。

为什么只补这一块
----------------
阶段 4 诊断（`s4_diag.py`）显示：test 有 **50.9%** 的窗口落在 q2∈[75,95°]，
而训练集同区间只占 **5.9%**。`T1_J2J3J5` 的 J2 误差 3.28 N·m（线性基线只要 0.34），
就是被这一块稀疏覆盖拖出来的。补采的目的不是「多采点数据」，而是**把 q2 高段的
姿态-力矩对应关系填密**，让任何模型（包括 NEXT）有得学。

设计原则
--------
1. **几何先过闸门**（发帧之前）：正弦包络是「各轴独立摆动」，最危险的是
   `center ± amp` 的 64 个角点，全部过 `pose_verdict`；
2. **q2 越高越要屈肘**：q2=95° 时若 J3 不够负，末端会伸出去撞台面/立柱；
3. **速度卡在 0.3 rad/s 下的 77%**（13.2 °/s），与阶段 3 一致；
4. J6 中心一律 +30°（行程单侧 [0,+343]，0° 是硬端点，R52）。

用法（VM，~/stage0/20261020_stage4）：
    python3 s4_topup_design.py --gate      # 几何 + 限位 + 速度判定
    python3 s4_topup_design.py --emit P2_J2mid   # 打印单段的 stage1_cmd 命令
"""

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"
sys.path.insert(0, str(STAGE3))
import s3_design as D  # noqa: E402

G = D.G
LIMITS = D.LIMITS
V_DESIGN = D.V_DESIGN          # 13.2 °/s
J6_HOME = D.J6_HOME            # 30°


def corners(center, amp):
    out = [[]]
    for c, a in zip(center, amp):
        nxt = []
        for b in out:
            if a == 0.0:
                nxt.append(b + [c])
            else:
                nxt.append(b + [c - a])
                nxt.append(b + [c + a])
        out = nxt
    return out


def limit_margin(q):
    return min(min(q[i] - LIMITS[i][0], LIMITS[i][1] - q[i]) for i in range(6))


def T(name, center, amp, freq, dur, note):
    return dict(name=name, center=list(center), amp=list(amp), freq=freq,
                dur=dur, kind="sine", note=note)


# 中心一律 J6=30；q3 随 q2 加深（屈肘），保证末端不伸出去。
CAND = [
    T("P1_J2lo",  (0, 68, -68, 0, 35, J6_HOME), (0, 14, 0, 0, 0, 0), 0.12, 30,
      "q2 54~82：覆盖 test 下沿"),
    T("P2_J2mid", (0, 80, -78, 0, 35, J6_HOME), (0, 14, 0, 0, 0, 0), 0.12, 30,
      "q2 66~94：test 主峰区"),
    T("P3_J2hi",  (0, 92, -92, 0, 35, J6_HOME), (0, 12, 0, 0, 0, 0), 0.10, 30,
      "q2 80~104：test 上沿，训练几乎空白"),
    T("P4_J2J3",  (0, 84, -86, 0, 35, J6_HOME), (0, 12, 12, 0, 0, 0), 0.12, 30,
      "q2/q3 联动：解 τ2 对 q3 的依赖"),
    T("P5_J2J5",  (0, 84, -80, 0, 42, J6_HOME), (0, 12, 0, 0, 10, 0), 0.12, 30,
      "q2/q5 联动：解 τ2 对 q5 的依赖"),
    T("P6_J2wide", (0, 80, -80, 0, 35, J6_HOME), (0, 20, 0, 0, 0, 0), 0.09, 30,
      "q2 60~100 大幅扫：单段覆盖整个缺口"),
    T("P7_J2slow", (0, 88, -86, 0, 35, J6_HOME), (0, 12, 0, 0, 0, 0), 0.05, 36,
      "慢速：降低惯性/摩擦混淆"),
]


def artic(t):
    """速度峰值（°/s）与位置峰峰（°）。"""
    pp = [2.0 * a for a in t["amp"]]
    vp = [2.0 * math.pi * t["freq"] * a for a in t["amp"]]
    return pp, vp


def verdict(arm, t):
    bad = []
    worst = (1e9, None)
    for q in corners(t["center"], t["amp"]):
        v = G.pose_verdict(arm, G.deg(*q))
        if v["bad"]:
            bad.append(f"角点 {tuple(round(x,1) for x in q)}: {v['bad']}")
            break
        c = min(v["zmin"] - G.pack()["zmin"], v["clr"] - G.pack()["clr"])
        if c < worst[0]:
            worst = (c, q)
    lm = min(limit_margin(q) for q in corners(t["center"], t["amp"]))
    pp, vp = artic(t)
    vmax = max(vp)
    if vmax > V_DESIGN:
        bad.append(f"速度 {vmax:.1f} > {V_DESIGN:.1f} °/s")
    if lm < 0:
        bad.append(f"限位越界 {lm:+.1f}°")
    if pp and max(pp) < 8.0:
        bad.append(f"激励不足 pp={max(pp):.1f}<8")
    return bad, worst, lm, vmax, pp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", action="store_true")
    ap.add_argument("--emit", default=None)
    ap.add_argument("--json", default=str(HERE / "topup_tracks.json"))
    args = ap.parse_args()

    G.set_pack("gripper")
    urdf, arm = D.load()

    if args.emit:
        t = next(x for x in CAND if x["name"] == args.emit)
        c = " ".join(f"{v:g}" for v in t["center"])
        a = " ".join(f"{v:g}" for v in t["amp"])
        print(f"{t['name']}|{c}|{a}|{t['freq']}|{t['dur']}")
        return

    print("包络 =", G.pack()["label"], "| MIN_TCP_Z =", G.MIN_TCP_Z_MM)
    print()
    print("| 段 | center | amp | f | 时长 | 峰峰 | 速度峰值 | 最差余量 | 限位余量 | 判 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    ok = 0
    for t in CAND:
        bad, worst, lm, vmax, pp = verdict(arm, t)
        flag = "PASS" if not bad else "FAIL: " + "; ".join(bad)
        if not bad:
            ok += 1
        print(f"| {t['name']} | {tuple(t['center'])} | {tuple(t['amp'])} | {t['freq']} | "
              f"{t['dur']}s | {max(pp):.0f}° | {vmax:.1f} | {worst[0]:.0f} mm | "
              f"{lm:+.0f}° | {flag} |")
    print()
    print(f"总判：{ok}/{len(CAND)} 放行")

    # q2 覆盖对比
    print()
    print("补采后 q2 覆盖分布（每条正弦按均匀相位近似）：")
    import numpy as np
    bins = [(0, 45), (45, 60), (60, 75), (75, 95), (95, 110), (110, 180)]
    add = {b: 0.0 for b in bins}
    for t in CAND:
        lo, hi = t["center"][1] - t["amp"][1], t["center"][1] + t["amp"][1]
        for b in bins:
            ov = max(0.0, min(hi, b[1]) - max(lo, b[0]))
            if ov > 0:
                add[b] += ov / (hi - lo) / len(CAND)
    for b in bins:
        bar = "#" * int(60 * add[b])
        print(f"  {b[0]:3d}-{b[1]:3d}°  {100*add[b]:5.1f}% {bar}")

    if args.gate or True:
        json.dump(CAND, open(args.json, "w"), ensure_ascii=False, indent=1)
        print(f"\n轨迹表 → {args.json}")


if __name__ == "__main__":
    main()
