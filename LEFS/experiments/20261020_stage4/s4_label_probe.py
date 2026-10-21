#!/usr/bin/env python3
"""阶段 4 前置核验：标签 τ_m = K·I 的符号、量级、稳定性。

为什么必须先做这个
------------------
NEXT 的监督标签是「measured joint torque τ_m」。本机**没有力矩传感器**，唯一的
力矩观测量是电流 `cur`（A）乘阶段 2 的 K（N·m/A）。所以整条链路的前提是

    τ_m,j = K_j · I_j

这一句成立。它有两个未知：
  * **符号** —— K 的绝对值已经标定，但「电流为正」对应「关节力矩为正」还是「为负」，
    阶段 2 的差分法只约束了 ΔI 与 Δτ 的关系，没有把绝对符号钉死；
  * **量级** —— K 已被证明是 SDK 的 4.00×/0.78×，若电流口径（母线 vs 相电流）错了，
    比值会整体偏掉一个数量级。

判据
----
Q1 符号：静止保持段（H1~H3）与极端姿态段（E1~E3）共 6 个姿态 × 4 个已标定轴 = 24 个点。
        若 24 点**全部同号**或**全部反号**，符号就定了；混号则 K·I 不能当力矩用。
Q2 量级：|K·I| / |τ_gravity|。URDF 连杆质量已知与实机不符（阶段 2 结论：J3 相关性 −0.95、
        J5 −1.00 全反号），所以**只要求落在 0.1~10 倍**，只用来抓「数量级错」。
Q3 稳定性：静止段内逐轴电流的 std/|mean|。这是标签噪声地板 —— 若它就有 20%，
        阶段 4 的 MAE ≤ 0.05 N·m 验收目标根本不可能达到，得先解决测量问题。

用法（VM，~/stage0/20261020_stage4）：
    python3 s4_label_probe.py
    python3 s4_label_probe.py --pose-frac 0.4   # 只取后 40% 作为静止窗口
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"
sys.path.insert(0, str(STAGE3))
import s3_design as D  # noqa: E402

# 阶段 2 主交付 calibration/k_table.json 的 K（N·m/A）。None = 未标定。
K_TABLE = {0: None, 1: 4.7185, 2: 4.7185, 3: 0.7446, 4: 0.7446, 5: None}
# SDK 系数，仅作未标定轴的旁证（J1 1.18125 / J6 0.95844）
K_SDK = {0: 1.18125, 1: 1.18125, 2: 1.18125, 3: 0.95844, 4: 0.95844, 5: 0.95844}
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]

POSES = ["H1_hold_A", "H2_hold_B", "H3_hold_C", "E1_J2high", "E2_J3deep", "E3_wrist"]


def load_ep(name):
    p = STAGE3 / "episodes" / name / "samples.jsonl"
    rows = [json.loads(l) for l in open(p, encoding="utf-8")]
    rows = [r for r in rows if r.get("q") and None not in r["q"] and None not in r["cur"]]
    t = np.array([r["t"] for r in rows], float)
    q = np.array([r["q"] for r in rows], float)
    cur = np.array([r["cur"] for r in rows], float)
    return t, q, cur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose-frac", type=float, default=0.4,
                    help="取段尾这一比例作为静止窗口（避开 pose 接近段），默认 0.4")
    ap.add_argument("--min-frac", type=float, default=0.02,
                    help="相对量级下限，低于此值的点不计入符号统计（默认 0.02 N·m）")
    args = ap.parse_args()

    urdf, arm = D.load()
    print("末端口径：URDF urdf 包 + 夹爪 end_mass（阶段 2 R41）")
    print()

    rows = []          # (pose, j, q_deg, I, tau_K, tau_u, ratio)
    for ep in POSES:
        t, q, cur = load_ep(ep)
        n = len(t)
        lo = int(n * (1.0 - args.pose_frac))
        qm = q[lo:].mean(axis=0)
        Im = cur[lo:].mean(axis=0)
        Isd = cur[lo:].std(axis=0)
        tg = np.asarray(arm.tau_gravity(list(qm)), float)
        print(f"=== {ep}  静止窗口 {n - lo}/{n} 帧  t={t[lo]:.1f}~{t[-1]:.1f} s ===")
        print("  轴   q(deg)    I均(A)   I波动%    K·I(N·m)   URDFτg      比值")
        for j in range(6):
            k = K_TABLE[j]
            ik = k * Im[j] if k is not None else float("nan")
            ratio = ik / tg[j] if (k is not None and abs(tg[j]) > 1e-9) else float("nan")
            ripple = 100.0 * Isd[j] / abs(Im[j]) if abs(Im[j]) > 1e-6 else float("nan")
            print("  %-4s %+8.2f  %+8.3f  %6.1f  %+9.3f  %+9.3f  %s"
                  % (JN[j], np.degrees(qm[j]), Im[j], ripple, ik, tg[j],
                     ("%+.2f" % ratio) if np.isfinite(ratio) else "  -"))
            if k is not None:
                rows.append((ep, j, np.degrees(qm[j]), Im[j], ik, tg[j], ratio))
        print()

    # ---------------- Q1 符号 ----------------
    print("## Q1 符号一致性")
    big = [r for r in rows if abs(r[5]) >= args.min_frac]
    same = sum(1 for r in big if np.sign(r[4]) == np.sign(r[5]))
    diff = len(big) - same
    print(f"  参与统计 {len(big)} 点（|τg| ≥ {args.min_frac} N·m），"
          f"同号 {same}，反号 {diff}")
    if same and not diff:
        print("  → 结论：**同号**，τ_m = +K·I")
    elif diff and not same:
        print("  → 结论：**反号**，τ_m = −K·I")
    else:
        print("  → 结论：**混号，K·I 不能当力矩用**")
    print()

    # ---------------- Q2 量级 ----------------
    print("## Q2 量级（|K·I| / |τg|）")
    rr = np.array([abs(r[6]) for r in big if np.isfinite(r[6])])
    if len(rr):
        print(f"  n={len(rr)}  中位数 {np.median(rr):.2f}  范围 {rr.min():.2f}~{rr.max():.2f}")
        ok = (rr >= 0.1) & (rr <= 10.0)
        print(f"  落在 0.1~10 倍：{ok.sum()}/{len(rr)}")
        print("  逐轴中位比值：",
              {JN[j]: round(float(np.median([abs(r[6]) for r in big if r[1] == j])), 2)
               for j in range(6) if any(r[1] == j for r in big)})
    print()

    # ---------------- Q3 稳定性 ----------------
    print("## Q3 静止段电流稳定性（std/|mean|，%）")
    print("  " + "  ".join(f"{jn:>6s}" for jn in JN))
    for ep in POSES:
        t, q, cur = load_ep(ep)
        n = len(t)
        lo = int(n * (1.0 - args.pose_frac))
        Im = cur[lo:].mean(axis=0)
        Isd = cur[lo:].std(axis=0)
        cells = []
        for j in range(6):
            v = 100.0 * Isd[j] / abs(Im[j]) if abs(Im[j]) > 1e-6 else float("nan")
            cells.append(("%6.1f" % v) if np.isfinite(v) else "     -")
        print(f"  {ep:12s} " + " ".join(cells))
    print()

    # ---------------- 未标定轴旁证 ----------------
    print("## 旁证：未标定轴用 SDK 系数估的量级（J1/J6）")
    for ep in POSES[:1]:
        t, q, cur = load_ep(ep)
        n = len(t)
        lo = int(n * (1.0 - args.pose_frac))
        qm = q[lo:].mean(axis=0)
        Im = cur[lo:].mean(axis=0)
        tg = np.asarray(arm.tau_gravity(list(qm)), float)
        for j in (0, 5):
            k = K_SDK[j]
            print("  %-4s I=%+.3f A  K_SDK·I=%+.3f  URDFτg=%+.3f  比值 %+.2f"
                  % (JN[j], Im[j], k * Im[j], tg[j],
                     (k * Im[j] / tg[j]) if abs(tg[j]) > 1e-9 else float("nan")))
    print()
    print(urdf is not None and "（URDF 只作量级参照：阶段 2 已判定连杆质量与实机不符）" or "")


if __name__ == "__main__":
    main()
