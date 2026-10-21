#!/usr/bin/env python3
"""s21_probe_j6.py —— 「只差 J6」的一对姿态，反推水瓶质心的**横向偏心**

为什么单独做这个探针
--------------------
用户明确说过：水瓶是斜着绑在夹爪上的 —— J6 归 0 时夹爪本身不竖直，所以水瓶
跟着腕部一起斜。这句话在数据上有一个**不需要空载**就能检验的推论：

    K15 = (0,45,-60,0,45,30)   J1..J5 完全相同，只有 J6 = 30°
    K14 = (0,45,-60,0,45,90)  同上，J6 = 90°

两姿态只差一个 J6（60°）。于是：
  * 臂自重、固件重力前馈、URDF 连杆质量误差 —— 全部**逐轴相同**，相减即消；
  * 法兰原点在 J6 转轴上 ⇒ p0 项也逐轴相同，相减即消；
  * 剩下的只有「质心绕工具轴转了 60°」这一个效应。

    ΔI_j = [ (A15_j − A14_j) · c6 ] / K_j

若质心严格在工具轴上（cx = cy = 0），ΔI 必须**全零**。所以 ΔI 非零 ⇒
水瓶确实斜着；而 ΔI 的**大小**直接给出横向偏心量。

注意这个探针**定不了 K**：ΔI 与 (c/K) 只以乘积形式出现。它只回答
「斜多少」，K 仍要靠 s21_pair_fit.py 的空载−带载差分。这里固定 K = K_SDK
作为量级基准，结论只当数量级看。

用法（本机，experiments/20261019_stage2 下）
    python s21_probe_j6.py
    python s21_probe_j6.py --last-s 30
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage2_design as S
import s21_design as G
import s21_fit as F

EP = F.EP_DIR
K_SDK = F.K_SDK
JN = F.JOINT_NAME

# (标签, J6 小的那对, J6 大的那对, 名义 J6 差)
PAIR = ("K15↔K14", "s21_K15", "s21_K14", 60.0)


def wls2(T, Y, W):
    """两参数加权最小二乘 y = t0·x0 + t1·x1（无常数项，纯线性过原点）。"""
    a00 = sum(w * t[0] * t[0] for w, t in zip(W, T))
    a01 = sum(w * t[0] * t[1] for w, t in zip(W, T))
    a11 = sum(w * t[1] * t[1] for w, t in zip(W, T))
    b0 = sum(w * t[0] * y for w, t, y in zip(W, T, Y))
    b1 = sum(w * t[1] * y for w, t, y in zip(W, T, Y))
    det = a00 * a11 - a01 * a01
    if abs(det) < 1e-12:
        return None
    x0 = (a11 * b0 - a01 * b1) / det
    x1 = (a00 * b1 - a01 * b0) / det
    chi2 = sum(w * (y - (x0 * t[0] + x1 * t[1])) ** 2 for w, t, y in zip(W, T, Y))
    return x0, x1, chi2, a11 / det, a00 / det


def main(argv=None):
    ap = argparse.ArgumentParser(description="只差 J6 的一对姿态反推横向偏心")
    ap.add_argument("--last-s", type=float, default=30.0)
    ap.add_argument("--stable-deg", type=float, default=0.3)
    ap.add_argument("--urdf", default=str(Path(__file__).resolve().parent.parent /
                                         "_vendor" / "urdf" / "piper_description.urdf"))
    a = ap.parse_args(argv)

    urdf = S.Urdf(Path(a.urdf))
    arm = S.Arm(urdf, end_mass=F.GRIPPER_MASS, end_com=urdf.gripper_com())

    lab, d_lo, d_hi, _ = PAIR
    p_lo, p_hi = EP / d_lo / "samples.jsonl", EP / d_hi / "samples.jsonl"
    for p in (p_lo, p_hi):
        if not p.exists():
            print(f"[ABORT] 缺 {p}", file=sys.stderr)
            return 2

    r_lo = F.build_row(arm, p_lo, a.last_s, a.stable_deg, True)
    r_hi = F.build_row(arm, p_hi, a.last_s, a.stable_deg, True)

    print(f"## {lab}：J1..J5 相同，只有 J6 差 {PAIR[3]:.0f}°")
    print(f"  q_lo = {','.join(f'{math.degrees(v):+.2f}' for v in r_lo['q'])}")
    print(f"  q_hi = {','.join(f'{math.degrees(v):+.2f}' for v in r_hi['q'])}")
    dq = [math.degrees(x - y) for x, y in zip(r_hi["q"], r_lo["q"])]
    print(f"  姿态差 = {','.join(f'{v:+.2f}' for v in dq)}  ← 只有第 6 轴该动")
    others = max(abs(dq[j]) for j in range(5))
    print(f"  前五轴最大偏差 {others:.3f}°  {'✓ 干净' if others < 0.5 else '✗ 定位不重复，结论打折'}")
    print()

    dI = [r_hi["cur"][j] - r_lo["cur"][j] for j in range(6)]
    sig = [math.hypot(r_hi["sig"][j], r_lo["sig"][j]) for j in range(6)]
    print("## 1. 实测 ΔI = I(J6=90°) − I(J6=30°)   （A）")
    print("| 关节 | ΔI | ±1σ | ΔI/σ（信噪比） |")
    print("| --- | --- | --- | --- |")
    for j in range(6):
        snr = abs(dI[j]) / max(sig[j], 1e-9)
        print(f"| {JN[j]} | {dI[j]:+.4f} | {sig[j]:.4f} | {snr:.1f}× |")
    print()

    # Δτ_j(c) = (A_hi − A_lo)_j · c  —— 只取前两维（绕工具轴旋转不影响 z 分量）
    D = [[r_hi["A"][j][i] - r_lo["A"][j][i] for i in range(2)] for j in range(6)]
    print("## 2. 几何矩阵 (A_hi − A_lo)·(cx, cy)   （N·m / mm）")
    print("| 关节 | ∂Δτ/∂cx | ∂Δτ/∂cy |")
    print("| --- | --- | --- |")
    for j in range(6):
        print(f"| {JN[j]} | {D[j][0]*1000:+.4f} | {D[j][1]*1000:+.4f} |")
    dz = max(abs(r_hi["A"][j][2] - r_lo["A"][j][2]) for j in range(6))
    print(f"\n  z 分量差分最大 {dz*1000:.6f} N·m/mm "
          f"{'✓ 绕工具轴旋转确实不改变 z 分量' if dz < 1e-6 else '✗ 异常'}")
    print()

    # 固定 K = K_SDK，解 ΔI_j = (D_j · c) / K_j  ⇒  ΔI_j·K_j = D_j·c
    idx = [j for j in range(6) if math.hypot(D[j][0], D[j][1]) > 1e-9]
    T = [[D[j][0], D[j][1]] for j in idx]
    Y = [dI[j] * K_SDK[j] for j in idx]
    W = [1.0 / max(sig[j] * K_SDK[j], 1e-12) ** 2 for j in idx]
    r = wls2(T, Y, W)
    if r is None:
        print("[ABORT] 几何矩阵退化，解不出横向偏心", file=sys.stderr)
        return 2
    cx, cy, chi2, cxx, cyy = r
    scale = math.sqrt(chi2 / max(len(idx) - 2, 1))
    sx, sy = math.sqrt(cxx) * scale, math.sqrt(cyy) * scale
    print("## 3. 横向偏心（固定 K = K_SDK 作量级基准）")
    print(f"  c6 横向 = ({cx*1000:+.1f}, {cy*1000:+.1f}) mm   "
          f"±({sx*1000:.1f}, {sy*1000:.1f})")
    print(f"  |c_xy| = {math.hypot(cx, cy)*1000:.1f} mm    χ² = {chi2:.2f} "
          f"（{len(idx)} 轴，自由度 {len(idx)-2}）")
    print()
    print("## 4. 逐轴回代（ΔI 预测 vs 实测，A）")
    print("| 关节 | 预测 | 实测 | 残差 |")
    print("| --- | --- | --- | --- |")
    for j in range(6):
        pred = (D[j][0] * cx + D[j][1] * cy) / K_SDK[j]
        print(f"| {JN[j]} | {pred:+.4f} | {dI[j]:+.4f} | {dI[j]-pred:+.4f} |")
    print()
    print("判读：")
    if math.hypot(cx, cy) * 1000 < 5.0:
        print("  横向偏心 < 5 mm ⇒ 质心基本在工具轴上，K15/K14 对几乎不提供信息，")
        print("  「斜着」只体现在沿轴分量（cz）上，那要靠空载−带载差分才能定。")
    else:
        print(f"  横向偏心 {math.hypot(cx, cy)*1000:.0f} mm ⇒ 水瓶确实斜着绑，")
        print("  且这个偏心量会让 K15/K14 之间产生可测力矩差，可作为差分拟合的独立交叉验证。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
