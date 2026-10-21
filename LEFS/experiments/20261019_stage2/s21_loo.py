#!/usr/bin/env python3
"""S2-1 留一法交叉验证 + J6 可辨识性诊断。

为什么需要这一步
----------------
全量拟合给出 K 之后，唯一能回答「这套 K 能不能拿去做力矩估计」的问题是：
**换一个没参与拟合的姿态，预测得准不准。** 残差 RMS 只说明「拟合得好不好」，
留一法说明「外推能力」，后者才是阶段 2 要交付的东西。

三项检验
--------
1. LOO：每次扣掉一个姿态，用剩下 9 个重解 c6 与 K，再预测被扣掉那个姿态的 ΔI。
2. c6 稳定性：10 次 LOO 各自解出的 c6 有多大散度（这是 c6 的真实不确定度，
   比任何解析协方差都诚实 —— 姿态之间并非相互独立）。
3. J6 可辨识性：把 J6 也放进拟合，看它的斜率是否显著非零。
   结构性预期是「不显著」（质心在转轴上 ⇒ τ6≈0），若显著就说明载荷偏心，
   需要改模型。

用法（本机 Windows，stage2 目录下）
    python s21_loo.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s21_pair_fit as P  # noqa: E402  build_pair / search_c / fit_for_c 全复用
import s21_fit as F       # noqa: E402
import stage2_design as S  # noqa: E402

JOINTS = (1, 2, 3, 4, 5)


def load_arm():
    urdf = S.Urdf(Path(__file__).resolve().parent.parent / "_vendor" / "urdf"
                  / "piper_description.urdf")
    return S.Arm(urdf, end_mass=F.GRIPPER_MASS, end_com=urdf.gripper_com())


def main():
    arm = load_arm()
    pairs = []
    for lab, ed, ld in P.DEFAULT_PAIRS:
        p = P.build_pair(arm, lab, ed, ld, 30.0, 0.3)
        if p is None:
            print(f"[skip] {lab}")
            continue
        pairs.append(p)
    n = len(pairs)
    print(f"[in] {n} 个姿态：{', '.join(p['lab'] for p in pairs)}")

    # --- 0. 全量基准
    c_all, chi_all = P.search_c(pairs, joints=JOINTS)
    out_all = P.fit_for_c(pairs, c_all, joints=JOINTS)
    K_all = {j: 1.0 / out_all[j]["a"] for j in JOINTS}
    print(f"\n[全量] c6 = [{c_all[0]*1000:+.1f}, {c_all[1]*1000:+.1f}, "
          f"{c_all[2]*1000:+.1f}] mm  χ² = {chi_all:.2f}")
    print("       K = " + "  ".join(f"{P.JN[j]} {K_all[j]:.3f}" for j in JOINTS))

    # --- 1+2. 留一法
    print("\n## 留一法（扣掉该姿态重新解 c6 与 K，再预测它）")
    print("| 留出姿态 | 预测残差 RMS (A) | 最大单轴残差 (A) | 该次的 K(J2/J3/J4/J5) | c6 (mm) |")
    print("| --- | --- | --- | --- | --- |")
    c_hist, kerr_hist = [], {j: [] for j in JOINTS}
    worst_overall = 0.0
    for k in range(n):
        sub = pairs[:k] + pairs[k + 1:]
        c, _ = P.search_c(sub, joints=JOINTS)
        out = P.fit_for_c(sub, c, joints=JOINTS)
        row = pairs[k]
        tp = P.tau_pay(row, c)
        res = []
        for j in JOINTS:
            pred = out[j]["a"] * tp[j] + out[j]["delta"]
            res.append(row["dI"][j] - pred)
            kerr_hist[j].append(1.0 / out[j]["a"] - K_all[j])
        rms = math.sqrt(sum(r * r for r in res) / len(res))
        worst_overall = max(worst_overall, max(abs(r) for r in res))
        c_hist.append(c)
        print(f"| {row['lab']:4s} | {rms:.4f} | {max(abs(r) for r in res):+.4f} | "
              + "/".join(f"{1.0/out[j]['a']:.2f}" for j in JOINTS)
              + f" | [{c[0]*1000:+.1f},{c[1]*1000:+.1f},{c[2]*1000:+.1f}] |")

    print(f"\n**留一法预测残差最大的单轴值 = {worst_overall:.4f} A**")

    # --- c6 稳定性
    mean_c = [sum(h[i] for h in c_hist) / n for i in range(3)]
    sd_c = [math.sqrt(sum((h[i] - mean_c[i]) ** 2 for h in c_hist) / (n - 1)) for i in range(3)]
    print(f"\n## c6 稳定性（{n} 次留一）")
    print(f"  全量解 = [{c_all[0]*1000:+.1f}, {c_all[1]*1000:+.1f}, {c_all[2]*1000:+.1f}] mm")
    print(f"  LOO 均值 = [{mean_c[0]*1000:+.1f}, {mean_c[1]*1000:+.1f}, {mean_c[2]*1000:+.1f}] mm")
    print(f"  LOO 标准差 = [{sd_c[0]*1000:.1f}, {sd_c[1]*1000:.1f}, {sd_c[2]*1000:.1f}] mm")

    # --- K 稳定性
    print("\n## K 稳定性（LOO 相对全量解的偏差）")
    print("| 关节 | 全量 K | LOO 最大偏离 | 相对 |")
    print("| --- | --- | --- | --- |")
    for j in JOINTS:
        m = max(abs(v) for v in kerr_hist[j])
        print(f"| {P.JN[j]} | {K_all[j]:.3f} | {m:.3f} | {m/abs(K_all[j])*100:.1f}% |")

    # --- 3. J6 可辨识性（把 J6 放进拟合）
    print("\n## J6 是否真的无信号（把 J6 也放进去解）")
    out6 = P.fit_for_c(pairs, c_all, joints=(1, 2, 3, 4, 5, 6))
    e6 = out6[6]
    K6 = 1.0 / e6["a"]
    sig6 = math.sqrt(e6["cov_a"]) * K6 * K6      # dK = d(1/a) = |da|/a²
    T6 = e6["T"]
    span6 = max(T6) - min(T6)
    y6 = e6["Y"]
    print(f"  τ6 名义跨度 = {span6:.4f} N·m")
    print(f"  ΔI6 实测范围 = {min(y6):+.4f} ~ {max(y6):+.4f} A（峰峰 {max(y6)-min(y6):.4f}）")
    print(f"  拟合 K6 = {K6:+.2f} ± {sig6:.2f} N·m/A   （|K|/σ = {abs(K6)/sig6:.1f}）")
    print(f"  判据：跨度 ≥ 6·|K|·σ_I ？  {span6:.3f} vs "
          f"{6*abs(K6)*0.03:.3f}  →  {'够' if span6 >= 6*abs(K6)*0.03 else '不够（结构性无解）'}")
    print(f"  结论：{'J6 不可标定（与预期一致：质心在转轴上）' if abs(K6)/sig6 < 3 else 'J6 有信号，需复查载荷模型'}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
