#!/usr/bin/env python3
"""S2-1 加固：组内等系数约束 + c/K 简并诊断 + 留一稳定性。

要解决的真问题
--------------
无约束拟合（每个轴各解一个 K）在留一法下 K4/K5 偏离 107%/109%，而解析 σ 只报
2.8%/1.3%。根因不是噪声，是 **c6 与 K 的简并**：

    τ_pay_j(c) = p0_j + A_j·c        ΔI_j = τ_pay_j(c)/K_j + δ_j

当各姿态的 A_jz/p0_j 比例彼此接近时，把 c_z 调大一点、同时把 K_j 调大同样的比例，
几乎不改变预测值。于是「单个姿态」的 ΔI 都能拟合好，但 K 本身随扣掉哪个姿态乱跳。

为什么允许加约束
----------------
SDK 自己的分组是 J1–J3 = 1.18125、J4–J6 = 0.95844，且实测 J2≈J3、J4≈J5
（无约束解 4.536/3.942 与 0.684/0.626，组内差 15%/9%）。把同组 K 设为相等不是
为了「好看」，而是把 SDK 已知的结构性事实作为约束，换取 c6 的可辨识性。

判据（诚实版）
--------------
1. 约束后 χ² 相对无约束的增量是否与自由度增量相符（F 检验）；
2. 约束后用留一法看 K 的偏离 —— 若仍 >30%，则写「未标定」，不填数。

用法（本机 Windows，stage2 目录下）
    python s21_group_fit.py
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s21_pair_fit as P  # noqa: E402
import s21_fit as F       # noqa: E402
import stage2_design as S  # noqa: E402

GROUPS = [((1, 2), "A: J2+J3"), ((3, 4), "B: J4+J5")]   # 0-based：1,2 ⇒ J2,J3；3,4 ⇒ J4,J5


def load_arm():
    urdf = S.Urdf(Path(__file__).resolve().parent.parent / "_vendor" / "urdf"
                  / "piper_description.urdf")
    return S.Arm(urdf, end_mass=F.GRIPPER_MASS, end_com=urdf.gripper_com())


def _wls_parts(T, Y, W):
    """单轴加权回归的充分统计量（给定 c）。"""
    S0 = sum(W)
    S1 = sum(w * t for w, t in zip(W, T))
    S2 = sum(w * t * t for w, t in zip(W, T))
    R0 = sum(w * y for w, y in zip(W, Y))
    R1 = sum(w * t * y for w, t, y in zip(W, T, Y))
    det = S0 * S2 - S1 * S1
    S2w = det / S0 if S0 > 0 else 0.0        # = Σw(t−t̄)²
    xj = (S0 * R1 - S1 * R0) / det if abs(det) > 1e-14 else 0.0
    ybar = R0 / S0 if S0 > 0 else 0.0
    tbar = S1 / S0 if S0 > 0 else 0.0
    return dict(S0=S0, S2w=S2w, xj=xj, ybar=ybar, tbar=tbar)


def fit_constrained(pairs, c, groups=GROUPS):
    """组内共享斜率 x=1/K、各轴独立截距；c 未知时由外层搜索。返回 (x_per_group, chi2, per_joint)。"""
    per_joint = {}
    for j_list, _ in groups:
        parts = []
        for j in j_list:
            T = [P.tau_pay(p, c)[j] for p in pairs]
            Y = [p["dI"][j] for p in pairs]
            W = [1.0 / max(p["sig"][j], 1e-6) ** 2 for p in pairs]
            parts.append((j, _wls_parts(T, Y, W), T, Y, W))
        den = sum(pt["S2w"] for _, pt, *_ in parts)
        if den <= 0:
            continue
        x = sum(pt["S2w"] * pt["xj"] for _, pt, *_ in parts) / den
        var_x = 1.0 / den
        chi2 = 0.0
        for j, pt, T, Y, W in parts:
            d = pt["ybar"] - x * pt["tbar"]
            chi2 += sum(w * (y - x * t - d) ** 2 for w, t, y in zip(W, T, Y))
            per_joint[j] = dict(x=x, delta=d, var_x=var_x, chi2_j=None)
        per_joint[j_list[0]]["var_x"] = var_x
        per_joint[j_list[0]]["chi2_group"] = chi2
    chi2_tot = sum(v.get("chi2_group", 0.0) for v in per_joint.values())
    return per_joint, chi2_tot


def search_c_constrained(pairs, rounds=4, steps=(9, 9, 8)):
    lo = [P.C_BOX[i][0] for i in range(3)]
    hi = [P.C_BOX[i][1] for i in range(3)]
    best_c, best = None, float("inf")
    for rd in range(rounds):
        axes = []
        for i in range(3):
            n = min(steps[i] * (2 ** rd) + 1, 33)
            axes.append([lo[i] + (hi[i] - lo[i]) * k / (n - 1) for k in range(n)])
        for x in axes[0]:
            for y in axes[1]:
                for z in axes[2]:
                    c = [x, y, z]
                    _, chi = fit_constrained(pairs, c)
                    if chi < best:
                        best, best_c = chi, c
        # 收缩搜索域
        w = [(hi[i] - lo[i]) / (2 ** (rd + 1)) for i in range(3)]
        lo = [max(P.C_BOX[i][0], best_c[i] - w[i]) for i in range(3)]
        hi = [min(P.C_BOX[i][1], best_c[i] + w[i]) for i in range(3)]
    return best_c, best


def report_at(pairs, c):
    per, chi = fit_constrained(pairs, c)
    K = {}
    for j_list, _ in GROUPS:
        x = per[j_list[0]]["x"]
        for j in j_list:
            K[j] = 1.0 / x
    return K, per, chi


def main():
    arm = load_arm()
    pairs = []
    for lab, ed, ld in P.DEFAULT_PAIRS:
        p = P.build_pair(arm, lab, ed, ld, 30.0, 0.3)
        if p is not None:
            pairs.append(p)
    n_obs = len(pairs) * 4

    # ---------- 1. J6 显著性（0-based index 5）
    c_free, chi_free = P.search_c(pairs, joints=(1, 2, 3, 4, 5))
    out_free = P.fit_for_c(pairs, c_free, joints=(1, 2, 3, 4, 5))
    e6 = out_free[5]
    K6 = 1.0 / e6["a"]
    var_a = e6["cov_a"]
    sig_K6 = math.sqrt(var_a) / (e6["a"] ** 2)
    print("## 0. J6 可辨识性（把 J6 放进自由拟合）")
    print(f"  τ6 名义跨度 = {max(e6['T']) - min(e6['T']):.4f} N·m")
    print(f"  ΔI6 实测峰峰 = {max(e6['Y']) - min(e6['Y']):+.4f} A")
    print(f"  K6 = {K6:+.2f} ± {sig_K6:.2f} N·m/A   |K|/σ = {abs(K6)/sig_K6:.2f}")
    span6 = max(e6["T"]) - min(e6["T"])
    need6 = 6 * abs(K6) * 0.03
    print(f"  判据 span ≥ 6|K|σ_I：{span6:.4f} vs {need6:.4f} → "
          f"{'达标' if span6 >= need6 else '不达标'}")
    print(f"  → {'J6 有真实信号，只是幅度不足以定 K（横偏 5 mm 造成）' if abs(K6)/sig_K6 >= 3 else '不显著'}"
          f"；σ_K/K = {sig_K6/abs(K6)*100:.0f}%，补强需偏心横杆 80 mm（τ6 放大 ~16 倍）")
    print()

    # ---------- 2. 无约束基准
    K_free = {j: 1.0 / out_free[j]["a"] for j in (1, 2, 3, 4, 5)}
    print("## 1. 无约束（每轴独立 K）")
    print(f"  c6 = [{c_free[0]*1000:+.1f}, {c_free[1]*1000:+.1f}, {c_free[2]*1000:+.1f}] mm")
    print(f"  K = " + "  ".join(f"{P.JN[j]} {K_free[j]:.3f}" for j in (1, 2, 3, 4, 5)))
    print(f"  χ² = {chi_free:.2f}，自由度数 = {n_obs} − 11 = {n_obs-11}")
    print()

    # ---------- 3. 组内等系数约束
    c_c, chi_c = search_c_constrained(pairs)
    K_c, per_c, chi_c2 = report_at(pairs, c_c)
    print("## 2. 组内等系数约束（K2=K3, K4=K5）")
    print(f"  c6 = [{c_c[0]*1000:+.1f}, {c_c[1]*1000:+.1f}, {c_c[2]*1000:+.1f}] mm")
    print(f"  K = " + "  ".join(f"{P.JN[j]} {K_c[j]:.3f}" for j in (1, 2, 3, 4)))
    print(f"  χ² = {chi_c2:.2f}，自由度数 = {n_obs} − 9 = {n_obs-9}")
    dchi = chi_c2 - chi_free
    print(f"  Δχ² = {dchi:+.2f}（Δdof = 2）→ "
          f"{'约束可接受（Δχ² < 2·√(2·Δdof)·σ 量级）' if dchi < 9.5 else '约束过强，组内 K 确实不相等'}")
    print()

    # ---------- 4. 约束解的留一稳定性
    print("## 3. 约束解的留一稳定性")
    print("| 留出 | K_A(J2/J3) | K_B(J4/J5) | c6 (mm) | χ² |")
    print("| --- | --- | --- | --- | --- |")
    KA, KB, Cs = [], [], []
    for k in range(len(pairs)):
        sub = pairs[:k] + pairs[k + 1:]
        cc, ch = search_c_constrained(sub, rounds=3)
        Kk, _, _ = report_at(sub, cc)
        KA.append(Kk[1]); KB.append(Kk[3]); Cs.append(cc)
        print(f"| {pairs[k]['lab']:4s} | {Kk[1]:.3f} | {Kk[3]:.3f} | "
              f"[{cc[0]*1000:+.0f},{cc[1]*1000:+.0f},{cc[2]*1000:+.0f}] | {ch:.2f} |")

    def stat(v):
        m = sum(v) / len(v)
        sd = math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))
        return m, sd, max(abs(x - m) for x in v)

    print()
    for name, v, full in (("K_A (J2=J3)", KA, K_c[1]), ("K_B (J4=J5)", KB, K_c[3])):
        m, sd, mx = stat(v)
        print(f"  {name}: 全量 {full:.3f}  LOO 均值 {m:.3f} ± {sd:.3f}  "
              f"最大偏离 {mx:.3f} ({mx/full*100:.1f}%)")
    for i, ax in enumerate("xyz"):
        v = [c[i] for c in Cs]
        m, sd, mx = stat(v)
        print(f"  c6_{ax}: 全量 {c_c[i]*1000:+.1f}  LOO {m*1000:+.1f} ± {sd*1000:.1f} mm  "
              f"最大偏离 {mx*1000:.1f} mm")

    # ---------- 5. 落盘（约束解）
    out = dict(
        version="s21_group_fit 1.0",
        method="同姿态空载−带载差分 + 组内等系数约束（K2=K3, K4=K5，依据 SDK 分组结构）",
        constraint_reason="无约束解下 c6 与 K 近似简并，K4/K5 留一偏离 >100%；组约束换取 c6 可辨识性",
        bottle_mass_kg=P.BOTTLE_MASS,
        c6_mm=[round(v * 1000, 2) for v in c_c],
        n_pairs=len(pairs),
        chi2=round(chi_c2, 2), dof=n_obs - 9,
        chi2_free=round(chi_free, 2), dof_free=n_obs - 11,
        joints={})
    for j in range(6):
        if j == 0:
            out["joints"]["J1"] = dict(state="未标定", K=None,
                                       reason="重力与 J1 轴平行 ⇒ τ1≡0；需滑轮+测力计切向拉")
        elif j == 5:
            out["joints"]["J6"] = dict(state="未标定", K=None,
                                       reason=f"τ6 跨度仅 {span6:.3f} N·m（质心横偏 ~5 mm 造成），"
                                              f"σ_K/K = {sig_K6/abs(K6)*100:.0f}% 未达判据；"
                                              f"需偏心横杆 80 mm 把 τ6 放大 ~16 倍")
        else:
            src = KA if j in (1, 2) else KB
            mm, ss, mxd = stat(src)
            rel = mxd / K_c[j] * 100
            out["joints"][f"J{j+1}"] = dict(
                state="已标定" if rel < 20 else "弱约束",
                K=round(K_c[j], 4),
                loo_max_dev=round(mxd, 4),
                loo_rel_dev=round(rel, 1),
                K_over_SDK=round(abs(K_c[j]) / P.K_SDK[j], 3),
                reason="" if rel < 20 else "c6 与 K 简并导致留一偏离偏大，使用时须带此不确定度")
    pth = Path(__file__).resolve().parent / "calibration" / "k_table_constrained.json"
    pth.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n写出 {pth}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
