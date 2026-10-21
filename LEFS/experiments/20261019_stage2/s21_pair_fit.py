#!/usr/bin/env python3
"""s21_pair_fit.py —— S2-1 主口径：**同姿态「空载 − 带载」差分**定 K

为什么不用「URDF 重力矩 vs 电流」直接回归
----------------------------------------
2026-10-20 12:2x 拿 6 个姿态试过一次，结论很硬：**不能用 URDF 的连杆质量当力矩真值**。
证据：J2 在 K3（J3=−110°）处，URDF 说保持力矩 1.71 N·m，实测电流 1.79 A 却是全部
姿态里最高的；同一条 J2 在 LIFT（4.43 N·m）只有 0.65 A。跨姿态的电流**与 URDF 力矩
反着走**，残差 4.6 N·m（3.7 倍）。这不是噪声，是 URDF 的连杆质量/质心与这台机器不符。

差分法的好处
------------
同一个姿态、同一台臂、同样的到达方向，**只把 0.525 kg 水瓶拿上/拿下**：

    ΔI_j = I_empty,j − I_loaded,j = τ_pay,j(c) / K_j + δ_j

  * `τ_pay(c)` 只依赖**运动学 + 已知质量 0.525 kg + 水瓶质心 c**，
    与 URDF 的连杆质量**完全无关**（这是它能用的根本原因）；
  * 臂自重、固件重力前馈、连杆质量误差在差分里**整体抵消**；
  * 摩擦/零点差只留下一个小偏置 δ_j（4 个参数），由拟合吸收。

未知数：c(3) + K(4) + δ(4) = 11；数据：N 姿态 × 4 轴。N=11 时 44 个方程，
且 c 由「各关节对载荷的敏感方向不同」定出来。

用法（本机 Windows，LEFS/experiments/20261019_stage2）
-----------------------------------------------------
  python s21_pair_fit.py                       # 默认清单
  python s21_pair_fit.py --last-s 30 --out calibration
  python s21_pair_fit.py --selftest
"""

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage2_design as S
import s21_design as G
import s21_fit as F

EP = F.EP_DIR
BOTTLE_MASS = F.BOTTLE_MASS
K_SDK = F.K_SDK
JN = F.JOINT_NAME

# 同一姿态下的「空载 episode / 带载 episode」配对
#
# 2026-10-20 12:5x 对齐到新姿态集：
#   * 去掉 K5（τ_pay 四轴全被覆盖、贡献 +0.000，且路径按真实速率算必死）
#   * 加 K9R（K2→K12 的安全中转点，顺便作 K9 的重复性对照）
#   * P1 直接复用 S2-0 的历史数据（s20_p1 空载 / s20_p1_load 带载）：
#     它与 K15/K14 同位形，τ_pay 逐轴相同，所以**空载轮不必重采 P1**。
DEFAULT_PAIRS = [
    ("LIFT", "s21e_LIFT", "s21_LIFT"),
    ("K3",   "s21e_K3",   "s21_K3"),
    ("K10",  "s21e_K10",  "s21_K10"),
    ("K15",  "s21e_K15",  "s21_K15"),
    ("K14",  "s21e_K14",  "s21_K14"),
    ("K9",   "s21e_K9",   "s21_K9"),
    ("K8",   "s21e_K8",   "s21_K8"),
    ("K2",   "s21e_K2",   "s21_K2"),
    ("K9R",  "s21e_K9R",  "s21_K9R"),
    ("K12",  "s21e_K12",  "s21_K12"),
    ("P1",   "s20_p1",    "s20_p1_load"),
]

C_BOX = ((-0.13, 0.13), (-0.13, 0.13), (0.00, 0.28))


def build_pair(arm, lab, e_dir, l_dir, last_s, stable_deg):
    pe = EP / e_dir / "samples.jsonl"
    pl = EP / l_dir / "samples.jsonl"
    if not pe.exists() or not pl.exists():
        return None
    e = F.build_row(arm, pe, last_s, stable_deg, False)
    l = F.build_row(arm, pl, last_s, stable_deg, True)
    return dict(lab=lab, q=l["q"], p0=l["p0"], A=l["A"],
                dI=[e["cur"][j] - l["cur"][j] for j in range(6)],
                sig=[math.hypot(e["sig"][j], l["sig"][j]) for j in range(6)],
                qe=e["q"], ql=l["q"])


def tau_pay(row, c):
    return [row["p0"][j] + sum(row["A"][j][i] * c[i] for i in range(3)) for j in range(6)]


def wls2(T, Y, W):
    n = len(T)
    S0 = sum(W); S1 = sum(w * t for w, t in zip(W, T))
    S2 = sum(w * t * t for w, t in zip(W, T))
    R0 = sum(w * y for w, y in zip(W, Y)); R1 = sum(w * t * y for w, t, y in zip(W, T, Y))
    det = S0 * S2 - S1 * S1
    if abs(det) < 1e-12:
        return None
    a = (S0 * R1 - S1 * R0) / det
    d = (S2 * R0 - S1 * R1) / det
    chi2 = sum(w * (y - a * t - d) ** 2 for w, t, y in zip(W, T, Y))
    return a, d, chi2, S0 / det, S2 / det


def fit_for_c(pairs, c, joints=(1, 2, 3, 4, 5)):
    out = {}
    for j in joints:
        T = [tau_pay(p, c)[j] for p in pairs]
        Y = [p["dI"][j] for p in pairs]
        W = [1.0 / max(p["sig"][j], 1e-6) ** 2 for p in pairs]
        r = wls2(T, Y, W)
        if r is None:
            out[j] = None
            continue
        a, d, chi2, ca, cd = r
        out[j] = dict(a=a, delta=d, chi2=chi2, cov_a=ca, cov_d=cd, T=T, Y=Y, W=W, n=len(pairs))
    return out


def total_chi2(pairs, c, joints=(1, 2, 3, 4, 5)):
    r = fit_for_c(pairs, c, joints)
    return sum(v["chi2"] for v in r.values() if v), r


def search_c(pairs, joints=(1, 2, 3, 4, 5), rounds=4, steps=(9, 9, 8)):
    lo = [C_BOX[i][0] for i in range(3)]
    hi = [C_BOX[i][1] for i in range(3)]
    best_c, best = None, float("inf")
    for rd in range(rounds):
        axes = []
        for i in range(3):
            n = min(steps[i] * (2 ** rd) + 1, 33)
            axes.append([lo[i] + (hi[i] - lo[i]) * k / (n - 1) for k in range(n)])
        for x in axes[0]:
            for y in axes[1]:
                for z in axes[2]:
                    chi, _ = total_chi2(pairs, [x, y, z], joints)
                    if chi < best:
                        best, best_c = chi, [x, y, z]
        span = [(hi[i] - lo[i]) / (2 ** (rd + 1)) for i in range(3)]
        lo = [max(C_BOX[i][0], best_c[i] - span[i]) for i in range(3)]
        hi = [min(C_BOX[i][1], best_c[i] + span[i]) for i in range(3)]
    return best_c, best


def report(pairs, out, last_s):
    c = out["c"]
    print("## 1. 水瓶质心（link6 系，拟合值）")
    print(f"  c6 = [{c[0]*1000:+.1f}, {c[1]*1000:+.1f}, {c[2]*1000:+.1f}] mm"
          f"   |c6| = {math.sqrt(sum(v*v for v in c))*1000:.1f} mm")
    print(f"  总 χ² = {out['chi']:.1f}（{len(pairs)} 个姿态 × 4 轴）")
    print()
    print("## 2. 逐轴 K（ΔI = τ_pay/K + δ）")
    print("| 关节 | K (N·m/A) | ±1σ | K/K_SDK | δ (A) | τ_pay 跨度 | 残差 RMS | 状态 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for j in range(1, 5):
        e = out["joints"][j]
        dof = max(e["n"] - 2, 1)
        scale = math.sqrt(e["chi2"] / dof)
        sa = math.sqrt(e["cov_a"]) * scale
        K = 1.0 / e["a"] if abs(e["a"]) > 1e-12 else float("inf")
        sigK = abs(sa / (e["a"] ** 2))
        wbar = sum(e["W"]) / len(e["W"])
        rms = math.sqrt(e["chi2"] / dof / wbar)
        span = max(e["T"]) - min(e["T"])
        ok = span >= 6 * abs(K) * math.sqrt(1.0 / wbar) and abs(sigK / K) < 0.30
        e.update(K=K, sigma_K=sigK, span=span, rms=rms, ok=ok, chi2_red=e["chi2"] / dof)
        print(f"| {JN[j]} | {K:+.3f} | {sigK:.3f} | {abs(K)/K_SDK[j]:.2f}× | "
              f"{e['delta']:+.4f} | {span:.2f} | {rms:.4f} | "
              f"{'已标定' if ok else '未标定（跨度/精度不足）'} |")
    print()
    print("## 3. 逐姿态残差（A）：实测 ΔI − 拟合 ΔI")
    print("| 姿态 | J2 | J3 | J4 | J5 |")
    print("| --- | --- | --- | --- | --- |")
    for k, p in enumerate(pairs):
        cells = []
        for j in range(1, 5):
            e = out["joints"][j]
            cells.append(f"{p['dI'][j] - (e['a']*e['T'][k] + e['delta']):+.4f}")
        print(f"| {p['lab']} | " + " | ".join(cells) + " |")
    print()
    print("## 4. 逐姿态 ΔI 与预测 Δτ")
    print("| 姿态 | ΔτJ2 | ΔτJ3 | ΔτJ4 | ΔτJ5 | ΔI2 | ΔI3 | ΔI4 | ΔI5 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for p in pairs:
        T = tau_pay(p, c)
        print(f"| {p['lab']} | " + " ".join(f"{T[j]:+.2f}" for j in (1, 2, 3, 4)) + " | "
              + " ".join(f"{p['dI'][j]:+.3f}" for j in (1, 2, 3, 4)) + " |")


def write_table(out, path, last_s, npairs):
    kt = dict(version="s21_pair_fit 1.0",
              method="同姿态空载−带载差分；τ_pay 只依赖运动学与已知质量，与 URDF 连杆质量无关",
              bottle_mass_kg=BOTTLE_MASS,
              c6_mm=[round(v * 1000, 2) for v in out["c"]],
              n_pairs=npairs, last_s=last_s, joints={})
    for i in range(6):
        if i in (0, 5):
            kt["joints"][JN[i]] = dict(
                state="未标定", K=None,
                reason=("重力与 J1 轴平行 ⇒ τ1≡0，静态法无解；需滑轮/弹簧秤切向拉" if i == 0
                        else "水瓶质心几乎在工具轴（=J6 转轴）上 ⇒ τ6≈0；需偏心横杆 80 mm"))
            continue
        e = out["joints"][i]
        kt["joints"][JN[i]] = dict(
            state="已标定" if e["ok"] else "未标定",
            K=round(e["K"], 4), sigma_K=round(e["sigma_K"], 4),
            delta_A=round(e["delta"], 5), tau_span=round(e["span"], 3),
            resid_rms_A=round(e["rms"], 5), chi2_red=round(e["chi2_red"], 2),
            K_over_SDK=round(abs(e["K"]) / K_SDK[i], 3),
            reason="" if e["ok"] else "τ 跨度或 K 的相对不确定度不满足判据")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(kt, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n写出 {path}")


def selftest(arm):
    n_ok = n_bad = 0

    def chk(name, cond, detail=""):
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print(f"  ✓ {name}")
        else:
            n_bad += 1
            print(f"  ✗ {name}  {detail}")

    truth_c = [0.02, -0.03, 0.095]
    truth_K = [0, 4.0, 3.6, -1.2, 2.8, 0]
    truth_d = [0, 0.01, -0.02, 0.005, 0.01, 0]
    qs = [G.deg(*v) for v in
          [(0, 45, -60, 0, 45, 0), (0, 30, -110, 0, -40, 0), (0, 75, -30, 0, -40, 0),
           (0, 60, -50, 60, 40, 0), (0, 60, -50, 90, 40, 90), (0, 120, -90, 90, 40, 0),
           (-45, 30, -80, 0, 50, 0), (0, 45, -60, 0, 45, 30)]]
    pairs = []
    for q in qs:
        r = F.build_row_fake(arm, q, truth_c, [1e9] * 6, [0] * 6)   # 只要 p0/A
        T = [r["p0"][j] + sum(r["A"][j][i] * truth_c[i] for i in range(3)) for j in range(6)]
        pairs.append(dict(lab="x", q=q, p0=r["p0"], A=r["A"],
                          dI=[T[j] / truth_K[j] + truth_d[j] if truth_K[j] else 0.0
                              for j in range(6)],
                          sig=[0.01] * 6))
    c, chi = search_c(pairs, rounds=3)
    out = dict(c=c, chi=chi, joints=fit_for_c(pairs, c))
    chk("A 合成 c 反解（±4 mm）", max(abs(c[i] - truth_c[i]) for i in range(3)) < 0.004,
        str([round(v * 1000, 1) for v in c]))
    ok = True
    for j in range(1, 5):
        K = 1 / out["joints"][j]["a"]
        if abs(K - truth_K[j]) / abs(truth_K[j]) > 0.02:
            ok = False
            print(f"     J{j+1} K={K:.3f} 真 {truth_K[j]:.3f}")
    chk("A 合成 K 四轴 < 2%", ok)
    chk("A 合成 δ 也回来",
        all(abs(out["joints"][j]["delta"] - truth_d[j]) < 0.01 for j in range(1, 5)))

    # 关键性质：差分对「臂自重误差」完全不敏感（故意把 c 猜错一点点看残差怎么变）
    chi_bad, _ = total_chi2(pairs, [0.0, 0.0, 0.15])
    chk("B 固定错误的 c 时 χ² 显著变大（说明 c 可辨识）", chi_bad > 10 * chi,
        f"{chi_bad:.1f} vs {chi:.1f}")

    print(f"\n自测：{n_ok} 过 / {n_bad} 败")
    return 0 if n_bad == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="S2-1 同姿态空载−带载差分定 K")
    ap.add_argument("--last-s", type=float, default=30.0)
    ap.add_argument("--stable-deg", type=float, default=0.3)
    ap.add_argument("--out", default="calibration/k_table.json")
    ap.add_argument("--report", default="calibration/fit_report.md",
                    help="把拟合报告同时写成 markdown（留档用）")
    ap.add_argument("--urdf", default=str(Path(__file__).resolve().parent.parent /
                                         "_vendor" / "urdf" / "piper_description.urdf"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)

    urdf = S.Urdf(Path(a.urdf))
    arm = S.Arm(urdf, end_mass=F.GRIPPER_MASS, end_com=urdf.gripper_com())
    if a.selftest:
        return selftest(arm)

    pairs = []
    for lab, ed, ld in DEFAULT_PAIRS:
        p = build_pair(arm, lab, ed, ld, a.last_s, a.stable_deg)
        if p is None:
            print(f"[skip] {lab}: 缺空载或带载 episode")
            continue
        pairs.append(p)
        print(f"[pair] {lab:5s} q={','.join(f'{math.degrees(v):+.1f}' for v in p['q'])}  "
              f"ΔI=" + " ".join(f"{p['dI'][j]:+.3f}" for j in range(6)))
    if len(pairs) < 4:
        print(f"[ABORT] 只有 {len(pairs)} 对，不够拟合", file=sys.stderr)
        return 2

    c, chi = search_c(pairs)
    out = dict(c=c, chi=chi, joints=fit_for_c(pairs, c))
    print()

    # report 全是 print，先抓到缓冲区：既要上屏，也要落成 md 留档。
    import contextlib
    import io
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        report(pairs, out, a.last_s)
    txt = buf.getvalue()
    print(txt)

    if a.report:
        rp = Path(a.report)
        rp.parent.mkdir(parents=True, exist_ok=True)
        head = (
            "# S2-1 力矩常数 K 标定报告\n\n"
            f"- 方法：**同姿态「空载 − 带载」差分**，`ΔI_j = τ_pay,j(c6)/K_j + δ_j`\n"
            f"- 载荷：水瓶 {BOTTLE_MASS:.3f} kg（连瓶带水带胶带，刚性固定在夹爪上，随腕部倾斜）\n"
            f"- 姿态数：{len(pairs)} 对；静止段取最后 {a.last_s:g} s\n"
            "- 为什么这个口径可信：τ_pay **只依赖运动学 + 已知质量 + 未知质心 c6**，"
            "与 URDF 连杆质量无关；臂自重、固件重力前馈、连杆质量误差在差分里整体抵消\n"
            "- 判据：`τ 跨度 ≥ 6·|K|·σ_I` 且 `σ_K/|K| < 30%`，否则写「未标定」，不填 SDK 系数\n\n"
            "---\n\n"
        )
        rp.write_text(head + txt, encoding="utf-8")
        print(f"写出 {rp}")

    write_table(out, a.out, a.last_s, len(pairs))
    return 0


if __name__ == "__main__":
    sys.exit(main())
