#!/usr/bin/env python3
"""阶段 5（L2）称重分析：τ_ext 是否忠实反映已知载荷。

链路
----
    τ_ext = τ_m − τ̂_free
    τ_m  = K · mean(I)            （静止窗口平均，噪声 ↓√N）
    τ̂_free = L1 模型(q, dq, q_cmd−q)   （残差版：网络输出 + 线性重力基线）
    τ_pay(Φ) = tau_point_force(q, link6, Φ, [0,0,−mg])   （URDF，对 Φ 线性）

判据（S5_PLAN.md §5.3）
-----------------------
1. 空载段 τ_ext 均值应接近 0（|·| < 0.03 N·m）：模型无系统偏置；
2. τ_ext 对质量线性，R² > 0.95；
3. 拟合出的质心 Φ 与阶段 2 的 (−7.2, +7.2, +80.9) mm 同量级；
4. 拟合残差 RMS < 0.05 N·m。

符号说明
--------
阶段 2 的差分式是 `ΔI = τ_pay/K + δ`，而 τ_ext = K·(I_loaded − I_free) = −τ_pay。
所以预期系数 `s ≈ −1`。这里**不预设符号**，让回归自己定，并报告它离 −1 有多远。

用法（本机，LEFS/experiments/20261020_stage4）：
    python s5_load_fit.py --selftest     # 合成数据验证拟合链路
    python s5_load_fit.py                # 真实数据（episodes/ 下 24 段）
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "20261019_stage3"))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))
from factr2_next.training.models import build_model  # noqa: E402
import s3_design as D  # noqa: E402
import stage2_design as S  # noqa: E402
from s4_build import K, K_STATE  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
GOOD_AXES = [1, 2, 3, 4]          # J2~J5，K 实测/弱约束
STEADY_S = 10.0
FS = 50.0

POSES = {
    # 全部 J6=+90：唯一能让「指闭合轴」在世界系里水平的 J6
    # U 族：掌心朝上（ax_z=+1.0000），重力 ∥ 工具轴
    "U1": (0, 30, -50, 0, -65, 50.6),
    "U2": (0, 45, -60, 0, -70, 50.6),
    "U3": (0, 60, -70, 0, -70, 50.6),
    # V 族：掌心竖直（ax_z=0.000），重力 ⊥ 工具轴
    "A": (0, 30, -50, 0, 25, 50.6),
    "B": (0, 45, -60, 0, 20, 50.6),
    "C": (0, 60, -70, 0, 15, 50.6),
}
# 两族各自拟合 Φ：工具轴与重力的夹角不同，激励到的 Φ 分量不同。
# 两族拟合出的 |Φ| 若接近，才说明称重成立（否则质量误差被 |Φ| 吸收）。
GROUPS_OF = {"U": ["U1", "U2", "U3"], "V": ["A", "B", "C"]}
POSE2GROUP = {p: g for g, ps in GROUPS_OF.items() for p in ps}
# 质量档：用户现有砝码 100/300/500 g（没有 1 kg，2026-10-20 17:42 确认）
MASS_TAGS = {"M0": 0.0, "M1": 0.1, "M2": 0.3, "M3": 0.5}
G = 9.81


def read_steady(ep_dir, steady_s=STEADY_S):
    """读一段的静止窗口：返回 q/dq/(q_cmd−q) 的逐帧数组（原始单位）。"""
    p = Path(ep_dir) / "samples.jsonl"
    rows = [json.loads(l) for l in open(p, encoding="utf-8")]
    rows = [r for r in rows
            if r.get("q") and r.get("dq") and r.get("cur") and None not in r["q"]]
    first = next((i for i, r in enumerate(rows) if r.get("q_cmd") is not None), 0)
    rows = rows[first:]
    n = int(steady_s * FS)
    rows = rows[-n:] if len(rows) > n else rows
    q = np.array([r["q"] for r in rows], float)
    dq = np.array([r["dq"] for r in rows], float)
    cmd = np.array([r["q_cmd"] for r in rows], float)
    cur = np.array([r["cur"] for r in rows], float)
    return q, dq, cmd, cur


def tau_free_mean(model, norm, lin_W, q, dq, dcmd, H):
    """对静止段取多个窗口预测并平均，得到 τ̂_free（N·m，6 维）。"""
    x = np.concatenate([q, dq, dcmd], axis=1)
    xn = (x - norm["x_mean"]) / norm["x_std"]
    idx = list(range(H - 1, len(x), 5))
    if not idx:
        idx = [len(x) - 1]
    wins = np.stack([xn[i - H + 1:i + 1] for i in idx]).astype(np.float32)
    with torch.no_grad():
        p = model(torch.from_numpy(wins)).numpy()
    resid = p * norm["y_std"] + norm["y_mean"]
    q_last = q[idx]
    out = []
    for k, i in enumerate(idx):
        A = np.hstack([q_last[k], 1.0])
        out.append(resid[k] + (np.asarray(lin_W, float).T @ A))
    return np.mean(out, axis=0)


def basis_tau_pay(arm, q, phi):
    """τ_pay 对 Φ 的基函数：返回 (6, 4)，第 0 列是 Φ=0 的常数项（恒为 0）。"""
    cols = []
    for k in range(4):
        p = np.zeros(3) if k == 0 else np.eye(3)[k - 1]
        cols.append(np.asarray(arm.tau_point_force(list(q), S.END_LINK, list(p * phi), [0, 0, -1.0 * G])))
    return np.stack(cols, axis=1)      # (6,4)，单位：N·m per (kg·Φ)


def _basis(arm, q, m, only):
    """返回 (p0, A3)：τ_pay = p0 + A3·Φ。

    **p0 不能省**（阶段 2 的 R47 同一个坑）：`tau_point_force(q, link6, local, F)` 的
    `local` 是**相对法兰的偏移**，`local=0` 时力作用在法兰原点，对 J2/J3 仍然有力矩。
    漏掉它，拟合会用 Φ 去补偿这个偏置，得到荒谬的质心（2026-10-20 17:2x 踩到，
    自测 RMS 从 0.008 恶化到 0.455）。
    """
    F = [0, 0, -m * G]
    p0 = np.asarray(arm.tau_point_force(list(q), S.END_LINK, [0.0, 0.0, 0.0], F))[only]
    cols = []
    for k in range(3):
        e = np.eye(3)[k]
        cols.append(np.asarray(arm.tau_point_force(list(q), S.END_LINK, list(e), F))[only])
    A3 = np.stack(cols, axis=1) - p0[:, None]      # 扣掉 p0，得到纯 Φ 项
    return p0, A3


def fit_phi(rows, arm, only=GOOD_AXES, fix_sign=-1.0):
    """固定符号系数 s，只拟合 Φ。

    为什么必须固定 s
    ----------------
    自测发现：把 s 也当未知量一起拟合，会得到 s=0.914、Φ=89.6 mm（真值 s=−1、Φ=81 mm），
    **乘积 s·Φ 不变** —— 结构性简并，和阶段 2 的「K 与 c6 简并」同型。
    只用 τ_ext 永远分不开「载荷重了 10%」和「质心远了 10%」。

    固定 s = −1 的依据：阶段 2 的差分式 `ΔI = τ_pay/K + δ`（K>0），
    而 τ_ext = K(I_loaded − I_free) = −τ_pay。
    """
    A_list, y_list = [], []
    for r in rows:
        if r["m"] == 0.0:
            continue
        qr = D.G.deg(*POSES[r["pose"]])
        p0, A3 = _basis(arm, qr, r["m"], only)
        A_list.append(A3)
        y_list.append(fix_sign * np.asarray(r["ext"])[only] - p0)
    if not A_list:
        return dict(phi_mm=[float("nan")] * 3, rms_nm=float("nan"), fix_sign=fix_sign,
                    note="无带载段，跳过 Φ 拟合")
    A = np.vstack(A_list)
    y = np.concatenate(y_list)
    phi, *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ phi - y) ** 2)))
    return dict(phi_mm=(phi * 1000).tolist(), rms_nm=rms, fix_sign=fix_sign)


def fit_phi_free(rows, arm, only=GOOD_AXES):
    """自由拟合 s 与 Φ（τ_ext = s·p0 + A3·c，c = s·Φ），只用来说明简并。"""
    A_list, y_list = [], []
    for r in rows:
        if r["m"] == 0.0:
            continue
        qr = D.G.deg(*POSES[r["pose"]])
        p0, A3 = _basis(arm, qr, r["m"], only)
        A_list.append(np.hstack([p0[:, None], A3]))
        y_list.append(np.asarray(r["ext"])[only])
    if not A_list:
        return dict(s=float("nan"), phi_mm=[float("nan")] * 3, rms_nm=float("nan"))
    A = np.vstack(A_list)
    y = np.concatenate(y_list)
    coeff, *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ coeff - y) ** 2)))
    s = float(coeff[0])
    phi = coeff[1:] / s if abs(s) > 1e-9 else np.full(3, np.nan)
    return dict(s=s, phi_mm=(phi * 1000).tolist(), rms_nm=rms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    ap.add_argument("--dir", default=str(HERE / "episodes"))
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--out", default=str(HERE / "runs" / "l2_load.json"))
    args = ap.parse_args()

    urdf, arm = D.load()

    if args.selftest:
        print("## 自测：用合成 τ_ext 验证拟合能还原 Φ 与符号")
        true_phi = np.array([-0.007, 0.007, 0.081])
        rows = []
        for tag, m in MASS_TAGS.items():
            if m == 0.0:
                continue
            for pose, qd in POSES.items():
                qr = D.G.deg(*qd)
                tp = np.asarray(arm.tau_point_force(list(qr), S.END_LINK,
                                                    list(true_phi), [0, 0, -m * G]))
                ext = -tp + np.random.RandomState(hash((tag, pose)) % 2**31).normal(
                    0, 0.01, 6)
                rows.append(dict(name=f"{tag}_{pose}", m=m, pose=pose, ext=ext))
        fit = fit_phi(rows, arm)
        free = fit_phi_free(rows, arm)
        print(f"  真值 Φ = {(true_phi*1000).round(1).tolist()} mm")
        print(f"  固定 s=−1 拟合 → Φ = {np.round(fit['phi_mm'],1).tolist()} mm，"
              f"RMS = {fit['rms_nm']:.4f}")
        print(f"  （对照）自由拟合 s = {free['s']:+.3f}，Φ = "
              f"{np.round(free['phi_mm'],1).tolist()} mm —— 与上面等价，说明简并")
        err = np.abs(np.array(fit["phi_mm"]) - true_phi * 1000).max()
        ok = err < 5.0 and fit["rms_nm"] < 0.05
        print(f"  最大 Φ 误差 {err:.1f} mm → {'通过' if ok else '未通过'}")
        return

    # ---------------- 真实数据 ----------------
    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}
    H = int(ck["history"])
    lin_W = ck.get("lin_W", None)
    model = build_model(ck["model"], ck["input_size"], ck["output_size"], H)
    model.load_state_dict(ck["model_state_dict"])
    model.eval()

    root = Path(args.dir)
    rows = []
    missing = []
    for tag, m in MASS_TAGS.items():
        for pose in POSES:
            name = f"{tag}_{pose}"
            d = root / name
            if not (d / "samples.jsonl").exists():
                missing.append(name)
                continue
            q, dq, cmd, cur = read_steady(d)
            tau_m = K * cur.mean(axis=0)
            tf = tau_free_mean(model, norm, lin_W, q, dq, cmd - q, H)
            ext = tau_m - tf
            rows.append(dict(name=name, m=m, pose=pose, ext=ext.tolist(),
                             tau_m=tau_m.tolist(), tau_free=tf.tolist(), n=len(q)))
    if missing:
        print(f"[缺段] {len(missing)} 个：{missing[:6]}{' …' if len(missing) > 6 else ''}")
    if not rows:
        print("没有任何可用段。先采集 24 段再跑。")
        return

    print(f"## 逐段 τ_ext（N·m），{len(rows)} 段")
    print("| 段 | m | J1 | J2 | J3 | J4 | J5 | J6 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in rows:
        e = r["ext"]
        print(f"| {r['name']} | {r['m']:g} | " + " | ".join(f"{v:+.4f}" for v in e) + " |")

    fits = {}
    for grp, plist in GROUPS_OF.items():
        sub_all = [r for r in rows if r["pose"] in plist]
        if not sub_all:
            continue
        tag = "竖直夹持（工具轴朝上）" if grp == "V" else "横向夹持（工具轴水平）"
        print()
        print(f"# ===== {grp} 组：{tag} =====")

        # ① 空载基线
        zero = [r for r in sub_all if r["m"] == 0.0]
        if zero:
            z = np.array([r["ext"] for r in zero])
            print("## ① 空载基线（应接近 0）")
            for j in GOOD_AXES:
                print(f"  {JN[j]}  mean {z[:,j].mean():+.4f}  |max| "
                      f"{np.abs(z[:,j]).max():.4f} N·m")
            worst = np.abs(z[:, GOOD_AXES]).max()
            print(f"  最大 |τ_ext| = {worst:.4f} → "
                  f"{'通过' if worst < 0.05 else '超过 0.05，用同姿态差分消掉'}")

        # ② 线性度
        print()
        print("## ② τ_ext 对质量的线性度（每姿态每轴）")
        print("| 姿态 | 轴 | 斜率 (N·m/kg) | R² |")
        print("| --- | --- | --- | --- |")
        r2s = []
        for pose in plist:
            sub = [r for r in sub_all if r["pose"] == pose]
            ms = np.array([r["m"] for r in sub])
            if len(set(ms.tolist())) < 2:
                continue
            for j in GOOD_AXES:
                ys = np.array([r["ext"][j] for r in sub])
                A = np.vstack([ms, np.ones_like(ms)]).T
                coef, *_ = np.linalg.lstsq(A, ys, rcond=None)
                pred = A @ coef
                ss_res = ((ys - pred) ** 2).sum()
                ss_tot = ((ys - ys.mean()) ** 2).sum()
                r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
                r2s.append(r2)
                print(f"| {pose} | {JN[j]} | {coef[0]:+.3f} | {r2:.3f} |")
        if r2s:
            print(f"  中位 R² = {np.nanmedian(r2s):.3f} → "
                  f"{'通过' if np.nanmedian(r2s) > 0.95 else '未达 0.95'}")

        # ③ 拟合质心
        print()
        print("## ③ 拟合质心 Φ（固定 s=−1，联合最小二乘，J2~J5）")
        fit = fit_phi(sub_all, arm)
        free = fit_phi_free(sub_all, arm)
        print(f"  Φ = {np.round(fit['phi_mm'],1).tolist()} mm  "
              f"|Φ| = {np.linalg.norm(fit['phi_mm']):.1f} mm")
        print(f"  对照：自由拟合 s = {free['s']:+.3f}、Φ = "
              f"{np.round(free['phi_mm'],1).tolist()} —— s·Φ 与上表相同，说明简并")
        print(f"  拟合残差 RMS = {fit['rms_nm']:.4f} N·m → "
              f"{'通过' if fit['rms_nm'] < 0.05 else '未达 0.05'}")
        fits[grp] = fit

    if len(fits) == 2:
        print()
        print("# ===== 两组交叉验证 =====")
        print("同一物体两种夹持方位，Φ 方向不同但**模长应接近**（都等于质心到法兰的距离）：")
        for grp, f in fits.items():
            print(f"  {grp}: |Φ| = {np.linalg.norm(f['phi_mm']):.1f} mm  Φ = "
                  f"{np.round(f['phi_mm'],1).tolist()}")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(rows=rows, fits=fits, missing=missing), open(args.out, "w"),
              ensure_ascii=False, indent=1)
    print(f"\n写入 {args.out}")


if __name__ == "__main__":
    main()
