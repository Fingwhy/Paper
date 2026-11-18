#!/usr/bin/env python3
"""阶段 5（L2）称重验收：Φ 辨识 + 质量盲测 + 最小可分辨质量。

原理
----
每个姿态 q、载荷质量 m、质心（link6 系）Φ 下，载荷对关节的力矩是

    τ_pay(q, m, Φ) = m · ( p0(q) + A3(q)·Φ )

其中 p0 是**力作用在法兰原点**时的力矩（不能省，见 R47/R57），
A3 是 Φ 三个分量各自的灵敏度（`_basis` 用 m=1 算出来）。
空载/带载同姿态差分把 L1 模型的系统偏置消掉后：

    τ_ext = K·(I_loaded − I_empty) = −τ_pay

所以 (a) 已知 m 与 τ_ext 可反解 Φ；(b) 已知 Φ 与 τ_ext 可反解 m。

为什么要两个方位（V 朝前 / U 掌心上）
--------------------------------------
τ = Φ × F 对「沿 F 平移 Φ」不变 ⇒ 若所有姿态里 F 在 link6 系方向相同，
Φ 有一个不可辨识方向（实测 V 族单独拟合时 A3 第三个奇异值 = 0，
零空间方向 (−0.635, 0.773, 0) 正好是重力方向）。U 族把工具轴转成竖直后
F 在 link6 系变成 (0,0,−1)，与 V 族正交 ⇒ 合并后满秩。

用法（本机，有 torch 的 venv）
    python s5_weigh.py
    python s5_weigh.py --holdout 0.5     # 指定盲测质量
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "20261019_stage3"))

import s3_design as D                    # noqa: E402
import stage2_design as S                # noqa: E402
from s5_load_fit import (POSES, GROUPS_OF, _basis, read_steady,  # noqa: E402
                         tau_free_mean)
from s4_build import K                   # noqa: E402
import torch                             # noqa: E402
from factr2_next.training.models import build_model   # noqa: E402

G = 9.81
GOOD_AXES = [1, 2, 3, 4]        # J2~J5（K 实测/弱约束）
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
MASSEYS = {"M0": 0.0, "M1": 0.1, "M2": 0.3, "M3": 0.5}


def load_rows(ckpt, root):
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}
    H = int(ck["history"])
    lin_W = ck.get("lin_W", None)
    model = build_model(ck["model"], ck["input_size"], ck["output_size"], H)
    model.load_state_dict(ck["model_state_dict"])
    model.eval()
    rows, missing = [], []
    for tag, m in MASSEYS.items():
        for pose in POSES:
            d = Path(root) / f"{tag}_{pose}"
            if not (d / "samples.jsonl").exists():
                if m > 0 or pose in GROUPS_OF["U"]:
                    missing.append(f"{tag}_{pose}")
                continue
            q, dq, cmd, cur = read_steady(d)
            tau_m = K * cur.mean(axis=0)
            tf = tau_free_mean(model, norm, lin_W, q, dq, cmd - q, H)
            rows.append(dict(name=f"{tag}_{pose}", m=m, pose=pose,
                             ext=(tau_m - tf).tolist(), n=len(q),
                             cur_std=(cur.std(axis=0)).tolist()))
    return rows, missing


def diff_against_empty(rows):
    """同姿态差分：τ_ext 相对该姿态的空载段。"""
    base = {r["pose"]: np.asarray(r["ext"]) for r in rows if r["m"] == 0.0}
    out = []
    for r in rows:
        if r["pose"] not in base:
            continue
        out.append(dict(name=r["name"], m=r["m"], pose=r["pose"],
                        ext=r["ext"], d=(np.asarray(r["ext"]) - base[r["pose"]])))
    return out


def basis_stack(arm, sel_rows, only=GOOD_AXES):
    A, y, mvec = [], [], []
    for r in sel_rows:
        q = D.G.deg(*POSES[r["pose"]])
        p0, A3 = _basis(arm, q, 1.0, only)
        # τ_ext = −m·(p0 + A3·Φ) → 移项： (−τ_ext)/m − p0 = A3·Φ
        # 方程： τ_ext = −m·(p0 + A3·Φ)  ⇔  m·A3·Φ = −τ_ext − m·p0
        # **必须在力矩空间加权**：若先除 m 再拟合，0.1 kg 档的噪声被放大 10 倍
        # （2026-10-20 19:0x 踩到，导致 RMS 1.08 N·m、盲测全废）。
        A.append(A3 * r["m"])
        y.append(-np.asarray(r["d"])[only] - r["m"] * p0)
        mvec.append(r["m"])
    return np.vstack(A), np.concatenate(y), np.array(mvec)


def fit_phi(arm, rows, only=GOOD_AXES):
    A, y, _ = basis_stack(arm, rows, only)
    s = np.linalg.svd(A, compute_uv=False)
    phi, *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ phi - y) ** 2)))
    return phi, dict(sing=s.tolist(), cond=float(s[0] / s[-1]) if s[-1] > 0 else float("inf"),
                     rms_nm=rms)


def predict_mass(arm, phi, row, only=GOOD_AXES):
    """已知 Φ，由 τ_ext 反解质量（最小二乘）。"""
    q = D.G.deg(*POSES[row["pose"]])
    p0, A3 = _basis(arm, q, 1.0, only)
    g = -(p0 + A3 @ phi)                     # 每 1 kg 产生的 τ_ext
    d = np.asarray(row["d"])[only]
    return float(g @ d / (g @ g)) if g @ g > 0 else float("nan")


def _blind(arm, tr, te, mh):
    phi_tr, _ = fit_phi(arm, tr)
    print(f"    标定 Φ = {np.round(phi_tr*1000,1).tolist()} mm")
    print("| 段 | 真实 | 预测 | 误差 | 相对 |")
    print("| --- | --- | --- | --- | --- |")
    errs = []
    for r in te:
        me = predict_mass(arm, phi_tr, r)
        e = me - r["m"]; errs.append(e)
        print(f"| {r['name']} | {r['m']:.2f} | {me:+.3f} | {e:+.3f} | "
              f"{100*e/r['m']:+.1f}% |")
    print(f"    平均绝对误差 {np.mean(np.abs(errs)):.3f} kg "
          f"（{100*np.mean(np.abs(errs))/mh:.1f}%）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    ap.add_argument("--dir", default=str(HERE / "episodes"))
    ap.add_argument("--holdout", type=float, default=0.5)
    ap.add_argument("--out", default=str(HERE / "runs" / "l2_weigh.json"))
    args = ap.parse_args()

    urdf, arm = D.load()
    rows, missing = load_rows(args.ckpt, args.dir)
    if missing:
        print(f"[缺段] {len(missing)}：{missing[:8]}{' …' if len(missing) > 8 else ''}")
    drows = diff_against_empty(rows)
    print(f"可用行 {len(drows)}（含空载基线），质量档 "
          f"{sorted({r['m'] for r in drows})}")

    # ---- ① 可辨识性：只用 V 族 vs V+U ----
    for label, sel in (("仅 V 族（朝前）", [r for r in drows if r["pose"] in GROUPS_OF["V"]]),
                       ("V + U 族", [r for r in drows if r["m"] > 0])):
        A, y, _ = basis_stack(arm, sel)
        s = np.linalg.svd(A, compute_uv=False)
        print(f"\n① 可辨识性 · {label}：行 {A.shape[0]} 列 {A.shape[1]}，"
              f"奇异值 {np.round(s, 3).tolist()}")
        if s[-1] < 1e-6 * s[0]:
            print("   → 秩亏（存在不可辨识方向），Φ 无法唯一确定")
        else:
            print(f"   → 满秩，条件数 {s[0]/s[-1]:.1f}")

    # ---- ② 联合拟合 Φ ----
    use = [r for r in drows if r["m"] > 0]
    phi, info = fit_phi(arm, use)
    print(f"\n② 联合拟合 Φ（fix s=−1，J2~J5）")
    print(f"   Φ = {np.round(phi*1000, 1).tolist()} mm   |Φ| = {np.linalg.norm(phi)*1000:.1f} mm")
    print(f"   模型残差 RMS = {info['rms_nm']:.4f} N·m   条件数 = {info['cond']:.1f}")

    # ---- ③ 每姿态每轴灵敏度（每 kg 的力矩）----
    print("\n③ 每姿态灵敏度（N·m / kg，J2~J5）")
    print("| 姿态 | J2 | J3 | J4 | J5 |")
    print("| --- | --- | --- | --- | --- |")
    for pose in POSES:
        q = D.G.deg(*POSES[pose])
        p0, A3 = _basis(arm, q, 1.0, GOOD_AXES)
        g = -(p0 + A3 @ phi)
        print(f"| {pose} | " + " | ".join(f"{v:+.3f}" for v in g) + " |")

    # ---- ④ 盲测：留一质量档 ----
    print("\n④ 盲测：逐档留出（训练集必须同时含 V 与 U 两族，否则秩亏）")
    for mh in sorted({r["m"] for r in use}):
        tr = [r for r in use if abs(r["m"] - mh) > 1e-9]
        te = [r for r in use if abs(r["m"] - mh) < 1e-9]
        if not tr or not te:
            continue
        A_tr, _, _ = basis_stack(arm, tr)
        s_tr = np.linalg.svd(A_tr, compute_uv=False)
        if s_tr[-1] < 1e-6 * s_tr[0]:
            print(f"\n  留出 {mh:g} kg：训练集含姿态 "
                  f"{sorted({r['pose'] for r in tr})} → 秩亏，跳过（没有 U 族数据）")
            continue
        print(f"\n  留出 {mh:g} kg（训练集姿态 {sorted({r['pose'] for r in tr})}）")
        _blind(arm, tr, te, mh)
    if False:
        phi_tr, _ = fit_phi(arm, tr)
        print(f"   标定用质量 {sorted({r['m'] for r in tr})} → Φ = "
              f"{np.round(phi_tr*1000,1).tolist()} mm")
        print("| 段 | 真实质量 | 预测质量 | 误差 | 相对误差 |")
        print("| --- | --- | --- | --- | --- |")
        errs = []
        for r in te:
            me = predict_mass(arm, phi_tr, r)
            e = me - r["m"]
            errs.append(e)
            print(f"| {r['name']} | {r['m']:.2f} | {me:+.3f} | {e:+.3f} | "
                  f"{100*e/r['m']:+.1f}% |")
        print(f"   平均绝对误差 {np.mean(np.abs(errs)):.3f} kg "
              f"（{100*np.mean(np.abs(errs))/args.holdout:.1f}%）")

    # ---- ④b 盲测：留一姿态（标定后换一个没见过的姿态能不能称）----
    print("\n④b 盲测：留一姿态（用其余姿态标定 Φ，预测该姿态各档质量）")
    for ph in POSES:
        tr = [r for r in use if r["pose"] != ph]
        te = [r for r in use if r["pose"] == ph]
        if not tr or not te:
            continue
        A_tr, _, _ = basis_stack(arm, tr)
        s_tr = np.linalg.svd(A_tr, compute_uv=False)
        if s_tr[-1] < 1e-6 * s_tr[0]:
            continue
        print(f"\n  留出姿态 {ph}（训练 {len(tr)} 行，测试 {len(te)} 行）")
        _blind(arm, tr, te, max(r["m"] for r in te))

    # ---- ⑤ 最小可分辨质量 ----
    print("\n⑤ 最小可分辨质量（按 3σ 帧间噪声，逐姿态）")
    stds = {r["pose"]: np.asarray(r["cur_std"]) for r in rows if r["m"] == 0.0}
    print("| 姿态 | 单帧 3σ 力矩 (N·m) | 换算质量 (kg) | 12 s 平均后 (kg) |")
    print("| --- | --- | --- | --- |")
    for pose in POSES:
        if pose not in stds:
            continue
        q = D.G.deg(*POSES[pose])
        p0, A3 = _basis(arm, q, 1.0, GOOD_AXES)
        g = -(p0 + A3 @ phi)
        s3 = 3.0 * np.abs(K[GOOD_AXES] * stds[pose][GOOD_AXES])
        # 沿 g 方向投影
        m_single = float(np.linalg.norm(s3) / np.linalg.norm(g))
        print(f"| {pose} | {np.linalg.norm(s3):.3f} | {m_single:.3f} | "
              f"{m_single/np.sqrt(600):.4f} |")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(phi_mm=(phi * 1000).tolist(), info=info,
                   rows=[dict(name=r["name"], m=r["m"], pose=r["pose"],
                              d=np.asarray(r["d"]).tolist()) for r in drows]),
              open(args.out, "w"), ensure_ascii=False, indent=1)
    print(f"\n写入 {args.out}")


if __name__ == "__main__":
    main()
