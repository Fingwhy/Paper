#!/usr/bin/env python3
"""换一个物体、不做任何预标定，直接称出质量（需要 2 个方位的两段数据）。

为什么 2 段就够、1 段不行
------------------------
载荷力矩 `τ_pay = m·(p0(q) + A3(q)·Φ)`。把未知量换成 (m, Ψ=mΦ)：

    −τ_ext = m·p0(q) + A3(q)·Ψ        ← 对 (m, Ψ) 是**线性**的，4 个未知量

每个姿态给 4 个方程（J2~J5），但**同一族的姿态里 p0 与 A3 张成的方向几乎一样**，
1 个姿态只有 3 个独立方向（实测 A 单独时第 4 个奇异值 = 0）⇒ m 定不出来。
换成「朝前 + 掌心朝上」两个方位后第 4 个奇异值升到 2.47（条件数 7.0），
m 与 Φ 可以同时解出——**不需要预先知道 Φ**。

实测（500 g 砝码，只用两段）：
    A + U1   → 0.468 kg（−6.5%）
    A+C+U1   → 0.476 kg（−4.7%）
    仅 A     → 0.604 kg（+20.8%，秩亏，不可用）

用法（本机）
    python s5_weigh_new.py --fwd NEW_A --up NEW_U1
    python s5_weigh_new.py --fwd NEW_A --up NEW_U1 --base-fwd M0_A --base-up M0_U1
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "20261019_stage3"))

import s3_design as D                                    # noqa: E402
from s5_load_fit import POSES, _basis, read_steady, tau_free_mean   # noqa: E402
from s4_build import K                                   # noqa: E402
from factr2_next.training.models import build_model      # noqa: E402

ONLY = [1, 2, 3, 4]          # J2~J5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fwd", required=True, help="朝前姿态那一段的目录名（如 NEW_A）")
    ap.add_argument("--up", required=True, help="掌心朝上那一段的目录名（如 NEW_U1）")
    ap.add_argument("--base-fwd", default="M0_A", help="朝前姿态的空载基线段")
    ap.add_argument("--base-up", default="M0_U1", help="掌心朝上姿态的空载基线段")
    ap.add_argument("--dir", default=str(HERE / "episodes"))
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    args = ap.parse_args()

    root = Path(args.dir)
    for name in (args.fwd, args.up):
        if not (root / name / "samples.jsonl").exists():
            sys.exit(f"找不到 {root / name}")

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}
    H = int(ck["history"])
    lin_W = ck.get("lin_W", None)
    model = build_model(ck["model"], ck["input_size"], ck["output_size"], H)
    model.load_state_dict(ck["model_state_dict"])
    model.eval()

    def tau_ext(name):
        q, dq, cmd, cur = read_steady(root / name)
        return K * cur.mean(axis=0) - tau_free_mean(model, norm, lin_W, q, dq, cmd - q, H)

    urdf, arm = D.load()
    rows, y = [], []
    for pose, seg, base in (("A", args.fwd, args.base_fwd), ("U1", args.up, args.base_up)):
        d = tau_ext(seg) - tau_ext(base)
        q = D.G.deg(*POSES[pose])
        p0, A3 = _basis(arm, q, 1.0, ONLY)
        rows.append(np.hstack([p0[:, None], A3]))
        y.append(-np.asarray(d)[ONLY])
        print(f"  {seg}  相对 {base}：Δτ(J2~J5) = "
              + " ".join(f"{v:+.3f}" for v in d[ONLY]) + " N·m")
    A = np.vstack(rows)
    y = np.concatenate(y)
    s = np.linalg.svd(A, compute_uv=False)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ x - y) ** 2)))
    m, psi = float(x[0]), x[1:]

    print()
    print(f"设计矩阵奇异值 {np.round(s, 3).tolist()}  条件数 {s[0]/s[-1]:.1f}")
    if s[-1] < 1e-3 * s[0]:
        print("！秩亏：两个姿态的方向太接近，无法分离 m 与 Φ（检查是否真的一朝前、一朝上）")
    print(f"残差 RMS = {rms:.4f} N·m")
    print()
    print(f"==> 质量 m = {m:+.3f} kg")
    print(f"    质心 Φ = {np.round(psi / m * 1000, 1).tolist()} mm"
          f"（|Φ| = {np.linalg.norm(psi / m) * 1000:.1f} mm）")
    print()
    print("对照：同一个 500 g 砝码用这两段解出 0.468 kg（−6.5%），"
          "三个姿态 0.476 kg（−4.7%）")


if __name__ == "__main__":
    main()
