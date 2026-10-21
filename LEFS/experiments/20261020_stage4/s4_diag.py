#!/usr/bin/env python3
"""按段 / 按轴的误差归因：找出 MAE 到底是被谁拖高的。

动机：v2 的测试集 MAE 0.373 里 J2 一个轴就贡献 1.36，而线性基线只要 0.29。
不定位到「哪一段、哪种工况」就只是在猜。这里按段拆开，并区分静止/运动。

用法：
    python s4_diag.py --ckpt runs/next_v2/model.pt
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))
from factr2_next.training.models import build_model  # noqa: E402
from s4_train import WindowDataset  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step.npz"))
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v2" / "model.pt"))
    ap.add_argument("--split", default="test")
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    H = int(npz["history"])
    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}

    ds = WindowDataset(npz, args.split, H)
    raw = ds.X.copy()
    X = (raw - norm["x_mean"]) / norm["x_std"]
    Y = (ds.Y[ds.centers] - norm["y_mean"]) / norm["y_std"]

    model = build_model(ck["model"], ck["input_size"], ck["output_size"], H)
    model.load_state_dict(ck["model_state_dict"])
    model.eval()

    preds = []
    with torch.no_grad():
        for i in range(len(ds)):
            c = int(ds.centers[i])
            x = torch.from_numpy(X[c - H + 1:c + 1]).unsqueeze(0)
            preds.append(model(x).squeeze(0).numpy())
    P = np.stack(preds) * norm["y_std"] + norm["y_mean"]
    R = Y * norm["y_std"] + norm["y_mean"]

    q_last = raw[ds.centers][:, 0:6]
    dq_last = raw[ds.centers][:, 6:12]
    moving = np.abs(dq_last).max(axis=1) > 0.01
    names = npz[f"{args.split}_ep_name"]

    # 线性基线
    y_tr = npz["train_Y"].astype(np.float64)
    q_tr = npz["train_X"][:, 0:6].astype(np.float64)
    A = np.hstack([q_tr, np.ones((len(q_tr), 1))])
    W, *_ = np.linalg.lstsq(A, y_tr, rcond=None)
    LP = np.hstack([q_last, np.ones((len(q_last), 1))]) @ W

    print(f"checkpoint {args.ckpt}  split={args.split}")
    print()
    print("| 段 | 窗口 | 运动% | NEXT MAE | 线性 MAE | 该段 q2 均值 | J2 NEXT | J2 线性 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for i, ep in enumerate(names):
        m = ds.owners == i
        if not m.any():
            continue
        n_mae = np.abs(P[m] - R[m]).mean()
        l_mae = np.abs(LP[m] - R[m]).mean()
        j2n = np.abs(P[m, 1] - R[m, 1]).mean()
        j2l = np.abs(LP[m, 1] - R[m, 1]).mean()
        print(f"| {ep} | {m.sum()} | {100*moving[m].mean():.0f}% | {n_mae:.4f} | "
              f"{l_mae:.4f} | {np.degrees(q_last[m,1].mean()):+.1f}° | {j2n:.4f} | {j2l:.4f} |")

    print()
    print("按 q2 分箱（看误差是否集中在高角度稀疏区）：")
    print("| q2 区间 | 窗口 | NEXT MAE | 线性 MAE | J2 NEXT | J2 线性 |")
    print("| --- | --- | --- | --- | --- | --- |")
    q2 = np.degrees(q_last[:, 1])
    for lo, hi in ((0, 30), (30, 45), (45, 60), (60, 75), (75, 95), (95, 130)):
        m = (q2 >= lo) & (q2 < hi)
        if m.sum() < 5:
            continue
        print(f"| {lo}-{hi}° | {m.sum()} | {np.abs(P[m]-R[m]).mean():.4f} | "
              f"{np.abs(LP[m]-R[m]).mean():.4f} | {np.abs(P[m,1]-R[m,1]).mean():.4f} | "
              f"{np.abs(LP[m,1]-R[m,1]).mean():.4f} |")

    print()
    print("train / test 在 q2 各分箱的窗口占比（训练覆盖）：")
    tr = WindowDataset(npz, "train", H)
    q2tr = np.degrees(tr.X[tr.centers][:, 1])
    q2te = q2
    for lo, hi in ((0, 30), (30, 45), (45, 60), (60, 75), (75, 95), (95, 130)):
        a = ((q2tr >= lo) & (q2tr < hi)).mean()
        b = ((q2te >= lo) & (q2te < hi)).mean()
        bar = "#" * int(40 * b)
        print(f"  {lo:3d}-{hi:3d}°  train {100*a:5.1f}%   test {100*b:5.1f}% {bar}")


if __name__ == "__main__":
    main()
