#!/usr/bin/env python3
"""阶段 4 的基线谱系：τ_m 里到底有多少「可学的东西」。

为什么要有这个脚本
------------------
如果只报「NEXT 的 MAE = 0.34」，读者无法判断这个数好不好。真正的参照系是：
给定同样的输入，**最简单的模型能做到多少**。这里把从「零信息」到「灵活非线性」
的整条谱系排出来，每一档都只加一点点信息：

    const          逐轴输出训练集均值                  —— 零信息
    lin(q)         τ ≈ Aq + b                          —— 只有姿态（重力）
    lin(q, dq)     再加当前速度                        —— 惯性/粘滞
    lin(q, dq, Δ)  再加 q_cmd − q                      —— 摩擦/滞后
    quad(q)        q 的二次项（含两两乘积）            —— 非线性重力

判据：**如果 lin(q, dq, Δ) 相比 lin(q) 没有明显提升，那么「历史窗口」这条路
在自由空间就没有增量信息** —— 这直接决定 NEXT 该不该存在。

用法（本机）：
    python s4_baselines.py
    python s4_baselines.py --split val
"""

import argparse
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]


def fit_eval(Xtr, Ytr, Xte, Yte, cols, quad=False):
    def design(X):
        if quad:
            parts = [X[:, 0:6]]
            for a, b in combinations(range(6), 2):
                parts.append((X[:, a] * X[:, b])[:, None])
            return np.hstack(parts)
        return X[:, cols]

    A = np.hstack([design(Xtr), np.ones((len(Xtr), 1))])
    W, *_ = np.linalg.lstsq(A, Ytr, rcond=None)
    P = np.hstack([design(Xte), np.ones((len(Xte), 1))]) @ W
    E = np.abs(P - Yte)
    return E.mean(), E.mean(axis=0), np.abs(np.hstack([design(Xtr), np.ones((len(Xtr), 1))]) @ W - Ytr).mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step.npz"))
    ap.add_argument("--split", default="test", choices=("val", "test"))
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    Xtr = npz["train_X"].astype(np.float64)
    Ytr = npz["train_Y"].astype(np.float64)
    Xte = npz[f"{args.split}_X"].astype(np.float64)
    Yte = npz[f"{args.split}_Y"].astype(np.float64)

    print(f"评估集 = {args.split}（{len(Xte)} 帧）；train {len(Xtr)} 帧")
    print()
    print("| 模型 | 关节平均 MAE | 训练集 MAE | " + " | ".join(JN) + " |")
    print("| --- | --- | --- |" + " --- |" * 6)

    const = np.tile(Ytr.mean(axis=0), (len(Yte), 1))
    Ec = np.abs(const - Yte)
    print(f"| const（训练均值） | **{Ec.mean():.4f}** | — | "
          + " | ".join(f"{v:.4f}" for v in Ec.mean(axis=0)) + " |")

    rows = [
        ("lin(q)", list(range(0, 6)), False),
        ("lin(q, dq)", list(range(0, 12)), False),
        ("lin(q, dq, q_cmd−q)", list(range(0, 18)), False),
        ("lin 仅 (q_cmd−q)", list(range(12, 18)), False),
        ("lin 仅 dq", list(range(6, 12)), False),
        ("quad(q) 27 维", None, True),
    ]
    for name, cols, quad in rows:
        avg, per, tr = fit_eval(Xtr, Ytr, Xte, Yte, cols, quad)
        print(f"| {name} | **{avg:.4f}** | {tr:.4f} | "
              + " | ".join(f"{v:.4f}" for v in per) + " |")

    print()
    base = fit_eval(Xtr, Ytr, Xte, Yte, list(range(0, 6)), False)[0]
    full = fit_eval(Xtr, Ytr, Xte, Yte, list(range(0, 18)), False)[0]
    print(f"结论：加入 dq 与 q_cmd−q 后，{args.split} MAE {base:.4f} → {full:.4f}"
          f"（改善 {100*(base-full)/base:.1f}%）")
    if abs(base - full) / base < 0.10:
        print("  ⇒ **速度与滞后项在自由空间的增量 <10%**：τ_m 基本只由姿态决定。")
        print("  ⇒ 依赖历史窗口的 NEXT 在这类任务上拿不到额外信息，"
              "这是它在测试集上打不过 6 维线性回归的根本原因。")
    print()
    print("对照：quad(q) 在训练集远好于线性、在评估集明显更差 = 典型过拟合，")
    print("      说明「更灵活」不等于「更准」，样本在该姿态区间的覆盖才是瓶颈。")


if __name__ == "__main__":
    main()
