#!/usr/bin/env python3
"""诊断：模型预测的「抖动」是否超过真值本身。

为什么需要
----------
逐段拆解发现：v9（best epoch=2，几乎没训练）在 test 的两段上明显赢过训练 10 轮的
seed1~3，而这两段恰好都是**小幅正弦**：
  * M7_J2J6  — J2 幅值 5°、J6 幅值 10°
  * V3_J1    — J1 幅值 10°
小幅动作意味着重力矩几乎不变、真值本身很平，误差主要来自「模型自己造出来的
波动」。假设：训练更久的模型把训练集里的高频细节（摩擦、量化噪声）也学了进去，
在小幅动作上表现为过冲；v9 只训练到第 2 个 epoch，输出更平滑，于是反而更准。

判据
----
对同一条连续窗口序列取一阶差分，比较
    ratio = mean|Δpred| / mean|Δref|
ratio > 1 ⇒ 预测比真值抖；≈1 ⇒ 平滑度接近真值。
这个指标不看误差大小，只看「谁在制造高频」，因此能区分
「系统性偏差」和「过拟合抖动」。

用法（本机，LEFS/experiments/20261020_stage4）：
    python s4_trace.py --data data/step_h75.npz \
        --ckpt runs/next_v9_h75/model.pt runs/next_v9_seed1/model.pt
"""

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from s4_ensemble import predict_one  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step_h75.npz"))
    ap.add_argument("--ckpt", nargs="+", required=True)
    ap.add_argument("--split", default="test")
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    names = [str(x) for x in npz[f"{args.split}_ep_name"]]
    rows = [predict_one(ck, npz, args.split) for ck in args.ckpt]
    own = rows[0]["owners"]
    Y = rows[0]["Y"]
    cks = [Path(c).parent.name.replace("next_v9_", "").replace("next_", "") for c in args.ckpt]

    print(f"{args.split} 集，{len(names)} 段。ratio = mean|Δpred| / mean|Δref|（>1 = 预测比真值抖）")
    print()
    print("| 段 | 关节平均 | " + " | ".join(cks) + " |")
    print("| --- | --- |" + " --- |" * len(cks))
    for oi, nm in enumerate(names):
        m = own == oi
        if not m.any():
            continue
        dY = np.abs(np.diff(Y[m], axis=0)).mean()
        cells = []
        for r in rows:
            dP = np.abs(np.diff(r["P"][m], axis=0)).mean()
            cells.append(f"{dP/max(dY,1e-9):.2f}")
        print(f"| {nm} | {dY*1000:.2f}e-3 | " + " | ".join(cells) + " |")

    # 逐轴看：哪个关节在抖
    print()
    print("逐轴 ratio（全部 test 段合并）：")
    dY = np.abs(np.diff(Y, axis=0)).mean(axis=0)
    print("| 模型 | " + " | ".join(JN) + " | 平均 |")
    print("| --- |" + " --- |" * 7)
    for ck, r in zip(cks, rows):
        dP = np.abs(np.diff(r["P"], axis=0)).mean(axis=0)
        ratio = dP / np.maximum(dY, 1e-9)
        print(f"| {ck} | " + " | ".join(f"{v:.2f}" for v in ratio)
              + f" | {ratio.mean():.2f} |")


if __name__ == "__main__":
    main()
