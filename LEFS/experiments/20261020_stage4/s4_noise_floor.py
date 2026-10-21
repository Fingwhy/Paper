#!/usr/bin/env python3
"""阶段 4 的「验收门能不能立」：标签 τ_m 的不可约噪声地板。

为什么必须单独算这个
--------------------
指南 §7.3 的初期工程目标是「关节平均 MAE ≤ 0.05 N·m」。那个数是**有力矩传感器**的
口径。本机没有力矩传感器，τ_m = K·I 是从电流反算的，而电流在静止保持时的**重复性**
就不是零（固件量化 + 齿隙微振 + 温漂）。如果这个地板本身就 > 0.05 N·m，
那么任何模型都不可能达到 0.05，硬报「未达标」是把测量问题算成模型问题。

两个估计量
----------
1. **重复性 std**：H1~H3 三段静止保持（同一姿态 30 s 不动）内 τ_m 的标准差。
   这是最干净的「同一真值重复测量散布」。
2. **差分白噪声 σ**：`std(diff(τ))/√2`。对慢变趋势（重力随姿态慢变）不敏感，
   只反映帧间的随机抖动。两者接近说明噪声确实是白的；差很多说明有低频漂移。

判据
----
    gate_j = max(0.05, 3 × σ_j)      # 3σ：模型误差小于标签噪声 1/3 才有意义
低于 gate 认为「到达测量分辨率」，不再追究。

用法（本机，LEFS/experiments/20261020_stage4）：
    python s4_noise_floor.py
"""

import json
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from s4_build import build_episode, K_STATE  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
HOLDS = ["H1_hold_A", "H2_hold_B", "H3_hold_C"]


def main():
    rows = []
    print("## 静止保持段的 τ_m 散布（同一姿态不动 30 s）")
    print("| 段 | 轴 | τ 均值 (N·m) | 重复性 std | 差分 σ | 相对 |")
    print("| --- | --- | --- | --- | --- | --- |")
    for ep in HOLDS:
        got = build_episode(ep)
        if got is None:
            print(f"| {ep} | - | 跳过 | - | - | - |")
            continue
        x, y, t, info = got
        y2 = y[50:]                       # 丢掉开头 1 s，避开接近段残余
        rep = y2.std(axis=0)
        d = np.diff(y2, axis=0)
        sig = d.std(axis=0) / np.sqrt(2.0)
        for j in range(6):
            rel = 100.0 * rep[j] / abs(y2[:, j].mean()) if abs(y2[:, j].mean()) > 1e-6 else float("nan")
            print(f"| {ep} | {JN[j]} | {y2[:,j].mean():+.3f} | {rep[j]:.4f} | "
                  f"{sig[j]:.4f} | {rel:.1f}% |")
            rows.append((ep, j, float(y2[:, j].mean()), float(rep[j]), float(sig[j])))

    # 取三段里最大的那个作为保守地板（最坏情况）
    print()
    print("## 噪声地板（三段取最大，保守）")
    print("| 轴 | K 状态 | 重复性 std | 3σ | 验收门 max(0.05, 3σ) | 与 0.05 比 |")
    print("| --- | --- | --- | --- | --- | --- |")
    floor = []
    gate = []
    for j in range(6):
        v = [r[3] for r in rows if r[1] == j]
        s = max(v) if v else float("nan")
        g = max(0.05, 3.0 * s)
        floor.append(s)
        gate.append(g)
        verdict = "自身即超 0.05" if 3 * s > 0.05 else "0.05 主导"
        print(f"| {JN[j]} | {K_STATE[j]} | {s:.4f} | {3*s:.4f} | {g:.4f} | {verdict} |")

    print()
    print(f"关节平均噪声地板 = {np.mean(floor):.4f} N·m，"
          f"3σ 平均 = {3*np.mean(floor):.4f} N·m")
    print(f"关节平均验收门 = {np.mean(gate):.4f} N·m")
    print()
    print("结论：")
    hard = [JN[j] for j in range(6) if 3 * floor[j] > 0.05]
    if hard:
        print(f"  **{', '.join(hard)} 的 3σ 已超过 0.05 N·m** —— 指南 §7.3 的"
              f"「关节平均 MAE ≤0.05」在本测量链路上**不可达**，")
        print("  它不是模型问题，是「没有力矩传感器、只有电流」的物理限制。")
        print("  验收改用：MAE 与该轴噪声地板之比，以及是否显著优于常数/线性基线。")
    else:
        print("  所有轴 3σ 均低于 0.05 N·m，指南目标可用。")

    out = Path(__file__).resolve().parent / "runs" / "noise_floor.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(floor_std=floor, gate_3sigma=gate, raw=rows),
              open(out, "w"), indent=2, ensure_ascii=False)
    print(f"\n写入 {out}")


if __name__ == "__main__":
    main()
