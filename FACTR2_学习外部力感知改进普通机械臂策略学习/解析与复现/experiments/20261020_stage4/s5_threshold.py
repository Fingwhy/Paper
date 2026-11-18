#!/usr/bin/env python3
"""阶段 5（L2）第 1 步：在**无接触**数据上定阈值初值，并算检测下限。

为什么这一步必须最先做
----------------------
L2 的全部前提是 `τ_ext = τ_m − τ̂_free` 在自由空间里接近 0。但「接近 0」到底是多少，
决定了后面所有场景设计：

* 如果 P99 残差是 0.3 N·m，那么小于 ~0.3 N·m 的接触力矩就**检不出来**（按 80 mm 力臂
  折算约 4 N 的接触力）。挡板场景必须设计得比这个更"硬"，否则测出来的 recall
  反映的是模型精度而不是检测算法。
* 这一步只用已有的 49 段**无接触**数据 + L1 模型，不需要机械臂、不需要接触工装。
  先算清楚，再去买/做挡板，避免白采。

阈值口径（指南 §8.2）
--------------------
1. 在一段**独立**的无接触数据上算 `score = Σ|τ_ext|`——这里用 **val 集**（5 段，未参与训练）；
2. `T_high` = score 的 P99，`T_low` = 0.6 × `T_high`（滞回，避免阈值附近抖动出假事件）；
3. **冻结后到另一组数据上验证**——这里用 **test 集**（5 段），检查误触发率。
   test 上不允许再回调阈值。

为什么给 4 轴和 6 轴两版
------------------------
J1/J6 的 K 是组结构外推、力矩本身接近噪声（阶段 4 §2），把它们算进 score 只会抬高地板。
4 轴版（J2~J5）是可辩护的主口径，6 轴版作为对照。

用法（本机）：
    python s5_threshold.py
    python s5_threshold.py --axes 6     # 用六轴
"""

import argparse
import json
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
import sys
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))
from factr2_next.training.models import build_model  # noqa: E402
from s4_train import WindowDataset  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]


def predict_ext(npz, split, ckpt, H):
    """返回 (score, ext, moving, centers, ep_names)。score 按 axis 子集求和。"""
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}
    ds = WindowDataset(npz, split, H)
    raw = ds.X.copy()
    X = (raw - norm["x_mean"]) / norm["x_std"]

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

    lin_W = ck.get("lin_W", None)
    if lin_W is not None:
        q = raw[ds.centers][:, 0:6].astype(np.float64)
        A = np.hstack([q, np.ones((len(q), 1))])
        P = P + (A @ np.asarray(lin_W, np.float64))

    # 注意：这里**没有**对 ds 做 apply_norm（只在局部构造 X 时归一化），
    # 所以 ds.Y 已是原始 τ（N·m），不能再乘 y_std 反归一化 —— 那会多除一个尺度，
    # 把 τ_ext 放大 2.5 倍并导致阈值定得虚高（2026-10-20 17:0x 踩到并修正）。
    Y = ds.Y[ds.centers].astype(np.float64)
    ext = Y - P
    dq = raw[ds.centers][:, 6:12]
    moving = np.abs(dq).max(axis=1) > 0.01
    return ext, moving, ds.centers, ds.owners, npz[f"{split}_ep_name"]


def hysteresis_events(score, t_high, t_low, min_hold=1):
    """滞回状态机：idle → active 记一次事件。返回事件数与每次的起止索引。"""
    state = "idle"
    events = []
    start = None
    hold = 0
    for i, s in enumerate(score):
        if state == "idle":
            if s > t_high:
                hold += 1
                if hold >= min_hold:
                    state = "active"
                    start = i - hold + 1
                    hold = 0
            else:
                hold = 0
        else:
            if s < t_low:
                events.append((start, i))
                state = "idle"
                hold = 0
    if state == "active":
        events.append((start, len(score) - 1))
    return events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step_v2.npz"))
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    ap.add_argument("--axes", type=int, default=4, choices=(4, 6),
                    help="4 = 只用 J2~J5（K 实测/弱约束，主口径）；6 = 全部")
    ap.add_argument("--low-ratio", type=float, default=0.6)
    ap.add_argument("--out", default=str(HERE / "runs" / "l2_threshold.json"))
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    H = int(npz["history"])
    idx = list(range(1, 5)) if args.axes == 4 else list(range(6))
    tag = "J2~J5" if args.axes == 4 else "J1~J6"
    print(f"checkpoint {args.ckpt}")
    print(f"score 口径：Σ|τ_ext| over {tag}    滞回 T_low = {args.low_ratio} × T_high")
    print()

    res = {}
    for split in ("val", "test"):
        ext, moving, centers, owners, names = predict_ext(npz, split, args.ckpt, H)
        score = np.abs(ext[:, idx]).sum(axis=1)
        # 按段统计时长（50 Hz）
        dur = len(score) / 50.0
        res[split] = dict(score=score, dur=dur, names=names, owners=owners,
                          ext=ext, moving=moving)

    v = res["val"]
    s_val = v["score"]
    T_high = float(np.percentile(s_val, 99))
    T_low = args.low_ratio * T_high
    print("## 无接触数据的 score 分布（val，5 段，阈值标定用）")
    print(f"  样本 {len(s_val)} 帧 / {v['dur']:.0f} s")
    for p in (50, 90, 99, 99.9, 100):
        print(f"  P{p:<5g} = {np.percentile(s_val, p):.4f} N·m")
    print(f"  → T_high = P99 = {T_high:.4f} N·m，T_low = {T_low:.4f} N·m")
    print()

    print("## 冻结阈值后在 test 上验证（不允许回调）")
    t = res["test"]
    ev = hysteresis_events(t["score"], T_high, T_low)
    rate = len(ev) / (t["dur"] / 60.0)
    print(f"  test {len(t['score'])} 帧 / {t['dur']:.0f} s（{t['dur']/60:.1f} min）")
    print(f"  误触发 {len(ev)} 次 → {rate:.2f} 次/分钟（验收门 ≤1）")
    if ev:
        lens = [(b - a + 1) / 50.0 for a, b in ev]
        print(f"  事件时长 中位 {np.median(lens):.2f} s，最长 {max(lens):.2f} s")
    print()

    # 检测下限：T_high 对应多大力
    print("## 检测下限（换算成接触力，力臂按 80 mm）")
    for lever in (0.06, 0.08, 0.10):
        print(f"  力臂 {lever*1000:.0f} mm  →  T_high 对应 {T_high/lever:.1f} N，"
              f"3σ 地板 {np.percentile(s_val,99.9)/lever:.1f} N")

    print()
    print("## 逐轴 τ_ext 的分布（val）")
    print("| 轴 | P50 | P99 | max |")
    print("| --- | --- | --- | --- |")
    for j in range(6):
        a = np.abs(v["ext"][:, j])
        print(f"| {JN[j]} | {np.percentile(a,50):.4f} | {np.percentile(a,99):.4f} | {a.max():.4f} |")

    print()
    print("## 结论")
    ok = rate <= 1.0
    print(f"  冻结阈值在 test 上误触发 {rate:.2f} 次/分 → {'满足' if ok else '不满足'} ≤1 次/分的门")
    print(f"  能可靠检出的最小接触力矩 ≈ {T_high:.2f} N·m"
          f"（80 mm 力臂约 {T_high/0.08:.1f} N）")
    print(f"  ⇒ 接触场景必须产生 **> {3*T_high:.1f} N·m** 的力矩变化，"
          f"否则测到的是模型误差，不是检测能力")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(checkpoint=args.ckpt, axes=tag, T_high=T_high, T_low=T_low,
                   low_ratio=args.low_ratio, val_frames=len(s_val), val_dur_s=v["dur"],
                   test_events=len(ev), test_rate_per_min=rate, test_dur_s=t["dur"],
                   val_pct={f"P{p}": float(np.percentile(s_val, p)) for p in (50, 90, 99, 99.9, 100)}),
              open(args.out, "w"), indent=2, ensure_ascii=False)
    print(f"\n写入 {args.out}")


if __name__ == "__main__":
    main()
