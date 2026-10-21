#!/usr/bin/env python3
"""阶段 4 第 3 步：测试集一次性评估（指南 §7.3）。

只读一次测试集
--------------
`test` 在这里**只出现一次**，不做任何调参。checkpoint 是训练时按 val 选出来的，
归一化参数也从 checkpoint 里读（复用训练集拟合的那一组，指南 §7.1 第 3 步）。

为什么必须给基线
----------------
τ_m 里重力占绝对主导，而重力矩是姿态的**慢变**函数。一个「输出训练集均值」的常数模型，
就能拿到不错的 MAE。如果只报 LSTM 的 MAE 而不报常数基线，读者无法判断网络到底学到了
什么。所以这里并排给三个：
  * **const** — 逐轴输出训练集均值（零信息基线）
  * **lin(q)** — 只用 q 的线性最小二乘（「重力矩 ≈ 姿态的线性函数」的一阶近似）
  * **NEXT** — LSTM 用 [q, dq, q_cmd−q] 的历史窗口
只有在**逐轴**和**静止/运动分开**的口径下全面优于前两者，才算学到了东西（而不是只学到均值）。

指标口径（与指南一致）
    MAE       = sum(|pred − ref|) / (N·J)         ← 关节平均，主判据 ≤0.05 N·m
    MAE_joint = mean(|pred[:,j] − ref[:,j]|)
    bias_joint= mean(pred[:,j] − ref[:,j])        ← 系统性偏差，抵消不掉
    P95_joint = |pred[:,j] − ref[:,j]| 的 95 分位
    P95 推理延迟 = 单窗口 batch=1 前向的 95 分位

用法（本机，LEFS/experiments/20261020_stage4）：
    python s4_eval.py --ckpt runs/next_v1/model.pt
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
import sys
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))
from factr2_next.training.models import build_model  # noqa: E402
from s4_train import WindowDataset, apply_norm  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
K_STATE = ["外推", "实测", "实测", "弱约束", "弱约束", "外推"]


def stats(pred, ref):
    err = pred - ref
    ae = np.abs(err)
    return dict(
        mae=float(ae.mean()),
        per_mae=ae.mean(axis=0).tolist(),
        per_bias=err.mean(axis=0).tolist(),
        per_p95=np.percentile(ae, 95, axis=0).tolist(),
        per_rmse=np.sqrt((err ** 2).mean(axis=0)).tolist(),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step.npz"))
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v1" / "model.pt"))
    ap.add_argument("--out", default=str(HERE / "runs" / "eval_test.json"))
    ap.add_argument("--latency-n", type=int, default=200)
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    H = int(npz["history"])
    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    norm = {k: np.asarray(v, dtype=np.float32) for k, v in ck["norm"].items()}

    # 训练集均值基线要用训练集的原始 y
    y_tr = npz["train_Y"].astype(np.float64)
    const_pred = y_tr.mean(axis=0)

    # 线性基线 tau ≈ A q + b（只用 q，用训练集拟合）
    Xtr = npz["train_X"]
    q_tr = Xtr[:, 0:6].astype(np.float64)
    A = np.hstack([q_tr, np.ones((len(q_tr), 1))])
    W, *_ = np.linalg.lstsq(A, y_tr, rcond=None)

    test = WindowDataset(npz, "test", H)
    raw_test = test.X.copy()                      # 归一化前留一份，用于算 |dq|、q
    apply_norm(test, norm)

    model = build_model(ck["model"], ck["input_size"], ck["output_size"], H)
    model.load_state_dict(ck["model_state_dict"])
    model.eval()

    preds = []
    with torch.no_grad():
        for i in range(len(test)):
            x, _ = test[i]
            preds.append(model(x.unsqueeze(0)).squeeze(0).numpy())
    P = np.stack(preds) * norm["y_std"] + norm["y_mean"]
    lin_W = ck.get("lin_W", None)
    if lin_W is not None:
        # 残差学习：网络输出残差，加回线性重力基线
        q_last_ = raw_test[test.centers][:, 0:6].astype(np.float64)
        A_last = np.hstack([q_last_, np.ones((len(q_last_), 1))])
        P = P + (A_last @ np.asarray(lin_W, np.float64))
    # 只取每个窗口「终点帧」的标签：窗口数与 step 数不同（每段前 H-1 帧没有完整历史）
    Y = test.Y[test.centers] * norm["y_std"] + norm["y_mean"]

    # 取出每个窗口「最后时刻」的原始 q / dq，做静止 vs 运动分组
    centers = test.centers
    q_last = raw_test[centers][:, 0:6]
    dq_last = raw_test[centers][:, 6:12]
    moving = np.abs(dq_last).max(axis=1) > 0.01          # rad/s

    lin_pred = np.hstack([q_last, np.ones((len(q_last), 1))]) @ W
    base_const = np.tile(const_pred, (len(Y), 1))

    res = dict(
        ckpt=str(args.ckpt),
        best_epoch=int(ck.get("epoch", -1)),
        n_windows=int(len(Y)),
        n_episodes=int(len(npz["test_ep_name"])),
        n_moving=int(moving.sum()),
        n_still=int((~moving).sum()),
        next=stats(P, Y),
        const=stats(base_const, Y),
        lin=stats(lin_pred, Y),
        next_still=stats(P[~moving], Y[~moving]) if (~moving).any() else None,
        next_moving=stats(P[moving], Y[moving]) if moving.any() else None,
        const_still=stats(base_const[~moving], Y[~moving]) if (~moving).any() else None,
        const_moving=stats(base_const[moving], Y[moving]) if moving.any() else None,
    )

    # 推理延迟：单窗口 batch=1，与部署口径一致（输入必须是 (1, H, 18)）
    x_win, _ = test[0]
    x1 = x_win.unsqueeze(0)
    with torch.no_grad():
        for _ in range(10):
            model(x1)
        lat = []
        for _ in range(args.latency_n):
            t = time.perf_counter()
            model(x1)
            lat.append((time.perf_counter() - t) * 1000.0)
    res["latency_ms"] = dict(
        p50=float(np.percentile(lat, 50)),
        p95=float(np.percentile(lat, 95)),
        mean=float(np.mean(lat)),
        note="CPU 单线程、batch=1，未做任何推理优化；不要与论文 1.76 ms（GPU/特定实现）比较",
    )

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(args.out, "w"), indent=2, ensure_ascii=False)

    # ---------------- 打印 ----------------
    print(f"checkpoint: {args.ckpt}  (best epoch {res['best_epoch']})")
    print(f"测试集：{res['n_episodes']} 段 / {res['n_windows']} 窗口 "
          f"（运动 {res['n_moving']}，静止 {res['n_still']}）")
    print()
    print("| 模型 | 关节平均 MAE | " + " | ".join(JN) + " |")
    print("| --- | --- |" + " --- |" * 6)
    for tag, key in (("常数（训练均值）", "const"), ("线性 q", "lin"), ("NEXT", "next")):
        r = res[key]
        cells = " | ".join(f"{v:.4f}" for v in r["per_mae"])
        mark = " **" if key == "next" else " "
        print(f"| {tag}{mark} | **{r['mae']:.4f}** | {cells} |")
    print()
    print("NEXT 逐轴细节（N·m）：")
    print("| 轴 | K 状态 | MAE | bias | P95 | RMSE |")
    print("| --- | --- | --- | --- | --- | --- |")
    n = res["next"]
    for j in range(6):
        print(f"| {JN[j]} | {K_STATE[j]} | {n['per_mae'][j]:.4f} | "
              f"{n['per_bias'][j]:+.4f} | {n['per_p95'][j]:.4f} | {n['per_rmse'][j]:.4f} |")
    print()
    if res["next_still"]:
        print(f"静止帧（{res['n_still']}）：NEXT MAE {res['next_still']['mae']:.4f} / "
              f"常数 {res['const_still']['mae']:.4f}")
    if res["next_moving"]:
        print(f"运动帧（{res['n_moving']}）：NEXT MAE {res['next_moving']['mae']:.4f} / "
              f"常数 {res['const_moving']['mae']:.4f}")
    print()
    print(f"推理延迟 P95 = {res['latency_ms']['p95']:.2f} ms  "
          f"(P50 {res['latency_ms']['p50']:.2f} ms, CPU batch=1)")
    print(f"\n写入 {args.out}")


if __name__ == "__main__":
    main()
