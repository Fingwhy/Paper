#!/usr/bin/env python3
"""5 折交叉验证：给「模型到底好不好」一个可信答案。

为什么必须做
------------
`S4_ENSEMBLE_REPORT.md` 已量化：8 次独立训练，单模型在 5 段 test 上的 MAE 在
0.1445~0.1689 之间摆动（17%），且 val 与 test 的排序相反。这说明**评估集本身**
是当前瓶颈 —— 只有 5 段 test，一段特别难就能翻转结论。在这种噪声下比较模型，
等于在噪声里挑数字。

做法
----
把 49 段全部拿进来，按 q2 中位数**分层蛇形**分成 5 折（每折 ~10 段）：
    第 i 折当 test → 其余 39 段里再分层抽 5 段当 val → 剩 34 段训练
跑满 5 折后，**每一段都恰好被预测过一次**，把这些预测拼起来算 MAE，
就是 out-of-fold（OOF）误差。它比单次 5 段 test 稳得多，因为：
  * 用满了 49 段的信息，不再由某一段的难度决定结论；
  * 每段都当过 test ⇒ 没有「这批段恰好适合这个模型」的运气。

注意口径
--------
每折训练集只有 34 段（原切分是 39 段），所以 **OOF MAE 天然会比 v9 的 0.1357 略高**。
这不是退步，是交叉验证的固有代价。要比较的是「配置 A vs 配置 B 的 OOF」，
而不是「OOF vs 单次切分」。

用法（本机 Windows，LEFS/experiments/20261020_stage4）：
    python s4_kfold.py --history 75 --epochs 10 --seed 0
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PY = sys.executable
sys.path.insert(0, str(HERE))

K_FOLDS = 5


def stratified_folds(eps_q2, k=K_FOLDS):
    """按 q2 中位数排序后**蛇形**分配，保证每折的高低角度分布相近。"""
    order = sorted(eps_q2, key=lambda e: (eps_q2[e], e))
    folds = [[] for _ in range(k)]
    for i, e in enumerate(order):
        r = i // k
        f = (i % k) if r % 2 == 0 else (k - 1 - i % k)
        folds[f].append(e)
    return folds


def pick_val(rest, eps_q2, n=5):
    """从训练折里按 q2 顺序**均匀间隔**抽 n 段做 val（覆盖全角度范围）。"""
    order = sorted(rest, key=lambda e: (eps_q2[e], e))
    idx = np.linspace(0, len(order) - 1, n).round().astype(int)
    idx = sorted(set(int(i) for i in idx))
    # 若因取整撞车导致不足 n 段，按顺序补齐
    j = 0
    while len(idx) < n and j < len(order):
        if j not in idx:
            idx.append(j)
        j += 1
    idx = sorted(idx[:n])
    val = [order[i] for i in idx]
    train = [e for e in rest if e not in set(val)]
    return train, val


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--splits", default=str(HERE / "splits_v2.json"))
    ap.add_argument("--history", type=int, default=75)
    ap.add_argument("--epochs", type=int, default=10)
    ap.add_argument("--batch", type=int, default=512)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(HERE / "runs" / "kfold"))
    ap.add_argument("--skip-train", action="store_true", help="只重算评估，不训练")
    args = ap.parse_args()

    sp = json.load(open(args.splits, encoding="utf-8"))
    per_ep = sp["per_ep"]
    eps = list(per_ep)
    eps_q2 = {e: float(per_ep[e]["q2_med"]) for e in eps}
    print(f"总段数 {len(eps)}，分层 5 折")

    folds = stratified_folds(eps_q2)
    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)

    from s4_ensemble import predict_one  # 延迟导入，避免 --help 时加载 torch

    results = []
    t0 = time.time()
    for i, test in enumerate(folds):
        rest = [e for e in eps if e not in set(test)]
        train, val = pick_val(rest, eps_q2)
        print(f"\n===== fold {i}: train {len(train)} / val {len(val)} / test {len(test)} =====")
        print(f"  test: {sorted(test)}")
        print(f"  q2_med: test {sorted(round(eps_q2[e]) for e in test)}")

        sp_path = out_root / f"fold{i}.json"
        json.dump({"sets": {"train": train, "val": val, "test": test},
                   "per_ep": per_ep, "why": f"5-fold 第 {i} 折（q2 分层蛇形）"},
                  open(sp_path, "w"), indent=1, ensure_ascii=False)

        npz_path = HERE / "data" / f"step_kf{i}.npz"
        ckpt = out_root / f"kf{i}" / "model.pt"

        if not args.skip_train:
            log = open(out_root / f"fold{i}.log", "w", encoding="utf-8")
            subprocess.run([PY, str(HERE / "s4_build.py"), "--splits", str(sp_path),
                            "--history", str(args.history), "--out", str(npz_path)],
                           check=True, stdout=log, stderr=subprocess.STDOUT)
            subprocess.run([PY, str(HERE / "s4_train.py"), "--data", str(npz_path),
                            "--out", str(out_root / f"kf{i}"), "--residual",
                            "--batch", str(args.batch), "--epochs", str(args.epochs),
                            "--seed", str(args.seed)],
                           check=True, stdout=log, stderr=subprocess.STDOUT)
            log.close()

        npz = np.load(npz_path, allow_pickle=True)
        r = predict_one(str(ckpt), npz, "test")
        Y, P, own = r["Y"], r["P"], r["owners"]
        names = [str(x) for x in npz["test_ep_name"]]
        seg = {}
        for oi, nm in enumerate(names):
            m = own == oi
            if m.any():
                seg[nm] = dict(n=int(m.sum()),
                               mae=float(np.abs(P[m] - Y[m]).mean()),
                               per_mae=np.abs(P[m] - Y[m]).mean(axis=0).tolist())
        fold_mae = float(np.abs(P - Y).mean())
        seg_mean = float(np.mean([v["mae"] for v in seg.values()]))
        print(f"  → 窗口加权 {fold_mae:.4f} / 段等权 {seg_mean:.4f}（best epoch {r['best_epoch']}）")
        results.append(dict(fold=i, best_epoch=r["best_epoch"],
                            n_windows=int(len(Y)), window_mae=fold_mae,
                            seg_mean=seg_mean, seg=seg,
                            train=train, val=val, test=sorted(test)))

    # ---------------- 汇总 ----------------
    all_seg = {}
    for r in results:
        all_seg.update(r["seg"])
    oof = float(np.mean([v["mae"] for v in all_seg.values()]))
    per_axis = np.mean([v["per_mae"] for v in all_seg.values()], axis=0)
    fold_seg = [r["seg_mean"] for r in results]
    summary = dict(
        history=args.history, epochs=args.epochs, seed=args.seed,
        n_segments=len(all_seg), n_folds=len(results),
        oof_mae=oof, oof_per_axis=per_axis.tolist(),
        fold_seg_mean=fold_seg,
        fold_seg_std=float(np.std(fold_seg)),
        fold_seg_min=float(np.min(fold_seg)), fold_seg_max=float(np.max(fold_seg)),
        note="oof_mae = 49 段段等权平均；每折训练集 34 段（比原切分 39 段少），故天然略高于单次切分",
        folds=results)
    json.dump(summary, open(out_root / "kfold_summary.json", "w"), indent=2, ensure_ascii=False)

    print(f"\n{'='*60}")
    print(f"5 折 OOF MAE（{len(all_seg)} 段，段等权）= {oof:.4f} N·m")
    print(f"各折段等权 MAE: {[round(x,4) for x in fold_seg]}")
    print(f"  折间 std {summary['fold_seg_std']:.4f} / 范围 {summary['fold_seg_min']:.4f}~{summary['fold_seg_max']:.4f}")
    print(f"逐轴: " + "  ".join(f"{n} {v:.3f}" for n, v in zip(["J1","J2","J3","J4","J5","J6"], per_axis)))
    print(f"\n总耗时 {(time.time()-t0)/60:.1f} min → {out_root/'kfold_summary.json'}")


if __name__ == "__main__":
    main()
