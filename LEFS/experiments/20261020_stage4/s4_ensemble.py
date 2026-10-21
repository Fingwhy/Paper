#!/usr/bin/env python3
"""阶段 4 补充实验：多随机种子**等权集成**评估。

为什么做这一件
--------------
v9（H=75 + 残差）的 test MAE 0.1445 已贴近标签噪声地板：J3 实测 0.343 / 地板 0.280
（1.2×，占总误差 40%），J4/J6 已经在 1.0~1.1×。再加容量或正则只会更过拟合
（v11 按 q2 加权采样反而 +6.7%）。剩下的降误差手段里，唯一不需要动硬件的就是
**集成**：不同 seed 的初始化和 batch 顺序不同，误差中互不相关的部分取平均后会
抵消，而所有模型共有的系统偏差（如 K 的标定误差、J3 漂移）不会被消掉。
因此增益主要来自「方差项」，预期 10% 量级，不是数量级。

口径（三条，避免自我欺骗）
--------------------------
1. **等权，不调权重**。按 val 挑权重、再到 test 上报，等于在测试集上选模型。
2. **同时报 val / test / merged（窗口加权）**。R64：val 和 test 各只有 5 段，
   单次切分的排序会不一致，只报一个数就是点估计。
3. **逐轴也报**。若集成只改善了 J1/J2（本来就有余量的轴），而 J3 没动，
   那就是在噪声里挑数字，不是真进步。

用法（本机 Windows，LEFS/experiments/20261020_stage4）：
    python s4_ensemble.py --data data/step_h75.npz \
        --ckpt runs/next_v9_h75/model.pt runs/next_v9_seed1/model.pt \
               runs/next_v9_seed2/model.pt runs/next_v9_seed3/model.pt \
        --out runs/ensemble_v9.json
"""

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))
from factr2_next.training.models import build_model  # noqa: E402
from s4_train import WindowDataset, apply_norm  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
K_STATE = ["外推", "实测", "实测", "弱约束", "弱约束", "外推"]


def stats(pred, ref):
    ae = np.abs(pred - ref)
    err = pred - ref
    return dict(mae=float(ae.mean()),
                per_mae=ae.mean(axis=0).tolist(),
                per_bias=err.mean(axis=0).tolist(),
                per_p95=np.percentile(ae, 95, axis=0).tolist())


def predict_one(ckpt, npz, split, batch=1024, device="cpu"):
    """单个 checkpoint 在某个 split 上出**物理单位**的预测（已加回残差基线）。"""
    H = int(npz["history"])
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    if int(ck["history"]) != H:
        raise SystemExit(f"{ckpt}: checkpoint H={ck['history']} 与数据 H={H} 不符")
    norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}

    ds = WindowDataset(npz, split, H)
    raw = ds.X.copy()                      # 归一化前留一份，取原始 q 算残差基线
    apply_norm(ds, norm)

    model = build_model(ck["model"], ck["input_size"], ck["output_size"], H).to(device)
    model.load_state_dict(ck["model_state_dict"])
    model.eval()

    out = []
    with torch.no_grad():
        for x, _ in DataLoader(ds, batch_size=batch, shuffle=False):
            out.append(model(x.to(device)).cpu().numpy())
    P = np.concatenate(out).astype(np.float64) * norm["y_std"] + norm["y_mean"]

    lw = ck.get("lin_W", None)
    if lw is not None:
        q = raw[ds.centers][:, 0:6].astype(np.float64)
        P = P + np.hstack([q, np.ones((len(q), 1))]) @ np.asarray(lw, np.float64)

    Y = ds.Y[ds.centers].astype(np.float64) * norm["y_std"] + norm["y_mean"]
    return dict(P=P, Y=Y, n=len(Y), best_epoch=int(ck.get("epoch", -1)),
                centers=ds.centers, owners=ds.owners, q=raw[ds.centers][:, 0:6],
                dq=raw[ds.centers][:, 6:12])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step_h75.npz"))
    ap.add_argument("--ckpt", nargs="+", required=True)
    ap.add_argument("--out", default=str(HERE / "runs" / "ensemble.json"))
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)

    # ---- 每个 split 只推理一次，之后所有组合都在内存里算 ----
    per_split = {}
    for split in ("val", "test"):
        rows = []
        for ck in args.ckpt:
            rows.append(predict_one(ck, npz, split, args.batch, args.device))
        # 元数据必须对齐：不同 seed 用的是同一份数据，窗口数/终点帧必须一致
        n0 = rows[0]["n"]
        for ck, r in zip(args.ckpt, rows):
            if r["n"] != n0:
                raise SystemExit(f"{ck} 在 {split} 上窗口数 {r['n']} != {n0}")
            if not np.array_equal(r["centers"], rows[0]["centers"]):
                raise SystemExit(f"{ck} 在 {split} 上的窗口终点与第一个 checkpoint 不一致")
        per_split[split] = rows

    Y_val = per_split["val"][0]["Y"]
    Y_test = per_split["test"][0]["Y"]

    def merged_mae(v, t):
        nv, nt = len(Y_val), len(Y_test)
        return (nv * v + nt * t) / (nv + nt)

    # ---- 单模型 ----
    singles = []
    for i, ck in enumerate(args.ckpt):
        sv = stats(per_split["val"][i]["P"], Y_val)
        st = stats(per_split["test"][i]["P"], Y_test)
        singles.append(dict(
            ckpt=ck, best_epoch=per_split["val"][i]["best_epoch"],
            val=sv, test=st,
            val_mae=sv["mae"], test_mae=st["mae"],
            merged_mae=merged_mae(sv["mae"], st["mae"])))

    # ---- 等权集成（全部模型） ----
    Pv = np.mean([r["P"] for r in per_split["val"]], axis=0)
    Pt = np.mean([r["P"] for r in per_split["test"]], axis=0)
    ev, et = stats(Pv, Y_val), stats(Pt, Y_test)

    # ---- 多样性：模型两两预测之差的平均绝对值（N·m） ----
    pairs = []
    for a, b in itertools.combinations(range(len(args.ckpt)), 2):
        d = np.abs(per_split["test"][a]["P"] - per_split["test"][b]["P"])
        pairs.append(dict(a=Path(args.ckpt[a]).parent.name,
                          b=Path(args.ckpt[b]).parent.name,
                          mean_abs_diff=float(d.mean()),
                          per_axis=d.mean(axis=0).tolist()))

    # ---- 子集集成：2~3 个的组合，看是否所有成员都在贡献 ----
    subsets = []
    if len(args.ckpt) > 2:
        for k in range(2, len(args.ckpt) + 1):
            for idx in itertools.combinations(range(len(args.ckpt)), k):
                pv = np.mean([per_split["val"][i]["P"] for i in idx], axis=0)
                pt = np.mean([per_split["test"][i]["P"] for i in idx], axis=0)
                a, b = stats(pv, Y_val), stats(pt, Y_test)
                subsets.append(dict(members=[Path(args.ckpt[i]).parent.name for i in idx],
                                    val_mae=a["mae"], test_mae=b["mae"],
                                    merged_mae=merged_mae(a["mae"], b["mae"])))

    # ---- 逐段拆解：判断差异到底来自哪一段 ----
    # 5 段的聚合 MAE 会掩盖段间差异。若某一两段主导（该段同时占窗口多数或误差极大），
    # 那「0.1445 vs 0.1686」只反映那一两段的偏好，不能当成模型优劣的结论。
    per_seg = []
    for split, Ysplit, P_ens in (("val", Y_val, Pv), ("test", Y_test, Pt)):
        names = npz[f"{split}_ep_name"]
        own = per_split[split][0]["owners"]
        for oi in range(len(names)):
            m = own == oi
            if not m.any():
                continue
            row = dict(split=split, ep=str(names[oi]), n=int(m.sum()))
            for i, ck in enumerate(args.ckpt):
                row[Path(ck).parent.name] = float(
                    np.abs(per_split[split][i]["P"][m] - Ysplit[m]).mean())
            row["ensemble"] = float(np.abs(P_ens[m] - Ysplit[m]).mean())
            per_seg.append(row)

    best_single = min(singles, key=lambda r: r["merged_mae"])
    res = dict(
        data=str(args.data), n_ckpt=len(args.ckpt),
        n_val_windows=int(len(Y_val)), n_test_windows=int(len(Y_test)),
        note="等权集成，权重未调；merged = (n_val·MAE_val + n_test·MAE_test)/(n_val+n_test)",
        singles=singles, best_single=best_single["ckpt"],
        ensemble=dict(val=ev, test=et, val_mae=ev["mae"], test_mae=et["mae"],
                      merged_mae=merged_mae(ev["mae"], et["mae"])),
        gain_vs_best_single_pct=float(
            100.0 * (1.0 - merged_mae(ev["mae"], et["mae"]) / best_single["merged_mae"])),
        gain_vs_best_single_test_pct=float(
            100.0 * (1.0 - et["mae"] / best_single["test_mae"])),
        diversity=pairs, subsets=subsets, per_seg=per_seg)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(args.out, "w"), indent=2, ensure_ascii=False)

    # ------------------------------ 打印 ------------------------------
    print(f"数据 {args.data}   val {len(Y_val)} 窗口 / test {len(Y_test)} 窗口   "
          f"{len(args.ckpt)} 个模型")
    print()
    print("| 模型 | best ep | val MAE | test MAE | merged MAE |")
    print("| --- | --- | --- | --- | --- |")
    for s in singles:
        nm = Path(s["ckpt"]).parent.name
        print(f"| {nm} | {s['best_epoch']} | {s['val_mae']:.4f} | "
              f"{s['test_mae']:.4f} | {s['merged_mae']:.4f} |")
    print(f"| **等权集成（{len(args.ckpt)}）** | - | **{ev['mae']:.4f}** | "
          f"**{et['mae']:.4f}** | **{merged_mae(ev['mae'], et['mae']):.4f}** |")
    print()
    print(f"相对最佳单模型：test {res['gain_vs_best_single_test_pct']:+.1f}% / "
          f"merged {res['gain_vs_best_single_pct']:+.1f}%")
    print()
    print("集成后逐轴（test，N·m）：")
    print("| 轴 | K 状态 | 最佳单模型 | 集成 | 变化 |")
    print("| --- | --- | --- | --- | --- |")
    bs = stats(per_split["test"][args.ckpt.index(best_single["ckpt"])]["P"], Y_test)
    for j in range(6):
        a, b = bs["per_mae"][j], et["per_mae"][j]
        print(f"| {JN[j]} | {K_STATE[j]} | {a:.4f} | {b:.4f} | {100*(b/a-1):+.1f}% |")
    print()
    full_names = [Path(c).parent.name for c in args.ckpt]
    short = [n.replace("next_v9_", "").replace("next_", "") for n in full_names]
    print("逐段 MAE（N·m）—— 差异是否集中在个别段：")
    print("| split | 段 | 窗口 | " + " | ".join(short) + " | 集成 | 集成 vs 该段最佳 |")
    print("| --- | --- | --- |" + " --- |" * (len(short) + 2))
    for r in per_seg:
        vals = [r[n] for n in full_names]
        b = min(vals)
        print(f"| {r['split']} | {r['ep']} | {r['n']} | "
              + " | ".join(f"{v:.4f}" for v in vals)
              + f" | {r['ensemble']:.4f} | {100*(r['ensemble']/b-1):+.1f}% |")
    print()
    # 段等权：每段先算自己的 MAE 再平均。窗口加权会被窗口多的段主导，
    # 而 5 段 test 里只要有一段特别难就能翻转结论 ⇒ 段等权更稳。
    print("10 段（val+test）等权 MAE —— 抗单段主导，用于跨模型比较：")
    for nm in full_names + ["ensemble"]:
        vals = [r[nm] for r in per_seg]
        print(f"  {nm:28s} {np.mean(vals):.4f}   "
              f"（最好段 {min(vals):.4f} / 最差段 {max(vals):.4f}）")
    print()
    if pairs:
        print("模型间预测差异（test，越大说明误差越不相关、集成越有用）：")
        for p in pairs:
            print(f"  {p['a']} vs {p['b']}: 平均 |Δ| = {p['mean_abs_diff']:.4f} N·m  "
                  f"逐轴 {[round(x,3) for x in p['per_axis']]}")
    if subsets:
        print()
        print("子集集成（看每个成员是否都在贡献）：")
        for s in subsets:
            print(f"  {'+'.join(s['members']):40s} merged {s['merged_mae']:.4f}  "
                  f"(val {s['val_mae']:.4f} / test {s['test_mae']:.4f})")
    print(f"\n写入 {args.out}")


if __name__ == "__main__":
    main()
