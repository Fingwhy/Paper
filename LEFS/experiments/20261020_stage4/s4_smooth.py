#!/usr/bin/env python3
"""诊断 + 廉价修复：对预测做**因果时间平滑**（EMA），看 MAE 能降多少。

背景（已在 s4_trace.py 中证实）
-------------------------------
同一批数据、同一结构，只是训练久了几个 epoch：
  * v9（best epoch=2）  预测一阶差分只有真值的 0.04~0.09 倍  —— 近乎平线
  * seed1~3（10 epoch） 是 0.23~0.54 倍                       —— 跟着真值动
而在小幅动作段（M7_J2J6、V3_J1）上，**越平越准**：真值本身几乎不变，
任何跟随高频的部分都是把标签噪声学进去了。

但这不意味着「越平越好」：在 P7_J2slow（真值变化最大的一段）上 v9 反而最差。
正确结论是 —— 模型需要**按信噪比自适应地平滑**，而不是一味跟随或一味冻结。

本脚本验证一个零训练成本的近似做法：
    P_s[t] = α·P[t] + (1−α)·P_s[t−1]      （每个段独立重置，纯因果，可部署）
α=1 等价于不平滑。α 在 **val** 上挑，test 只报数（不在 test 上选参数）。

用法（本机，LEFS/experiments/20261020_stage4）：
    python s4_smooth.py --data data/step_h75.npz \
        --ckpt runs/next_v9_h75/model.pt runs/next_v9_seed1/model.pt
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from s4_ensemble import predict_one  # noqa: E402

ALPHAS = [1.0, 0.5, 0.3, 0.2, 0.15, 0.10, 0.07, 0.05, 0.03, 0.02]


def ema_by_segment(P, owners, alpha):
    """段内因果 EMA；段首用当帧初始化（不跨段泄漏）。"""
    if alpha >= 1.0:
        return P
    out = np.empty_like(P)
    prev = None
    cur = -1
    for t in range(len(P)):
        if owners[t] != cur:
            cur = owners[t]
            prev = P[t].copy()
        else:
            prev = alpha * P[t] + (1.0 - alpha) * prev
        out[t] = prev
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step_h75.npz"))
    ap.add_argument("--ckpt", nargs="+", required=True)
    ap.add_argument("--out", default=str(HERE / "runs" / "smooth.json"))
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    data = {}
    for split in ("val", "test"):
        rows = [predict_one(ck, npz, split) for ck in args.ckpt]
        data[split] = dict(rows=rows, owners=rows[0]["owners"], Y=rows[0]["Y"])

    names = [Path(c).parent.name.replace("next_v9_", "").replace("next_", "") for c in args.ckpt]
    nv, nt = len(data["val"]["Y"]), len(data["test"]["Y"])

    def merged(v, t):
        return (nv * v + nt * t) / (nv + nt)

    # 逐个模型 + 等权集成，扫 alpha
    series = {}
    for i, nm in enumerate(names):
        series[nm] = [data[s]["rows"][i]["P"] for s in ("val", "test")]
    series[f"ens{len(names)}"] = [
        np.mean([r["P"] for r in data[s]["rows"]], axis=0) for s in ("val", "test")]

    table = {}
    for nm, (Pv, Pt) in series.items():
        rows = []
        for a in ALPHAS:
            Qv = ema_by_segment(Pv, data["val"]["owners"], a)
            Qt = ema_by_segment(Pt, data["test"]["owners"], a)
            mv = float(np.abs(Qv - data["val"]["Y"]).mean())
            mt = float(np.abs(Qt - data["test"]["Y"]).mean())
            rows.append(dict(alpha=a, val=mv, test=mt, merged=merged(mv, mt)))
        table[nm] = rows

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(n_val=nv, n_test=nt, alphas=ALPHAS, table=table),
              open(args.out, "w"), indent=2, ensure_ascii=False)

    # ------------------------------ 打印 ------------------------------
    print(f"val {nv} 窗口 / test {nt} 窗口。merged = val/test 窗口加权。")
    for nm, rows in table.items():
        r1 = rows[0]
        best_val = min(rows, key=lambda r: r["val"])
        best_mg = min(rows, key=lambda r: r["merged"])
        print()
        print(f"### {nm}")
        print("α         val      test     merged")
        for r in rows:
            mark = ""
            if r["alpha"] == best_val["alpha"]:
                mark += "  ← val 最优"
            if r["alpha"] == best_mg["alpha"]:
                mark += "  ← merged 最优"
            print(f"{r['alpha']:<6.2f}  {r['val']:.4f}   {r['test']:.4f}   "
                  f"{r['merged']:.4f}{mark}")
        print(f"  不平滑(α=1) merged {r1['merged']:.4f} → "
              f"val 选中 α={best_val['alpha']:.2f} 时 merged {best_val['merged']:.4f} "
              f"（{100*(best_val['merged']/r1['merged']-1):+.1f}%），"
              f"其 test {best_val['test']:.4f} "
              f"（{100*(best_val['test']/r1['test']-1):+.1f}%）")


if __name__ == "__main__":
    main()
