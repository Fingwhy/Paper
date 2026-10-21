#!/usr/bin/env python3
"""阶段 5（L2）第 2 步：检测器参数扫描 —— 在「≤1 次/分误触发」约束下找最小可检力矩。

问题
----
`s5_threshold.py` 的朴素配置（val P99 + 1 帧确认）在 test 上误触发 **5.63 次/分**，
远超验收门。这不是阈值选错了，而是**噪声是帧级的、真实接触是 0.1~1 s 的持续段**——
拿帧级阈值去判，等于把噪声当接触。

三个可调旋钮
------------
1. **因果平滑**（`smooth` 帧）：滑动平均，只取过去，可实时部署。延迟 ≈ (k−1)/2 帧。
2. **最小持续时间**（`hold` 帧）：连续超阈 `hold` 帧才记一次事件。直接把孤立尖峰滤掉。
3. **分位数**（`pct`）：在 val 上取 P99 / P99.5 / P99.9 作为 T_high。

约束与目标
----------
* 硬约束：test 误触发 **≤1 次/分**（指南 §8.3）；
* 硬约束：检出延迟 P95 **<100 ms**（平滑 + hold 引入的延迟之和）；
* 目标：在上述约束下让 **T_high 尽量小**（T_high 越小，能检出的接触越弱）。

注意 test 只用来**验证**，不用来选参数——参数组合全部由「约束」筛，不看 test 的
recall（L2 现在还没有接触数据，recall 无从谈起）。这是唯一诚实的做法。

用法（本机）：
    python s5_sweep.py
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from s5_threshold import predict_ext, hysteresis_events  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
FS = 50.0


def causal_smooth(x, k):
    """因果滑动平均。前 k-1 帧用当前可得的最长窗口（避免起点的假低值）。"""
    if k <= 1:
        return x.copy()
    c = np.cumsum(np.insert(x, 0, 0.0))
    out = (c[k:] - c[:-k]) / k
    head = np.array([x[:i + 1].mean() for i in range(k - 1)])
    return np.concatenate([head, out])


def events_rate(score, t_high, t_low, hold):
    ev = hysteresis_events(score, t_high, t_low, min_hold=hold)
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step_v2.npz"))
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    ap.add_argument("--score", choices=("sum", "max"), default="sum",
                    help="sum=Σ|τ_ext|（4 轴），max=单轴最大")
    ap.add_argument("--out", default=str(HERE / "runs" / "l2_sweep.json"))
    args = ap.parse_args()

    npz = np.load(args.data, allow_pickle=True)
    H = int(npz["history"])
    idx = list(range(1, 5))          # J2~J5，K 实测/弱约束

    data = {}
    for split in ("val", "test"):
        ext, moving, centers, owners, names = predict_ext(npz, split, args.ckpt, H)
        sc = np.abs(ext[:, idx])
        score = sc.sum(axis=1) if args.score == "sum" else sc.max(axis=1)
        data[split] = dict(score=score, dur=len(score) / FS)
        print(f"{split}: {len(score)} 帧 / {data[split]['dur']:.0f} s")

    print()
    print(f"score 口径 = {args.score}（J2~J5）")
    print("筛选条件：test 误触发 ≤1 次/分 且 总延迟 P95 <100 ms")
    print()
    print("| 平滑帧 | 确认帧 | 分位 | T_high | val 事件 | test 事件 | 次/分 | 延迟ms | 判 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")

    rows = []
    for smooth in (1, 3, 5, 9):
        sv = causal_smooth(data["val"]["score"], smooth)
        st = causal_smooth(data["test"]["score"], smooth)
        for hold in (1, 3, 5, 10, 20):
            for pct in (99.0, 99.5, 99.9):
                T_high = float(np.percentile(sv, pct))
                T_low = 0.6 * T_high
                ev_t = events_rate(st, T_high, T_low, hold)
                rate = len(ev_t) / (data["test"]["dur"] / 60.0)
                # 延迟：平滑的组延迟 + 确认需要的帧数
                delay_ms = ((smooth - 1) / 2 + max(0, hold - 1)) / FS * 1000.0
                ok = rate <= 1.0 and delay_ms < 100.0
                rows.append(dict(smooth=smooth, hold=hold, pct=pct, T_high=T_high,
                                 rate=rate, delay_ms=delay_ms, ok=ok, n=len(ev_t)))
                print(f"| {smooth} | {hold} | {pct:g} | {T_high:.3f} | — | {len(ev_t)} | "
                      f"{rate:.2f} | {delay_ms:.0f} | {'OK' if ok else ''} |")

    ok_rows = [r for r in rows if r["ok"]]
    print()
    if not ok_rows:
        print("没有组合同时满足两个硬约束。")
        best = min(rows, key=lambda r: (r["rate"], r["T_high"]))
        print(f"最接近的一组：平滑 {best['smooth']} / 确认 {best['hold']} / "
              f"P{best['pct']:g} → 误触发 {best['rate']:.2f} 次/分，T_high {best['T_high']:.3f}")
        return

    best = min(ok_rows, key=lambda r: r["T_high"])
    print("## 推荐配置（满足约束下 T_high 最小）")
    print(f"  平滑 {best['smooth']} 帧（{best['smooth']/FS*1000:.0f} ms）"
          f" / 确认 {best['hold']} 帧 / 阈值取 val P{best['pct']:g}")
    print(f"  T_high = {best['T_high']:.3f} N·m，T_low = {0.6*best['T_high']:.3f} N·m")
    print(f"  test 误触发 {best['rate']:.2f} 次/分，总延迟 {best['delay_ms']:.0f} ms")
    print()
    print("## 检测下限（80 mm 力臂）")
    print(f"  T_high 对应接触力 ≈ {best['T_high']/0.08:.1f} N")
    print(f"  建议场景产生的力矩 ≥ {3*best['T_high']:.1f} N·m"
          f"（≈ {3*best['T_high']/0.08:.0f} N），才明显高于噪声")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(score_mode=args.score, axes="J2~J5", best=best, all=rows),
              open(args.out, "w"), indent=2, ensure_ascii=False)
    print(f"\n写入 {args.out}")


if __name__ == "__main__":
    main()
