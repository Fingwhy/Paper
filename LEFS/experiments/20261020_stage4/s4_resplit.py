#!/usr/bin/env python3
"""阶段 4：按姿态分层重切分（替换阶段 3 的按段随机切分）。

为什么必须重做
--------------
阶段 4 诊断（`s4_diag.py`）暴露：阶段 3 的 `splits.json` 是**按段随机**切的，
而 42 段的姿态分布极不均匀（大部分段的 q2 在 27~45°，只有 6 段在高角度）。
结果 test 抽到的 3 段全在 q2=60~82°，于是

    train  q2∈[75,95°] 占  5.9%
    test   q2∈[75,95°] 占 50.9%

这不是「独立同分布测试集」，任何非线性模型都会在这块崩（二次多项式 train 0.107 / test 0.516）。
指南 §7.3 的验收门写的是「独立同分布测试集」，所以**切分不改，L1 的结论就不能用**。

分层依据
--------
按每段 q2 的中位数分箱（q2 是承载重力矩的主力轴，也是误差的主要来源），
每个箱内按 8:1:1 分配。这样三集合在 q2 上的分布天然一致，同时**仍按整段分配**
（禁令一：不跨 episode 切窗）。

判据（全部通过才写文件）：
  1. train 时长 ≥ 9 min
  2. 任一集合在任一 q2 分箱的占比，与全局占比偏差 ≤ 10 个百分点
  3. 六关节覆盖峰峰 ≥ 8°
  4. 无段重复出现在两个集合
  5. train/val/test 段数比例在 8:1:1 ±0.06

用法（本机）：
    python s4_resplit.py --report      # 只看分配与判据，不写文件
    python s4_resplit.py               # 写 splits_v2.json
"""

import argparse
import hashlib
import json
import random
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"
TOPDIR = HERE / "episodes"          # 补采段（若为空则只用阶段 3 的 42 段）
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]

# q2 分箱（度）。边界按阶段 3 实际分布定：27~45 最密，75~95 最稀。
BINS = [(0, 40), (40, 60), (60, 75), (75, 95), (95, 180)]
SEED = 20261020


def find_episodes():
    eps = {}
    for root in (STAGE3 / "episodes", TOPDIR):
        if not root.is_dir():
            continue
        for d in sorted(root.iterdir()):
            if not d.is_dir() or not (d / "samples.jsonl").exists():
                continue
            eps[d.name] = d
    return eps


def ep_stats(path):
    rows = [json.loads(l) for l in open(path / "samples.jsonl", encoding="utf-8")]
    rows = [r for r in rows if r.get("q") and None not in r["q"]]
    q = np.array([r["q"] for r in rows], float)
    t = np.array([r["t"] for r in rows], float)
    q2 = np.degrees(q[:, 1])
    return dict(n=len(rows), dur=float(t[-1] - t[0]), q2_med=float(np.median(q2)),
                q2_lo=float(np.percentile(q2, 5)), q2_hi=float(np.percentile(q2, 95)),
                pp=[float((q[:, i].max() - q[:, i].min()) * 57.29578) for i in range(6)])


def bin_of(v):
    for i, (lo, hi) in enumerate(BINS):
        if lo <= v < hi:
            return i
    return len(BINS) - 1


def largest_remainder(counts, total, weights):
    """按权重把 total 个名额分给各组，最大余数法。"""
    raw = [c / sum(counts) * total for c in counts]
    base = [int(np.floor(x)) for x in raw]
    rem = total - sum(base)
    order = sorted(range(len(raw)), key=lambda i: raw[i] - base[i], reverse=True)
    for i in order[:rem]:
        base[i] += 1
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--out", default=str(HERE / "splits_v2.json"))
    ap.add_argument("--test-frac", type=float, default=0.10)
    ap.add_argument("--val-frac", type=float, default=0.10)
    args = ap.parse_args()

    eps = find_episodes()
    if not eps:
        print("找不到任何 episode。")
        return
    st = {n: ep_stats(p) for n, p in eps.items()}
    names = sorted(st)
    print(f"共 {len(names)} 段（阶段 3 {sum(1 for n in names if (STAGE3/'episodes'/n).is_dir())} 段"
          f" + 补采 {sum(1 for n in names if (TOPDIR/n).is_dir())} 段）")

    # 按 q2 中位数分箱
    buckets = {i: [] for i in range(len(BINS))}
    for n in names:
        buckets[bin_of(st[n]["q2_med"])].append(n)
    print("\n分层（按 q2 中位数）：")
    for i, (lo, hi) in enumerate(BINS):
        g = sorted(buckets[i], key=lambda n: st[n]["q2_med"])
        print(f"  {lo:3d}-{hi:3d}°  {len(g):2d} 段  " + " ".join(g[:8]) +
              (" …" if len(g) > 8 else ""))

    # 每个箱内按 8:1:1 分配
    n_test = [max(1, int(round(len(buckets[i]) * args.test_frac))) if buckets[i] else 0
              for i in range(len(BINS))]
    n_val = [max(1, int(round(len(buckets[i]) * args.val_frac))) if buckets[i] else 0
             for i in range(len(BINS))]
    # 小箱不能把 test+val 吃光
    for i in range(len(BINS)):
        while n_test[i] + n_val[i] > len(buckets[i]) - 1 and (n_test[i] + n_val[i]) > 0:
            if n_test[i] >= n_val[i]:
                n_test[i] -= 1
            else:
                n_val[i] -= 1

    rng = random.Random(SEED)
    sets = {"train": [], "val": [], "test": []}
    for i in range(len(BINS)):
        g = list(buckets[i])
        rng.shuffle(g)
        sets["test"] += g[:n_test[i]]
        sets["val"] += g[n_test[i]:n_test[i] + n_val[i]]
        sets["train"] += g[n_test[i] + n_val[i]:]
    for k in sets:
        sets[k].sort()

    # ---------------- 判据 ----------------
    tot_bin = np.array([sum(1 for n in names if bin_of(st[n]["q2_med"]) == i)
                        for i in range(len(BINS))], float)
    glob = tot_bin / tot_bin.sum()
    print("\n各集合在 q2 分箱上的占比（判据：与全局偏差 ≤10 个百分点）：")
    print("| 集合 | 段数 | " + " | ".join(f"{lo}-{hi}°" for lo, hi in BINS) + " |")
    print("| --- | --- |" + " --- |" * len(BINS))
    dev_max = 0.0
    for k in ("train", "val", "test"):
        cnt = np.array([sum(1 for n in sets[k] if bin_of(st[n]["q2_med"]) == i)
                        for i in range(len(BINS))], float)
        frac = cnt / max(cnt.sum(), 1)
        dev_max = max(dev_max, float(np.max(np.abs(frac - glob))))
        print(f"| {k} | {len(sets[k])} | " +
              " | ".join(f"{100*f:.0f}%" for f in frac) + " |")

    dur = {k: sum(st[n]["dur"] for n in sets[k]) for k in sets}
    ratio = {k: len(sets[k]) / len(names) for k in sets}
    cov = [max(st[n]["pp"][j] for n in names) for j in range(6)]
    dup = (set(sets["train"]) & set(sets["val"])) | (set(sets["train"]) & set(sets["test"])) \
        | (set(sets["val"]) & set(sets["test"]))

    gates = dict(
        train_over_9min=dur["train"] >= 540,
        bin_dev_le_10pct=dev_max <= 0.10,
        coverage_6joint=min(cov) >= 8.0,
        no_cross_split=len(dup) == 0,
        ratio_ok=all(abs(ratio[k] - w) <= 0.06
                     for k, w in (("train", 0.8), ("val", 0.1), ("test", 0.1))),
    )
    print(f"\ntrain {len(sets['train'])} 段 / {dur['train']:.0f} s（{dur['train']/60:.1f} min）"
          f"  val {len(sets['val'])} 段 / {dur['val']:.0f} s"
          f"  test {len(sets['test'])} 段 / {dur['test']:.0f} s")
    print(f"比例 {', '.join(f'{k}={v:.3f}' for k, v in ratio.items())}（目标 8:1:1 ±0.06）")
    print(f"六关节覆盖峰峰 {[round(c,1) for c in cov]}（门槛 8.0）")
    print(f"q2 分箱最大偏差 {100*dev_max:.1f} 个百分点")
    for k, v in gates.items():
        print(f"  {'OK ' if v else 'FAIL'} {k}")
    ok = all(gates.values())
    print(f"\n总判：{'通过' if ok else '未通过'}")

    payload = dict(
        created="2026-10-20",
        stage="stage4_resplit",
        why="阶段 3 按段随机切分导致 test 50.9% 落在 train 仅 5.9% 覆盖的 q2 区间；改为按 q2 中位数分层",
        seed=SEED, bins=BINS, glob_frac=glob.tolist(),
        counts={k: len(v) for k, v in sets.items()},
        durations_s=dur, ratio=ratio, coverage_pp_deg=cov,
        max_bin_dev=dev_max, gates=gates, sets=sets,
        per_ep={n: st[n] for n in names},
    )
    blob = json.dumps({k: payload[k] for k in ("sets", "bins", "seed")}, sort_keys=True)
    payload["hash_sha256"] = hashlib.sha256(blob.encode()).hexdigest()

    if args.report:
        print("\n（--report：未写文件）")
        return
    if not ok:
        print("\n判据未全过，不写文件。先修分层参数。")
        return
    json.dump(payload, open(args.out, "w"), indent=1, ensure_ascii=False)
    print(f"\n写入 {args.out}  sha256={payload['hash_sha256'][:16]}…")


if __name__ == "__main__":
    main()
