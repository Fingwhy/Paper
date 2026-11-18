#!/usr/bin/env python
"""FIRST 阈值标定：一次算完 f_t 并缓存，然后扫分位 × 滞回比。

背景（2026-10-21）：
    按论文 §5 的滞回判据（T_high/T_low），把 T_high 取自由空间的 P99，
    实测误报 6.9~9.6 次/分，远超 ≤1 次/分的门。根因有两层：
      1) **P99 本身不对**：32.9 min 里 P99 只切掉 0.33 min，剩下的峰足够多；
      2) **J3 低频漂移**：即使 M0_A（空载静止）也有 19 个超标尖峰，
         与称重误差是同一个根源（J3 在 12 s 内可漂 1.2 N·m）。
    所以本脚本扫 T_high 的分位与滞回比，找可行工作点。

用法：
    python s4_first_sweep.py            # 算 f_t 并缓存 → 扫描
    python s4_first_sweep.py --reuse    # 直接读缓存
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import s4_first_seg as M  # noqa: E402

CACHE = HERE / "runs" / "l2_first_ft.npz"


def collect(pred):
    calib = sorted(Path(p).name for p in
                   glob.glob(str(HERE.parent / "20261019_stage3" / "episodes" / "*")))
    load = [e for e in (f"{t}_{x}" for t in ("M1", "M2", "M3") for x in ("A", "B", "C"))
            if (HERE / "episodes" / e / "samples.jsonl").exists()]
    out = {}
    for tag, eps in (("calib", calib), ("load", load)):
        for ep in eps:
            r = M.tau_ext_of(ep, pred)
            if r is None:
                continue
            te, t, _ = r
            body = te[pred.H - 1:]                      # 丢无完整历史的段首
            out[ep] = dict(f_all=M.contact_score(body, None),
                           f_good=M.contact_score(body, M.GOOD_AXES),
                           dur=float(t[-1] - t[0]))
            print(f"  {ep:14s} {len(body):5d} 帧  {t[-1]-t[0]:5.1f} s  "
                  f"f_all 中位 {np.median(out[ep]['f_all']):.3f}")
    return out


def save(d, path):
    keys = list(d)
    np.savez_compressed(
        path,
        keys=np.array(keys),
        f_all=np.concatenate([d[k]["f_all"] for k in keys]),
        f_good=np.concatenate([d[k]["f_good"] for k in keys]),
        dur=np.array([d[k]["dur"] for k in keys]),
        n=np.array([len(d[k]["f_all"]) for k in keys]),
    )


def load(path):
    z = np.load(path, allow_pickle=True)
    keys = [str(k) for k in z["keys"]]
    out, i = {}, 0
    for k, n in zip(keys, z["n"]):
        out[k] = dict(f_all=z["f_all"][i:i + n], f_good=z["f_good"][i:i + n],
                      dur=float(z["dur"][list(z["keys"]).index(k)]) if False else None)
        i += n
    # dur 单独按 keys 顺序取
    for k, dd in zip(keys, z["dur"]):
        out[k]["dur"] = float(dd)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reuse", action="store_true")
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    ap.add_argument("--data", default=str(HERE / "data" / "step_v2.npz"))
    ap.add_argument("--out", default=str(HERE / "runs" / "l2_first_sweep.json"))
    args = ap.parse_args()

    if args.reuse and CACHE.exists():
        d = load(CACHE)
        print(f"读缓存 {CACHE.name}（{len(d)} 段）")
    else:
        pred = M.NextPredictor(Path(args.ckpt), Path(args.data))
        print(f"用 {Path(args.ckpt).parent.name} 算 f_t ...")
        d = collect(pred)
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        save(d, CACHE)
        print(f"缓存 → {CACHE.name}")

    # **命名歧义坑（2026-10-21）**：阶段 3 也有 M1_J2J3 / M2_J2J3J5 / M3_J1J4 这些段名，
    # 它们以 M1/M2/M3 开头但**完全无载荷**。用 startswith 分类会把它们当成阶段 5 的
    # 带载段（recall 当然是 0%），同时把它们从标定集里挤掉（39 段而不是 42 段）。
    # 阶段 5 的段名格式固定是 `M<数字>_<A|B|C|U1|U2|U3>`，用它精确匹配。
    import re
    LOAD_RE = re.compile(r"^M[0-3]_(A|B|C|U1|U2|U3)$")
    loadv = {k: v for k, v in d.items() if LOAD_RE.match(k)}
    calib = {k: v for k, v in d.items() if not LOAD_RE.match(k)}
    dur_min = sum(v["dur"] for v in calib.values()) / 60.0
    print(f"\n标定集 {len(calib)} 段 / {dur_min:.1f} min；带载 {len(loadv)} 段\n")

    res = {}
    for mode, key in (("all", "f_all"), ("good", "f_good")):
        F = np.concatenate([v[key] for v in calib.values()])
        print(f"### 口径 {mode}（{'全 6 轴（论文）' if mode=='all' else '只 J2~J5'}）"
              f"  f_t 中位 {np.median(F):.3f}  P99 {np.percentile(F,99):.3f}  "
              f"max {F.max():.3f}")
        print("| T_high 规则 | T_high | 误报/分 | recall 均值 | recall 最差 | 判 |")
        print("|---|---|---|---|---|---|")
        best = None
        for q in (99.0, 99.5, 99.9, 99.95, 99.99):
            for ratio in (0.6, 0.8):
                th = float(np.percentile(F, q)); tl = th * ratio
                n = sum(len(M.phase_labels(M.hysteresis(v[key], th, tl))[1])
                        for v in calib.values())
                rate = n / dur_min
                recs = [float(M.hysteresis(v[key], th, tl).mean()) for v in loadv.values()]
                ok = rate <= 1.0
                star = ""
                if ok and (best is None or np.mean(recs) > best[4]):
                    best = (q, ratio, th, rate, float(np.mean(recs)))
                    star = " ←"
                print(f"| P{q:g} × {ratio:g} | {th:.3f} | **{rate:.2f}** | "
                      f"{100*np.mean(recs):.1f}% | {100*np.min(recs):.1f}% | "
                      f"{'PASS' + star if ok else 'FAIL'} |")
        res[mode] = dict(median=float(np.median(F)), p99=float(np.percentile(F, 99)),
                         max=float(F.max()), dur_min=dur_min,
                         best=dict(quantile=best[0], ratio=best[1], t_high=best[2],
                                   rate_per_min=best[3], recall=best[4]) if best else None)
        print()
        if best:
            print(f"  **口径 {mode} 的可行工作点**：T_high = P{best[0]:g} = {best[2]:.3f}，"
                  f"T_low = {best[1]:g}×，误报 {best[3]:.2f} 次/分，recall {100*best[4]:.1f}%")
            # 该点下 M1（0.1 kg）能否检出
            th, tl = best[2], best[2] * best[1]
            per = {k: float(M.hysteresis(v[key], th, tl).mean()) for k, v in loadv.items()}
            print("  逐段 recall：" + "  ".join(f"{k} {100*v:.0f}%" for k, v in sorted(per.items())))
        print()

    Path(args.out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"写入 {args.out}")


if __name__ == "__main__":
    main()
