#!/usr/bin/env python
"""按论文 §5（FIRST）的方法做接触阶段分割，并在我们的数据上验收。

论文原文（§5）：
    我们根据估计的外部关节力矩 τ̂ext,t，用标量接触分数 f_t = ||τ̂ext,t||_1 来判定接触。
    为避免噪声造成的频繁切换，我们采用带迟滞的阈值判定：
    c_t = 1 if f_t ≥ T_high；c_t = 0 if f_t ≤ T_low；否则保持先前取值。
    接触起始 = c_t 由 0 变 1 的时间步。
    阶段标签 s_t：Contact（c_t=1）；Pre-Contact（每个接触起始之前的 F 个时间步，
    F = 控制频率，即 1 秒）；其余 Free Space。

为什么要在这里做
----------------
阶段 5 之前只做了「阈值扫描 + 连续帧确认」（`s5_sweep.py`），那是工程近似。
论文用的是**标准施密特触发（滞回）**，而且多一个 Pre-Contact 标签——后者是 FIRST
上采样的直接输入，不做它就没法进阶段 6。

本脚本在**已知真值**的数据上验收分割器：
    · 自由空间数据（阶段 3 的 42 段 + 阶段 5 的 M0 空载）→ 真值应 0 个事件
      ⇒ 误报率 = 事件数 / 分钟
    · 带载数据（M1/M2/M3）→ 应整段判为 Contact（载荷是持续接触）
      ⇒ recall = 被判为 Contact 的帧占比

口径说明（重要）
----------------
论文 f_t 是**全 6 轴**的 L1 范数。但我们的 J1/J6 的 K 是「按组外推」的
（4.7185 由 J2/J3 借来），会把这两轴的电流噪声放大 4 倍，直接污染 f_t。
所以本脚本同时给出两种口径：
    all  = 全 6 轴（论文口径）
    good = 只 J2~J5（我们 K 实测/弱约束的 4 轴）

用法
----
    python s4_first_seg.py
    python s4_first_seg.py --axes good --ckpt runs/next_v7_resid/model.pt
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
sys.path.insert(0, str(HERE.parent / "20261019_stage3"))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))

import torch  # noqa: E402
from factr2_next.training.models import build_model  # noqa: E402
from s4_build import build_episode, K  # noqa: E402

GOOD_AXES = [1, 2, 3, 4]          # J2~J5
FPS = 50.0
MIN_HISTORY = 25


# --------------------------------------------------------------------------- #
class NextPredictor:
    """NEXT → 逐帧 τ̂_free。按段重置历史（训练时窗口不跨段，推理也必须一致）。"""

    def __init__(self, ckpt: Path, data: Path):
        npz = np.load(data, allow_pickle=True)
        self.H = int(npz["history"])
        ck = torch.load(ckpt, map_location="cpu", weights_only=False)
        self.norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}
        self.lin_W = ck.get("lin_W", None)
        self.model = build_model(ck["model"], ck["input_size"], ck["output_size"], self.H)
        self.model.load_state_dict(ck["model_state_dict"])
        self.model.eval()

    def per_frame(self, x: np.ndarray) -> np.ndarray:
        """x: (N,18) → (N,6)。段首 H-1 帧用首帧补齐，长度不变。"""
        x = x.astype(np.float32)
        pad = np.repeat(x[:1], self.H - 1, axis=0)
        xp = np.vstack([pad, x])
        win = np.lib.stride_tricks.sliding_window_view(xp, self.H, axis=0)
        win = np.transpose(win, (0, 2, 1))                       # (N,H,18)
        win = ((win - self.norm["x_mean"]) / self.norm["x_std"]).astype(np.float32)
        out = []
        with torch.no_grad():
            for i in range(0, len(win), 4096):
                out.append(self.model(torch.from_numpy(win[i:i + 4096].copy())).numpy())
        P = np.concatenate(out) * self.norm["y_std"] + self.norm["y_mean"]
        if self.lin_W is not None:
            q = x[:, 0:6].astype(np.float64)
            P = P + np.hstack([q, np.ones((len(q), 1))]) @ np.asarray(self.lin_W, np.float64)
        return P


def tau_ext_of(ep: str, pred: NextPredictor):
    """返回 (tau_ext (N,6), t (N,), 有效帧数)。y 就是 τ_m = K·I。"""
    got = build_episode(ep)
    if got is None:
        return None
    x, y, t, info = got
    return y.astype(np.float64) - pred.per_frame(x).astype(np.float64), t, info["n_used"]


# --------------------------------------------------------------------------- #
def contact_score(tau_ext: np.ndarray, axes: list[int] | None) -> np.ndarray:
    T = tau_ext if axes is None else tau_ext[:, axes]
    return np.abs(T).sum(axis=1)


def hysteresis(f: np.ndarray, t_high: float, t_low: float) -> np.ndarray:
    """论文 §5：f≥T_high→1；f≤T_low→0；否则保持（标准施密特触发）。"""
    c = np.zeros(len(f), dtype=np.int8)
    prev = 0
    for i, v in enumerate(f):
        if v >= t_high:
            prev = 1
        elif v <= t_low:
            prev = 0
        c[i] = prev
    return c


def phase_labels(c: np.ndarray, fps: float = FPS) -> tuple[np.ndarray, np.ndarray]:
    """论文 §5：0=Free Space, 1=Pre-Contact（起始前 F 步）, 2=Contact。"""
    starts = np.where(np.diff(c.astype(np.int8)) == 1)[0] + 1
    lab = np.where(c == 1, 2, 0).astype(np.int8)
    F = int(round(fps))
    for s in starts:
        lab[max(0, s - F):s] = 1
    return lab, starts


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=str(HERE / "runs" / "next_v7_resid" / "model.pt"))
    ap.add_argument("--data", default=str(HERE / "data" / "step_v2.npz"))
    ap.add_argument("--axes", choices=("all", "good", "both"), default="both")
    ap.add_argument("--t-high-quantile", type=float, default=99.0)
    ap.add_argument("--t-low-ratio", type=float, default=0.6)
    ap.add_argument("--out", default=str(HERE / "runs" / "l2_first_seg.json"))
    args = ap.parse_args()

    pred = NextPredictor(Path(args.ckpt), Path(args.data))
    print(f"checkpoint = {Path(args.ckpt).parent.name}   H = {pred.H}   "
          f"（数据 {Path(args.data).name}）")

    # ---- 段清单 ----
    # **标定 T_high 只能用「训练分布内」的段**。实测发现 M0_U1/U2/U3（掌心朝上，
    # J5=-65° 超出训练集 -10~45°）的自由空间 f_t 中位数高达 **14.8~15.2 N·m**，
    # 是分布内的 10 倍以上（外推失败）。用它标定会把 T_high 抬到 20+，
    # 导致真带载也检不出来（2026-10-21 踩到）。
    calib_eps = sorted(Path(p).name for p in
                       glob.glob(str(HERE.parent / "20261019_stage3" / "episodes" / "*")))
    extra_free = [f"M0_{x}" for x in ("A", "B", "C", "U1", "U2", "U3")]
    free_eps = calib_eps + extra_free          # 报告用（含 U 族，标注外推）
    # 带载段：只列实际存在的（M1/M2 只跑了 V 族）
    load_eps = [e for e in (f"{t}_{x}" for t in ("M1", "M2", "M3")
                            for x in ("A", "B", "C", "U1", "U2", "U3"))
                if any((d / e / "samples.jsonl").exists() for d in
                       (HERE / "episodes", HERE.parent / "20261019_stage3" / "episodes"))]

    modes = {"all": None, "good": GOOD_AXES} if args.axes == "both" else \
            {"all": None} if args.axes == "all" else {"good": GOOD_AXES}
    result = {}

    for mname, axes in modes.items():
        print(f"\n{'='*64}\n口径 {mname}：f_t 累加 {'全部 6 轴' if axes is None else 'J2~J5'}\n{'='*64}")

        # ---- ① 标定阈值：只用分布内的 42 段 ----
        fs, durs = [], []
        for ep in calib_eps:
            r = tau_ext_of(ep, pred)
            if r is None:
                continue
            te, t, n = r
            f = contact_score(te[pred.H - 1:], axes)      # 丢掉无完整历史的段首
            fs.append(f)
            durs.append((t[-1] - t[0]))
        tall = np.concatenate(fs)
        t_high = float(np.percentile(tall, args.t_high_quantile))
        t_low = float(t_high * args.t_low_ratio)
        free_minutes = sum(durs) / 60.0
        print(f"标定集：阶段 3 的 {len(fs)} 段 / {free_minutes:.1f} min（全部在 NEXT 训练分布内），"
              f"f_t 中位 {np.median(tall):.3f} P99 {t_high:.3f}")
        print(f"      T_high = {t_high:.4f}   T_low = {t_low:.4f}（{args.t_low_ratio:g}×）")

        # ---- ② 自由空间：数误报事件 ----
        print(f"\n① 自由空间的误报（真值应为 0 个接触事件）")
        print("| 段 | 时长 | f_t 中位 | 事件数 | 判 |")
        print("|---|---|---|---|---|")
        n_evt = 0
        bad = []
        for ep in free_eps:
            r = tau_ext_of(ep, pred)
            if r is None:
                continue
            te, t, _ = r
            f = contact_score(te[pred.H - 1:], axes)
            c = hysteresis(f, t_high, t_low)
            _, starts = phase_labels(c)
            n_evt += len(starts)
            if len(starts) > 0:
                bad.append((ep, len(starts)))
            print(f"| {ep} | {t[-1]-t[0]:.1f} s | {np.median(f):.3f} | {len(starts)} | "
                  f"{'✓' if len(starts)==0 else '✗'} |")
        rate = n_evt / free_minutes
        print(f"\n  **合计 {n_evt} 个误报 / {free_minutes:.1f} min = {rate:.2f} 次/分**"
              f"（门 ≤1 次/分）→ {'PASS' if rate <= 1.0 else 'FAIL'}")

        # ---- ③ 带载段：应整段判为 Contact ----
        print(f"\n② 带载段的 recall（载荷是持续接触，应几乎全程判为 Contact）")
        print("| 段 | f_t 中位 | Contact 帧占比 | 判 |")
        print("|---|---|---|---|")
        recs = []
        for ep in load_eps:
            r = tau_ext_of(ep, pred)
            if r is None:
                continue
            te, t, _ = r
            f = contact_score(te[pred.H - 1:], axes)
            c = hysteresis(f, t_high, t_low)
            rec = float(c.mean())
            recs.append(rec)
            print(f"| {ep} | {np.median(f):.3f} | {100*rec:.1f}% | {'✓' if rec>0.9 else '✗'} |")
        if recs:
            print(f"\n  **平均 recall = {100*np.mean(recs):.1f}%**")

        result[mname] = dict(t_high=t_high, t_low=t_low, free_minutes=free_minutes,
                             n_false_events=n_evt, rate_per_min=rate,
                             false_detail=bad, recall_load=recs,
                             quantile=args.t_high_quantile, low_ratio=args.t_low_ratio)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\n写入 {args.out}")
    print("\n【口径提醒】论文 f_t 用全 6 轴；我们的 J1/J6 的 K 是组外推（4.7185），")
    print("会把这两轴的电流噪声放大 4 倍。若 'all' 口径的误报明显高于 'good'，")
    print("说明是 K 外推造成的，不是检测算法问题——这一点要写进论文。")


if __name__ == "__main__":
    main()
