#!/usr/bin/env python
"""FIRST 第 1 步落地：把拧盖记录切成 (观测历史, 动作块) 样本，并做接触分相。

运行机器与目录（Windows 本机）
------------------------------
    C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6/

论文原文（§5 Segmenting Trajectories into Contact Phases）
--------------------------------------------------------
    f_t = ||τ̂_ext,t||_1                                （标量接触分数）
    c_t = 1   if f_t ≥ T_high
    c_t = 0   if f_t ≤ T_low
    c_t = c_{t-1}   otherwise                          （滞回，避免抖动）
    T_onset = { t | c_{t-1}=0, c_t=1 }
    s_t = Contact       if c_t = 1
          Pre-Contact   if ∃t_onset: t_onset-F ≤ t < t_onset   （F = 控制频率 = 1 s）
          Free Space    otherwise

本数据集的三处口径偏离（必须写明，写报告时不能含糊）
--------------------------------------------------
1. **f_t 只用 J6，不是 6 轴 L1 范数**。原因：NEXT 在拧盖姿态下是分布外
   （J1=150° 超训练范围 15 倍、J6=170° 超 80°），6 轴 τ̂_ext 不可信（曾达 15 N·m）。
   而拧盖的外部扭矩本来就只作用在 J6 上，所以 f_t = |K6·i6| − τ_f 是这个任务里
   物理上正确的接触分数。
2. **T_high / T_low 沿用项目既有约定**：取自由空间数据（阶段 3 的 42 段自由运动）
   上 f_t 的 P99.9 作为 T_high，T_low = 0.6·T_high。论文没给阈值确定方法，
   这是我们之前的空白填补方案，此处保持同一口径以便比较。
3. **动作只有 J6 一个轴**：原始记录只落了 qcmd6（`stage1_l0.py record` 的字段限制），
   J1~J5 在拧盖时锁位不动。所以动作块是 qcmd6 的未来增量，不是 6 轴。

用法
----
    python s6_first_dataset.py                 # 只报告分相构成
    python s6_first_dataset.py --save data/first_ds.npz
"""
from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"

# ---- 物理常数（见 MASTER_INDEX §1）----
K6 = 0.3889          # N·m/A，J6 力矩常数（正反转差分标定，σ/K 4.6%）
TAU_F6 = 0.86         # N·m，J6 摩擦（空载 ΔI·K6/2）

# ---- 序列口径 ----
FPS = 100.0           # 重采样到 100 Hz（原始记录 174~187 Hz）
H = 50                # 观测历史 = 0.5 s（论文 NEXT 用 50 步）
K_CHUNK = 10          # 动作块 = 0.1 s
F_PRECONTACT = int(FPS)   # 1 s，论文里 F = 控制频率


# ---------------------------------------------------------------- 工具
def load_free_space_i6() -> np.ndarray:
    """阶段 3 的 42 段自由空间记录里的 |i6|，用来定阈值。"""
    vals = []
    for p in sorted(glob.glob(str(STAGE3 / "episodes" / "*" / "samples.jsonl"))):
        for line in open(p, encoding="utf-8"):
            r = json.loads(line)
            c = r.get("cur")
            if c and len(c) > 5 and c[5] is not None:
                vals.append(abs(float(c[5])))
    return np.asarray(vals, dtype=np.float64)


def contact_score(i6: np.ndarray) -> np.ndarray:
    """f_t = |K6·i6| − τ_f，下限截到 0（负值表示低于摩擦，等价于无外部力矩）。"""
    return np.maximum(K6 * np.abs(i6) - TAU_F6, 0.0)


def hysteresis(f: np.ndarray, t_high: float, t_low: float) -> np.ndarray:
    """施密特触发：过 T_high 置 1，落 T_low 以下清 0，中间保持。"""
    c = np.zeros(len(f), dtype=np.int8)
    on = False
    for i, v in enumerate(f):
        if on:
            if v <= t_low:
                on = False
        else:
            if v >= t_high:
                on = True
        c[i] = 1 if on else 0
    return c


def phase_labels(c: np.ndarray) -> np.ndarray:
    """0 = Free Space, 1 = Pre-Contact, 2 = Contact（论文式 (7)）。"""
    s = np.where(c == 1, 2, 0).astype(np.int8)
    onsets = np.where((c[1:] == 1) & (c[:-1] == 0))[0] + 1
    for t_on in onsets:
        lo = max(0, t_on - F_PRECONTACT)
        # 只覆盖原本是 Free Space 的帧，不把更早的 Contact 翻成 PC
        seg = slice(lo, t_on)
        s[seg] = np.where(s[seg] == 0, 1, s[seg])
    return s


def clean_contact(c: np.ndarray, min_on: int, min_off: int) -> np.ndarray:
    """形态学清理：去掉过短的接触段、填掉过短的断开。

    为什么必须做 —— 拧盖时 J6 是 stick-slip 运动，接触分数会在阈值附近以 ~15 Hz
    反复穿越，滞回也压不住（实测每段 150~190 个「事件」）。论文的 FIRST 假设接触
    事件是**离散且稀疏**的（它们每个任务只有几次），所以必须在滞回之后再要求
    「持续足够久才算一次接触」。min_on/min_off 就是给这个假设一个可量化的门槛，
    论文没有写，这是我们补的口径。
    """
    c = c.copy()
    # 1) 填空洞：连续 0 段长度 < min_off ⇒ 认为没真正断开，填成 1
    i = 0
    n = len(c)
    while i < n:
        if c[i] == 0:
            j = i
            while j < n and c[j] == 0:
                j += 1
            if 0 < i and j < n and (j - i) < min_off:
                c[i:j] = 1
            i = j
        else:
            i += 1
    # 2) 去毛刺：连续 1 段长度 < min_on ⇒ 认为只是尖峰，清成 0
    i = 0
    while i < n:
        if c[i] == 1:
            j = i
            while j < n and c[j] == 1:
                j += 1
            if (j - i) < min_on:
                c[i:j] = 0
            i = j
        else:
            i += 1
    return c


def load_episode(csv_path: Path) -> dict:
    import pandas as pd
    d = pd.read_csv(csv_path)
    return {
        "name": csv_path.stem,
        "t": d["t_s"].to_numpy(dtype=np.float64),
        "q": d[[f"q{i}" for i in range(1, 7)]].to_numpy(dtype=np.float64),      # deg
        "dq": d[[f"dq{i}" for i in range(1, 7)]].to_numpy(dtype=np.float64),    # deg/s
        "qcmd6": d["qcmd6"].to_numpy(dtype=np.float64),                        # deg
        "i6": d["i6"].to_numpy(dtype=np.float64),
        "grip_ang": d["grip_ang_mm"].to_numpy(dtype=np.float64),
        "grip_eff": d["grip_eff_nm"].to_numpy(dtype=np.float64),
    }


def resample(ep: dict, fps: float) -> dict:
    t = ep["t"]
    t0, t1 = t[0], t[-1]
    n = int(np.floor((t1 - t0) * fps)) + 1
    tn = t0 + np.arange(n) / fps
    out = {"t": tn - t0}
    for k in ("q", "dq", "qcmd6", "i6", "grip_ang", "grip_eff"):
        v = ep[k]
        if v.ndim == 1:
            out[k] = np.interp(tn, t, v)
        else:
            out[k] = np.stack([np.interp(tn, t, v[:, j]) for j in range(v.shape[1])], axis=1)
    return out


def make_samples(ep: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(观测, 动作块, 起始帧索引)。

    观测 = 过去 H 帧的 [q(6), dq(6), f_t(1), grip_ang(1), grip_eff(1)] = 15 维
    动作 = 未来 K_CHUNK 步的 Δqcmd6（增量，绝对指令是斜坡，增量才是近似常值）
    """
    q = ep["q"]
    dq = ep["dq"]
    f = contact_score(ep["i6"])[:, None]
    g = np.stack([ep["grip_ang"], ep["grip_eff"]], axis=1)
    state = np.concatenate([q, dq, f, g], axis=1).astype(np.float32)   # (T,15)
    qcmd = ep["qcmd6"].astype(np.float32)
    T = len(state)
    idx = np.arange(H - 1, T - K_CHUNK)
    obs = np.stack([state[i - H + 1:i + 1].ravel() for i in idx])
    act = np.stack([np.diff(qcmd[i:i + K_CHUNK + 1]) for i in idx])
    return obs, act, idx


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default=str(HERE / "data" / "[1-5].*.csv"))
    ap.add_argument("--fps", type=float, default=FPS)
    ap.add_argument("--save", default="")
    ap.add_argument("--thigh", type=float, default=None, help="手动指定 T_high（默认自由空间 P99.9）")
    ap.add_argument("--min-on", type=float, default=0.30, help="一次接触至少要持续多少秒（默认 0.3）")
    ap.add_argument("--min-off", type=float, default=0.30, help="断开至少要持续多少秒（默认 0.3）")
    a = ap.parse_args()

    files = sorted(glob.glob(a.glob))
    if not files:
        print("没找到数据文件：", a.glob)
        return 1

    # 1) 阈值
    i6_free = load_free_space_i6()
    f_free = contact_score(i6_free)
    t_high = a.thigh if a.thigh is not None else float(np.percentile(f_free, 99.9))
    t_low = 0.6 * t_high
    print("自由空间（阶段 3，42 段，n=%d）的接触分数 f_t：" % len(f_free))
    print("  P50 %.3f   P99 %.3f   P99.9 %.3f   max %.3f N·m"
          % (np.percentile(f_free, 50), np.percentile(f_free, 99),
             np.percentile(f_free, 99.9), f_free.max()))
    print("阈值：T_high = %.3f   T_low = %.3f N·m（T_high 取自由空间 P99.9）\n"
          % (t_high, t_low))

    # 2) 逐段
    min_on = int(round(a.min_on * a.fps))
    min_off = int(round(a.min_off * a.fps))
    print("清理口径：接触至少持续 %.2f s（%d 帧），断开至少 %.2f s（%d 帧）\n"
          % (a.min_on, min_on, a.min_off, min_off))
    print("%-30s %6s %7s %7s %7s %6s %6s %7s" %
          ("段", "帧数", "F 占比", "PC 占比", "C 占比", "原始事件", "清理后", "f 中位"))
    obs_all, act_all, ph_all, ep_all = [], [], [], []
    for i, fp in enumerate(files):
        ep = resample(load_episode(Path(fp)), a.fps)
        f = contact_score(ep["i6"])
        c_raw = hysteresis(f, t_high, t_low)
        ev_raw = int(((c_raw[1:] == 1) & (c_raw[:-1] == 0)).sum())
        c = clean_contact(c_raw, min_on, min_off)
        s = phase_labels(c)
        n = len(s)
        nF, nPC, nC = [int((s == v).sum()) for v in (0, 1, 2)]
        ev = int(((c[1:] == 1) & (c[:-1] == 0)).sum())
        print("%-30s %6d %6.1f%% %6.1f%% %6.1f%% %7d %6d %7.2f"
              % (Path(fp).stem[:30], n, 100 * nF / n, 100 * nPC / n, 100 * nC / n,
                 ev_raw, ev, np.median(f)))
        o, ac, idx = make_samples(ep)
        obs_all.append(o); act_all.append(ac)
        ph_all.append(s[idx]); ep_all.append(np.full(len(idx), i, dtype=np.int16))

    obs = np.concatenate(obs_all); act = np.concatenate(act_all)
    ph = np.concatenate(ph_all); epid = np.concatenate(ep_all)
    tot = len(ph)
    print("\n合计 %d 段 / %d 个样本（观测 %d 维，动作 %d 维）" % (len(files), tot, obs.shape[1], act.shape[1]))
    for v, nm in ((0, "Free Space"), (1, "Pre-Contact"), (2, "Contact")):
        m = ph == v
        print("  %-12s %6d  %5.1f%%" % (nm, m.sum(), 100 * m.sum() / tot))

    if a.save:
        out = HERE / a.save if not os.path.isabs(a.save) else Path(a.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out, obs=obs, act=act, phase=ph, ep=epid,
                            names=np.array([Path(f).stem for f in files]),
                            t_high=t_high, t_low=t_low)
        print("\n已保存", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
