#!/usr/bin/env python3
"""阶段 4 第 1 步：把 42 段 jsonl 构造成 NEXT 的训练张量。

设计依据（指南 §7.1 六步的前两步）
----------------------------------
1. **核验字段、单位、同步与有效掩码** —— 全部在 `--report` 里打印，不靠嘴说。
2. **按冻结切分生成因果窗口，标签取窗口最后时刻的 τ_m** —— 窗口在这里不切，
   只按段存 `(N, 18)` 的 step 数组 + `ep_start` 索引；切窗交给 Dataset，
   这样**天然保证窗口不跨 episode**（禁令一）。

三个必须记住的口径
------------------
* **标签 τ_m = K_j · I_j**，K 来自阶段 2 `calibration/k_table.json`。
  J2=J3=4.7185、J4=J5=0.7446 是**实测**；J1/J6 未标定，按 SDK 的组结构
  （J1–J3 同组、J4–J6 同组，且实测 K2=K3、K4=K5 完全相等，组内等系数被数据接受）
  **外推**成 4.7185 / 0.7446。⇒ J1/J6 的 τ_m 含「推定」成分，评估时单列。
* **符号取正**（τ_m = +K·I）。依据：阶段 2 差分法 `ΔI = τ_pay/K + δ` 拟合出 K>0，
  χ²/dof = 17.97/31；且 J2（承载主力轴、URDF 相对关系最可靠）的 4 个静止姿态
  K·I 与 URDF 重力矩**全部同号**，而 J4/J5 全部反号 —— 后者正是阶段 2 已判定的
  「URDF 腕部质量错」。**不能拿 URDF 当绝对符号基准**（R41）。
* **段首截断**：每段前 11~35% 是 pre-pose 接近段，`q_cmd` 为 None
  （`flags=cmd_interrupted`）。第三项输入 `q_cmd − q` 在这些帧算不出来，
  从**第一个 q_cmd 非空**的帧开始保留。这里丢的是「臂从上一姿态挪到本段 center」，
  不是本段本体。

用法（本机，LEFS/experiments/20261020_stage4）：
    python s4_build.py --report      # 只看统计，不写文件
    python s4_build.py               # 写 data/step.npz
"""

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"
# 段可以来自两处：阶段 3 的 42 段 + 阶段 4 补采的稀疏区段。
EP_DIRS = [STAGE3 / "episodes", HERE / "episodes"]
SPLITS = STAGE3 / "splits.json"          # 默认；--splits 可覆盖为 splits_v2.json

# 阶段 2 主交付的 K（N·m/A）。J1/J6 为组外推，state 标注在 K_STATE 里。
K = np.array([4.7185, 4.7185, 4.7185, 0.7446, 0.7446, 0.7446], dtype=np.float64)
K_STATE = ["外推(组)", "实测", "实测", "弱约束", "弱约束", "外推(组)"]
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]

# 实质标记：出现即该帧不可用于训练。qcmd_out_of_window 是**无害**的
# （录制先起/后停，用最近指令保持），它跟 no_q_cmd 一起写在 stage1_l0.BENIGN_FLAGS 里。
HARD_FLAGS = ("cmd_interrupted", "stale:q_cmd")

HISTORY = 25          # 50 Hz × 0.5 s（论文 H=50 是 100 Hz 下的同一时长）


def ep_path(ep):
    for d in EP_DIRS:
        p = d / ep / "samples.jsonl"
        if p.exists():
            return p
    raise FileNotFoundError(f"找不到段 {ep}（查过 {[str(d) for d in EP_DIRS]}）")


def load_episode(ep):
    rows = [json.loads(l) for l in open(ep_path(ep), encoding="utf-8")]
    return rows


def build_episode(ep, verbose=False):
    """返回 (x, y, t, info)。x 形状 (N,18) = [q, dq, q_cmd-q]。"""
    rows = load_episode(ep)
    n_raw = len(rows)

    # --- 掩码 1：四路字段齐全 ---
    ok = []
    for r in rows:
        if r.get("q") is None or r.get("dq") is None or r.get("cur") is None:
            continue
        if any(f in r.get("flags", ()) for f in HARD_FLAGS):
            continue
        ok.append(r)
    # --- 掩码 2：找第一个 q_cmd 非空的帧，段首接近段整段丢掉 ---
    first = next((i for i, r in enumerate(ok) if r.get("q_cmd") is not None), None)
    if first is None:
        return None
    ok = ok[first:]
    if len(ok) <= HISTORY:
        return None

    q = np.array([r["q"] for r in ok], dtype=np.float64)
    dq = np.array([r["dq"] for r in ok], dtype=np.float64)
    cmd = np.array([r["q_cmd"] for r in ok], dtype=np.float64)
    cur = np.array([r["cur"] for r in ok], dtype=np.float64)
    t = np.array([r["t"] for r in ok], dtype=np.float64)

    x = np.concatenate([q, dq, cmd - q], axis=1).astype(np.float32)
    y = (cur * K[None, :]).astype(np.float32)
    info = dict(ep=ep, n_raw=n_raw, n_used=len(ok), dropped_head=first,
                ratio=len(ok) / n_raw)
    if verbose:
        dt = np.diff(t)
        print(f"  {ep:12s} 原 {n_raw:5d} → 用 {len(ok):5d}（丢段首 {first:4d} 帧, "
              f"保留 {100*len(ok)/n_raw:4.1f}%）  dt 中位 {np.median(dt)*1000:.1f} ms")
    return x, y, t, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="store_true", help="只报告，不写文件")
    ap.add_argument("--out", default=str(HERE / "data" / "step.npz"))
    ap.add_argument("--splits", default=str(SPLITS), help="切分清单，默认阶段 3 的 splits.json；"
                                                         "分层重切后用 splits_v2.json")
    ap.add_argument("--verbose", action="store_true", help="逐段打印")
    ap.add_argument("--history", type=int, default=HISTORY,
                    help=f"历史窗口步数，默认 {HISTORY}（50 Hz × 0.5 s）。"
                         "论文 H=50 是 100 Hz 下的 0.5 s；我们 50 Hz 下要等时长应是 25，"
                         "但论文 A.2 消融显示 H 越大越好（10→25→50 单调改善），"
                         "所以另设 H=50（1.0 s）、H=75（1.5 s）做对照")
    args = ap.parse_args()

    sp = json.load(open(args.splits, encoding="utf-8"))
    sets = sp["sets"]
    print("切分清单：", {k: len(v) for k, v in sets.items()},
          " sha256 =", str(sp.get("hash_sha256", "?"))[:16], "…",
          f"（{Path(args.splits).name}）")
    print("标签：τ_m = K·I，K =", dict(zip(JN, K.tolist())))
    print("K 状态：", dict(zip(JN, K_STATE)))
    print()

    store = {}
    for split in ("train", "val", "test"):
        xs, ys, ts, starts, names, infos = [], [], [], [0], [], []
        for ep in sets[split]:
            got = build_episode(ep, verbose=args.verbose)
            if got is None:
                print(f"  [跳过] {ep} 无可用帧")
                continue
            x, y, t, info = got
            xs.append(x); ys.append(y); ts.append(t)
            starts.append(starts[-1] + len(x))
            names.append(ep); infos.append(info)
        X = np.concatenate(xs); Y = np.concatenate(ys); T = np.concatenate(ts)
        store[split] = dict(X=X, Y=Y, T=T, ep_start=np.array(starts, np.int64),
                            ep_name=np.array(names),
                            ep_used=np.array([i["n_used"] for i in infos], np.int64),
                            ep_raw=np.array([i["n_raw"] for i in infos], np.int64))
        print(f"{split:5s}: {X.shape[0]:6d} 帧 / {len(names)} 段 / "
              f"{sum(i['n_used'] for i in infos)/50:.1f} s（各段 t 独立从 0 起，不能跨段相减）")

    print()
    print("## 字段与单位核验")
    for split in ("train",):
        X, Y = store[split]["X"], store[split]["Y"]
        print("  x 三块 [q, dq, q_cmd-q] 各 6 维，共 %d 维" % X.shape[1])
        for name, sl in (("q (rad)", slice(0, 6)), ("dq (rad/s)", slice(6, 12)),
                         ("q_cmd-q (rad)", slice(12, 18))):
            blk = X[:, sl]
            print(f"  {name:16s} 范围 [{blk.min():+.4f}, {blk.max():+.4f}]  "
                  f"std {blk.std(axis=0).round(4).tolist()}")
        print("  y = τ_m (N·m)  逐轴范围/标准差：")
        for j in range(6):
            v = Y[:, j]
            print(f"    {JN[j]}  [{v.min():+7.3f}, {v.max():+7.3f}]  "
                  f"mean {v.mean():+7.3f}  std {v.std():6.3f}  ({K_STATE[j]})")

    # ---- dq 单位旁证：正弦段速度峰值应等于 2πf·amp ----
    print()
    print("## dq 单位旁证（正弦段：速度峰值理论 = 2πf·amp）")
    for ep, f, amp in (("S1_2_lo", 0.10, 6.0), ("S1_2_hi", 0.35, 6.0),
                       ("S1_3_lo", 0.10, 6.0), ("S1_4_lo", 0.10, 6.0)):
        res = build_episode(ep)
        if res is None:
            continue
        x, y, t, info = res
        try:
            js = {"S1_2_lo": 1, "S1_2_hi": 1, "S1_3_lo": 2, "S1_4_lo": 3}[ep]
        except KeyError:
            continue
        dq_peak = np.abs(x[:, 6 + js]).max()
        theory = 2 * np.pi * f * np.deg2rad(amp)
        print(f"  {ep:9s} J{js+1} dq 峰值 {dq_peak:.4f} rad/s，理论 {theory:.4f}，"
              f"比 {dq_peak/theory:.2f}")

    if args.report:
        print("\n（--report：未写文件）")
        return

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out,
        history=args.history,
        K=K, K_state=np.array(K_STATE), joint=np.array(JN),
        **{f"{s}_{k}": v for s, dd in store.items() for k, v in dd.items()
           if k in ("X", "Y", "T", "ep_start", "ep_name", "ep_used", "ep_raw")},
    )
    print(f"\n写入 {out}  ({out.stat().st_size/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
