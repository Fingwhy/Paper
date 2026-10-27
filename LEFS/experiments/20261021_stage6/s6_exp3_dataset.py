#!/usr/bin/env python3
"""实验三·全量 —— 逐点对齐数据集（先出数据，再出图）。

对齐：两机时钟偏差 OFF = -8.2s（臂侧 VM epoch = 传感器 Windows epoch + OFF）。
三份 raw 独立互相关交叉验证：q3 -8.2 / q3b -8.3 / q3c -8.3，r = 0.74~0.82。

输出：
1. `runs/exp3_dataset.md`   每段对齐后的逐点数据集（同一时间轴，臂侧 vs 传感器）
2. `runs/exp3_dataset.csv`  同上，CSV 便于核对
"""
from __future__ import annotations

import bisect
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
K6 = 0.3889
OFF = -8.2   # 臂侧 VM epoch = 传感器 Windows epoch + OFF

SEGMENTS = [
    ("松1", "松", "q3",  (20.0, 63.0), "q3"),
    ("松2", "松", "q3",  (63.0, 124.0), "q3"),
    ("松3", "松", "q3",  (175.0, 239.0), "q3"),
    ("紧1", "紧", "q3b", (40.0, 70.0), "q3b"),
    ("紧2", "紧", "q3b", (142.0, 200.0), "q3b"),
    ("紧3", "紧", "q3c", (0.0, 70.0), "q3c"),
]


def load_arm(f):
    rows = [json.loads(l) for l in open(HERE / "runs" / "arm_raw" / f"arm6_{f}.jsonl")]
    t = [r["t"] for r in rows]
    tau = [r["i6"] * K6 for r in rows]
    return t, tau


def load_sen(f):
    lines = open(HERE / "runs" / f"sensor_{f}.jsonl", encoding="utf-8").read().splitlines()
    rows = [json.loads(l) for l in lines if '"tau"' in l]
    t = [r["t"] for r in rows]
    tau = [r["tau"] for r in rows]
    return t, tau


def interp(t_target, t_src, v_src):
    k = bisect.bisect_left(t_src, t_target)
    if k == 0:
        return v_src[0]
    if k >= len(t_src):
        return v_src[-1]
    t0, t1 = t_src[k - 1], t_src[k]
    v0, v1 = v_src[k - 1], v_src[k]
    if t1 == t0:
        return v0
    return v0 + (v1 - v0) * (t_target - t0) / (t1 - t0)


def main():
    md = ["# 实验三 · 全量 —— 逐点对齐数据集", "",
          f"- K6 = {K6:.4f} N·m/A（冻结）",
          f"- 对齐偏差 OFF = {OFF:+.1f}s（臂侧 VM epoch = 传感器 Windows epoch + OFF）",
          "- 数据源：臂侧 = VM `raw/q3*.jsonl` 的 0x256 帧解析 J6 电流 × K6；传感器 = `runs/sensor_q3*.jsonl`",
          "- 时间轴：统一用「传感器 Windows epoch − 传感器 t0」的段内时间（秒）", ""]

    csv_rows = [["段", "方向", "段内时间s", "臂侧K6·i6(N·m)", "传感器τ(N·m)", "差值(臂-传感)"]]

    for label, dirc, sf, (lo, hi), armf in SEGMENTS:
        at_abs, av = load_arm(armf)
        st_abs, sv = load_sen(sf)

        # 对齐到传感器时基：臂侧 t 转换成 Windows 时基 = 臂侧 VM epoch - OFF
        # OFF=-8.2 => 臂侧Win = 臂侧VM - (-8.2) = 臂侧VM + 8.2
        at_win = [x - OFF for x in at_abs]

        # 段内时间（相对传感器 t0）
        t0_sen = st_abs[0]
        st = [x - t0_sen for x in st_abs]
        at = [x - t0_sen for x in at_win]

        # 截取段窗口
        m_sen = [i for i in range(len(st)) if lo <= st[i] <= hi]
        m_arm = [i for i in range(len(at)) if lo <= at[i] <= hi]
        ts = [st[i] for i in m_sen]; vs = [sv[i] for i in m_sen]
        ta = [at[i] for i in m_arm]; va = [av[i] for i in m_arm]

        if len(ta) < 3 or len(ts) < 3:
            md.append(f"## {label} —— 数据不足，跳过")
            continue

        # 输出数据集：用 0.5s 网格，臂侧插值到传感器网格（传感器是较稀的 50Hz）
        md.append(f"## {label}　·　{dirc}方向　（段内 {lo:.0f}~{hi:.0f}s，共 {len(ts)} 个传感器点 / {len(ta)} 个臂侧点）")
        md.append("")
        md.append("| 段内时间s | 臂侧 K6·i6 (N·m) | 传感器 τ (N·m) | 差值 (臂−传感) |")
        md.append("|---|---|---|---|")

        # 传感器网格，每 0.5s 取一个（避免表格过长）
        grid = [x for x in ts if (x - lo) % 0.5 < 0.02]
        if len(grid) > 40:
            grid = grid[:: max(1, len(grid) // 40)]
        for g in grid:
            v_sen = interp(g, ts, vs)
            v_arm = interp(g, ta, va)
            md.append(f"| {g:6.2f} | {v_arm:+7.2f} | {v_sen:+7.2f} | {v_arm - v_sen:+6.2f} |")
            csv_rows.append([label, dirc, f"{g:.2f}", f"{v_arm:.3f}", f"{v_sen:.3f}", f"{v_arm - v_sen:.3f}"])
        md.append("")

        # 峰值/均值
        if dirc == "松":
            a_peak = min(va); s_peak = min(vs)
        else:
            a_peak = max(va); s_peak = max(vs)
        a_mean = sum(va) / len(va); s_mean = sum(vs) / len(vs)
        md.append(f"- 臂侧峰值 {a_peak:+.2f} N·m，均值 {a_mean:+.2f} N·m")
        md.append(f"- 传感器峰值 {s_peak:+.2f} N·m，均值 {s_mean:+.2f} N·m")
        md.append(f"- 峰值比（臂/传感）= {abs(a_peak) / abs(s_peak):.2f}")
        md.append("")

    (HERE / "runs" / "exp3_dataset.md").write_text("\n".join(md), encoding="utf-8")
    with open(HERE / "runs" / "exp3_dataset.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerows(csv_rows)
    print("\n".join(md))
    print(f"\n已输出 runs/exp3_dataset.md（{len(csv_rows) - 1} 行数据）与 runs/exp3_dataset.csv")


if __name__ == "__main__":
    main()
