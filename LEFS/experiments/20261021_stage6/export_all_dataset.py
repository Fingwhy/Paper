#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导出「所有实验」的对齐数据集：实验一/二/三 的臂侧 K6·i6 vs 传感器 τ。

参数（2026-10-27 修正后）
------------------------
K6 = 0.3889 N·m/A（旧 10.7882，高估 27.74 倍）
k  = 0.001 N·m/count（旧 0.02774，仪表 Sd=3 直接读 N·m）

对齐
----
* 实验一/二：`L = 传感器 t0 − (臂 started + skew)`，已由报告反解为 −7.74 / −7.18 s
* 实验三　　：`OFF = −8.2 s`（臂 VM epoch = 传感器 Win epoch + OFF，三份 raw 互相关 −8.2/−8.3/−8.3）

输出
----
* runs/ALL_DATASET.csv   长表（实验, 段, 时间s, 臂侧, 传感器, 差值）
* runs/ALL_DATASET.md    说明 + 各段统计
"""
from __future__ import annotations

import bisect
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
K6 = 0.3889
G = 9.80665

# 实验一/二：(标签, sensor, samples, L)
EXP12 = [
    ("一·拧紧", "sensor_q1.jsonl", "q1_samples.jsonl", -7.74),
    ("二·松开", "sensor_q2.jsonl", "q2_samples.jsonl", -7.18),
]

# 实验三：(段, 方向, sensor 文件, 窗口[s], arm_raw 文件)
EXP3 = [
    ("松1", "松", "q3",  (20.0, 63.0), "q3"),
    ("松2", "松", "q3",  (63.0, 124.0), "q3"),
    ("松3", "松", "q3",  (175.0, 239.0), "q3"),
    ("紧1", "紧", "q3b", (40.0, 70.0), "q3b"),
    ("紧2", "紧", "q3b", (142.0, 200.0), "q3b"),
    ("紧3", "紧", "q3c", (0.0, 70.0), "q3c"),
]
OFF3 = -8.2


def load_sensor(f):
    lines = open(HERE / "runs" / f, encoding="utf-8").read().splitlines()
    rows = [json.loads(l) for l in lines if '"tau"' in l]
    return [r["t"] for r in rows], [r["tau"] for r in rows], rows


def load_arm_samples(f):
    rows = [json.loads(l) for l in open(HERE / "runs" / f, encoding="utf-8") if l.strip()]
    t = [r["t"] for r in rows]
    i6 = [r["cur"][5] for r in rows]
    return t, i6


def load_arm_raw(f):
    rows = [json.loads(l) for l in open(HERE / "runs" / "arm_raw" / f"arm6_{f}.jsonl")]
    return [r["t"] for r in rows], [r["i6"] for r in rows]


def interp(x, ts, vs):
    k = bisect.bisect_left(ts, x)
    if k == 0:
        return vs[0]
    if k >= len(ts):
        return vs[-1]
    t0, t1 = ts[k - 1], ts[k]
    v0, v1 = vs[k - 1], vs[k]
    return v0 if t1 == t0 else v0 + (v1 - v0) * (x - t0) / (t1 - t0)


def main():
    rows_out = []   # (实验, 段, 时间s, 臂侧, 传感器, 差值)
    stats = []

    # ---------- 实验一/二 ----------
    for tag, sf, af, L in EXP12:
        st_abs, sv, raw_rows = load_sensor(sf)
        at_rel, ai6 = load_arm_samples(af)
        s0 = st_abs[0]
        # 时间网格：传感器相对时间，20 Hz
        span = st_abs[-1] - s0
        grid = []
        x = 0.0
        while x <= span:
            grid.append(x)
            x += 0.05
        rec = []
        for g in grid:
            ta_rel = g + L                       # 臂侧相对时间 = 传感器相对时间 + L
            v_arm = interp(ta_rel, at_rel, ai6) * K6
            v_sen = interp(g + s0, st_abs, sv)
            rec.append((g, v_arm, v_sen))
            rows_out.append([tag, "全段", f"{g:.2f}", f"{v_arm:.4f}", f"{v_sen:.4f}",
                             f"{v_arm - v_sen:.4f}"])
        a = [r[1] for r in rec]; s = [r[2] for r in rec]
        n = len(rec)
        mae = sum(abs(a[i] - s[i]) for i in range(n)) / n
        r = _corr(a, s)
        stats.append((tag, "全段", n, min(a), max(a), min(s), max(s), mae, r))

    # ---------- 实验三 ----------
    for seg, dirc, sf, (lo, hi), armf in EXP3:
        st_abs, sv, _ = load_sensor(f"sensor_{sf}.jsonl")
        at_abs, ai6 = load_arm_raw(armf)
        s0 = st_abs[0]
        at_rel = [x - OFF3 - s0 for x in at_abs]   # 臂侧换算到传感器时基后的段内时间
        st_rel = [x - s0 for x in st_abs]
        grid = []
        x = lo
        while x <= hi:
            grid.append(x)
            x += 0.05
        rec = []
        for g in grid:
            v_arm = interp(g, at_rel, ai6) * K6
            v_sen = interp(g, st_rel, sv)
            rec.append((g, v_arm, v_sen))
            rows_out.append(["三·全量", seg, f"{g:.2f}", f"{v_arm:.4f}", f"{v_sen:.4f}",
                             f"{v_arm - v_sen:.4f}"])
        a = [r[1] for r in rec]; s = [r[2] for r in rec]
        n = len(rec)
        mae = sum(abs(a[i] - s[i]) for i in range(n)) / n
        r = _corr(a, s)
        stats.append(("三·全量", seg, n, min(a), max(a), min(s), max(s), mae, r))

    # ---------- 写 CSV ----------
    with open(HERE / "runs" / "ALL_DATASET.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["实验", "段", "时间s", "臂侧K6·i6(N·m)", "传感器τ(N·m)", "差值(臂-传感)"])
        w.writerows(rows_out)

    # ---------- 写 MD ----------
    md = ["# 全部实验数据集（2026-10-27 修正版）", "",
          f"- **K6 = {K6} N·m/A**（旧 10.7882，高估 27.74 倍）",
          "- **传感器 k = 0.001 N·m/count**（仪表 Sd=3 直接读 N·m；挂重 500g×120mm 验证 +0.87%）",
          "- 对齐：实验一 L=−7.74s、实验二 L=−7.18s、实验三 OFF=−8.2s",
          f"- 采样：20 Hz，共 **{len(rows_out)} 行** → `runs/ALL_DATASET.csv`", "",
          "## 各段统计", "",
          "| 实验 | 段 | 点数 | 臂侧min | 臂侧max | 传感器min | 传感器max | MAE | 相关r |",
          "|---|---|---|---|---|---|---|---|---|"]
    for tag, seg, n, amin, amax, smin, smax, mae, r in stats:
        md.append(f"| {tag} | {seg} | {n} | {amin:+.3f} | {amax:+.3f} | "
                  f"{smin:+.3f} | {smax:+.3f} | {mae:.4f} | {r:+.3f} |")

    md += ["", "## 怎么用", "",
           "```python",
           "import pandas as pd",
           "df = pd.read_csv('runs/ALL_DATASET.csv')",
           "df.groupby(['实验','段']).agg({'臂侧K6·i6(N·m)':'mean','传感器τ(N·m)':'mean'})",
           "```", "",
           "## 备注", "",
           "- 实验一/二：臂侧来自 `q1/q2_samples.jsonl`（200 Hz parse 后 50 Hz），i6 = cur[5]",
           "- 实验三：臂侧来自 `runs/arm_raw/arm6_q3*.jsonl`（VM raw 0x256 帧解析，绝对 epoch）",
           "- 传感器：`sensor_q*.jsonl` 的 `tau` 字段（已由 raw×0.001 重算）",
           "- **松方向弹开冲击**传感器能测、电流测不到，是物理盲区，不是误差"]

    (HERE / "runs" / "ALL_DATASET.md").write_text("\n".join(md), encoding="utf-8")
    print("\n".join(md))


def _corr(a, b):
    n = len(a)
    ma = sum(a) / n; mb = sum(b) / n
    cov = sum((a[i] - ma) * (b[i] - mb) for i in range(n))
    va = sum((a[i] - ma) ** 2 for i in range(n))
    vb = sum((b[i] - mb) ** 2 for i in range(n))
    return cov / ((va * vb) ** 0.5) if va * vb > 0 else 0.0


if __name__ == "__main__":
    main()
