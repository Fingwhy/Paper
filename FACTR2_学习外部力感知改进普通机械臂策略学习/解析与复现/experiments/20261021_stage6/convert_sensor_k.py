#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器历史数据换算修正：把乘过错误 k=0.02774 的 tau 恢复成原始读数。

背景（2026-10-27 验证）
--------------------
用「轴一侧挂 500g、力臂 120.0mm」做判决实验：
    理论 τ = 0.5 × 9.80665 × 0.120 = 0.5884 N·m
    仪表读数（0x56, Sd=3）= 593.5 counts -> 0.5935 N·m
    偏差 +0.87%  => 仪表直接给的就是 N·m，**不需要任何 k**（k 恒等于 0.001）
之前用的 k=0.02774 高估了 27.98 倍。

做法
----
raw（0x56 寄存器值）是原始读数，永远可信。因此统一重算：
    tau_new = raw × 0.001      （Sd=3 时 raw/1000 就是 N·m）
原文件备份到 runs/_old_k/，header 里记下换算来源，便于追溯。

用法
----
    python convert_sensor_k.py --dry-run     # 只看会改哪些、多少条
    python convert_sensor_k.py               # 真正转换（先自动备份）
"""
from __future__ import annotations

import argparse
import glob
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
K_NEW = 0.001
K_OLD = 0.02774
BACKUP = HERE / "runs" / "_old_k"


def convert_one(path: Path, dry: bool) -> dict:
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return dict(file=path.name, skipped="空文件")

    hdr = None
    out = []
    n_changed = 0
    for i, line in enumerate(lines):
        s = line.strip()
        if not s:
            continue
        try:
            d = json.loads(s)
        except json.JSONDecodeError:
            out.append(line)
            continue

        if d.get("type") == "header":
            hdr = dict(d)
            hdr["k_nm_per_count"] = K_NEW
            hdr["k_note"] = ("2026-10-27 挂重验证：仪表 Sd=3 直接读 N·m，无需 k 标定；"
                             f"此文件 tau 已由 k={K_OLD} 修正为 raw×{K_NEW}")
            hdr["tau_converted_from_k"] = K_OLD
            out.append(json.dumps(hdr, ensure_ascii=False))
            continue

        if "tau" in d and "raw" in d:
            old = d["tau"]
            new = round(d["raw"] * K_NEW, 6)
            if abs(old - new) > 1e-9:
                n_changed += 1
            d["tau"] = new
            out.append(json.dumps(d, ensure_ascii=False))
        else:
            out.append(line)

    if not dry and n_changed:
        BACKUP.mkdir(parents=True, exist_ok=True)
        bak = BACKUP / path.name
        if not bak.exists():
            shutil.copy2(path, bak)
        path.write_text("\n".join(out) + "\n", encoding="utf-8")

    old_k = hdr.get("tau_converted_from_k", "?") if hdr else "?"
    return dict(file=path.name, n=len(out), changed=n_changed, old_k=old_k)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="只统计不写文件")
    ap.add_argument("--pattern", default="runs/sensor_*.jsonl", help="匹配模式")
    a = ap.parse_args()

    files = sorted(Path(HERE).glob(a.pattern)) + [HERE / "runs" / "sync_test1_sensor.jsonl"]
    print(f"{'文件':<30} {'行数':>7} {'改动':>7}  原k")
    print("-" * 62)
    total = 0
    for f in files:
        if not f.exists():
            continue
        r = convert_one(f, a.dry_run)
        if "skipped" in r:
            print(f"{r['file']:<30} {'-':>7} {'-':>7}  {r['skipped']}")
            continue
        print(f"{r['file']:<30} {r['n']:>7} {r['changed']:>7}  {r['old_k']}")
        total += r["changed"]
    print("-" * 62)
    print(f"总计将改写 {total} 条记录" + ("（dry-run，未写盘）" if a.dry_run else ""))
    if not a.dry_run and total:
        print(f"原文件已备份到 {BACKUP.relative_to(HERE)}/")


if __name__ == "__main__":
    main()
