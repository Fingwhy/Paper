#!/usr/bin/env python3
"""判决实验：力臂到底起不起作用（倍比检验）。

原理（不依赖任何标定系数，只看比例）：

    同一重物 m，挂在两个力臂 L1、L2 上
    τ1/τ2 = L1/L2
    传感器读数变化 Δcounts1 / Δcounts2 应该 = L1/L2

    * 若比值 ≈ L1/L2  -> 力臂真的起作用，标定方法对
    * 若比值 ≈ 1       -> 力臂没起作用（重物实际挂在轴上），标定方法错

用法（Windows，D2XX）
--------------------
    # 力臂 120.2mm 与 60.1mm 各挂一次
    & $PY verify_k_ratio.py --port D2XX --l1 120.2 --l2 60.1 --mass 0.500
    # 若用 COM 口： --port COM3 --baud 115200

流程：先空载采 3 s 零点 -> 提示挂 L1 -> 采 3 s -> 提示挂 L2 -> 采 3 s -> 打印比值判定
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import torque485 as T  # noqa: E402

G = 9.80665


def sample(c, a, seconds, label):
    vals = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        v = T._read_torque(c, a)
        if v is not None:
            vals.append(v[0])
        time.sleep(a.interval)
    if not vals:
        print(f"  [{label}] 采样失败")
        return None
    mean = sum(vals) / len(vals)
    sd = (sum((x - mean) ** 2 for x in vals) / max(1, len(vals) - 1)) ** 0.5
    print(f"  [{label}] raw 均值 {mean:+.2f} counts  (n={len(vals)}, sd={sd:.2f})")
    return mean


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--parity", default="N")
    ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--interval", type=float, default=0.02)
    ap.add_argument("--l1", type=float, required=True, help="第一个力臂 mm（长的那个）")
    ap.add_argument("--l2", type=float, required=True, help="第二个力臂 mm（短的那个，建议一半）")
    ap.add_argument("--mass", type=float, required=True, help="重物质量 kg")
    ap.add_argument("--seconds", type=float, default=3.0, help="每个点采样秒数")
    ap.add_argument("--nm-per-count", type=float, default=0.001,
                    help="对比用的 k（默认现值 0.02774）")
    a = ap.parse_args()

    c = T.make_client(a.port, a.baud, a.parity, timeout=a.timeout)
    if not c.connect():
        print("串口打不开")
        return 1
    try:
        print("\n[1/3] 空载采零点：请确保杆上什么都没挂，轴水平")
        input("      准备好后按回车…")
        z = sample(c, a, a.seconds, "零点")
        if z is None:
            return 1

        print(f"\n[2/3] 挂 {a.mass*1000:.0f} g 在力臂 L1 = {a.l1} mm 处（杆端，轴水平）")
        input("      挂好后按回车…")
        s1 = sample(c, a, a.seconds, f"L1={a.l1}mm")
        if s1 is None:
            return 1

        print(f"\n[3/3] 挂同一个重物在力臂 L2 = {a.l2} mm 处（更靠近轴心）")
        input("      挂好后按回车…")
        s2 = sample(c, a, a.seconds, f"L2={a.l2}mm")
        if s2 is None:
            return 1
    finally:
        c.close()

    d1 = abs(s1 - z)
    d2 = abs(s2 - z)
    tau1 = a.mass * G * a.l1 / 1000
    tau2 = a.mass * G * a.l2 / 1000

    print("\n" + "=" * 60)
    print(f"零点 raw        = {z:+.2f}")
    print(f"L1={a.l1:.1f}mm  raw = {s1:+.2f}   Δ = {d1:.2f} counts   理论 τ = {tau1:.4f} N·m")
    print(f"L2={a.l2:.1f}mm  raw = {s2:+.2f}   Δ = {d2:.2f} counts   理论 τ = {tau2:.4f} N·m")
    print(f"理论力臂比 L1/L2 = {a.l1/a.l2:.2f}")
    if d2 > 0.5:
        r = d1 / d2
        print(f"实测计数比 Δ1/Δ2 = {r:.2f}")
        print()
        if abs(r - a.l1 / a.l2) / (a.l1 / a.l2) < 0.25:
            print("判定：比值 ≈ 力臂比  ->  力臂确实起作用，标定方法有效 ✅")
        elif abs(r - 1.0) < 0.25:
            print("判定：比值 ≈ 1  ->  力臂没起作用（重物实际挂在轴上）❌ 标定方法有问题")
        else:
            print("判定：介于两者之间，需检查（杆是否真的绕轴扭转 / 力臂量法）")
        print()
        print("附带：用现值 k 反算")
        print(f"  L1 反解 k = {tau1/d1:.5f} N·m/count" if d1 > 0.5 else "  L1 信号太小")
        print(f"  L2 反解 k = {tau2/d2:.5f} N·m/count" if d2 > 0.5 else "  L2 信号太小")
        print(f"  现值 k = {a.nm_per_count:.5f}")
    else:
        print(f"判定：L2 信号太小（Δ={d2:.2f} counts），无法算比值。换更长的 L2 重试")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
