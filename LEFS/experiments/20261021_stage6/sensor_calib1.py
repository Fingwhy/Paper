#!/usr/bin/env python
"""传感器单点标定：不碰仪表、不用砝码，只用水瓶 + 钢尺。

运行机器与目录（Windows 本机）
------------------------------
    C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6/

为什么不用仪表的 SH
------------------
说明书里 SH 是**标定动作**（加上负载、输入显示值），不是可读参数，读不到当前值；
而且它要求负载 ≥ 半量程（2.5 N·m），改坏了也回不去（CLR 备份是厂家内部用的）。
所以这里改用软件系数：只记录两个读数，算一个乘数，仪表纹丝不动。

原理
----
读数与力矩线性：  τ = (S − S0) × k

* S0 = 空载读数（含零点偏移）
* S1 = 挂已知力矩 τ_known 时的读数
* k  = τ_known / (S1 − S0)

用法
----
    # 1) 先让传感器空载，然后跑（会先采 3 s 零点）
    python sensor_calib1.py --port COM3

    # 挂上已知力矩后按回车，再输入水重与力臂（或直接输入已知力矩）
    # 2) 之后所有读数都加 --nm-per-count <k>

已知力矩怎么来：τ = m · g · L（水瓶 500 ml ≈ 0.500 kg，L 用钢尺量到轴心）
    0.500 kg × 0.200 m × 9.80665 = 0.981 N·m
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


def sample_mean(c, a, seconds=3.0):
    vals, raw = [], []
    t_end = time.time() + seconds
    while time.time() < t_end:
        v = T._read_torque(c, a)
        if v is not None:
            raw.append(v[0])
            vals.append(v[1])
        time.sleep(0.02)
    if not vals:
        return None, None, 0
    return sum(raw) / len(raw), sum(vals) / len(vals), len(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--parity", default="N")
    ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--span", type=float, default=5.0)
    ap.add_argument("--full-display", type=float, default=5.0)
    ap.add_argument("--sec", type=float, default=3.0, help="每次采样秒数")
    a = ap.parse_args()

    c = T.make_client(a.port, a.baud, a.parity, timeout=0.3)
    if not c.connect():
        print("串口打不开：", a.port)
        return 1

    try:
        print("第 1 步：请确认传感器**空载**（轴水平、不挂任何东西）")
        input("  准备好后按回车采样零点…")
        S0r, S0, n0 = sample_mean(c, a, a.sec)
        if S0 is None:
            print("  采样失败")
            return 1
        print(f"  空载：raw={S0r:.1f}  读数={S0:+.4f}（按仪表口径）  样本 {n0}\n")

        print("第 2 步：挂上已知力矩（水瓶挂在轴上的水平杆端）")
        s = input("  输入 水重kg,力臂m  （例 0.5,0.2）；或直接输入已知力矩 N·m：").strip()
        if not s:
            print("  没输入，退出")
            return 1
        if "," in s:
            m, L = [float(x) for x in s.split(",")]
            tau_known = m * G * L
            print(f"  已知力矩 = {m:g} × 9.80665 × {L:g} = {tau_known:.4f} N·m")
        else:
            tau_known = float(s)
            print(f"  已知力矩 = {tau_known:.4f} N·m")
        if abs(tau_known) > a.span:
            print(f"  ！超过量程 ±{a.span} N·m，请减小力臂或水重")
            return 1
        input("  挂好并稳定后按回车采样…")
        S1r, S1, n1 = sample_mean(c, a, a.sec)
        if S1 is None:
            print("  采样失败")
            return 1
        print(f"  加载：raw={S1r:.1f}  读数={S1:+.4f}  样本 {n1}\n")

        d = S1 - S0
        if abs(d) < 1e-9 or abs(S1r - S0r) < 1:
            print("！两次读数几乎一样 —— 力矩没进传感器。查：轴是否水平、杆是否夹紧、盖子/转接头是否连上")
            return 2
        # 用 raw 差值算更稳（不受默认换算口径影响）
        k = tau_known / (S1r - S0r)                    # N·m / count
        tau0 = -S0r * k                                # 零点补偿
        resid = (S1r - S0r) * k - tau_known
        print("=" * 62)
        print(f"每 count 对应：  k = {k:.6f} N·m/count")
        print(f"                         （默认假设是 0.001000，相差 {k / 0.001:.2f} 倍）")
        print(f"零点补偿：        --zero-nm {tau0:+.4f}")
        print(f"校验残差：        {resid:+.5f} N·m")
        print()
        print("以后读数都加这两个参数：")
        print(f"  python torque485.py --port COM3 --baud 115200 --addr 1 "
              f"--nm-per-count {k:.6f} --zero-nm {tau0:+.4f} cypy")
        print(f"  python sensor_rec.py --port COM3 --nm-per-count {k:.6f} "
              f"--zero-nm {tau0:+.4f} --seconds 60 --out runs/x.jsonl")
        print("=" * 62)
        print()
        print("建议再做一个点验证（换个力臂，例如 0.1 m），读数应线性。")
    finally:
        c.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
