#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""带载前只读探针：量当前六轴电流 / 姿态 / 末端位置，给出电流守卫的推荐阈值。

为什么需要
----------
`pose` 的电流守卫默认 `--max-current-a 3.0`，那是**空载**定的。
绑上 0.525 kg 水瓶以后，J2/J3 在低位保持时就可能 3~5 A，一开跑就 abort（假警报）。
这个脚本**只读、不发任何帧**，量出实际值再定阈值。

用法（VM）
----------
    python3 load_probe.py --iface can0 --sec 2
"""

import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser(description="六轴电流/姿态只读探针（不发帧）")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--sec", type=float, default=2.0, help="采样时长 s，默认 2")
    ap.add_argument("--margin-a", type=float, default=1.5,
                    help="推荐阈值 = 实测峰值 + 这个余量，默认 1.5 A")
    a = ap.parse_args()

    try:
        from piper_sdk import C_PiperInterface_V2
    except Exception as e:                                    # noqa: BLE001
        print(f"[ABORT] 导入 piper_sdk 失败：{e}", file=sys.stderr)
        return 2

    p = C_PiperInterface_V2(a.iface)
    # 0.6.1 的 ConnectPort() 方法体没有 return，返回 None。绝不能 `if not ...`，
    # 否则 None 会被当成失败（R40 那条坑）。
    if p.ConnectPort() is False:
        print("[ABORT] ConnectPort 返回 False：can0 没起来？", file=sys.stderr)
        return 2
    time.sleep(1.0)

    n = max(1, int(a.sec * 100))
    cur = [[] for _ in range(6)]
    q = [[], [], [], [], [], []]
    z = []
    for _ in range(n):
        ms = p.GetArmHighSpdInfoMsgs()
        for i in range(6):
            cur[i].append(getattr(ms, f"motor_{i+1}").current * 0.001)
        js = p.GetArmJointMsgs()
        # 注意：joint_x 的单位已经是 **0.001 度**（不是弧度）。2026-10-20 11:0x 第一版
        # 又乘了 57.29，读到 J5=1351°，一眼就是错的。这里只除 1000。
        for i in range(6):
            q[i].append(getattr(js.joint_state, f"joint_{i+1}") * 0.001)
        z.append(p.GetArmEndPoseMsgs().end_pose.Z_axis / 1000.0)
        time.sleep(0.01)

    en = p.GetArmEnableStatus()
    en = en if isinstance(en, (list, tuple)) else [en] * 6

    print(f"使能        : {all(en)}  逐轴={list(en)}")
    print(f"六轴角 (deg): [" + ", ".join(f"{sum(v)/len(v):+.3f}" for v in q) + "]")
    print(f"末端 Z (mm) : {sum(z)/len(z):+.1f}")
    print()
    print("| 轴 | 电流均值 (A) | 电流峰 | 峰峰 |")
    print("| --- | --- | --- | --- |")
    peak_all = 0.0
    for i in range(6):
        v = cur[i]
        m = sum(v) / len(v)
        lo, hi = min(v), max(v)
        peak_all = max(peak_all, abs(lo), abs(hi))
        print(f"| J{i+1} | {m:+.3f} | {max(abs(lo), abs(hi)):.3f} | {hi-lo:.3f} |")

    rec = round(peak_all + a.margin_a, 1)
    print()
    print(f"实测最大 |I| = {peak_all:.2f} A  ⇒  推荐 --max-current-a {rec}")
    print(f"（默认 3.0 A 是空载定的；带载后必须按上面这个数给，否则一开跑就假 abort）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
