#!/usr/bin/env python3
"""夹爪致动诊断（走 piper_sdk，含 CAN_CTRL 模式切换）。

为什么另起一个脚本，不直接改 stage0_gripper_check.py：
那个脚本用自建 socket 收发（单进程、逐帧判成败、失败不推进轨迹），
而「切控制模式」必须调 MotionCtrl_2(0x151)。两套收发混在一个进程里会互相抢总线（R30）。
所以这里整条链路走 SDK，只回答一个问题：**夹爪收到指令到底会不会动**。

2026-10-20 背景：`stage0_gripper_check.py actuate --span 5000` 报告「未跟随」，
但那时 bit6 已经读不到使能、也没切 CAN 指令控制模式。三种可能没被分开：
  ① 没进 CAN_CTRL，0x159 被忽略；
  ② 机械卡住（这才是「装没装到位」的真问题）；
  ③ 编码器坏了（指头会动但读数不动）。

本脚本把 ①②③ 拆开：它先切模式、再清错、再使能，然后**慢速张大到 50 mm 再收回**，
每 0.2 s 打印一行。跑的时候人盯着夹爪指头：
  - 指头动 + 读数动  → 正常，之前的「未跟随」是模式问题；
  - 指头动 + 读数不动 → 编码器/反馈问题，机械没问题；
  - 指头不动 + 读数不动 → 机械卡死或驱动器不执行，属「没装到位」。

用法（VM，~/stage0/20261019_stage0/scripts/）：
    python3 stage0_gripper_probe_sdk.py
    python3 stage0_gripper_probe_sdk.py --to-mm 20 --ramp-s 2
"""
from __future__ import annotations

import argparse
import sys
import time

UNIT = 0.001  # 0.001 mm / count
STROKE_MAX = 70.0  # mm，官方行程上限


def read_gripper(p) -> dict:
    m = p.GetArmGripperMsgs()
    st = m.gripper_state
    raw = int(getattr(st, "grippers_angle", 0))
    return {"raw": raw, "mm": raw * UNIT,
            "effort": int(getattr(st, "grippers_effort", 0)) * 0.001,
            "status": int(getattr(st, "status_code", 0)),
            "hz": float(getattr(m, "Hz", 0.0) or 0.0)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="夹爪致动诊断（SDK 路径 + CAN_CTRL 切换）")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--to-mm", type=float, default=50.0, help="张开目标行程，mm")
    ap.add_argument("--ramp-s", type=float, default=3.0, help="张开/收回各自的斜坡时长")
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--effort", type=int, default=1000, help="0.001 N·m，默认 1 N·m")
    ap.add_argument("--print-every", type=float, default=0.2)
    a = ap.parse_args(argv)

    to_raw = int(round(min(a.to_mm, STROKE_MAX) / UNIT))

    from piper_sdk import C_PiperInterface_V2

    p = C_PiperInterface_V2(a.iface)
    p.ConnectPort()
    time.sleep(0.5)  # 等首帧，否则读到默认值（全 0）

    g0 = read_gripper(p)
    print(f"[init ] 行程 {g0['mm']:.2f} mm  力矩 {g0['effort']:.3f} N·m  "
          f"status 0x{g0['status']:02X}  Hz {g0['hz']:.1f}")
    if g0["hz"] < 50 and g0["raw"] == 0 and g0["status"] == 0:
        print("[拒绝] 没收到 0x2A8 反馈（Hz 太低或全 0），先查末端线与上电。", file=sys.stderr)
        return 2

    print("[mode ] MotionCtrl_2(0x01 CAN_CTRL, 0x01 MOVE_J, 30%)")
    p.MotionCtrl_2(0x01, 0x01, 30, 0x00)
    time.sleep(0.3)

    print("[clr  ] GripperCtrl(0, effort, 0x02) 失能并清除错误")
    p.GripperCtrl(0, a.effort, 0x02, 0)
    time.sleep(0.3)

    print("[en   ] GripperCtrl(0, effort, 0x01) 使能")
    p.GripperCtrl(0, a.effort, 0x01, 0)
    time.sleep(0.5)
    g = read_gripper(p)
    print(f"[en   ] status 0x{g['status']:02X}  bit6={bool(g['status'] >> 6 & 1)}")
    if not (g["status"] >> 6 & 1):
        print("[警告] bit6=0：驱动器仍未使能，后面的致动不会生效。仍然继续发，看现象。")

    peak = g["raw"]
    t0 = time.time()

    def ramp(start_raw: int, end_raw: int, dur: float) -> None:
        nonlocal peak
        t_start = time.time()
        nextp = t_start
        while True:
            u = (time.time() - t_start) / dur
            if u >= 1.0:
                u = 1.0
            uu = u * u * (3 - 2 * u)  # smoothstep
            tgt = int(round(start_raw + (end_raw - start_raw) * uu))
            p.GripperCtrl(tgt, a.effort, 0x01, 0)
            now = read_gripper(p)
            peak = max(peak, now["raw"])
            if time.time() >= nextp:
                print(f"  t={time.time() - t0:5.2f}s  目标 {tgt * UNIT:6.2f} mm   "
                      f"反馈 {now['mm']:6.2f} mm   力矩 {now['effort']:+.3f} N·m   "
                      f"status 0x{now['status']:02X}")
                nextp += a.print_every
            if u >= 1.0:
                break
            time.sleep(0.005)

    print(f"[open ] 0 → {to_raw * UNIT:.1f} mm，斜坡 {a.ramp_s}s")
    ramp(g["raw"], to_raw, a.ramp_s)
    time.sleep(a.hold_s)

    print(f"[close] {to_raw * UNIT:.1f} mm → 0，斜坡 {a.ramp_s}s")
    ramp(to_raw, 0, a.ramp_s)
    time.sleep(a.hold_s)

    gend = read_gripper(p)
    print(f"[end  ] 反馈跨度 = {(peak - g0['raw']) * UNIT:.2f} mm"
          f"（起始 {g0['mm']:.2f}，峰值 {peak * UNIT:.2f}，结束 {gend['mm']:.2f}）")

    p.GripperCtrl(0, a.effort, 0x00, 0)  # 失能，交还被动状态
    time.sleep(0.2)
    glast = read_gripper(p)
    print(f"[off  ] 已失能，status 0x{glast['status']:02X}  bit6={bool(glast['status'] >> 6 & 1)}")

    span = (peak - g0["raw"]) * UNIT
    if span >= 0.4 * (to_raw * UNIT):
        print("[结论] 跟随：反馈行程随指令变化 → 机械与编码器都正常。")
        return 0
    print("[结论] 未跟随：反馈几乎没动。若你目视指头也没动 → 机械卡死/未装到位；"
          "若指头动了但读数不动 → 编码器或反馈解析问题。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
