#!/usr/bin/env python3
# s3_clear_j6.py —— 试清 J6 驱动锁存故障（**只发使能帧，不发任何运动帧**）
#
# 背景（2026-10-20 14:0x）
# ----------------------
# S1_6_lo 让 J6 走 0 → +15°（正常）→ 回 0 → 往负方向时在 −0.5° 顶住，
# 电流冲到 −12.8 A 并持续 58.7 s，驱动打出「过流 + 驱动错误 + 堵转保护」并失能 J6。
# 之后 S1_6_mid 的 pre-pose 因「六轴未全使能」零帧 Abort，整批停在 17/42。
#
# 本脚本只做一件事：对 J6 重复发 Disable→Enable，看锁存能否清掉，然后回读。
# 全程不打开下发通道、不写 0x155/0x157，机械臂不会动（它本来就不动）。
#
# 用法（VM，~/stage0/20261019_stage3 下）：
#   python3 s3_clear_j6.py            # 试 3 轮
#   python3 s3_clear_j6.py --tries 5

import argparse
import time

from piper_sdk import C_PiperInterface

FAIL_FLAGS = [
    ("voltage_too_low", "母线欠压"),
    ("driver_overcurrent", "驱动过流"),
    ("driver_overheating", "驱动过温"),
    ("motor_overheating", "电机过温"),
    ("collision_status", "碰撞保护"),
    ("driver_error_status", "驱动错误"),
    ("stall_status", "堵转保护"),
]


def snapshot(piper):
    en = piper.GetArmEnableStatus()
    low = piper.GetArmLowSpdInfoMsgs()
    m = getattr(low, "motor_6")
    bad = [txt for attr, txt in FAIL_FLAGS if getattr(m.foc_status, attr)]
    js = piper.GetArmJointMsgs().joint_state
    j6 = getattr(js, "joint_6") / 1000.0
    return en, bad, j6, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--motor", type=int, default=6)
    ap.add_argument("--tries", type=int, default=3)
    args = ap.parse_args()

    piper = C_PiperInterface(args.iface)
    if piper.ConnectPort() is False:  # 0.6.1 返回 None，只有 False 才失败（R33）
        raise SystemExit("ConnectPort() 返回 False：can0 打不开")

    time.sleep(0.5)
    en, bad, j6, m = snapshot(piper)
    print(f"[before] 使能={en}  J6={j6:+.2f}°  故障={'、'.join(bad) if bad else '无'}")
    if en[args.motor - 1] and not bad:
        print("J6 已使能且无故障，无需处理")
        return

    for k in range(1, args.tries + 1):
        piper.DisableArm(args.motor)
        time.sleep(0.30)
        piper.EnableArm(args.motor, 0x02)
        time.sleep(0.60)
        en, bad, j6, m = snapshot(piper)
        ok = en[args.motor - 1] and not bad
        print(f"[try {k}] 使能 J6={en[args.motor - 1]}  角={j6:+.2f}°  "
              f"故障={'、'.join(bad) if bad else '无'}  {'✓' if ok else '✗'}")
        if ok:
            for _ in range(3):
                time.sleep(0.5)
                en, bad, j6, m = snapshot(piper)
                print(f"  [确认] 使能 J6={en[args.motor - 1]}  故障={'、'.join(bad) if bad else '无'}")
            print("\n结论：故障已清，J6 恢复使能。可以断点续跑，但**先确认 J6 负向为什么顶住**再重启批。")
            return

    print("\n结论：连发 %d 轮都没清掉 —— 锁存是硬件级（驱动错误/堵转），"
          "只能断电重启清，或者 J6 轴本身有机械阻挡。" % args.tries)
    print("断电前必须先把臂挪到安全位/垫好，否则断电即自由下落。")


if __name__ == "__main__":
    main()
