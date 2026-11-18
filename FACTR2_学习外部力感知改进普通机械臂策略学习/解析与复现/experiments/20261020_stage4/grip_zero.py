#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪标零（分步执行，必须失能状态下手动推合两指）。

为什么会需要它
--------------
夹爪读数是「相对零点」的。零点一旦写错，读数整体偏移，症状非常像机械卡死：

    命令张开 → 电机使劲（effort 几百）但开度纹丝不动
    命令闭合 → 却能顺畅走到底

因为控制器以为「还没到目标」，于是一直朝开口方向推，而实际已经顶在机械上限。
本次实测偏移约 **-70 mm**：读数 0 实际是 **全开 70 mm**。

正确顺序（**使能状态标零必错** —— 使能帧里自带目标角，会先把爪跑到那个位置再固化零点）
----------------------------------------------------------------------------------
    ① disable  失能
    ② 手动把两指推合到全闭，推住别松
    ③ zero     标零
    ④ 松手
    ⑤ enable   重新使能

用法（分步，方便人在旁边配合）
------------------------------
    python3 grip_zero.py read         # 先看看当前读数
    python3 grip_zero.py disable      # 失能
    #   >>> 这时手动把两指推合到全闭并推住 <<<
    python3 grip_zero.py read         # 确认读数跟随着动作（能跟随=编码器好，只是零点偏）
    python3 grip_zero.py zero         # 标零
    #   >>> 松手 <<<
    python3 grip_zero.py enable       # 重新使能
    python3 grip_zero.py read         # 应为 ~0 全闭；再张开验证
"""
import sys
import time

from piper_sdk import C_PiperInterface as Piper

IFACE = sys.argv[2] if len(sys.argv) > 2 else "can0"
DT = 0.02


def connect(iface):
    p = Piper(iface)
    p.ConnectPort()
    return p


def read_grip(p, settle=0.4):
    time.sleep(settle)
    st = p.GetArmGripperMsgs().gripper_state
    return int(st.grippers_angle), int(st.grippers_effort)


def main():
    stage = sys.argv[1] if len(sys.argv) > 1 else "read"
    p = connect(IFACE)

    if stage == "read":
        for _ in range(3):
            a, e = read_grip(p, 0.2)
            print(f"  开度 {a/1000:7.2f} mm   effort {e:+6d}")
        return 0

    if stage == "disable":
        print("① 失能夹爪（关节不动，只关夹爪电机）")
        for _ in range(10):
            p.GripperCtrl(0, 0, 0x00, 0)
            time.sleep(DT)
        a, e = read_grip(p)
        print(f"  已失能：开度 {a/1000:.2f} mm  effort {e:+d}")
        print("  >>> 现在手动把两指推合到全闭并推住，然后运行：")
        print("      python3 grip_zero.py read")
        print("      python3 grip_zero.py zero")
        return 0

    if stage == "zero":
        a0, e0 = read_grip(p)
        print(f"③ 标零前：开度 {a0/1000:.2f} mm  effort {e0:+d}")
        print("   发送 GripperCtrl(0,0,0x00,0xAE)")
        for _ in range(10):
            p.GripperCtrl(0, 0, 0x00, 0xAE)
            time.sleep(DT)
        a1, e1 = read_grip(p, 0.6)
        print(f"   标零后：开度 {a1/1000:.2f} mm  effort {e1:+d}")
        if abs(a1) < 500:
            print("   ✓ 零点已归位（读数≈0）")
        else:
            print(f"   ！读数仍为 {a1/1000:.2f} mm —— 两指可能没推到位，或编码器有问题")
        print("  >>> 可以松手了，然后运行：python3 grip_zero.py enable")
        return 0

    if stage == "enable":
        print("⑤ 重新使能夹爪")
        for _ in range(15):
            p.GripperCtrl(0, 600, 0x01, 0)
            time.sleep(DT)
        a, e = read_grip(p, 0.6)
        print(f"  已使能：开度 {a/1000:.2f} mm  effort {e:+d}")
        return 0

    print("未知 stage：", stage)
    return 1


if __name__ == "__main__":
    sys.exit(main())
