#!/usr/bin/env python3
# s3_j6_live.py —— J6 角度/电流实时打印（**只读**，不发任何帧）
#
# 用途：J6 驱动被保护失能时，人手绕工具轴转夹爪，看编码器还报不报数、
# 以及往哪个方向、转到多少度会顶住。
#
# 2026-10-20 的疑问：J6 故障后 samples 里 q6 一直是 +4.59°、cur6 一直是 −9.517 A，
# 两个值都纹丝不动 —— 到底是「真没动」还是「驱动故障后反馈冻结了」，
# 只能靠人转一下看数字变不变来区分。
#
# 用法（VM，~/stage0/20261019_stage3 下）：
#   python3 s3_j6_live.py --seconds 300
#   输出重定向到文件再回看：python3 s3_j6_live.py --seconds 300 > raw/j6_live.log

import argparse
import time

from piper_sdk import C_PiperInterface

FAIL_FLAGS = [
    ("voltage_too_low", "欠压"),
    ("driver_overcurrent", "过流"),
    ("driver_overheating", "驱动过温"),
    ("motor_overheating", "电机过温"),
    ("collision_status", "碰撞"),
    ("driver_error_status", "驱动错误"),
    ("stall_status", "堵转"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--period", type=float, default=0.2)
    args = ap.parse_args()

    piper = C_PiperInterface(args.iface)
    if piper.ConnectPort() is False:
        raise SystemExit("ConnectPort() 返回 False：can0 打不开")

    t0 = time.time()
    last = None
    print("t(s)   J1      J2      J3      J5      J6(deg)   cur6(A)  使能  异常")
    while time.time() - t0 < args.seconds:
        js = piper.GetArmJointMsgs().joint_state
        q = [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]
        j6 = q[5]
        low = piper.GetArmLowSpdInfoMsgs()
        m = getattr(low, "motor_6")
        cur = m.bus_current / 1000.0
        en = m.foc_status.driver_enable_status
        bad = ",".join(txt for attr, txt in FAIL_FLAGS if getattr(m.foc_status, attr)) or "-"
        mark = ""
        if last is not None and abs(j6 - last) >= 0.05:
            mark = "  <== J6 变了 %+.2f" % (j6 - last)
        last = j6
        print("%6.1f  %+7.2f %+7.2f %+7.2f %+7.2f  %+8.2f  %+8.3f  %-5s %s%s"
              % (time.time() - t0, q[0], q[1], q[2], q[4], j6, cur, str(en), bad, mark), flush=True)
        time.sleep(args.period)


if __name__ == "__main__":
    main()
