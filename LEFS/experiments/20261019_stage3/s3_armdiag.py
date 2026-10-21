#!/usr/bin/env python3
# s3_armdiag.py —— 只读诊断：机械臂为什么不动 / 为什么某轴使不上
#
# 不下发任何运动指令，也不改模式、不改使能。只读 0x2A1 / 0x261~0x266 / 0x251~0x256。
# 打印：
#   1) ctrl_mode（0x2A1）与 arm_status 状态码（急停 / 碰撞 / 抱闸未开 / 关节异常…）
#   2) 六轴使能位（0x261~0x266 的 foc_status.driver_enable_status）
#   3) 六轴故障位（欠压 / 过流 / 驱动过温 / 电机过温 / 碰撞 / 驱动错误 / 堵转）
#   4) 六轴角度、电压、驱动温度、电机温度、母线电流
#
# 用法（VM，~/stage0/20261019_stage3 下）：
#   python3 s3_armdiag.py
#   python3 s3_armdiag.py --iface can0 --seconds 2

import argparse
import time

from piper_sdk import C_PiperInterface

CTRL_MODE = {0x00: "STANDBY(0x0)", 0x01: "CAN_CTRL(0x1)", 0x02: "TEACHING_MODE(0x2)"}
ARM_STATUS = {
    0x00: "正常",
    0x01: "急停",
    0x02: "无解",
    0x03: "奇异点",
    0x04: "目标角度超过限",
    0x05: "关节通信异常",
    0x06: "关节抱闸未打开",
    0x07: "机械臂发生碰撞",
    0x08: "拖动示教时超速",
    0x09: "关节状态异常",
    0x0A: "其它异常",
    0x0B: "示教记录",
    0x0C: "示教执行",
    0x0D: "示教暂停",
    0x0E: "主控NTC过温",
    0x0F: "释放电阻NTC过温",
}


def flags(foc):
    bad = []
    if foc.voltage_too_low:
        bad.append("母线欠压")
    if foc.driver_overcurrent:
        bad.append("驱动过流")
    if foc.driver_overheating:
        bad.append("驱动过温")
    if foc.motor_overheating:
        bad.append("电机过温")
    if foc.collision_status:
        bad.append("碰撞保护")
    if foc.driver_error_status:
        bad.append("驱动错误")
    if foc.stall_status:
        bad.append("堵转保护")
    return "、".join(bad) if bad else "正常"


def joint_deg(piper):
    js = piper.GetArmJointMsgs().joint_state
    return [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()

    piper = C_PiperInterface(args.iface)
    ret = piper.ConnectPort()
    if ret is False:  # 0.6.1 返回 None，只有明确 False 才算失败（R33）
        raise SystemExit("ConnectPort() 返回 False：can0 打不开")

    t0 = time.time()
    seen = set()
    while time.time() - t0 < args.seconds:
        st = piper.GetArmStatus()
        seen.add(getattr(st, "time_stamp", id(st)))
        time.sleep(0.01)

    st = piper.GetArmStatus()
    body = getattr(st, "arm_status", st)
    cmode = int(getattr(body, "ctrl_mode", -1))
    astat = int(getattr(body, "arm_status", -1))
    mfeed = getattr(body, "mode_feed", None)

    print("=== 0x2A1 机械臂状态 ===")
    print(f"  唯一时间戳 {len(seen)}（{args.seconds:.1f}s 内，>10 说明反馈在跑）")
    print(f"  ctrl_mode = {CTRL_MODE.get(cmode, f'未知({cmode})')}")
    print(f"  arm_status= 0x{astat:02X} {ARM_STATUS.get(astat, f'未知({astat})')}"
          + (f"   mode_feed=0x{mfeed:02X}" if isinstance(mfeed, int) else ""))
    print(f"  使能六轴 = {piper.GetArmEnableStatus()}   （全 True 才算全臂使能）")

    low = piper.GetArmLowSpdInfoMsgs()
    print("\n=== 0x261~0x266 逐轴诊断 ===")
    print(f"  {'轴':<4}{'使能':<7}{'电压V':<8}{'驱动C':<8}{'电机C':<8}{'母线A':<9}异常")
    for i in range(1, 7):
        m = getattr(low, f"motor_{i}")
        print(
            f"  J{i:<3}{str(m.foc_status.driver_enable_status):<7}"
            f"{m.vol / 10.0:<8.1f}{m.foc_temp:<8d}{m.motor_temp:<8d}"
            f"{m.bus_current / 1000.0:<9.3f}{flags(m.foc_status)}"
        )

    print("\n=== 六轴角（deg） ===")
    print("  [" + ", ".join(f"{v:+.2f}" for v in joint_deg(piper)) + "]")


if __name__ == "__main__":
    main()
