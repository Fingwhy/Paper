#!/usr/bin/env python3
"""张开 → 等你放 5 秒 → 小力矩渐进闭合。

为什么要一个独立脚本：`grip.py open` 和 `grip.py close` 是两个进程，
中间会插进 SDK 重连（各 ~1 s），5 秒的窗口就不准了。这里一个进程跑完，
5 秒就是从「已完全张开」开始计时。

闭合力默认 1200（1.2 N·m）——比之前的 2.5 更轻，足够夹住几百克，
又不会把薄壁物体压变形。
"""

import argparse
import time

from piper_sdk import C_PiperInterface

DT = 0.04


def angle_of(p):
    st = p.GetArmGripperMsgs().gripper_state
    return int(st.grippers_angle), int(st.grippers_effort)


def send(p, ang, effort, n=2):
    for _ in range(n):
        p.GripperCtrl(int(ang), int(effort), 0x01, 0)
        time.sleep(DT / n)


def ramp_to(p, target, effort, step=1500, lag=3000, stop_on_contact=False):
    cur, _ = angle_of(p)
    while (cur < target) if target > cur else (cur > target):
        cur = min(target, cur + step) if target > cur else max(target, cur - step)
        send(p, cur, effort)
        meas, _ = angle_of(p)
        if stop_on_contact and abs(meas - cur) > lag:
            return meas, True
        if not stop_on_contact and ((target > cur and meas < cur - lag) or
                                    (target < cur and meas > cur + lag)):
            cur = meas
    return angle_of(p)[0], False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--hold-s", type=float, default=5.0, help="张开后等你放物体的秒数")
    ap.add_argument("--open-angle", type=int, default=70000)
    ap.add_argument("--open-effort", type=int, default=400)
    ap.add_argument("--close-effort", type=int, default=1200)
    args = ap.parse_args()

    p = C_PiperInterface(args.iface)
    if p.ConnectPort() is False:
        raise SystemExit("连不上 can0")
    time.sleep(1.0)

    a0, _ = angle_of(p)
    print(f"1) 张开：{a0}（{a0/1000:.1f} mm）→ {args.open_angle/1000:.1f} mm，"
          f"effort {args.open_effort}")
    ramp_to(p, args.open_angle, args.open_effort)
    a1, _ = angle_of(p)
    print(f"   已张开 {a1}（{a1/1000:.1f} mm）")

    print(f"2) >>> 请在 {args.hold_s:g} 秒内置入物体 <<<")
    for s in range(int(args.hold_s), 0, -1):
        print(f"   {s} …", flush=True)
        time.sleep(1)

    print(f"3) 轻力闭合（effort {args.close_effort}，1.5 mm/级，碰到即停）")
    a2, hit = ramp_to(p, 0, args.close_effort, stop_on_contact=True)
    print(f"   夹到接触，开度 {a2}（{a2/1000:.1f} mm ≈ 物体宽度）"
          f"{'（接触检测触发）' if hit else ''}")

    # 压进 0.6 mm 形成稳定夹持力，力上限仍是 close_effort
    hold = max(0, a2 - 600)
    for _ in range(12):
        send(p, hold, args.close_effort)
    a3, e3 = angle_of(p)
    print(f"4) 保持：目标 {hold}，实测 {a3}（{a3/1000:.1f} mm），effort {e3}")


if __name__ == "__main__":
    main()
