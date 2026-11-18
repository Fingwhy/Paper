#!/usr/bin/env python3
"""夹爪温柔控制：小力矩 + 渐进目标，不猛夹。

为什么重写（2026-10-20 17:50 用户反馈「关得太快、力太大，容易把物体或臂弄坏」）
----------------------------------------------------------------------------------
旧写法 `GripperCtrl(0, 3000, ...)` 每 100 ms 发一次，是把目标**一步**设成 0（全闭）
再配 3 N·m 力矩 —— 相当于让夹爪以最大力撞上去。对 100 g 的物体完全没必要，
而且夹爪闭合到底时的冲击会传到腕部。

新写法三条：
1. **目标渐进**：从当前开度按 `--step`（默认 1.5 mm）逐级逼近，每级 40 ms，
   任何一级都不许跳。
2. **力矩压低**：默认 `--effort 600`（0.6 N·m），空载 100 g 靠摩擦已足够；
   500 g 用 `--effort 1000`。
3. **接触即停**：每级之后回读开度，若实测落后目标 > 3 mm，说明夹到东西了，
   立即停止下降并在**当前实测开度**上保持 —— 不再往目标里顶。

子命令
------
  open   [--angle 70000] [--effort 500]     慢慢张开
  close  [--effort 600]                     慢慢闭合，夹到即停
  hold   [--angle N] [--effort 800] [--secs] 在指定/当前开度轻力保持
  status                                    只读

例子（VM，~/stage0/20261020_stage4）：
  python3 grip.py status
  python3 grip.py open --angle 70000 --effort 500
  python3 grip.py close --effort 600
  python3 grip.py hold --effort 800 --secs 600
"""

import argparse
import sys
import time

from piper_sdk import C_PiperInterface

STEP_DEFAULT = 1500      # 1.5 mm 每级
DT = 0.04                # 每级 40 ms
LAG_MM = 3000            # 实测落后目标 3 mm ⇒ 判为夹到东西


def angle_of(piper):
    st = piper.GetArmGripperMsgs().gripper_state
    return int(st.grippers_angle), int(st.grippers_effort)


def send(piper, ang, effort, n=2):
    for _ in range(n):
        piper.GripperCtrl(int(ang), int(effort), 0x01, 0)
        time.sleep(DT / n)


def do_open(piper, target, effort, step):
    cur, _ = angle_of(piper)
    print(f"张开：{cur} → {target}（effort {effort}，每级 {step}）")
    while cur < target:
        cur = min(target, cur + step)
        send(piper, cur, effort)
        meas, eff = angle_of(piper)
        if meas < cur - LAG_MM:
            cur = meas                      # 跟不上就按实测走，别硬顶
    time.sleep(0.3)
    a, e = angle_of(piper)
    print(f"完成：开度 {a}（{a/1000:.1f} mm），effort {e}")


def do_close(piper, effort, step):
    cur, _ = angle_of(piper)
    print(f"闭合：{cur} → 0（effort {effort}，每级 {step}，夹到即停）")
    held = None
    while cur > 0:
        cur = max(0, cur - step)
        send(piper, cur, effort)
        meas, eff = angle_of(piper)
        if meas > cur + LAG_MM:             # 实测比目标大 ⇒ 被物体挡住
            held = meas
            print(f"  夹到接触：目标 {cur}，实测 {meas} ⇒ 停在实测开度")
            break
    if held is None:
        held, _ = angle_of(piper)
    # 保持目标要**略微小于**实测接触开度，否则「目标=实测」等于不施力，
    # 实测 effort 只有 −0.5 N·m（2026-10-20 18:0x 踩到：物体在重力下会沉）。
    # 压进 0.6 mm，真正的夹持力由 --effort 上限决定。
    squeeze = min(int(step * 0.4), 600)
    hold_ang = max(0, held - squeeze)
    send(piper, hold_ang, effort, n=12)
    a, e = angle_of(piper)
    print(f"完成：夹持开度 {a}（{a/1000:.1f} mm ≈ 物体宽度），"
          f"保持目标 {hold_ang}（压进 {squeeze}），effort {e}")


def do_hold(piper, ang, effort, secs):
    if ang is None:
        ang, _ = angle_of(piper)
    print(f"保持开度 {ang}，effort {effort}，{secs} s（Ctrl-C 结束）")
    t0 = time.time()
    while secs <= 0 or time.time() - t0 < secs:
        piper.GripperCtrl(int(ang), int(effort), 0x01, 0)
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("open", "close", "hold", "status"))
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--angle", type=int, default=None)
    ap.add_argument("--effort", type=int, default=None)
    ap.add_argument("--step", type=int, default=STEP_DEFAULT)
    ap.add_argument("--secs", type=int, default=600)
    args = ap.parse_args()

    piper = C_PiperInterface(args.iface)
    ret = piper.ConnectPort()
    if ret is False:                       # 注意：0.6.1 的 ConnectPort 返回 None，不能用 not
        print("连不上 can0", file=sys.stderr)
        return 2
    time.sleep(1.0)

    if args.cmd == "status":
        a, e = angle_of(piper)
        # 注意：gripper_state.status_code 是 int（不是结构体），不能取 .driver_enable_status
        print(f"开度 {a}（{a/1000:.1f} mm）  effort {e}")
        return 0

    if args.cmd == "open":
        do_open(piper, args.angle if args.angle is not None else 70000,
                args.effort if args.effort is not None else 500, args.step)
    elif args.cmd == "close":
        do_close(piper, args.effort if args.effort is not None else 600, args.step)
    else:
        do_hold(piper, args.angle,
                args.effort if args.effort is not None else 800, args.secs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
