#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stage0_sniff_151.py —— 抓 SDK 发出的 0x151 原始字节（只读观察，不改状态）

为什么需要它
------------
`s10` 那次 pose 发了 1063 帧 `0x155~0x157` 全部发送成功，臂一动没动。根因是**没有先发
`0x151`（运动模式帧）**：臂在「收到 0x151 之前」会忽略关节指令，帧照收、照回传状态，
只是不执行 —— 与 09:47 夹爪那次 `0x159` 被忽略是同一个机制。

要自己发 0x151 就得知道字节布局。文档只写到「Byte2 速度百分比 0~100」，但**没写
是否缩放**，而速度是安全相关量：猜错会让臂以非预期速度运动。所以先在总线上
把 SDK 实际发出的那一帧抄下来，再照抄进自建 socket。

做法：另开一个只收不发的 socketcan 实例，用 SDK 发一次速度 30% 的 0x151，
从监听侧把帧取出来。总线是广播的，别的 socket 能收到本机发出的帧。
"""
import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--speed", type=int, default=30, help="要观察的速度百分比")
    args = ap.parse_args()

    import can
    import piper_sdk

    # 监听侧：只收，不参与发送
    listener = can.Bus(interface="socketcan", channel=args.iface)
    piper = piper_sdk.C_PiperInterface_V2(args.iface)
    piper.ConnectPort()
    time.sleep(0.4)

    print(f"[sniff] 发一帧 MotionCtrl_2(ctrl=0x01, move=0x01, speed={args.speed}, mit=0x00)")
    piper.MotionCtrl_2(0x01, 0x01, args.speed, 0x00)

    got = []
    t0 = time.time()
    while time.time() - t0 < 0.5:
        m = listener.recv(0.05)
        if m and m.arbitration_id == 0x151:
            got.append(bytes(m.data))

    if not got:
        print("[FAIL] 0.5 s 内没抓到 0x151。可能 loopback 关闭（ip link set can0 type can loopback on）",
              file=sys.stderr)
        return 2

    for i, d in enumerate(got[:3]):
        print(f"[fx{i}] {d.hex(' ')}   len={len(d)}")
    print(f"[n  ] 抓到 {len(got)} 帧")

    # 判读：byte2 是否等于原始百分比，还是有缩放
    b = got[0]
    print(f"[解读] ctrl={b[0]:#04x} move={b[1]:#04x} speed_byte={b[2]} "
          f"（请求 {args.speed}）mit={b[3]:#04x} residence={b[4]} install={b[5]:#04x}")
    if b[2] == args.speed:
        print("[结论] byte2 就是原始百分比，无缩放 → 自建 socket 可直接照抄")
    else:
        print(f"[结论] byte2 ≠ 请求值，存在缩放：{args.speed} → {b[2]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
