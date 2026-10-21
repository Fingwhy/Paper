#!/usr/bin/env python3
"""拆装载荷期间的刚性保持。

为什么需要它：PiPER 在 CAN_CTRL 下靠「持续收到位置指令」维持伺服锁定。实测掉线
22 s 内姿态没变（0.01°），说明短时空闲不会失能；但用户拆水瓶要几分钟，而且人就在
臂旁边用力撕胶带，这段时间不能靠「应该不会掉」——必须每拍发帧把臂钉住。

与 `stage1_cmd.py pose --hold-s` 的区别：
  * 不发斜坡、不跑守卫、不落盘，只做一件事：把**拆装开始那一刻的实测角**锁成常量目标，
    每 10 ms 发一次。目标刻意**不跟随实测**——若跟随，臂在负载突然卸掉时会被"实测"
    拖着慢慢漂（这正是 R43 里吃过的亏）。
  * 目标必须落在固件∩URDF 硬包络内（R44），否则固件会整臂不动或把臂拽回界内。

用法（VM）：
    python3 hold_pose.py can0 900
Ctrl-C 或 kill 结束；结束时臂保持最后姿态（不会失能）。
"""
import math
import sys
import time

sys.path.insert(0, "/home/zzz/stage0/20261019_stage1_l0")
import stage1_cmd as C  # noqa: E402  复用已验证的自建发帧 Tx 与硬包络

HZ = 100.0


def read_q_deg(piper):
    js = piper.GetArmJointMsgs()
    return [getattr(js.joint_state, f"joint_{i + 1}") * 0.001 for i in range(6)]


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    sec = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0

    import piper_sdk

    p = piper_sdk.C_PiperInterface_V2(iface)
    ret = p.ConnectPort()
    if ret is False:
        print(f"[hold] ConnectPort 明确失败，中止")
        return 2
    time.sleep(0.8)

    # 等到确实有反馈（唯一时间戳 >1），否则读到的 0 会被当成目标
    q0, seen = None, 0
    t0 = time.time()
    while time.time() - t0 < 3.0:
        js = p.GetArmJointMsgs()
        seen += 1
        q = read_q_deg(p)
        if any(abs(v) > 1e-6 for v in q):
            q0 = q
            break
        time.sleep(0.02)
    if q0 is None:
        print("[hold] 3 s 内没有读到有效关节角，中止（CAN 是否还在？）")
        return 3

    import json
    from pathlib import Path

    lim_path = Path("/home/zzz/stage0/20261019_stage0/env/joint_limits.json")
    limits = json.loads(lim_path.read_text(encoding="utf-8")) if lim_path.exists() else None
    if limits is None:
        print(f"[hold] 警告：{lim_path} 不存在，只用 URDF 限位兜底")
    env = C.safe_envelope(limits, tol_deg=2.0)
    tgt, clamped = [], []
    for i, v in enumerate(q0, start=1):
        lo, hi = env[i]
        w = min(max(v, lo), hi)
        if abs(w - v) > 1e-6:
            clamped.append(f"J{i} {v:+.2f}→{w:+.2f}")
        tgt.append(w)

    print(f"[hold] 目标锁定 = {['%+.2f' % v for v in tgt]}")
    if clamped:
        print(f"[hold] 被硬包络夹紧：{', '.join(clamped)}  （不会下发界外值）")
    print(f"[hold] 发 {HZ:.0f} Hz × {sec:.0f} s。Ctrl-C 结束。")

    tx = C.Tx(iface)
    tgt_rad = [math.radians(v) for v in tgt]
    dt = 1.0 / HZ
    t_end = time.time() + sec
    n_tx = n_fail = 0
    next_t = time.time()
    try:
        while time.time() < t_end:
            if tx.send_joint(tgt_rad):
                n_tx += 3
            else:
                n_fail += 3
            next_t += dt
            sl = next_t - time.time()
            if sl > 0:
                time.sleep(sl)
            else:
                next_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        tx.close()

    # 收尾核对：有没有被负载拽走
    q1 = read_q_deg(p)
    dmax = max(abs(a - b) for a, b in zip(q1, tgt))
    print(f"[hold] 发送 {n_tx} 帧成功 / {n_fail} 失败")
    print(f"[hold] 结束实测 = {['%+.2f' % v for v in q1]}")
    print(f"[hold] 与锁定目标最大偏差 = {dmax:.3f}°  "
          f"{'（保持在位）' if dmax < 1.0 else '（注意：有偏移，检查是否被碰/负载已卸）'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
