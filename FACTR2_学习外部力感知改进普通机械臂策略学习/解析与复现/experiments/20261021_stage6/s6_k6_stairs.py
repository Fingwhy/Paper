#!/usr/bin/env python3
"""K6 往返阶梯标定：每级转一步、停几秒，往返各 N 级。

为什么要阶梯而不是匀速拧
------------------------
2026-10-26 的四次实验证明：瓶盖不动时，夹爪与盖之间是**打滑**关系，
匀速拧时传感器读到的是"打滑阈值 + 弹性释放"，臂侧读到的是"J6 摩擦"，
两者不是同一个量（详见 S6_TORQUE_CAP1.md 的诊断）。

阶梯法把问题拆开：
* **保持段**里 J6 不动 ⇒ 没有滑动摩擦、没有弹性释放，两边都是静平衡；
* 每级一个平台 ⇒ 得到 N 个 (i6, τ) 点，可以拟合；
* **往返各做一遍** ⇒ 摩擦与迟滞取平均后被消掉，剩下的就是 K6。

安全
----
* 速率默认 2 °/s，总行程默认 6×5°=30°（往返回到原角度）
* J6 电流上限默认 0.35 A（≈3.8 N·m，保护 5 N·m 量程的传感器）
* 任一步电流超限立即停发并报错，不中断使能
* 记录臂侧起始 epoch（VM 时钟），配对时用 7.97 s 的时钟偏差换算
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

import twist_log as T          # 复用连接/读数/Hold 实现

HERE = Path(__file__).resolve().parent
DT_SEND = 0.02


def move_to(p, hold, q_now, q_target, rate, max_cur6, reissue, grip_ang, grip_eff):
    """按速率插值把 J6 移到目标角，其它轴保持不变。"""
    span = q_target[5] - q_now[5]
    dur = abs(span) / max(rate, 0.1)
    n = max(2, int(round(dur / DT_SEND)))
    q0 = list(q_now)
    t0 = time.perf_counter()
    nxt = t0
    k = 0
    while True:
        now = time.perf_counter()
        if k <= n and now >= nxt:
            f = min(1.0, k / n)
            qcmd = list(q0)
            qcmd[5] = q0[5] + span * f
            hold.send(qcmd)
            if k % reissue == 0 and grip_eff is not None:
                p.GripperCtrl(max(0, int(grip_ang * 1000) - 300), grip_eff, 0x01, 0)
            k += 1
            nxt += DT_SEND
        cur = T.read_cur(p)
        if abs(cur[5]) > max_cur6:
            raise RuntimeError(f"J6 电流 {cur[5]:+.3f} A 超 {max_cur6} A（≈"
                               f"{abs(cur[5]) * T.K6:.1f} N·m），停发")
        if k > n and now - t0 > dur + 0.3:
            break
    return T.read_q(p)


def main():
    ap = argparse.ArgumentParser(description="K6 往返阶梯标定")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--step", type=float, default=5.0, help="每级转多少度")
    ap.add_argument("--steps", type=int, default=6, help="往、返各几级")
    ap.add_argument("--hold", type=float, default=5.0, help="每级保持秒数（取后 2 s 平均）")
    ap.add_argument("--rate", type=float, default=2.0, help="转动速率 deg/s")
    ap.add_argument("--grip-effort", type=int, default=2000)
    ap.add_argument("--no-grip", action="store_true",
                    help="不碰夹爪（夹持由 weigh_arm.py close 的后台 hold 维持）")
    ap.add_argument("--grip-squeeze", type=int, default=300)
    ap.add_argument("--reissue", type=int, default=2, help="每几拍重发夹爪")
    ap.add_argument("--speed-pct", type=int, default=20)
    ap.add_argument("--max-cur6", type=float, default=0.35, help="J6 电流上限 A")
    ap.add_argument("--out", default=str(HERE / "runs" / "stairs.json"))
    a = ap.parse_args()

    p = T.connect(a.iface)
    q0 = T.read_q(p)
    ang0, ge0 = T.read_grip(p)
    print(f"[init] q = {['%+.2f' % v for v in q0]}")
    print(f"[init] 夹爪 {ang0:.2f} mm，effort {ge0:+.3f}")
    print(f"[init] EPOCH {time.time():.3f}")

    p.MotionCtrl_2(0x01, 0x01, a.speed_pct, 0x00)
    if not a.no_grip:
        p.GripperCtrl(max(0, int(ang0 * 1000) - a.grip_squeeze), a.grip_effort, 0x01, 0)
    hold = T.Hold(a.iface)
    hold.send(q0)
    time.sleep(1.5)

    rows = []
    q = list(q0)
    base = q0[5]
    # 正向：base+1s … base+N s；反向：base+(N-1)s … base（逐级退回，不跳）
    plans = [(k, +1) for k in range(1, a.steps + 1)] + \
            [(k, +1) for k in range(a.steps - 1, -1, -1)]
    if len(plans) > 1 and plans[a.steps][0] == plans[a.steps - 1][0]:
        pass  # 正向末级与反向首级相同，不会重复发送（下面按 target 判重）

    try:
        for k, sign in plans:
            tgt = list(q)
            tgt[5] = base + sign * k * a.step
            q = move_to(p, hold, q, tgt, a.rate, a.max_cur6, a.reissue, ang0,
                        None if a.no_grip else a.grip_effort)

            # 保持：取最后 2 s 的均值
            samples = []
            t_end = time.perf_counter() + a.hold
            while time.perf_counter() < t_end:
                tt = time.perf_counter()
                cur = T.read_cur(p)
                qq = T.read_q(p)
                ga, ge = T.read_grip(p)
                samples.append((time.time(), qq[5], cur[5], ga, ge))   # 用绝对时钟，便于跨机配对
                hold.send(qq)
                time.sleep(max(0.0, DT_SEND - (time.perf_counter() - tt)))

            ta = np.array([s[0] for s in samples])
            sel = ta >= (ta[-1] - 2.0)
            i6 = float(np.mean([s[2] for s, m in zip(samples, sel) if m]))
            q6 = float(np.mean([s[1] for s, m in zip(samples, sel) if m]))
            ga = float(np.mean([s[3] for s, m in zip(samples, sel) if m]))
            ge = float(np.mean([s[4] for s, m in zip(samples, sel) if m]))
            rows.append(dict(step=sign * k, q6=q6, i6=i6, grip_mm=ga, grip_eff=ge,
                             t_epoch=float(np.mean([s[0] for s, m in zip(samples, sel) if m]))))
            print(f"[step {sign*k:+d}] q6={q6:+8.3f}  i6={i6:+8.4f} A "
                  f"(K6 折算 {i6 * T.K6:+7.2f} N·m)  夹爪 {ga:.2f} mm  effort {ge:+.2f}")
    except Exception as e:                                   # noqa: BLE001
        print(f"[ABORT] {type(e).__name__}: {e}")
    finally:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        json.dump(dict(k_sensor=0.001, k6_used=T.K6, step=a.step, hold=a.hold,
                       rate=a.rate, grip_effort=a.grip_effort, rows=rows),
                  open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
        print(f"已保存 {a.out}（{len(rows)} 级）")
        print("提示：传感器侧还需同时记录，事后按 epoch + 时钟偏差 7.97 s 配对。")


if __name__ == "__main__":
    main()
