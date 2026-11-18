#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""why_stuck.py —— 只看数据、不动机器人：把 s20_p1 这次「没动」拆开看

要回答三个问题
--------------
1. J1/J4 到底动没动？（如果完全没动，说明 0x155 整帧被丢弃，而不是「动得慢」）
2. 电流是多少？（顶住障碍电流会飙；被丢弃则电流接近「只抵抗重力」的水平）
3. 0x2A1 的 arm_status / motion_status 在过程中有没有变化（保护态？）

用法：python3 why_stuck.py raw/s20_p1.jsonl
"""
import json
import struct
import sys

ANGLE_IDS = {0x2A5: (1, 2), 0x2A6: (3, 4), 0x2A7: (5, 6)}
HS_IDS = {0x251 + i: i + 1 for i in range(6)}


def i32(buf, off):
    """CAN 帧是**大端**（与 stage1_l0.py 的 decode_joint_pair 一致）。"""
    return int.from_bytes(buf[off:off + 4], "big", signed=True)


def i16(buf, off):
    return int.from_bytes(buf[off:off + 2], "big", signed=True)


def main(path):
    last_q = {}
    last_cur = {}
    last_st = None
    t0 = None
    rows = []
    next_emit = 0.0

    with open(path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except Exception:
                continue
            if "id" not in rec:
                continue
            t = rec["t"]
            me = rec["id"]
            buf = bytes.fromhex(rec["d"])
            if t0 is None:
                t0 = t
            if me in ANGLE_IDS:
                a, b = ANGLE_IDS[me]
                last_q[a] = i32(buf, 0) * 0.001
                last_q[b] = i32(buf, 4) * 0.001
            elif me in HS_IDS:
                last_cur[HS_IDS[me]] = i16(buf, 2) * 0.001
            elif me == 0x2A1:
                last_st = (buf[0], buf[1], buf[2], buf[4])
            el = t - t0
            if el >= next_emit:
                rows.append((el, dict(last_q), dict(last_cur), last_st))
                next_emit += 1.0

    print(f"样本 {len(rows)} 行（每 1 s 一行）")
    print("  t(s) |        J1        J2        J3        J4        J5        J6 "
          "|      I1     I2     I3     I4     I5     I6 | ctrl sta  mdl  mov")
    for el, q, cur, st in rows:
        if el > 130:
            break
        qs = " ".join(f"{q.get(k, float('nan')):9.3f}" for k in range(1, 7))
        cs = " ".join(f"{cur.get(k, float('nan')):6.2f}" for k in range(1, 7))
        print(f"{el:6.0f} | {qs} | {cs} | {st}")

    # 首末对比
    if len(rows) >= 2:
        a, b = rows[0][1], rows[-1][1]
        print("\n首→末 位移 (deg)：" + " ".join(
            f"J{k}={b.get(k, 0) - a.get(k, 0):+.3f}" for k in range(1, 7)))
        cur_all = [abs(v) for _, _, c, _ in rows for v in c.values() if v is not None]
        if cur_all:
            cur_all.sort()
            print(f"电流 |I| 中位 {cur_all[len(cur_all) // 2]:.2f} A，"
                  f"最大 {cur_all[-1]:.2f} A，样本 {len(cur_all)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "raw/s20_p1.jsonl")
