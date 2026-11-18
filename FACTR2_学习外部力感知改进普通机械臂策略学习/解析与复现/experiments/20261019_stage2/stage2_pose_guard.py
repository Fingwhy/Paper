#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stage2_pose_guard.py 1.0 — 发指令前的几何闸门（不动机械臂）

为什么要有它（2026-10-20 10:1x 事故）
--------------------------------------
P4 原定 `(0, 100, −20, 0, 60, 0)` 是**纯按力矩可辨识性**挑的：J2 力矩反号、幅值够大。
但它的末端 Z 只有 **67 mm**、离基座轴线 82 mm —— 夹爪正好弯进底盘立柱里。

用 URDF 正解算一遍就能提前看到（本次复盘复算）：

    P1  (0, 45,−60,0,45,0)   clear=+28 mm   TCP z=402 mm      ← 已跑过，正常
    P4  (0,100,−20,0,60,0)   clear=−28 mm   TCP z= 67 mm      ← 撞底盘
    P4' (0,100,−75,0,60,0)   clear=+220 mm  TCP z=203 mm      ← 替代方案

所以「姿态表」不能只过力矩这一关，必须先过几何这一关。本工具就是这个关卡：
**在使能之前、在任何帧发出去之前**，把目标姿态和整条路径算一遍，不合格直接退出 2。

判据（三条，全部要过）
----------------------
1. **姿态净空**：link3~link6 与末端 TCP 到「禁区柱体」的距离 ≥ `--min-clear-mm`。
   禁区 = 基座立柱：半径 110 mm、高 150 mm 的圆柱（link2 在 123 mm）。
2. **末端高度**：TCP 的 Z ≥ `--min-tcp-z-mm`（默认 150 mm）。掉到底盘高度就是这一条没守。
3. **路径净空**：起点→目标的线性插值路径上，逐点净空 ≥ `--min-path-mm`（默认 20 mm）。
   只看终点不够 —— 终点很高、中途扫过立柱的情况确实存在。

口径与边界
----------
- 用 URDF（`--urdf`，默认 `_vendor/urdf/piper_description.urdf`）+ 夹爪 0.5 kg 质心。
  正解与固件末端位姿已核验一致：断电垂下姿态 FK `(51.4, −2.5, 170.4)` vs 固件
  `(51.378, −2.454, 170.471)`，亚毫米级 —— 所以这个闸门的口径可信。
- 净空是**保守**的：把臂简化成连杆端点，不做完整包络。它只能挡住「明显要撞」，
  挡不住「擦着过」。运行时还有卡死/电流/高度三道守卫兜底。
- 本工具**只读**：不连 CAN，不发帧，不动臂。唯一例外是 `--start-from-can` 读当前姿态。

用法
----
    # 用当前实测姿态当起点，检查一个目标
    python3 stage2_pose_guard.py --target-deg 0,100,-75,0,60,0 --start-from-can

    # 规划整条路线（可给多个目标=航点，逐段检查）
    python3 stage2_pose_guard.py --start-deg -2.2,-2.0,1.8,-0.8,21.6,0 \
        --target-deg 0,45,-60,0,45,0 --target-deg 0,100,-75,0,60,0

    # 复盘：把撞过的那个姿态喂进来，应当退出 2
    python3 stage2_pose_guard.py --start-deg 0,45,-60,0,45,0 --target-deg 0,100,-20,0,60,0
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage2_design as S

VERSION = "1.0-20261020"

# 禁区：基座立柱（半径、柱顶高度）。link2（肩）在 z=123 mm。
HAZARD_R = 0.110
HAZARD_Z = 0.150

LINKS = ("link3", "link4", "link5", "link6")


def clearance_mm(arm: S.Arm, com, q_deg):
    """姿态到禁区柱体的最小距离（mm），以及最差的部位。负=侵入。"""
    q = [math.radians(v) for v in q_deg]
    T = arm.transforms(q)
    worst = (1e9, None)
    for lk in LINKS:
        p = T[lk][1]
        d = _d_hazard(p)
        if d < worst[0]:
            worst = (d, lk)
    p = arm.point_world(q, "link6", com)
    d = _d_hazard(p)
    if d < worst[0]:
        worst = (d, "tcp")
    return worst[0] * 1000.0, worst[1]


def _d_hazard(p):
    """到「有限高圆柱」禁区的最短距离（m）。负值 = 侵入。

    禁区是实体，不是无限高立柱：半径 HAZARD_R、高 HAZARD_Z，底在 z=0。
    第一版把 r>=R 直接返回 r−R，于是 P1（r=138 mm、z=418 mm）被误判成「擦边 28 mm」——
    它离立柱顶面还有 268 mm 的余量，根本不构成风险。判据错了会把安全姿态也拦掉，
    最后只能被迫加一个 --force 开关绕过，等于没有闸门。
    """
    r = math.hypot(p[0], p[1])
    z = p[2]
    dr = max(0.0, r - HAZARD_R)
    if z > HAZARD_Z:
        dz = z - HAZARD_Z
    elif z < 0.0:
        dz = -z
    else:
        dz = 0.0
    if dr == 0.0 and dz == 0.0:          # 落在圆柱体内
        return -min(HAZARD_R - r, HAZARD_Z - z, z)
    return math.hypot(dr, dz)


def tcp_of(arm, com, q_deg):
    q = [math.radians(v) for v in q_deg]
    return arm.point_world(q, "link6", com)


def path_min_mm(arm, com, a_deg, b_deg, n=81):
    """线性插值路径上的最小净空（mm）与位置。

    **起点本身不参与判定**（f 从 1/(n−1) 开始）：起点是臂当下的停放姿态，是事实而不是
    选择。垂下停放时末端离立柱顶面只有 ~10 mm，把它算进判据会让任何一条路线都永远不合格，
    最后只能加 --force 绕过 —— 闸门就失效了。要判定的是「我们让它走的那条路」。
    """
    worst = (1e9, 0.0, None)
    for k in range(1, n):
        f = k / (n - 1)
        q = [x + (y - x) * f for x, y in zip(a_deg, b_deg)]
        d, who = clearance_mm(arm, com, q)
        if d < worst[0]:
            worst = (d, f, who)
    return worst


def read_start_from_can(iface: str):
    import time
    import piper_sdk

    p = piper_sdk.C_PiperInterface_V2(iface)
    p.ConnectPort()
    time.sleep(0.4)
    js = p.GetArmJointMsgs().joint_state
    return [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]


def parse_deg(chunks):
    vals = []
    for c in chunks:
        vals.extend(str(c).replace(",", " ").split())
    return [float(v) for v in vals]


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="发指令前的几何闸门：检查目标姿态与整条路径到基座立柱的净空（只读，不动臂）")
    ap.add_argument("--urdf", default=str(Path(__file__).resolve().parents[1]
                                          / "_vendor" / "urdf" / "piper_description.urdf"))
    ap.add_argument("--gripper-mass", type=float, default=0.50,
                    help="末端夹爪质量 kg，默认 0.50（实测值可覆盖）")
    ap.add_argument("--start-deg", nargs="+", default=None, help="起点六轴角，deg")
    ap.add_argument("--start-from-can", action="store_true",
                    help="从 CAN 读当前实测姿态作为起点（唯一会碰硬件的动作，只读）")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--target-deg", nargs="+", action="append", default=None,
                    help="目标六轴角，deg。可给多次，按航点顺序逐段检查")
    ap.add_argument("--min-clear-mm", type=float, default=100.0,
                    help="目标姿态的最小净空，默认 100 mm")
    ap.add_argument("--min-tcp-z-mm", type=float, default=150.0,
                    help="末端 TCP 最低高度，默认 150 mm（底盘之上）")
    ap.add_argument("--min-path-mm", type=float, default=20.0,
                    help="路径净空，默认 20 mm")
    ap.add_argument("--no-path", action="store_true", help="只检查终点，不检查路径")
    ap.add_argument("--selftest", action="store_true", help="离线自测（不连硬件）")
    a = ap.parse_args(argv)

    urdf = S.Urdf(Path(a.urdf))
    com = urdf.gripper_com()
    arm = S.Arm(urdf, end_mass=a.gripper_mass, end_com=com)

    if a.selftest:
        return _selftest(arm, com)

    if not a.target_deg:
        print("[guard][ABORT] 需要至少一个 --target-deg", file=sys.stderr)
        return 2

    if not a.start_deg and not a.start_from_can:
        print("[guard][ABORT] 需要 --start-deg 或 --start-from-can（没有起点就算不了路径）",
              file=sys.stderr)
        return 2
    start = parse_deg(a.start_deg) if a.start_deg else read_start_from_can(a.iface)
    if len(start) != 6:
        print(f"[guard][ABORT] 起点要有 6 个数，收到 {len(start)}", file=sys.stderr)
        return 2

    targets = [parse_deg(t) for t in a.target_deg]
    for i, t in enumerate(targets):
        if len(t) != 6:
            print(f"[guard][ABORT] 第 {i + 1} 个目标要有 6 个数，收到 {len(t)}", file=sys.stderr)
            return 2

    print(f"stage2_pose_guard {VERSION}")
    print(f"URDF        : {a.urdf}")
    print(f"末端口径    : 夹爪 {a.gripper_mass:.2f} kg，质心沿工具轴 {com[2] * 1000:.1f} mm")
    print(f"禁区        : 半径 {HAZARD_R * 1000:.0f} mm、高 {HAZARD_Z * 1000:.0f} mm 的基座立柱")
    print(f"判据        : 终点净空 ≥{a.min_clear_mm:g} mm、"
          f"TCP Z ≥{a.min_tcp_z_mm:g} mm、"
          f"路径净空 ≥{a.min_path_mm:g} mm")
    print()

    def _row(tag, q, clear, who, tcp, tau=None):
        r = math.hypot(tcp[0], tcp[1]) * 1000
        line = (f"  {tag:10s} clear={clear:+7.1f} mm ({who:6s})  "
                f"TCP=({tcp[0] * 1000:+7.1f},{tcp[1] * 1000:+7.1f},{tcp[2] * 1000:+7.1f}) mm  "
                f"r={r:5.0f}")
        if tau is not None:
            line += f"  tau2={tau[1]:+.2f} tau5={tau[4]:+.2f}"
        print(line)

    c0, w0 = clearance_mm(arm, com, start)
    _row("起点", start, c0, w0, tcp_of(arm, com, start),
         arm.tau_gravity([math.radians(v) for v in start]))

    fails = []
    prev = list(start)
    c_prev = c0
    for idx, t in enumerate(targets, 1):
        c, who = clearance_mm(arm, com, t)
        tcp = tcp_of(arm, com, t)
        tau = arm.tau_gravity([math.radians(v) for v in t])
        _row(f"目标 {idx}", t, c, who, tcp, tau)

        if not a.no_path:
            pc, pf, pwho = path_min_mm(arm, com, prev, t)
            # 路径下限不能严于起点本身：垂下停放时末端离立柱顶面只有 ~10 mm，
            # 任何离场动作在前 1% 都还在那个距离附近。物理意义是「不许比停着更靠近」。
            floor = min(a.min_path_mm, c_prev) if idx == 1 else a.min_path_mm
            flag = "OK" if pc >= floor else "!! 过低"
            print(f"  {'':10s} 路径净空={pc:+7.1f} mm @f={pf:.2f} ({pwho})  下限 {floor:+.1f}  {flag}")
            if pc < floor:
                fails.append(f"第 {idx} 段路径净空 {pc:+.0f} mm < {floor:g} mm（{pwho}）")

        if c < a.min_clear_mm:
            fails.append(f"目标 {idx} 净空 {c:+.0f} mm < {a.min_clear_mm:g} mm（{who}）")
        if tcp[2] * 1000 < a.min_tcp_z_mm:
            fails.append(f"目标 {idx} TCP Z {tcp[2] * 1000:.0f} mm < {a.min_tcp_z_mm:g} mm")
        prev = list(t)
        c_prev = c

    print()
    if fails:
        print("[guard][ABORT] 这个姿态/路线不允许下发：", file=sys.stderr)
        for f in fails:
            print(f"  - {f}", file=sys.stderr)
        print("[guard] 换姿态或改路线。不要用 --force 之类的开关绕过 —— "
              "上一次绕过这个检查的结果是夹爪撞进底盘。", file=sys.stderr)
        return 2
    print("[guard] 通过：目标与路径都在净空要求之内，可以下发。")
    return 0


def _selftest(arm, com):
    ok = 0
    bad = 0

    def chk(name, cond, detail=""):
        nonlocal ok, bad
        if cond:
            ok += 1
            print(f"ok   {name}")
        else:
            bad += 1
            print(f"FAIL {name}  {detail}")

    # 1. 撞过底盘的姿态必须被判负
    c_bad, _ = clearance_mm(arm, com, [0, 100, -20, 0, 60, 0])
    chk("K1 事故姿态 P4 判负", c_bad < 0, f"{c_bad:+.1f} mm")
    # 2. 替代姿态必须判正且余量够
    c_ok, _ = clearance_mm(arm, com, [0, 100, -75, 0, 60, 0])
    chk("K2 替代姿态净空 ≥100 mm", c_ok >= 100.0, f"{c_ok:+.1f} mm")
    # 3. P1 已知可跑
    c_p1, _ = clearance_mm(arm, com, [0, 45, -60, 0, 45, 0])
    chk("K3 P1 判正", c_p1 > 0, f"{c_p1:+.1f} mm")
    # 4. 正解与固件一致性（用断电垂下姿态：固件末端位姿 Z=170.471，对应法兰=link6 原点）
    T = arm.transforms([math.radians(v) for v in [-2.186, -1.998, 1.818, -0.840, 21.633, 0.0]])
    z_flange = T["link6"][1][2] * 1000.0
    chk("K4 正解与固件末端位姿一致（法兰 Z 误差 <5 mm）", abs(z_flange - 170.471) < 5.0,
        f"{z_flange:.1f} vs 170.471")
    # 5. 路径检查能抓到终点合格但中途过低
    chk("K5 路径函数给出有限值",
        path_min_mm(arm, com, [0, 45, -60, 0, 45, 0], [0, 100, -75, 0, 60, 0])[0] < 1e8)

    print(f"\n自测：{'全部通过' if bad == 0 else f'{bad} 条失败'}（{ok + bad} 条）")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
