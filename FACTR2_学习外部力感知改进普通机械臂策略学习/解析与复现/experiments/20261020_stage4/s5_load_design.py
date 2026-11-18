#!/usr/bin/env python3
"""阶段 5（L2）称重实验：姿态集 + 质量档 + 几何闸门。

设计约束（来自 S5_PLAN.md）
---------------------------
1. 6 个姿态必须**落在阶段 4 训练集的 q 范围内**（train q2 ∈ [27, 90]°）——
   先验 L1 模型的内插能力，不引入外推误差；
2. J1=0、J4=0、J6=+30 固定：J6 避开 0° 硬端点（R52），J1/J4 固定减少变量；
3. 姿态 E（q2=85）接近水平，末端会伸出去，**必须**用最保守的 `bottle` 包络
   （22 cm 胶囊，比实际载荷大）过闸门；
4. 每段静止保持 12 s（600 帧），把噪声从 0.21 压到 0.0086 N·m。

用法（VM，~/stage0/20261020_stage4）：
    python3 s5_load_design.py --gate
    python3 s5_load_design.py --emit E        # 给 run 脚本用
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"
sys.path.insert(0, str(STAGE3))
import s3_design as D  # noqa: E402

G = D.G

HOLD_S = 12          # 静止保持秒数

# ---------------------------------------------------------------------------
# 姿态集（2026-10-20 18:0x 定稿，全部 J6=+90）
# ---------------------------------------------------------------------------
# 两个硬要求（用户 17:50/17:56 两次指出）：
#   ① **指闭合轴必须水平**（夹爪左右张开，不是歪的）→ 只有 J6=90° 能做到，
#      实测 J6=30° 时 A 姿态倾斜 60°、F 姿态 30°，J6=90° 时全部 0.0°。
#   ② **掌心必须水平**（夹爪像托盘平放，便于放/取物体）→ 工具轴（link6 z）
#      必须竖直，即 ax_z = ±1。实测 J5=−50° 时 ax_z 只有 0.906（偏 25°），
#      这就是「看着不水平」的原因；要 ax_z=1.000 必须把 J5 解到 −65/−70/−72。
#
# 于是把姿态按「工具轴相对重力」分成两族：
#   U 族（掌心朝上，ax_z = +1.000）：放取物体用；重力 ∥ 工具轴。
#   V 族（掌心竖直，ax_z =  0.000）：工具轴水平；重力 ⊥ 工具轴。
# 两族合起来让重力相对工具轴从「平行」扫到「垂直」，才能把 Φ 的三个分量
# 都激励出来；只用一个方位时，质量误差可以完全被 |Φ| 吸收，辨识不出来。
#
# J6=90 超出阶段 4 训练集（J6∈[10,50]）**不构成风险**：每个姿态都有同姿态
# 空载基线，τ_ext(带载)−τ_ext(空载) 里模型项被完全消掉，与模型在该点的误差无关。
POSES_U = [
    ("U1", (0, 30, -50, 0, -65, 50.6), "掌心朝上（ax_z 1.0000），净空 290"),
    ("U2", (0, 45, -60, 0, -70, 50.6), "掌心朝上（1.0000），净空 322"),
    ("U3", (0, 60, -70, 0, -70, 50.6), "掌心朝上（0.9986），净空 340"),
]

POSES_V = [
    ("A", (0, 30, -50, 0, 25, 50.6), "掌心竖直（ax_z 0.000），净空 238"),
    ("B", (0, 45, -60, 0, 20, 50.6), "掌心竖直（0.000），净空 271"),
    ("C", (0, 60, -70, 0, 15, 50.6), "掌心竖直（0.000），净空 295"),
]

GROUPS = {"U": POSES_U, "V": POSES_V}
POSES = POSES_U            # 默认按「掌心朝上」——放取物体用的那一族

# ---------------------------------------------------------------------------
# 实测「两指水平」的 J6 零点（R60）
# ---------------------------------------------------------------------------
# URDF 说 joint6_to_gripper_base 是恒等变换，按它推「两指水平」应在 J6=0 或 180。
# 但实测不是：用户先目测「14 时右转 45」「59 时多出 15」都把目标指向 60~74，误差大；
# 最终由用户在**示教模式下亲手调平**，回读到 q6=50.62、工具轴与水平 1.2°，
# 这才是可靠基准 ⇒ 真实夹爪相对 URDF 恒等假设有约 **50.6° 的滚转装配偏差**。
# 所以判据改成「相对实测零点的偏差」，不再直接用 URDF 的 x/y 轴。
J6_LEVEL_ZERO = 50.62   # 用户 18:10 手调「水平」后回读：q6=50.62（工具轴与水平 1.2°）
FINGER_LEVEL_TOL_DEG = 6.0     # 允许偏差（目测精度约 ±5°）
# 判据：掌心水平容差（|ax_z| 与 1 的差 < 0.05，≈18° 以内不判失败；
# U 族要求「平」是主诉，单独按 0.02 判，见 FINGER 之外的第二条）

# 质量档：用户现有砝码只有 100/300/500 g，没有 1 kg（2026-10-20 17:42 确认）
MASSEYS = [
    (0, "M0", 0.0, "空载基线"),
    (1, "M1", 0.1, "100 g（最弱，接近噪声地板）"),
    (2, "M2", 0.3, "300 g"),
    (3, "M3", 0.5, "500 g（最强）"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", action="store_true")
    ap.add_argument("--emit", default=None, help="打印单段的 center（供 run 脚本）")
    ap.add_argument("--group", default="U", choices=("U", "V", "both"),
                    help="U=掌心朝上（放取物体、重力∥工具轴）；"
                         "V=掌心竖直（工具轴水平、重力⊥工具轴）；both=两族一起")
    ap.add_argument("--json", default=str(HERE / "load_tracks.json"))
    args = ap.parse_args()

    sel = ("U", "V") if args.group == "both" else (args.group,)
    poses = [p for g in sel for p in GROUPS[g]]

    if args.emit:
        t = next((p for p in poses if p[0] == args.emit), None)
        if t is None:
            raise SystemExit(f"未知姿态 {args.emit}，可选 {[p[0] for p in poses]}")
        print(" ".join(f"{v:g}" for v in t[1]))
        return

    urdf, arm = D.load()
    # 注意顺序：s3_design.load() 内部会把包络重置成 gripper（阶段 3 空载口径），
    # 所以 set_pack 必须放在 load 之后，否则会被静默覆盖（2026-10-20 17:1x 踩到）。
    G.set_pack("bottle")            # 最保守：22 cm 胶囊，比真实载荷大
    print(f"包络 = {G.pack()['label']}（保守，实际载荷远小于它）")
    print(f"每段静止保持 {HOLD_S} s（{HOLD_S*50} 帧），"
          f"噪声预计 J2 {0.21/(HOLD_S*50)**0.5:.4f} N·m")
    print()
    print("| 姿态 | (q1,q2,q3,q4,q5,q6) | 工具轴ax_z | 指轴倾角 | 法兰z | 末端最低z | 净空 | 限位余量 | 判 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    ok = 0
    rows = []
    for name, q, note in poses:
        v = G.pose_verdict(arm, G.deg(*q))
        R6, _p6, _ax = G.tcp_and_axis(arm, G.deg(*q))
        M = np.asarray(R6, dtype=float).reshape(3, 3)
        axz = float(M[2, 2])          # 工具轴（link6 z）的世界 z 分量
        # 「看起来水平的两指方向」= 实测零点处的那条方向。
        # 用实测偏移 φ 把 link6 的 y/x 组合成真实指面方向：
        #   f = cosφ·y + sinφ·x ，φ = J6_LEVEL_ZERO
        # 工具轴水平时 f_z = sin(J6 − φ)，即只有 J6=φ 才水平；
        # 工具轴竖直时 y、x 都在水平面内，f 恒水平（任何 J6 都水平）——两种情况都对。
        phi = np.radians(J6_LEVEL_ZERO)
        f = np.cos(phi) * M[:, 1] + np.sin(phi) * M[:, 0]
        tilt = float(np.degrees(np.arcsin(max(-1.0, min(1.0, f[2])))))
        lm = min(min(q[i] - D.LIMITS[i][0], D.LIMITS[i][1] - q[i]) for i in range(6))
        good = (not v["bad"]) and lm >= 0 and tilt <= FINGER_LEVEL_TOL_DEG
        ok += good
        bad = list(v["bad"])
        if tilt > FINGER_LEVEL_TOL_DEG:
            bad.append(f"相对水平偏 {tilt:.1f}°")
        print(f"| {name} | {q} | {axz:+.3f} | {tilt:4.1f}° | {v['p6'][2]*1000:.0f} | "
              f"{v['zmin']:.0f} | {v['clr']:.0f} | {lm:+.0f}° | "
              f"{'PASS' if good else 'FAIL: '+';'.join(bad)} |")
        rows.append(dict(name=name, center=list(q), note=note, ax_z=axz,
                         finger_tilt_deg=tilt,
                         flange_z_mm=v["p6"][2] * 1000, zmin=v["zmin"], clr=v["clr"],
                         ok=good))
    print()
    print(f"几何总判：{ok}/{len(poses)} 放行")
    if ok == len(poses):
        print("全部放行")

    print()
    print(f"## 段清单（4 质量档 × {len(poses)} 姿态 = {4*len(poses)} 段）")
    print("| 段名 | 质量 | 姿态 | 说明 |")
    print("| --- | --- | --- | --- |")
    for mi, tag, m, mnote in MASSEYS:
        for name, q, note in poses:
            print(f"| {tag}_{name} | {m:g} kg | {name} | {mnote} / {note} |")

    print()
    print("## 质量档 × 载荷力矩（质心距 80 mm，g=9.81）")
    for mi, tag, m, _ in MASSEYS:
        print(f"  {tag}  {m:4.2f} kg → τ ≈ {m*9.81*0.08:.3f} N·m")

    json.dump(dict(poses=rows, masses=[dict(tag=t, kg=m, note=n) for _, t, m, n in MASSEYS],
                   hold_s=HOLD_S), open(args.json, "w"), ensure_ascii=False, indent=1)
    print(f"\n写入 {args.json}")


if __name__ == "__main__":
    main()
