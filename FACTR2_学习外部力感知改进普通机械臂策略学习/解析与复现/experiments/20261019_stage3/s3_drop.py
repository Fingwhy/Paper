#!/usr/bin/env python3
"""算出本段 pre-pose 允许的最大末端下降量（mm），供 run_s3.sh 传给 --max-drop-mm。

为什么需要它
------------
run_s3.sh 原来把 `--max-drop-mm` **写死 400.0**（第 37 行有 `MAX_DROP_MM` 这个变量，
但两处调用根本没引用它）。2026-10-20 14:4x 的后果：
`E2_J3deep (0,60,-140,0,35,30)` → `E3_wrist (0,90,-60,60,50,90)` 计划就要降约 400 mm，
实测降了 **400.3 mm**，差 0.3 mm 触发高度守卫 → pre-pose 退出码 4 → 整批停在 33/42，
连带 V1/V2/V3/T1/T2/T3 六段全没跑。

口径必须与守卫一致
------------------
守卫监视的是**固件末端位姿 z**（`GetArmEndPoseMsgs`）。2026-10-20 14:5x 实测标定：
固件 z=377.6 mm 时，URDF `link6` 原点 z=377.5 mm、xyz=(25.2,−3.1,377.5) 与固件逐位一致；
而带夹爪质心偏移 `com=(0,0,0.0376)` 时 z=388.2 mm，明显对不上。
⇒ **固件末端位姿 == URDF link6 原点，不含工具偏置**，所以这里一律用 `com=(0,0,0)`。
（旧记录 R45 说「两点偏差 −15~0 mm」是拿 com=0.0376 比出来的，那个 10 mm 就是错的来源。）

R45 的公式：阈值 = |计划下降| + 150 mm。加 150 是给「到位残差 0.5°」和模型误差留余量。

用法
----
    python3 s3_drop.py --iface can0 --target-deg 0,90,-60,60,50,90
    输出一行整数（mm），给 `--max-drop-mm` 用。
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import s3_design as D          # noqa: E402
import stage2_pose_guard as P  # noqa: E402

COM0 = (0.0, 0.0, 0.0)         # 与固件末端位姿同口径（见上面「口径必须与守卫一致」）


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--target-deg", required=True,
                    help='目标姿态，逗号或空格分隔，如 "0,90,-60,60,50,90"')
    ap.add_argument("--margin", type=float, default=150.0,
                    help="在 |计划下降| 之上再放宽多少 mm，默认 150（R45）")
    ap.add_argument("--min-mm", type=float, default=150.0,
                    help="下限，默认 150（不允许比它更松，也不允许比它更严）")
    ap.add_argument("--fallback-mm", type=float, default=600.0,
                    help="读不到真实起始姿态时用的兜底阈值，默认 600")
    args = ap.parse_args(argv)

    tgt = P.parse_deg([args.target_deg])
    if len(tgt) != 6:
        ap.error(f"--target-deg 需要 6 个值，收到 {len(tgt)} 个")

    arm = D.load()[1]

    q0 = P.read_start_from_can(args.iface)
    # 六个关节全为 0.0 是 SDK 收不到帧时的默认值（R18），不是真实姿态。
    if not q0 or all(abs(v) < 1e-9 for v in q0):
        print(f"[drop][WARN] CAN 没读到有效姿态（{q0}），改用兜底阈值 "
              f"{args.fallback_mm:g} mm —— 这一段的下降守卫会变松，请盯一下日志",
              file=sys.stderr)
        print(f"{args.fallback_mm:.0f}")
        return 0

    z0 = P.tcp_of(arm, COM0, q0)[2] * 1000.0
    z1 = P.tcp_of(arm, COM0, tgt)[2] * 1000.0
    drop = z0 - z1
    thr = max(args.min_mm, abs(drop) + args.margin)
    print(f"[drop] 起始 z={z0:.0f} mm → 目标 z={z1:.0f} mm，计划下降 {drop:+.0f} mm "
          f"⇒ 允许 {thr:.0f} mm", file=sys.stderr)
    print(f"{thr:.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
