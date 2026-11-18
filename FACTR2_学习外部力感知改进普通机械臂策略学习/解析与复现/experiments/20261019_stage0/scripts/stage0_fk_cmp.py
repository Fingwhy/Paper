#!/usr/bin/env python3
"""判 2° 口径：SDK 的 FK（dh_is_offset=0/1）与固件自报末端位姿比谁更像。

背景（2026-10-20）：臂完全靠自重垂下时，固件读 J2 = −2.003°、J3 = +1.798°，
而固件自报的限位是 J2∈[0,180]、J3∈[−170,0]，两边各越界约 2°。手册 §10.3 有一条
「固件 ≥ S-V1.6-3 DH 变了（J2/J3 偏移 2°），SDK 里 dh_is_offset=1」。
两种解释必须分开：真 2° 偏移 / 「自然垂下」本身不是零点。

做法：读当前 q，分别用 dh_is_offset=0 和 1 算 FK（法兰位姿），
和固件 `GetArmEndPoseMsgs()` 报的位姿比。谁的平移残差小，谁的 DH 就是固件在用的。

用法（VM）：
    python3 stage0_fk_cmp.py            # 0 和 1 各连一次，顺序跑
"""
from __future__ import annotations

import math
import sys
import time


def read_q(piper):
    js = piper.GetArmJointMsgs().joint_state
    return [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]


def read_pose(piper):
    ep = piper.GetArmEndPoseMsgs().end_pose
    return [ep.X_axis / 1000.0, ep.Y_axis / 1000.0, ep.Z_axis / 1000.0,
            ep.RX_axis / 1000.0, ep.RY_axis / 1000.0, ep.RZ_axis / 1000.0]


def one(dh: int, n_frames: int = 60) -> dict:
    from piper_sdk import C_PiperInterface_V2

    p = C_PiperInterface_V2("can0", dh_is_offset=dh)
    p.ConnectPort()
    time.sleep(0.6)
    p.EnableFkCal()
    time.sleep(0.5)

    q = read_q(p)
    pose_fw = read_pose(p)

    # FK 多采几帧取最后一次，避免首帧默认值
    fk = None
    for _ in range(n_frames):
        fk = p.GetFK("feedback")
        time.sleep(0.01)
    # GetFK 返回 6 个 link 的位姿，-1 表示 joint6（末端法兰）
    fk6 = fk[-1]

    res = {"dh": dh, "q": q, "pose_fw": pose_fw,
           "fk_type": type(fk6).__name__, "fk6": fk6}
    try:
        p.DisconnectPort()
    except Exception:
        pass
    time.sleep(0.3)
    return res


def as_xyz(v):
    """FK 返回值可能是 list/tuple，也可能是带 x/y/z 的对象。"""
    if isinstance(v, (list, tuple)):
        return list(v)[:3]
    out = []
    for k in ("x", "y", "z", "X", "Y", "Z"):
        if hasattr(v, k):
            out.append(float(getattr(v, k)))
    return out[:3]


def main() -> int:
    print("顺序连两次（dh_is_offset=0 然后 1），约 4 s")
    r0 = one(0)
    r1 = one(1)

    print(f"\nq(deg)      = {[round(x, 3) for x in r0['q']]}")
    print(f"固件 pose   = {[round(x, 3) for x in r0['pose_fw']]}")
    print(f"FK 返回类型 = {r0['fk_type']}")

    print(f"\n{'dh':>3}  {'FK-位置 (mm)':>34}  {'对固件的平移残差 (mm)':>22}")
    best = None
    for r in (r0, r1):
        xyz = as_xyz(r["fk6"])
        if len(xyz) < 3:
            print(f"{r['dh']:>3}  取不到 xyz：{r['fk6']!r}")
            continue
        d = math.dist(xyz, r["pose_fw"][:3])
        print(f"{r['dh']:>3}  {str([round(v, 3) for v in xyz]):>34}  {d:>22.3f}")
        if best is None or d < best[1]:
            best = (r["dh"], d)

    if best:
        print(f"\n[判] 残差更小的是 dh_is_offset={best[0]}（{best[1]:.3f} mm）")
        if best[0] == 1:
            print("[判] ⇒ 固件用的是「带 2° 偏移」的 DH：喂给 URDF 的 q 需要减掉这个偏移。")
        else:
            print("[判] ⇒ 固件用的是「无偏移」的 DH：那 J2/J3 的 ±2° 不是 DH 造成的，"
                  "要另找原因（自然垂下位置本身不等于零点 / 编码器零点未校）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
