#!/usr/bin/env python3
"""stage0_set_zero.py -- 单关节零点设置（0x475，set_zero=0xAE），带前置守卫与回读判据

在【机器人机 / VM 上】运行（Ubuntu + conda 环境 ros + piper_sdk 0.6.1 + can0 已拉起）。

为什么不用官方 demo piper_set_joint_zero.py
--------------------------------------------
三个都会咬人的副作用：

1. 它启动就 `EnablePiper()` → `MotionCtrl(0x01,0x01,30,0)` → `JointCtrl(0,0,0,0,0,0)`，
   臂若不在零点附近，**回车那一刻会大幅运动**；
2. 它无条件发 `GripperCtrl(0,1000,0x01,0)`，与「标定期夹爪断电」的约定冲突，多一帧
   无响应指令（R33：它返回 None，失败只写 logger）；
3. `JointConfig()` 同样返回 None，**打印的「设置成功」不来自任何回读**（R31/R33 同型）。

本脚本的规矩
------------
1. **不自动回零**（`JointCtrl(0,...,0)` 会让臂动），设置完只回读，不动臂；
2. **不默认失能/使能**：`--disable-first` 才会动，且只失能目标关节（其余关节仍抱住，
   臂不会掉）；使能也要显式 `--enable-after`；
3. **三道守卫任一不过就拒绝发帧**：
       G1 数据活：采样窗口内时间戳唯一值 ≥ 50（挡住 R18「读全 0 其实是没收到帧」）
       G2 静止：窗口内目标关节角度极差 ≤ --still-tol（运动中以「当前位姿」设零点，
         零点会带进速度/滞后误差）
       G3 目标合法：joint ∈ 1..6；7 号（夹爪行程零点）直接拒绝 —— 本脚本不读 0x2A8，
         拿不到回读判据，按纪律就不该发这种帧
4. **判据只有回读**：设完重读，目标关节 |q| ≤ --tol 且其余关节不动，才算成功。

用法
----
    python3 stage0_set_zero.py check                    # 只读：固件/使能/六轴角/位姿
    python3 stage0_set_zero.py set --joint 6 --disable-first --enable-after
    python3 stage0_set_zero.py set --joint 6 --yes       # 已自行失能摆好位，跳过最终确认
    python3 stage0_set_zero.py selftest                  # 离线，任何机器都能跑

产物：env/zero_set_J<joint>_<时间戳>.json（--out 可改）
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# 手册 §2.2.3 给出的零点末端位姿（法兰口径）。只作人工旁证，不参与任何计算：
# 装了夹爪后若固件按带爪 TCP 算，Z 会多出约 37.6 mm。
REF_ZERO_POSE = {"X": 54.952, "Y": 0.0, "Z": 203.386,
                 "RX": 0.0, "RY": 85.0, "RZ": 0.0}

# 守卫阈值（可被命令行覆盖）
MIN_UNIQUE_STAMPS = 50        # G1：1.5 s @200 Hz 期望 ~300，取 50 已是宽容下限
DEFAULT_STILL_TOL = 0.20      # G2：静止判定，单位度
DEFAULT_SET_TOL = 0.15        # 设完目标关节应落在 0 ± 该值
DEFAULT_SIDE_TOL = 0.20       # 设完其余关节相对 before 的允许变化

VERSION = "1.2"


# ---------------------------------------------------------------- 纯函数（可离线自测）

def encode_joint_config(joint: int, set_zero: bool = True,
                        max_joint_acc: int = 0x7FFF) -> bytes:
    """0x475 关节设置帧，8 字节。对齐 SDK 的 ArmMsgJointConfig 位描述。

    Byte0 关节号 1-7 / Byte1 设零点 0xAE / Byte2 加速度参数是否生效（0=不生效）
    Byte3-4 最大加速度 uint16 大端（0x7FFF=设定无效，V1.5-2 及以后有效）
    Byte5 清错 0xAE / Byte6-7 保留

    加速度位写 0x7FFF 而不是 500 是有意的：参数不生效时也不该夹带一个新值，
    免得哪天固件改成「总是解析」时把加速度悄悄改掉。
    """
    if not 1 <= joint <= 7:
        raise ValueError(f"joint 越界：{joint}")
    if not 0 <= max_joint_acc <= 500 and max_joint_acc != 0x7FFF:
        raise ValueError(f"max_joint_acc 越界：{max_joint_acc}")
    return bytes([joint & 0xFF,
                  0xAE if set_zero else 0x00,
                  0x00,
                  (max_joint_acc >> 8) & 0xFF,
                  max_joint_acc & 0xFF,
                  0x00,
                  0x00,
                  0x00])


def spread(values: list[float]) -> float:
    """极差。窗口内目标关节用它判静止。"""
    return (max(values) - min(values)) if values else float("inf")


def judge_still(q_series: list[float], tol: float) -> tuple[bool, float]:
    sp = spread(q_series)
    return (sp <= tol), sp


def judge_settled(q_after: list[float], q_before: list[float], joint: int,
                  tol: float, side_tol: float) -> dict:
    """设完的判据：目标关节贴 0，其余关节不许被连坐。joint 为 1-based。"""
    idx = joint - 1
    target = q_after[idx]
    side = [abs(q_after[i] - q_before[i]) for i in range(6) if i != idx]
    return {
        "target_deg": round(target, 4),
        "target_ok": abs(target) <= tol,
        "side_max_delta_deg": round(max(side), 4) if side else 0.0,
        "side_ok": (max(side) <= side_tol) if side else True,
        "ok": abs(target) <= tol and ((max(side) <= side_tol) if side else True),
    }


# ---------------------------------------------------------------- 与机器人交互

def _load_sdk():
    try:
        from piper_sdk import C_PiperInterface_V2
    except Exception as exc:  # pragma: no cover - 只在真机上走
        raise SystemExit(
            f"导入 piper_sdk 失败：{exc!r}\n"
            "本机 piper_sdk 装在系统 python3 的 ~/.local 下，不需要 conda。\n"
            "先确认 `python3 -c \"import piper_sdk; print(piper_sdk.__file__)\"` 能打印路径。")
    return C_PiperInterface_V2


def connect(iface: str):
    cls = _load_sdk()
    piper = cls(iface)
    ret = piper.ConnectPort()
    # piper_sdk 0.6.1 的 ConnectPort() 方法体没有 return，返回 None；老版本返回 bool。
    # 所以不能写 `if not ret` —— 那会把 None 当成 False 直接判失败（2026-10-20 实测踩到）。
    # 真正的连通性判据放在后面 G1（唯一时间戳计数），不靠返回值。
    if ret is False:
        raise SystemExit(f"ConnectPort() 明确返回 False：{iface} 打不开。"
                         f"先查 `ip -br link show {iface}`")
    time.sleep(0.3)  # 等首帧，否则读到的是新建对象的默认值（全 0）
    return piper


def read_state(piper) -> dict:
    msgs = piper.GetArmJointMsgs()
    js = msgs.joint_state
    q = [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]
    ep = piper.GetArmEndPoseMsgs().end_pose
    pose = [ep.X_axis / 1000.0, ep.Y_axis / 1000.0, ep.Z_axis / 1000.0,
            ep.RX_axis / 1000.0, ep.RY_axis / 1000.0, ep.RZ_axis / 1000.0]
    return {"q_deg": q, "pose": pose,
            "time_stamp": float(getattr(msgs, "time_stamp", 0.0) or 0.0),
            "hz": float(getattr(msgs, "Hz", 0.0) or 0.0)}


def sample(piper, seconds: float, period: float = 0.005) -> dict:
    """按固定周期读，统计「时间戳唯一值」判数据是否真在更新。"""
    t_end = time.time() + seconds
    stamps: set[float] = set()
    q_rows: list[list[float]] = []
    last = None
    while time.time() < t_end:
        st = read_state(piper)
        stamps.add(st["time_stamp"])
        q_rows.append(st["q_deg"])
        last = st
        time.sleep(period)
    if last is None:
        raise SystemExit("没有采到任何样本：检查 can0 是否 UP、臂是否上电")
    q_cols = [[row[i] for row in q_rows] for i in range(6)]
    return {"q_last": last["q_deg"], "pose": last["pose"],
            "n_samples": len(q_rows), "n_unique_stamps": len(stamps),
            "hz_field": last["hz"], "q_cols": q_cols}


def enable_state(piper) -> dict:
    """逐轴使能状态。

    坑：0.6.1 的 GetArmEnableStatus() 返回**六个 bool 的 list**，不是单个 bool。
    写 `bool(GetArmEnableStatus())` 永远得到 True（非空列表恒真），2026-10-20 实测
    因此把「全臂失能」误报成「使能 True」。必须逐轴取。
    """
    try:
        lst = list(piper.GetArmEnableStatus())
        return {"enabled": all(lst) if lst else None,
                "per_joint": [bool(x) for x in lst],
                "any_enabled": any(lst) if lst else None}
    except Exception as exc:
        return {"enabled": None, "per_joint": None, "error": repr(exc)}


def firmware(piper) -> str:
    try:
        return str(piper.GetPiperFirmwareVersion())
    except Exception as exc:
        return f"unavailable({exc!r})"


def fmt(v: list[float], nd: int = 3) -> str:
    return "[" + ", ".join(f"{x:.{nd}f}" for x in v) + "]"


# ---------------------------------------------------------------- 子命令

def cmd_check(args) -> int:
    piper = connect(args.iface)
    st = read_state(piper)
    smp = sample(piper, max(0.4, args.settle))
    en = enable_state(piper)
    print(f"固件        : {firmware(piper)}")
    print(f"使能        : {en.get('enabled')}  逐轴={en.get('per_joint')}")
    print(f"六轴角 (deg): {fmt(smp['q_last'])}")
    print(f"末端位姿    : {fmt(smp['pose'])}")
    print(f"数据活性    : {smp['n_unique_stamps']} 个唯一时间戳 / {smp['n_samples']} 次采样"
          f"（Hz 字段 {smp['hz_field']:.1f}）")
    print(f"手册零点旁证: {REF_ZERO_POSE}")
    ok = smp["n_unique_stamps"] >= MIN_UNIQUE_STAMPS
    print(f"G1 数据活   : {'通过' if ok else '不通过（读数可能是默认值，禁止设零点）'}")
    return 0 if ok else 2


def hold_joints(piper, joints, gap: float = 0.08) -> None:
    """逐轴发 0x471 使能。byte0 是「电机序号」不是位掩码，只能一个一个发。"""
    for j in joints:
        piper.EnableArm(j)
        time.sleep(gap)


def cmd_hold(args) -> int:
    """使能 J1..J5，把臂抱成刚体；J6 留给你手拧。

    为什么必须先做这一步：上电后六轴默认全失能，整条臂是软的。设 J6 零点时
    J6 必须失能（否则电机抱死，手拧不动），如果此时没有别的关节抱住，拧 J6
    会把整条臂拖着转 —— G2「静止」判据过不去，噪声也会写进零点基准。
    """
    piper = connect(args.iface)
    print(f"[env] 固件={firmware(piper)}  接口={args.iface}  版本={VERSION}")
    before = sample(piper, 0.5)
    if before["n_unique_stamps"] < MIN_UNIQUE_STAMPS and not args.force:
        print(f"[拒绝] G1 不通过：唯一时间戳 {before['n_unique_stamps']} < {MIN_UNIQUE_STAMPS}。"
              "没收到帧时读到的是默认值（R18），这次不发力矩。", file=sys.stderr)
        return 2
    print(f"[before] 六轴角 = {fmt(before['q_last'])}")
    print(f"[hold ] 逐轴使能 J{args.from_joint}..J{args.to_joint}"
          f"（区间外的关节不动）")
    hold_joints(piper, range(args.from_joint, args.to_joint + 1))
    time.sleep(0.5)
    after = sample(piper, 0.8)
    dq = [a - b for a, b in zip(after["q_last"], before["q_last"])]
    print(f"[after ] 六轴角 = {fmt(after['q_last'])}")
    print(f"[after ] 使能瞬间位移 = {fmt(dq)}  （超过 1° 说明臂在抱死前动过，正常现象但记下来）")
    print(f"[after ] 数据活性 = {after['n_unique_stamps']} 个唯一时间戳")
    print(f"[hold ] 完成。J{args.from_joint}..J{args.to_joint} 已锁死。")
    return 0


def cmd_release(args) -> int:
    """只失能目标关节，把「手拧」这一步单独拆出来做。

    为什么拆开：设零点的流程里，人手停顿的时间不可控。把「松开关节」和
    「写零点」分成两条命令，中间由操作者确认，比一个长 sleep 可靠 ——
    人没拧到位就不会有第二条命令。其余五轴保持原使能状态，臂不会塌。
    """
    joint = args.joint
    if not 1 <= joint <= 6:
        print(f"[拒绝] joint 只能是 1..6，收到 {joint}", file=sys.stderr)
        return 2
    piper = connect(args.iface)
    print(f"[env] 固件={firmware(piper)}  接口={args.iface}  版本={VERSION}")
    before = sample(piper, 0.5)
    if before["n_unique_stamps"] < MIN_UNIQUE_STAMPS and not args.force:
        print(f"[拒绝] G1 不通过：唯一时间戳 {before['n_unique_stamps']} < {MIN_UNIQUE_STAMPS}",
              file=sys.stderr)
        return 2
    print(f"[before] 六轴角 = {fmt(before['q_last'])}")
    print(f"[release] 只失能 J{joint}，其余五轴保持使能")
    piper.DisableArm(joint)
    time.sleep(0.4)
    after = sample(piper, 0.8)
    dq = [a - b for a, b in zip(after["q_last"], before["q_last"])]
    print(f"[after ] 六轴角 = {fmt(after['q_last'])}")
    print(f"[after ] 失能瞬间位移 = {fmt(dq)}（超过 1° 说明失能前臂在动）")
    print(f"\nJ{joint} 现在是软的，可以手拧了。把标记对准中间凹槽后，再跑 set（不要加 --disable-first）。")
    return 0


def cmd_set(args) -> int:
    joint = args.joint
    if not 1 <= joint <= 6:
        print(f"[拒绝] joint 只能是 1..6，收到 {joint}。7 号是夹爪行程零点：本脚本不读 0x2A8，"
              "没有回读判据，按规矩不该发这种帧。真要动它，用官方 "
              "piper_set_joint_zero_cpv.py 的 mode 2 → 2（捏紧闭合再标定）。", file=sys.stderr)
        return 2

    piper = connect(args.iface)
    fw = firmware(piper)
    print(f"[env] 固件={fw}  接口={args.iface}  版本={VERSION}")

    if args.hold_first:
        others = [j for j in range(1, 7) if j != joint]
        print(f"[hold ] 先抱住 J{others}，只留 J{joint} 是软的")
        hold_joints(piper, others)
        time.sleep(0.4)

    if args.disable_first:
        print(f"[失能] 只失能 J{joint}（其余关节保持抱住，臂不会掉）")
        piper.DisableArm(joint)
        time.sleep(0.2)
        input("现在手动把 J"
              f"{joint} 转到限位位置（J6 = 对准中间凹槽），摆好并松手后按回车：")
        time.sleep(0.3)

    smp = sample(piper, args.settle)
    q_before = list(smp["q_last"])
    print(f"[before] 六轴角 = {fmt(q_before)}")
    print(f"[before] 数据活性 = {smp['n_unique_stamps']} 个唯一时间戳 / {smp['n_samples']} 次采样")

    if smp["n_unique_stamps"] < MIN_UNIQUE_STAMPS and not args.force:
        print(f"[拒绝] G1 不通过：唯一时间戳 {smp['n_unique_stamps']} < {MIN_UNIQUE_STAMPS}。"
              "很可能根本没收到帧，此时读到的全 0 是默认值（R18），设零点会把任意姿态永久定义为 0。"
              "先排除线缆/上电/接口，确认真要这么做再加 --force。", file=sys.stderr)
        return 2

    still_ok, sp = judge_still(smp["q_cols"][joint - 1], args.still_tol)
    print(f"[guard] G2 静止：J{joint} 窗口极差 {sp:.4f}°（阈值 {args.still_tol}°）"
          f" → {'通过' if still_ok else '不通过'}")
    if not still_ok and not args.force:
        print("[拒绝] 目标关节还在动。设零点会把速度/滞后误差一起写进基准。"
              "等它停稳（或加大 --still-tol）再试。", file=sys.stderr)
        return 2

    if not args.yes:
        print(f"\n即将发送 0x475：joint={joint}, set_zero=0xAE。"
              f"当前 J{joint} 读数 {q_before[joint - 1]:.3f}° 将被定义为 0。")
        if input("确认输入 SET-ZERO：").strip() != "SET-ZERO":
            print("[取消] 未发帧。")
            return 1

    frame = encode_joint_config(joint, set_zero=True)
    print(f"[send] 0x475 <- {frame.hex(' ')}")
    piper.JointConfig(joint, 0xAE, 0x00, 0x7FFF, 0x00)

    time.sleep(0.4)
    after = sample(piper, 0.6)
    q_after = list(after["q_last"])
    judge = judge_settled(q_after, q_before, joint, args.tol, args.side_tol)
    print(f"[after ] 六轴角 = {fmt(q_after)}")
    print(f"[judge ] J{joint} = {judge['target_deg']:.4f}°（容差 ±{args.tol}）→ "
          f"{'贴 0' if judge['target_ok'] else '未贴 0'}")
    print(f"[judge ] 其余关节最大变化 {judge['side_max_delta_deg']:.4f}°（容差 {args.side_tol}）"
          f" → {'未连坐' if judge['side_ok'] else '有连坐，需排查'}")

    if args.enable_after:
        print(f"[使能] J{joint}")
        piper.EnableArm(joint)
        time.sleep(0.2)
        print(f"[使能] 回读 = {enable_state(piper).get('enabled')}")

    rec = {
        "tool": "stage0_set_zero.py", "version": VERSION,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "iface": args.iface, "firmware": fw,
        "joint": joint, "frame_0x475": frame.hex(" "),
        "before": {"q_deg": [round(v, 6) for v in q_before],
                   "pose": [round(v, 3) for v in smp["pose"]],
                   "n_unique_stamps": smp["n_unique_stamps"],
                   "n_samples": smp["n_samples"]},
        "after": {"q_deg": [round(v, 6) for v in q_after],
                  "pose": [round(v, 3) for v in after["pose"]],
                  "n_unique_stamps": after["n_unique_stamps"]},
        "guards": {"G1_data_alive": smp["n_unique_stamps"] >= MIN_UNIQUE_STAMPS,
                   "G2_still": still_ok, "G2_spread_deg": round(sp, 4)},
        "judge": judge,
        "flags": {"disable_first": args.disable_first, "enable_after": args.enable_after,
                  "force": args.force},
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rec, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[out  ] {out}")

    if not judge["ok"]:
        print("[失败] 回读判据未通过。不要假设「已设置成功」——先跑 check 复核，"
              "必要时用 CPV 模式从机械限位重新对齐。", file=sys.stderr)
        return 3
    print(f"[通过] J{joint} 零点已生效，其余关节未受影响。")
    return 0


def cmd_selftest(args) -> int:
    """离线自测：编码字节序 + 判据边界。不需要 CAN、不需要 piper_sdk。"""
    fails = []

    def ck(name, cond):
        print(f"  {'ok  ' if cond else 'FAIL'} {name}")
        if not cond:
            fails.append(name)

    f = encode_joint_config(6)
    ck("0x475 长度 8", len(f) == 8)
    ck("0x475 byte0=关节号", f[0] == 6)
    ck("0x475 byte1=0xAE", f[1] == 0xAE)
    ck("0x475 byte2=0 加速度不生效", f[2] == 0x00)
    ck("0x475 byte3-4=0x7FFF 大端", f[3] == 0x7F and f[4] == 0xFF)
    ck("0x475 byte5=0 不清错", f[5] == 0x00)
    ck("0x475 不设零时 byte1=0", encode_joint_config(1, set_zero=False)[1] == 0x00)

    for bad in (0, 8, -1):
        try:
            encode_joint_config(bad)
            ck(f"关节号 {bad} 应被拒", False)
        except ValueError:
            ck(f"关节号 {bad} 被拒", True)

    ck("静止：极差 0.05 通过", judge_still([1.00, 1.03, 1.05], 0.2)[0])
    ck("静止：极差 0.9 不通过", not judge_still([1.0, 1.9], 0.2)[0])

    qb = [0, 0, 0, 0, 0, 37.5]
    qa = [0, 0, 0, 0, 0, 0.03]
    j = judge_settled(qa, qb, 6, 0.15, 0.2)
    ck("贴 0 判通过", j["ok"] and j["side_ok"])

    qa_bad = [0, 0, 0, 0, 0, 12.0]
    ck("未贴 0 判失败", not judge_settled(qa_bad, qb, 6, 0.15, 0.2)["ok"])

    qa_side = [0, 0, 0, 0, 2.0, 0.0]
    ck("连坐被识别", not judge_settled(qa_side, qb, 6, 0.15, 0.2)["side_ok"])

    ck("极差函数空列表返回 inf", spread([]) == float("inf"))

    print(f"\n自测：{'全部通过' if not fails else f'{len(fails)} 条失败 → {fails}'}")
    return 0 if not fails else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="PiPER 单关节零点设置（0x475/0xAE），带守卫与回读判据")
    p.add_argument("--version", action="version", version=f"stage0_set_zero {VERSION}")
    sub = p.add_subparsers(dest="cmd", required=True)

    pc = sub.add_parser("check", help="只读：固件/使能/六轴角/位姿/数据活性")
    pc.add_argument("--iface", default="can0")
    pc.add_argument("--settle", type=float, default=0.8)
    pc.set_defaults(func=cmd_check)

    ps = sub.add_parser("set", help="对指定关节设零点（0xAE）")
    ps.add_argument("--joint", type=int, required=True, help="1..6（J6 = 装夹爪后要重设的那个）")
    ps.add_argument("--iface", default="can0")
    ps.add_argument("--settle", type=float, default=1.5, help="设零前采样窗口，秒")
    ps.add_argument("--still-tol", type=float, default=DEFAULT_STILL_TOL, help="静止判定阈值（度）")
    ps.add_argument("--tol", type=float, default=DEFAULT_SET_TOL, help="设完目标关节容差（度）")
    ps.add_argument("--side-tol", type=float, default=DEFAULT_SIDE_TOL, help="其余关节允许变化（度）")
    ps.add_argument("--disable-first", action="store_true", help="先只失能目标关节，再手动摆位")
    ps.add_argument("--hold-first", action="store_true",
                    help="先使能其余五轴把臂抱成刚体（拧 J6 时不拖着整条臂动）")
    ps.add_argument("--enable-after", action="store_true", help="设完重新使能目标关节")
    ps.add_argument("--force", action="store_true", help="跳过 G1/G2 拦截（危险）")
    ps.add_argument("--yes", action="store_true", help="跳过最终 SET-ZERO 确认")
    ps.add_argument("--out", default=None)
    ps.set_defaults(func=cmd_set)

    pt = sub.add_parser("selftest", help="离线自测")
    pt.set_defaults(func=cmd_selftest)

    ph = sub.add_parser("hold", help="使能 J1..J5，把臂抱成刚体（J6 留软）")
    ph.add_argument("--iface", default="can0")
    ph.add_argument("--from-joint", type=int, default=1)
    ph.add_argument("--to-joint", type=int, default=5)
    ph.add_argument("--force", action="store_true")
    ph.set_defaults(func=cmd_hold)

    pr = sub.add_parser("release", help="只失能指定关节，单独把「手拧」拆成一步")
    pr.add_argument("--joint", type=int, required=True)
    pr.add_argument("--iface", default="can0")
    pr.add_argument("--force", action="store_true")
    pr.set_defaults(func=cmd_release)

    args = p.parse_args(argv)
    if args.cmd == "set" and not args.out:
        ts = time.strftime("%Y%m%d_%H%M%S")
        args.out = f"env/zero_set_J{args.joint}_{ts}.json"
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
