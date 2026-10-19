#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stage1_cmd.py —— L0-b 的「下发即记录」层。

为什么必须单独有这个文件，而不是在记录器里补几行
------------------------------------------------
`stage1_l0.py record` 只收帧、不下发。没有它的时候，`parse --qcmd` 的 q_cmd 全是
null，样本全带 `no_q_cmd`。这时如果拿实测 q 顶替 q_cmd，策略学到的就是
`q_obs → q_obs` 的恒等映射：**loss 很低、验证集漂亮、机器人根本不响应指令**。
所以在 L1 之前，指令侧必须有独立、可核对、带时间戳的落盘。

三条硬规矩
----------
1. **记录的是"发出去的值"，不是"想发/算出来的值"**。夹紧、限幅、跳帧之后实际写入
   CAN 的那一帧才写进 CSV，并用**发送前后同一时刻的 wall clock** 打戳。
   *2.0 起这条是可执行的，不再靠自觉*：不用 piper_sdk 的 `JointCtrl()`（它返回 None，
   发送失败只在自己 logger 里打一行），改为自建 socket 逐帧判定；发失败的那一拍
   **不写 CSV**，`q_prev` 也不推进 —— 否则下一拍增量会变成两拍，轨迹被悄悄拉长。
2. **不允许插值补点**。掉一拍就少一行，让下游 `_qcmd_at` 用二分取"最近一次真实下发"。
   补点会伪造出机器从未收到的指令。
3. **模式不对就不发**。`ctrl_mode != CAN_CTRL(0x1)` 时直接中止（R19：示教模式下指令
   很可能被忽略，跑完只会得到一份"看着很成功"的废数据）。

产物
----
- `<out>.csv`   : `t_wall,j1..j6`（rad，无表头，供 `stage1_l0.py parse --qcmd` 直接吃）
- `<out>.meta.json` : 轨迹参数、起始时刻、clamp/skip 计数、模式、是否被中止

用法
----
    # 不接臂，先在 Windows 上把链路跑通（生成 CSV 不发送）
    python stage1_cmd.py hold  --dry-run --t0 1000000 --duration 5 --out cmd/hold

    # 接臂：先只保持当前姿态（最温和的一次运动）
    python stage1_cmd.py hold  --iface can0 --duration 10 --out cmd/hold

    # 低速正弦，验 q_cmd → q 的延迟与跟踪误差
    python stage1_cmd.py sine  --iface can0 --duration 60 --hz 100 \
        --amp-deg 5 --freq-hz 0.25 --out cmd/sine5

    # 阶跃，测纯延迟
    python stage1_cmd.py step  --iface can0 --duration 20 --amp-deg 3 --out cmd/step

判据
----
`hold` 跑完，`stage1_l0.py parse` 出来的 `q` 应该**稳定在 q0 附近、不抖**。
如果 `q` 完全不动而 `q_cmd` 在变 —— 指令没生效，回 R19 查模式。
如果 `q` 跟着动但滞后很多 —— 记下 `q_cmd` 与 `q` 的互相关峰值时刻，那就是这条路
的真实延迟，L1 的时序对齐要用它，不能用猜的。
"""

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

CMD_VERSION = "stage1_cmd/2.0"
DEFAULT_HZ = 100.0

# 与 stage1_l0.py 的 NOMINAL_DT 同一口径：实测 5 ms / 200 Hz。
# 下发 100 Hz、反馈 200 Hz 是刻意的——下发慢于反馈，才能从反馈里看出「指令已生效但还没走到」。
SOURCE_HZ = 200.0

RAD_PER_DEG = math.pi / 180.0
CTRL_CAN = 1          # ctrl_mode: CAN_CTRL(0x1)

# ------------------------------------------------------------------ 安全闸门


class Abort(Exception):
    """预检不通过。宁可什么都不做，也不要发一帧可疑的指令。"""


def _fmt_mode(ctrl: dict) -> str:
    if not ctrl:
        return "未知"
    return f"{ctrl.get('name')}(0x{ctrl.get('code'):x})" if ctrl.get("code") is not None else str(ctrl)


def read_status(piper):
    """返回 {ctrl_mode:{name,code}, arm_status:{...}}，解析不出就给 None。

    为什么不直接 getattr(st,'ctrl_mode')：0.6.1 打印出来是
    `Control Mode: TEACHING_MODE(0x2)` 这种文本摘要，属性上可能是 IntEnum 也可能是
    str，两种形态都见过。按文本正则先解，失败再退回属性，比赌一种形态稳。
    """
    import re
    try:
        st = piper.GetArmStatus()
    except Exception:
        return {}
    raw = " ".join(str(st).split())
    out = {}
    for key, label in (("ctrl_mode", "Control Mode"), ("arm_status", "Arm Status")):
        m = re.search(label + r":\s*([A-Za-z_]+)\s*\(0x([0-9A-Fa-f]+)\)", raw)
        if m:
            out[key] = {"name": m.group(1), "code": int(m.group(2), 16)}
            continue
        v = getattr(st, key, None)
        if v is not None:
            out[key] = {"name": getattr(v, "name", str(v)),
                        "code": getattr(v, "value", None)}
    return out


def read_q_rad(piper):
    """当前关节角（rad）。SDK 给的是 0.001 度整数。"""
    js = piper.GetArmJointMsgs().joint_state
    return [getattr(js, f"joint_{i}") / 1000.0 * RAD_PER_DEG for i in range(1, 7)]


def read_enable(piper):
    try:
        if hasattr(piper, "GetArmEnableStatus"):
            return [bool(x) for x in piper.GetArmEnableStatus()]
    except Exception:
        pass
    return None


# ------------------------------------------------------------------ 下发（自己发帧）
#
# JointCtrl 的三条帧：0x155=J1J2、0x156=J3J4、0x157=J5J6，各两个 int32 大端 / 0.001°。

JOINT_CTRL_IDS = (0x155, 0x156, 0x157)

# 官方 URDF 标称限位（experiments/_vendor/urdf/piper_description.urdf）。
# 固件 0x473 与它逐关节一致，**只有 J6 例外**：固件 ±180°，URDF ±120°（R28）。
# SDK 的 JointCtrl() 内部按 URDF 夹紧；我们绕开 SDK 自己发帧之后，这道网要自己保留。
# 取两者交集 —— 宁窄勿宽：窄了只是少走一点，宽了可能撞限位。
URDF_DEG = {1: (-150.0, 150.0), 2: (0.0, 179.91), 3: (-170.0, 0.0),
            4: (-99.98, 99.98), 5: (-69.90, 69.90), 6: (-120.0, 120.0)}


def safe_envelope(limits):
    """固件限位 ∩ URDF 标称 → {关节号: (lo_deg, hi_deg)}。"""
    env = {}
    for i in range(1, 7):
        d = (limits or {}).get(f"joint_{i}") or {}
        fw_lo, fw_hi = d.get("min_deg"), d.get("max_deg")
        u_lo, u_hi = URDF_DEG[i]
        env[i] = (u_lo if fw_lo is None else max(fw_lo, u_lo),
                  u_hi if fw_hi is None else min(fw_hi, u_hi))
    return env


class Tx:
    """自己建 socket 发 JointCtrl，逐帧判定成功与否。

    为什么不直接用 piper_sdk 的 `JointCtrl()`：它的返回值**恒为 None**，发送失败
    只在 SDK 自己的 logger 里打一行 `[ERROR] ... send failed`，调用方无法知道这一帧
    到底有没有进总线。15:26 那次就是约 80 帧没发出去，而 CSV 照写 4000 行 ——
    直接违反本文件第一条硬规矩（只记录真的发出去的值）。
    自己建 socket 后 `bus.send()` 会抛异常，可以逐帧判定。
    """

    def __init__(self, iface: str):
        import can
        self._can = can
        self.bus = can.Bus(interface="socketcan", channel=iface)
        self.n_ok = 0
        self.n_fail = 0
        self.last_err = None

    def send_joint(self, q_rad) -> bool:
        vals = [int(round(math.degrees(v) * 1000.0)) for v in q_rad]
        ok = True
        for i, cid in enumerate(JOINT_CTRL_IDS):
            data = (vals[2 * i].to_bytes(4, "big", signed=True)
                    + vals[2 * i + 1].to_bytes(4, "big", signed=True))
            try:
                # 5 ms 超时：宁可这一拍发失败（CSV 少一行，下游看得见），
                # 也不要阻塞 20 ms 把后面的节拍全带偏。
                self.bus.send(self._can.Message(arbitration_id=cid, data=data,
                                                is_extended_id=False), timeout=0.005)
                self.n_ok += 1
            except Exception as exc:
                ok = False
                self.n_fail += 1
                self.last_err = repr(exc)
        return ok

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


# ------------------------------------------------------------------ 轨迹
#
# 轨迹一律是「相对 q0 的小幅激励」，不是绝对位置规划。理由：这一阶段要的是
# 「指令发出去了、臂跟不跟」，不是「走到某个目标点」。绝对目标一旦和真实限位
# 不符（R26：限位还没回读成功），就会撞限位或触发保护。


def traj_hold(q0, t_rel, args):
    return list(q0)


def traj_sine(q0, t_rel, args):
    """以 q0 为中心的正弦。

    相位必须满足「t=0 时六轴都在 q0」。1.0 版给了固定 60° 错相，结果 t=0 处
    第 2~6 轴的 sin(ph) ≠ 0，第一拍就跳了 4.33°（被单周期限幅夹成 1°）。
    那是轨迹设计的错，不是执行的问题——**限幅计数把它暴露出来了**，否则这 4.33°
    的突跳会直接发给臂。现在默认 phase_spread=0（同相），需要错相时显式开。
    """
    out = []
    for i in range(6):
        amp = args.amp_deg[i] * RAD_PER_DEG
        ph = 2 * math.pi * args.phase_spread * (i / 6.0)
        out.append(q0[i] + amp * math.sin(2 * math.pi * args.freq_hz * t_rel + ph))
    return out


def traj_step(q0, t_rel, args):
    """在 t=duration/2 处跳 amp，其余保持。用来量纯延迟（不看跟踪，只看起始拐点）。"""
    if t_rel < args.duration / 2.0:
        return list(q0)
    return [q0[i] + args.amp_deg[i] * RAD_PER_DEG for i in range(6)]


TRAJ = {"hold": traj_hold, "sine": traj_sine, "step": traj_step}


# ------------------------------------------------------------------ 限幅


def clamp_step(q_prev, q_want, max_step_rad):
    """单周期最大变化量限制。

    这是这一层唯一无条件的保护：即使真实限位未知（当前就是未知），只要每周期
    走得足够小，任何时刻都还有机会在撞上之前停住。
    """
    out, clamped = [], False
    for a, b in zip(q_prev, q_want):
        d = b - a
        if abs(d) > max_step_rad:
            b = a + math.copysign(max_step_rad, d)
            clamped = True
        out.append(b)
    return out, clamped


def clamp_limits(q, env):
    """按「固件 ∩ URDF」包络夹紧；env 缺省时原样返回。"""
    if not env:
        return q, False
    out, hit = [], False
    for i, v in enumerate(q, start=1):
        lo, hi = env.get(i, (None, None))
        if lo is None or hi is None:
            out.append(v)
            continue
        if v < lo * RAD_PER_DEG:
            out.append(lo * RAD_PER_DEG)
            hit = True
        elif v > hi * RAD_PER_DEG:
            out.append(hi * RAD_PER_DEG)
            hit = True
        else:
            out.append(v)
    return out, hit


# ------------------------------------------------------------------ 主流程


def run(args):
    out_csv = Path(args.out).with_suffix(".csv")
    out_meta = Path(args.out).with_suffix(".meta.json")
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    limits = None
    if args.limits:
        limits = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    env = safe_envelope(limits)
    print("[cmd] 实际安全包络（固件 ∩ URDF，deg）："
          + "  ".join(f"J{i}[{env[i][0]:+.1f},{env[i][1]:+.1f}]" for i in range(1, 7)))
    if limits:
        for i in range(1, 7):
            d = limits.get(f"joint_{i}") or {}
            fw = (d.get("min_deg"), d.get("max_deg"))
            if fw[0] is not None and (abs(env[i][0] - fw[0]) > 1.0
                                      or abs(env[i][1] - fw[1]) > 1.0):
                print(f"[cmd][NOTE] J{i} 被 URDF 收紧：固件 [{fw[0]:+.2f},{fw[1]:+.2f}] → "
                      f"实际 [{env[i][0]:+.2f},{env[i][1]:+.2f}]（见 R28，宁可少走）")

    piper = None
    q0 = None
    mode = {}

    if args.dry_run:
        q0 = [0.10, 1.20, -1.30, 0.20, 0.30, -0.40]
        print(f"[cmd] dry-run：不下发、不连 CAN，用固定初始姿态生成轨迹 {q0}")
        if not limits:
            print("[cmd][WARN] 未提供 --limits：只做单周期增量限制，不做绝对限位夹紧")
    else:
        import piper_sdk                                 # 延迟导入：dry-run 时不需要
        # 固件 S-V1.8-8 官方建议用 V3 类（R20）。有就用，没有就退回默认类，
        # 并在日志里说清楚用的是哪个——不要让人以为一定跑在 V3 上。
        cls = getattr(piper_sdk, "C_PiperInterface_V3", None) or piper_sdk.C_PiperInterface
        print(f"[cmd] 连接 {args.iface}（{cls.__name__}）")
        piper = cls(args.iface) if args.iface else cls()
        piper.ConnectPort()
        time.sleep(0.25)                                 # 等状态帧到位，否则读到默认值

        mode = read_status(piper)
        ctrl = (mode.get("ctrl_mode") or {}).get("code")
        print(f"[cmd] ctrl_mode = {_fmt_mode(mode.get('ctrl_mode'))}")

        # ---- 预检一：模式 ----
        if ctrl != CTRL_CAN:
            msg = (f"ctrl_mode 是 {_fmt_mode(mode.get('ctrl_mode'))}，不是 CAN_CTRL(0x1)。\n"
                   f"  示教/待机模式下 CAN 运动指令很可能被忽略，跑完只会得到一份"
                   f"「看着成功」的废数据。\n"
                   f"  先切到程序控（注意：0xFC 是随动主从，不是程序控，别再用 --set-slave）：\n"
                   f"    python scripts/stage0_probe.py --phase sdk --allow-write --to-can-ctrl\n"
                   f"  确认无误后仍要跑：加 --force-mode")
            if not args.force_mode:
                raise Abort(msg)
            print(f"[cmd][WARN] 已用 --force-mode 越过模式检查。原因：{msg.splitlines()[0]}")

        # ---- 预检二：使能 ----
        en = read_enable(piper)
        if en is not None and not all(en):
            raise Abort(f"六轴未全使能：{en}。使能不是本脚本的职责（FIRST_ARM 红线），"
                        f"先在厂家上位机或按现场规程使能。")

        q0 = read_q_rad(piper)
        print(f"[cmd] 当前姿态 (deg) = {[round(math.degrees(v), 2) for v in q0]}")

        if max(abs(v) for v in q0) < 1e-6:
            raise Abort("六个关节读数全为 0 —— 这是 SDK 没收到帧时的默认值，不是真实姿态"
                        "（R18）。先确认总线有反馈帧，再跑这个脚本。")

        if not limits:
            print("[cmd][WARN] 未提供 --limits：只做单周期增量限制，不做绝对限位夹紧。"
                  "限位还没回读成功之前，这是已知的残余风险。")

        # ---- 预检三：人确认 ----
        if not args.yes:
            # hold 不吃 --amp-deg，但 argparse 给了默认值 [3.0]*6。
            # 15:21 那次 hold 就印出「幅值 [3.0]*6 deg」，而它一个字节都不会动 ——
            # 这种「提示说要做 A、实际做 B」比没有提示更危险：人会照着去理解行为。
            if args.kind == "hold":
                print(f"\n即将下发 hold 轨迹：{args.duration:g} s @ {args.hz:g} Hz，"
                      f"保持启动时读到的姿态不变（无幅值、无频率），"
                      f"单周期上限 {args.max_step_deg:g} deg")
                print(f"启动姿态 = {[round(math.degrees(v), 2) for v in q0]} deg "
                      f"（若与你目视不符，先别继续）")
            else:
                print(f"\n即将下发 {args.kind} 轨迹：{args.duration:g} s @ {args.hz:g} Hz，"
                      f"幅值 {args.amp_deg} deg，单周期上限 {args.max_step_deg:g} deg")
            print("确认臂周围无人、无遮挡、手能碰到急停。输入 yes 继续：", end=" ", flush=True)
            if sys.stdin.readline().strip().lower() not in ("yes", "y"):
                raise Abort("操作者未确认，已中止。")

    make = TRAJ[args.kind]
    max_step = args.max_step_deg * RAD_PER_DEG

    # 下发通道：自己建 socket，逐帧核对。dry-run 不建。
    tx = None
    if not args.dry_run:
        tx = Tx(args.iface or "can0")
        print(f"[cmd] 下发通道已开：{args.iface or 'can0'}（自建 socket，逐帧判定成败）")

    # step 的意图是「一跳就到」，但那和单周期限幅直接冲突。
    # 限幅优先（安全先于测量），代价是阶跃被斜坡化成若干拍，测出来的就不再是纯延迟。
    # 与其让人拿到一份看着像阶跃、其实是斜坡的数据去估延迟，不如现在就说破。
    if args.kind == "step" and max(args.amp_deg) > args.max_step_deg:
        n_ramp = math.ceil(max(args.amp_deg) / args.max_step_deg)
        print(f"[cmd][NOTE] step 幅值 {max(args.amp_deg):g} deg > 单周期上限 "
              f"{args.max_step_deg:g} deg，会被斜坡化成约 {n_ramp} 拍。"
              f"要测纯延迟，把 --max-step-deg 调到 > {max(args.amp_deg):g}。")

    hz = args.hz
    dt = 1.0 / hz
    n = max(int(args.duration * hz), 1)

    t0 = args.t0 if args.t0 is not None else time.time()
    q_prev = list(q0)
    n_clamped = n_lim = n_skipped = 0
    n_send_failed = 0
    consec_fail = 0
    rows = []
    aborted = False
    started = time.time()
    # 节拍用「真实时钟」推进，与 t0 解耦。
    # 否则 --t0 一旦被显式指定（测试时故意给一个过去的时间点，好让 CSV 落进
    # 合成 raw 的时间域），deadline 就全在过去，每一拍都被记成「落后」——
    # 这个假计数会让人以为主机调度撑不住 100 Hz。
    base_real = started

    def _on_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        for k in range(n):
            deadline = base_real + k * dt
            sleep = deadline - time.time()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -2 * dt:            # 落后两拍以上：不追帧，记下来
                n_skipped += 1

            # 真实运行时 t_wall 就是发帧前那一刻；--t0 只用于测试伪造时间基准
            t_wall = (t0 + k * dt) if args.t0 is not None else time.time()
            q_want = make(q0, k * dt, args)
            q_want, c1 = clamp_limits(q_want, env)
            q_want, c2 = clamp_step(q_prev, q_want, max_step)
            n_lim += int(c1)
            n_clamped += int(c2)

            ok = True
            if tx is not None:
                ok = tx.send_joint(q_want)

            if ok:
                # 只有**真的进了总线**的值才写 CSV。q_prev 也只在成功时推进：
                # 失败那一拍臂根本没收到，若把 q_prev 挪过去，下一拍的增量就会
                # 悄悄变成两拍，凭空把轨迹拉长、把斜率减半。
                q_prev = q_want
                rows.append(f"{t_wall:.6f}," + ",".join(f"{v:.6f}" for v in q_want))
                consec_fail = 0
            else:
                n_send_failed += 1
                consec_fail += 1
                if consec_fail >= 30:            # 100 Hz 下约 0.3 s
                    print(f"[cmd][ABORT] 连续 {consec_fail} 拍发送失败，总线已不可用，停止下发。"
                          f"最后一处错误：{tx.last_err}", file=sys.stderr)
                    break

        t_end = time.time()
    except KeyboardInterrupt:
        aborted = True
        t_end = time.time()
        print("\n[cmd] 收到中断，停止下发")
        if piper is not None and tx is not None:
            try:                              # 停在「当前实际位置」，不是停在轨迹上
                q_hold = read_q_rad(piper)
                if tx.send_joint(q_hold):
                    print(f"[cmd] 已下发保持帧：{[round(math.degrees(v), 2) for v in q_hold]} deg")
                else:
                    print(f"[cmd][WARN] 保持帧发送失败：{tx.last_err}")
            except Exception as exc:
                print(f"[cmd][WARN] 保持帧发送失败：{exc!r}")

    if tx is not None:
        tx.close()

    out_csv.write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")
    meta = {
        "v": CMD_VERSION,
        "kind": args.kind,
        "iface": args.iface,
        "dry_run": bool(args.dry_run),
        "hz": hz,
        "source_hz": SOURCE_HZ,
        "duration": args.duration,
        "t0_wall": t0,
        "t_start_wall": started,
        "t_end_wall": t_end,
        "n": len(rows),
        "n_skipped": n_skipped,
        "n_step_clamped": n_clamped,
        "n_limit_clamped": n_lim,
        "n_send_failed": n_send_failed,
        "n_tx_frames_ok": (None if tx is None else tx.n_ok),
        "n_tx_frames_fail": (None if tx is None else tx.n_fail),
        "last_tx_error": (None if tx is None else tx.last_err),
        "tx_ids": [f"0x{c:03X}" for c in JOINT_CTRL_IDS],
        "envelope_deg": {f"joint_{i}": list(env[i]) for i in range(1, 7)},
        "aborted": aborted,
        "q0_rad": q0,
        # hold 不用幅值/频率；把它们写成 null，别让 meta 里留一对没意义的默认值
        "amp_deg": (None if args.kind == "hold" else args.amp_deg),
        "freq_hz": (None if args.kind == "hold" else args.freq_hz),
        "max_step_deg": args.max_step_deg,
        "ctrl_mode": mode.get("ctrl_mode"),
        "limits_loaded": bool(limits),
        "csv_columns": ["t_wall", "j1", "j2", "j3", "j4", "j5", "j6"],
        "unit": "rad",
        "note": "t_wall 是「发送前一刻」的 epoch 秒；掉拍不补点，缺行即为真实丢发",
    }
    out_meta.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[cmd] 写出 {out_csv}（{len(rows)} 行）与 {out_meta.name}")
    if n_send_failed:
        print(f"[cmd][WARN] {n_send_failed} 拍发送失败，**没有写进 CSV** —— "
              f"这些指令臂根本没收到，轨迹在时间上有缺口（下游会看到缺行，这是对的）。"
              f"最后一处错误：{None if tx is None else tx.last_err}")
    if n_clamped:
        print(f"[cmd] 单周期限幅触发 {n_clamped} 次 —— 轨迹比 --max-step-deg 陡，"
              f"实际发的比算的小；CSV 里记的是实际值")
    if n_lim:
        print(f"[cmd] 限位夹紧触发 {n_lim} 次")
    if n_skipped:
        print(f"[cmd] 落后 >2 拍 {n_skipped} 次 —— 主机调度不足，考虑降 --hz")
    if aborted:
        print("[cmd] 本次被中断，CSV 只含中断前已实际下发的行（这是对的，别补）")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="L0-b 指令层：下发并记录 q_cmd（与 stage1_l0.py parse --qcmd 配套）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="CSV 无表头：t_wall,j1..j6，单位 rad，epoch 秒。")
    ap.add_argument("kind", choices=list(TRAJ), help="hold 保持 / sine 正弦 / step 阶跃")
    ap.add_argument("--iface", default="can0", help="CAN 接口名，默认 can0")
    ap.add_argument("--out", required=True, help="输出前缀（会生成 .csv 与 .meta.json）")
    ap.add_argument("--duration", type=float, default=10.0, help="时长 s，默认 10")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help=f"下发频率，默认 {DEFAULT_HZ:g}")
    ap.add_argument("--amp-deg", type=float, nargs="+", default=[3.0],
                    help="幅值（度）。给 1 个数则六轴同幅，给 6 个数则逐轴指定")
    ap.add_argument("--freq-hz", type=float, default=0.25, help="正弦频率 Hz，默认 0.25")
    ap.add_argument("--phase-spread", type=float, default=0.0,
                    help="正弦逐轴错相（单位=周期数），默认 0（同相，t=0 时不离零位）。"
                         "设成 1.0 则六轴均分一整个周期；注意错相会让 t=0 处有一部分轴不在 q0，"
                         "首拍可能被 --max-step-deg 夹住")
    ap.add_argument("--max-step-deg", type=float, default=1.0,
                    help="单周期最大角度变化（度），默认 1.0；这是唯一无条件的保护")
    ap.add_argument("--limits", default=None,
                    help="joint_limits.json（由 stage0_probe --query-limits 产出）")
    ap.add_argument("--dry-run", action="store_true", help="不连 CAN、不下发，只生成 CSV")
    ap.add_argument("--force-mode", action="store_true",
                    help="[危险] 越过 ctrl_mode != CAN_CTRL 的检查")
    ap.add_argument("--yes", action="store_true", help="跳过人工确认（脚本化时用）")
    ap.add_argument("--t0", type=float, default=None,
                    help="起始 epoch 秒；测试时可固定，默认取当前时间")
    args = ap.parse_args(argv)

    amp = list(args.amp_deg)
    if len(amp) == 1:
        amp = amp * 6
    elif len(amp) != 6:
        ap.error(f"--amp-deg 要么 1 个数，要么 6 个数，收到 {len(amp)} 个")
    args.amp_deg = amp

    try:
        return run(args)
    except Abort as exc:
        print(f"\n[cmd][ABORT] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
