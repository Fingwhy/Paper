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


def safe_envelope(limits, tol_deg=0.0):
    """固件限位 ∩ URDF 标称 → {关节号: (lo_deg, hi_deg)}，再向外放宽 tol_deg。

    tol 不是「宽容」，是必须先修的偏差：实测自然零位上 **J2 = −2.003°、
    J3 = +1.798°**，而固件自报 J2∈[0,180]、J3∈[−170,0]。若不放宽，`hold` 的第一拍
    就会被限位夹紧把臂从 −2° 拽到 0° —— **限幅本身造成了一次没人下达的移动**，
    比撞限位更隐蔽（撞限位至少有声音/报警）。tol 只影响边界外侧几度的窄带，
    正常目标（例如 P1 的 J2=0、P4 的 J2=100）完全不受影响。
    """
    env = {}
    for i in range(1, 7):
        d = (limits or {}).get(f"joint_{i}") or {}
        fw_lo, fw_hi = d.get("min_deg"), d.get("max_deg")
        u_lo, u_hi = URDF_DEG[i]
        lo = u_lo if fw_lo is None else max(fw_lo, u_lo)
        hi = u_hi if fw_hi is None else min(fw_hi, u_hi)
        env[i] = (lo - tol_deg, hi + tol_deg)
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


def traj_pose(q0, t_rel, args):
    """pose 不用「时间函数」驱动，见 pose_core：它按「已走比例」插值并做到位判定。"""
    return [v * RAD_PER_DEG for v in args.target_deg]


TRAJ = {"hold": traj_hold, "sine": traj_sine, "step": traj_step, "pose": traj_pose}


# ------------------------------------------------------------ 轨迹路径的运行时守卫
#
# 为什么单独抽成对象，而不是塞在 run() 的 for 循环里
# ------------------------------------------------
# 2026-10-20 13:5x 的事故：J6 负向顶死，电流 12.8 A 持续 58.7 s，sine 一路跑到
# duration 结束、退出码 0，把废数据当成功收下。原因是 **sine/step/hold 这条路径
# 之前完全没有反馈侧守卫**（pose 走 pose_core，三道守卫齐全）。
#
# 补守卫时不再把它写成一段内联 if：内联代码只能靠真机撞障碍来验证，而这种验证
# 代价就是再烧一次驱动。抽成对象之后可以用假读数离线跑（见 selftest 的 G 组），
# 「守卫真的会在第 N 拍停」这件事才有办法证明。

class TrajGuard:
    """电流超限 + 卡死 两道守卫。feed() 返回 None=继续，返回字符串=ABORT 原因。"""

    def __init__(self, cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                 stall_eps_deg=0.08, stall_lead_deg=2.0, q0=None, every=2):
        self.cur_limit_a = cur_limit_a
        self.cur_grace_s = cur_grace_s
        self.win_ticks = max(1, int(stall_win_ticks))
        self.eps_rad = stall_eps_deg * RAD_PER_DEG
        self.eps_deg = stall_eps_deg
        self.lead_rad = stall_lead_deg * RAD_PER_DEG
        self.lead_deg = stall_lead_deg
        self.every = max(1, int(every))
        self.over_since = None
        self.over_axis = None
        self.over_val = None
        self.win_cmd = list(q0) if q0 is not None else None
        self.win_meas = list(q0) if q0 is not None else None
        self.win_tick = 0
        self.hits = {"current": 0, "stall": 0}

    def feed(self, tick, t_now, cur, q_meas, q_cmd):
        if tick % self.every:
            return None

        # ---- 电流 ----
        if self.cur_limit_a and cur:
            imax = max(abs(c) for c in cur)
            if imax > self.cur_limit_a:
                if self.over_since is None:          # 首次超限：开始计时
                    self.over_since = t_now
                    self.over_axis = 1 + max(range(len(cur)), key=lambda i: abs(cur[i]))
                    self.over_val = imax
                elif t_now - self.over_since >= self.cur_grace_s:
                    self.hits["current"] = 1
                    return (f"电流守卫：J{self.over_axis} {self.over_val:.2f} A 超 "
                            f"{self.cur_limit_a:g} A 且持续 {t_now - self.over_since:.2f} s"
                            f"（阈值 {self.cur_grace_s:g} s）: 判为顶住障碍/堵转，原地抱住")
            else:
                self.over_since = None               # 落回阈值内就清计时

        # ---- 卡死：窗口内「指令走了、实测没走」----
        #
        # 2026-10-20 14:2x 补的第三个条件（lead）。只有前两条会**误杀正弦峰顶**：
        # 峰顶前后净位移是「上去又下来」，窗口内 |q_cmd - win_cmd| 本来就接近 0
        # （实测 S1_6_mid：指令净走 0.168°、实测 0.031°），于是被判成卡死。
        # 这不是磨损也不是障碍，是**判据本身不适用于振荡轨迹**。
        #
        # 真正的卡死一定伴随「滞后」：指令已经跑到实测前面很多。正常跟踪时滞后
        # 只有 0.1~0.5° 量级（峰顶更小），真卡死时会一路长到几度几十度
        # （13:5x 事故里 J6 指令领先实测 30°）。所以加一条必要条件：
        # **当前指令-实测偏差必须超过 --stall-lead-deg（默认 2°）**。
        # 这样：正弦峰顶滞后 0.14° → 不报；合法 step 跳变 → 实测在走，prog 大 → 不报；
        # 真卡死 → 滞后大 + 实测不走 → 立刻报。
        if q_meas is None or q_cmd is None or self.win_cmd is None:
            return None
        self.win_tick += self.every
        if self.win_tick < self.win_ticks:
            return None
        cmd_prog = max(abs(a - b) for a, b in zip(q_cmd, self.win_cmd))
        meas_prog = max(abs(a - b) for a, b in zip(q_meas, self.win_meas))
        lead = max(abs(a - b) for a, b in zip(q_cmd, q_meas))
        if (lead > self.lead_rad and cmd_prog >= self.eps_rad
                and meas_prog < self.eps_rad):
            self.hits["stall"] = 1
            return (f"卡死：窗口内指令走了 {math.degrees(cmd_prog):.3f}°，"
                    f"实测只走 {math.degrees(meas_prog):.3f}°（阈值 {self.eps_deg:g}°），"
                    f"滞后 {math.degrees(lead):.2f}° 超 {self.lead_deg:g}°")
        self.win_cmd = list(q_cmd)
        self.win_meas = list(q_meas)
        self.win_tick = 0
        return None


# ------------------------------------------------------------------ pose 核心
#
# 为什么单独抽成函数而不是塞进 run() 的循环里
# ------------------------------------------
# 因为「到了没有」这一条**不能靠时间推断**。S2-0/S2-1 要把臂停在固定角上采静态电流，
# 若靠「跑了 N 拍就算到位」，一旦被外部挡住、限幅夹住、或某轴减速器卡涩，脚本会
# 打印成功、CSV 也完整，而实际姿态差几十度 —— 后面所有力矩反推全错，且错得看不出来。
# 抽成函数后可以注入「卡住」的假读数做自测（F/G 两条），这是唯一能证明守卫真的生效的办法。


def pose_core(q0_deg, target_deg, rate_deg_s, hz, hold_s, tx, read_q_deg,
              arrive_tol_deg=0.5, arrive_cycles=20, settle_s=3.0, log=print,
              keepalive=None, keepalive_every=100,
              read_currents=None, cur_max_a=3.0, cur_grace_s=0.3,
              read_z_mm=None, z_drop_max_mm=None, z_min_mm=None,
              stall_win_s=1.0, stall_eps_deg=0.08,
              max_cycles=None, freeze_frames=5,
              hard_env=None, launch_s=3.0, launch_eps_deg=0.05, lead_max_deg=2.0):
    """伺服步进到绝对目标角 → 到位判定 → 保持。

    2026-10-20 10:1x 事故后重写（R42）
    -----------------------------------
    旧版斜坡是**按时间**推进的：`q_want = q0 + f*Δ`，f 只跟 tick 有关。
    臂一旦卡住（夹爪顶到底盘），f 照样往前走，于是「指令越走越远、实测不动」，
    控制器持续把目标压在物理障碍上 —— 这是**主动往障碍里推**，比不动更危险。
    而且唯一的「没动」检查只在第 motion_guard_cycles 拍查**一次**，卡在中间查不出来。

    新版三条改变：
      1. **指令跟实测走**：每拍 `q_cmd = q_meas + clamp(目标-实测, ±rate*dt)`。
         臂卡住时指令就停在实测位置，误差不会自己长大，不会再往障碍里顶。
      2. **进度守卫是连续的**：每个 stall_win_s 窗口比一次「这个窗口里指令走了多少、
         实测走了多少」。指令走了而实测没走 ⇒ 卡死，立刻 ABORT。
      3. **三条物理量守卫**：电流（顶住障碍时飙）、末端高度（往下掉）、总时长。
         任一触发都跳到 `_freeze()`：把目标改写回**当前实测姿态**再发几帧，
         让控制器原地抱住而不是继续推，然后退出。

    2026-10-20 10:53 第二次修正（S2-0 第一次重跑，100.9 s 一动不动）
    ---------------------------------------------------------------
    现象：J1 目标 −2.126°（完全合法）100 s 内一格没动，J2/J3 只挪了 1.9°，
          电流 0.6~1.2 A（保持，不是顶住）。指令 CSV 显示命令**只领先实测 0.06°**。
    根因：**「跟实测走」在起点越限时会自锁**。停放姿态 J2=−1.998°、J3=+1.818°
          在固件限位（0~180 / −170~0）之外；每拍指令 = 实测 + 一步 = −1.94°，**仍在界外**，
          固件不执行 ⇒ 实测不动 ⇒ 下一拍还是 −1.94°。指令永远出不了非法区。
          固件是六轴**同步插值**的，一个轴在界外就整体不动，所以合法的 J1 也陪着一动不动。
    修法两条：
      1. `hard_env`（固件∩URDF，**不**外扩）：每拍把 q_cmd 夹进这个硬包络。
         首拍就会变成 J2=0.0 / J3=0.0，即「先合法、再走路」——位移只有 0.13°，不额外冒险。
      2. `launch_s` 起步守卫：开跑 3 s 后实测位移仍 < `launch_eps_deg` 就立刻报「起步失败」，
         不再像这次白等 100 s（那时才超时，日志里看不出是哪一步的问题）。

    2026-10-20 11:05 第三次修正（S2-0 第二次重跑，指令全合法、仍然一动不动）
    ---------------------------------------------------------------------
    实测：把越限的两轴都夹到边界后，六轴目标**全部合法**，但 100 s 里 J1 只挪 0.10°。
    对比事故前那次成功的 P1：那是**按时间**的斜坡，目标一次领先几十度，臂就动了。
    结论：固件不会因为「目标比自己远 0.06°」而产生速度（0.06° 是 6°/s × 1/100 Hz 的一步）。
    改成「虚拟目标匀速前进 + 有界领先」：
        q_virt 每拍按 rate*dt 向目标走（它决定速率，不受实测影响）；
        实发指令 = q_virt，但**永不比实测领先超过 lead_max_deg（默认 2°）**。
      · 走得动时：指令≈实测+2°，固件按自己的速度追，整体仍受 rate 限制（速率由 q_virt 定）；
      · 被挡住时：领先量被 2° 封顶，绝不会像旧斜坡那样把目标一路顶到 60° 外；
      · 卡死判定改比 q_virt（「本应走多少」），因为卡住时实发指令会被 lead 夹住而不动。

    参数
    ----
    read_q_deg     : 无参可调用，返回实测六轴角（deg）。到位判定只看它。
    read_currents  : 无参可调用，返回六轴电流（A）。None=不做电流守卫。
    read_z_mm      : 无参可调用，返回末端法兰 Z（mm，基座坐标系）。
    z_drop_max_mm  : 相对**起始**高度最多允许下降多少 mm（None=不查）。
    stall_win_s    : 进度窗口长度，默认 1.0 s。
    stall_eps_deg  : 窗口内实测位移低于此值即判「没动」，默认 0.08°。
    max_cycles     : 总时长上限（拍）。None = 自动按斜坡理论时长 ×3 + 保持段。
    hard_env       : {1..6: (lo, hi)} 硬包络（deg）。每拍把 q_cmd 夹进去，避免「指令落在
                     限位外 ⇒ 固件整帧不执行 ⇒ 自锁」。None = 不夹（旧行为，容易自锁）。
    launch_s       : 起步观察窗（s）。窗口内实测几乎没动 ⇒ 立刻判定起步失败。
    launch_eps_deg : 起步窗口要求的实测位移（deg），默认 0.05。

    返回 dict：reached / aborted / abort_reason / rows / n_fail / n_ramp / n_hold
              / hold_start / q_end_deg / residual_deg / ramp_s / max_delta_deg / guards
    """
    dt = 1.0 / hz
    delta = [t - s for s, t in zip(q0_deg, target_deg)]
    max_d = max(abs(x) for x in delta) if delta else 0.0
    rate = max(abs(rate_deg_s), 1e-6)
    ramp_s = max_d / rate
    step = rate * dt                         # 每拍最大前进量，由速率折算
    n_ramp = max(1, int(math.ceil(ramp_s * hz)))
    if max_cycles is None:
        max_cycles = int((ramp_s * 3.0 + hold_s + 10.0) * hz)

    rows = []                                # 每拍真的发出去的六轴角（deg）
    n_fail = 0
    consec = 0
    aborted = False
    abort_reason = None
    reached = False
    guards = {"stall": 0, "current": 0, "height": 0, "deadline": 0, "launch": 0}

    q_meas = list(read_q_deg())
    q_cmd = list(q_meas)
    q_init = list(q_meas)                    # 起步守卫的比较基准
    q_virt = list(q0_deg)                    # 虚拟目标：按速率匀速前进，不受实测影响
    launch_done = False

    def _clamp_env(q):
        """把指令夹进硬包络（固件∩URDF，不外扩）。

        为什么必须夹：停放姿态可能本来就在固件限位外（本机 J2 −2.0°／J3 +1.8°），
        「实测 + 一步」还是在界外，固件不执行 ⇒ 实测不动 ⇒ 指令永远出不去（自锁）。
        夹到边界上只挪 0.1~0.2°，是「先合法、再走路」，不是额外运动。
        """
        if not hard_env:
            return q
        out = []
        for i, v in enumerate(q, 1):
            lo, hi = hard_env.get(i, (None, None))
            if lo is not None and v < lo:
                v = lo
            if hi is not None and v > hi:
                v = hi
            out.append(v)
        return out

    # 起始高度（只有拿到末端 Z 才有意义）
    z_start = None
    if read_z_mm is not None:
        try:
            z_start = float(read_z_mm())
        except Exception:
            z_start = None

    stall_win = max(4, int(stall_win_s * hz))
    win_meas = list(q_meas)
    win_cmd = list(q_virt)
    win_tick = 0

    cur_over = 0
    cur_grace = max(1, int(cur_grace_s * hz))

    t_base = time.monotonic()
    tick = [0]

    def _pace():
        tick[0] += 1
        s = t_base + tick[0] * dt - time.monotonic()
        if s > 0:
            time.sleep(s)

    def _ka():
        if keepalive is not None and keepalive_every > 0 and tick[0] % keepalive_every == 0:
            keepalive()

    def _send(q_deg):
        nonlocal n_fail, consec
        ok = tx.send_joint([v * RAD_PER_DEG for v in q_deg])
        if ok:
            consec = 0
        else:
            n_fail += 1
            consec += 1
        return ok

    def _residual(meas):
        return [m - t for m, t in zip(meas, target_deg)]

    def _freeze():
        """把目标改写回当前实测姿态：原地抱住，不再往障碍方向压。

        为什么不是直接失能：失能会让臂自由掉落（手册 §10.2）。
        抱住 + 报错 + 等人介入，是唯一不引入新风险的动作。
        """
        m = list(read_q_deg())
        for _ in range(max(1, freeze_frames)):
            _send(m)
            _pace()

    def _guard_current(meas_now):
        """顶住障碍时电流会持续升高 —— 比「位移为 0」更早、更直接。"""
        nonlocal cur_over
        if read_currents is None or cur_max_a is None or cur_max_a <= 0:
            return None
        try:
            cur = [abs(float(x)) for x in read_currents()]
        except Exception:
            return None
        if not cur:
            return None
        if max(cur) > cur_max_a:
            cur_over += 1
            if cur_over >= cur_grace:
                guards["current"] += 1
                return (f"电流超限：最大 {max(cur):.2f} A > {cur_max_a:.2f} A，"
                        f"持续 {cur_over} 拍（各轴 {[round(x, 2) for x in cur]}）")
        else:
            cur_over = 0
        return None

    def _guard_height(_meas_now):
        """末端往下掉超过允许值即停。卡进底盘前，Z 一定先掉。"""
        if read_z_mm is None:
            return None
        try:
            z = float(read_z_mm())
        except Exception:
            return None
        if z_min_mm is not None and z < z_min_mm:
            guards["height"] += 1
            return f"末端高度 {z:.1f} mm 低于硬下限 {z_min_mm:.1f} mm"
        if z_drop_max_mm is not None and z_start is not None:
            if (z_start - z) > z_drop_max_mm:
                guards["height"] += 1
                return (f"末端高度 {z:.1f} mm 比起始 {z_start:.1f} mm 下降了 "
                        f"{z_start - z:.1f} mm，超过允许 {z_drop_max_mm:.1f} mm")
        return None

    # ---------------- 主循环（伺服步进） ----------------
    quiet = 0
    cycles = 0
    while True:
        if cycles >= max_cycles:
            guards["deadline"] += 1
            aborted = True
            abort_reason = f"超过总时长上限 {max_cycles / hz:.1f} s 仍未到位"
            break
        cycles += 1

        q_meas = list(read_q_deg())
        err = [t - m for t, m in zip(target_deg, q_meas)]
        emax = max(abs(e) for e in err)

        if emax <= arrive_tol_deg:
            quiet += 1
            if quiet >= arrive_cycles:
                reached = True
                break
        else:
            quiet = 0

        # ---- 虚拟目标匀速前进 + 指令只许领先实测 lead_max_deg ----
        # 为什么不是「实测 + 一步」：2026-10-20 11:0x 实测证明，固件收到**比自己当前位置
        # 只远 0.06°** 的目标时不产生速度（100 s 只挪了 0.1°），而事故前那次成功的 P1
        # 用的是「一次推到几十度外」的绝对目标。所以：
        #   * 虚拟目标按 --rate-deg-s 匀速走（这是速率上限的来源）；
        #   * 实际发的指令 = 虚拟目标，但**不许比实测领先超过 lead_max_deg**（R42 的
        #     「永不往障碍里顶」由这条保证：卡住时最多领先 2°，不是 60°）。
        q_virt = [v + max(-step, min(step, t - v)) for v, t in zip(q_virt, target_deg)]
        q_cmd = []
        for v, m in zip(q_virt, q_meas):
            lo, hi = m - lead_max_deg, m + lead_max_deg
            q_cmd.append(hi if v > hi else (lo if v < lo else v))
        q_cmd = _clamp_env(q_cmd)

        # ---- 起步守卫：开跑 launch_s 后实测还几乎没动 ⇒ 别等满 deadline ----
        if not launch_done and max_d > 1.0:
            moved = max(abs(a - b) for a, b in zip(q_meas, q_init))
            if (time.monotonic() - t_base) >= launch_s:
                launch_done = True
                if moved < launch_eps_deg:
                    guards["launch"] += 1
                    aborted = True
                    abort_reason = (f"起步失败：{launch_s:g} s 内实测只走了 {moved:.3f}°"
                                    f"（< {launch_eps_deg:g}°），指令却已发 {len(rows)} 拍。"
                                    f"通常是「指令被固件判为越限/未使能/模式不对」，"
                                    f"不是卡在障碍上（电流若正常就更说明没在顶）")
                    break
            elif moved >= launch_eps_deg * 4:
                launch_done = True           # 已经走起来了，不必再等满窗口

        t_wall = time.time()
        if _send(q_cmd):
            rows.append((t_wall, list(q_cmd)))
        else:
            if consec >= 30:
                aborted = True
                abort_reason = f"连续 {consec} 拍发送失败：{getattr(tx, 'last_err', None)}"
                break

        for reason in (_guard_current(q_meas), _guard_height(q_meas)):
            if reason:
                aborted = True
                abort_reason = reason
                break
        if aborted:
            break

        # ---- 进度守卫（连续）：指令走了、实测没走 = 卡死 ----
        win_tick += 1
        if win_tick >= stall_win:
            # 比的是**虚拟目标**而不是实发指令：卡住时实发指令被 lead 夹住不动，
            # 拿它比会永远「没进展」而漏判。虚拟目标的推进＝「我们本想要它走多少」。
            cmd_prog = max(abs(a - b) for a, b in zip(q_virt, win_cmd))
            meas_prog = max(abs(a - b) for a, b in zip(q_meas, win_meas))
            if cmd_prog >= stall_eps_deg and meas_prog < stall_eps_deg:
                guards["stall"] += 1
                aborted = True
                abort_reason = (f"卡死：窗口 {stall_win_s:g} s 内本应走 {cmd_prog:.3f}°，"
                                f"实测只走 {meas_prog:.3f}°（阈值 {stall_eps_deg:g}°）")
                break
            win_meas = list(q_meas)
            win_cmd = list(q_virt)
            win_tick = 0

        _ka()
        _pace()

    # 卡死/超限/失败 → 原地抱住，绝不继续往原目标推
    if aborted:
        log(f"[pose][ABORT] {abort_reason}")
        log("[pose][FREEZE] 把目标改写成当前实测姿态，原地抱住（不失能、不掉臂）")
        _freeze()
        q_end = list(read_q_deg())
        return {
            "reached": False, "aborted": True, "abort_reason": abort_reason,
            "not_executed": False,
            "rows": rows, "n_fail": n_fail, "n_ramp": n_ramp, "n_hold": 0,
            "hold_start": len(rows), "q_end_deg": q_end,
            "residual_deg": _residual(q_end), "ramp_s": ramp_s,
            "max_delta_deg": max_d, "guards": guards,
        }

    # 未到位但没触发守卫（理论上到不了这里，除非 arrive_tol 太严）
    if not reached:
        log(f"[pose][WARN] 未触发守卫但也没判到位，残差 "
            f"{max(abs(v) for v in _residual(q_meas)):.3f}°")

    # ---------------- 保持段（只做在到位之后） ----------------
    hold_start = len(rows)
    n_hold = 0
    if reached:
        n_hold = max(0, int(hold_s * hz))
        for _ in range(n_hold):
            q_meas = list(read_q_deg())
            t_wall = time.time()
            if _send(target_deg):
                rows.append((t_wall, list(target_deg)))
            elif consec >= 30:
                aborted = True
                abort_reason = f"保持段连续 {consec} 拍发送失败：{getattr(tx, 'last_err', None)}"
                break
            for reason in (_guard_current(q_meas), _guard_height(q_meas)):
                if reason:
                    aborted = True
                    abort_reason = reason
                    break
            if aborted:
                break
            _ka()
            _pace()
        if aborted:
            log(f"[pose][ABORT] {abort_reason}")
            log("[pose][FREEZE] 保持段中断，原地抱住")
            _freeze()

    q_end = list(read_q_deg())
    return {
        "reached": reached,
        "aborted": aborted,
        "abort_reason": abort_reason,
        "not_executed": False,
        "rows": rows,
        "n_fail": n_fail,
        "n_ramp": 0,
        "n_hold": n_hold,
        "hold_start": hold_start,
        "q_end_deg": q_end,
        "residual_deg": _residual(q_end),
        "ramp_s": ramp_s,
        "max_delta_deg": max_d,
        "guards": guards,
    }


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


# ------------------------------------------------------------------ pose 子命令

class FakeTx:
    """dry-run / 自测用的假通道：不碰 CAN，恒定成功（或恒定失败）。

    track 给一个 {"q": [...]} 字典时，会把「上一条指令」写回去，用来模拟完美跟随，
    这样 dry-run 也能走完「到位判定 → 保持」这一段，而不是永远卡在未到位。
    """

    def __init__(self, ok=True, track=None):
        self.ok = ok
        self.track = track
        self.n_ok = 0
        self.n_fail = 0
        self.last_err = None
        self.sent = []

    def send_joint(self, q_rad):
        if self.ok:
            self.n_ok += 1
            self.sent.append(list(q_rad))
            if self.track is not None:
                self.track["q"] = [math.degrees(v) for v in q_rad]
            return True
        self.n_fail += 1
        self.last_err = "FakeTx: 注入的发送失败"
        return False


def check_pose_target(target, env, allow_outside=False):
    """守卫 F：目标必须整体落在包络内，否则抛 Abort。返回越界清单（空表示通过）。"""
    bad = []
    for i in range(1, 7):
        lo, hi = env[i]
        if not (lo <= target[i - 1] <= hi):
            bad.append((i, target[i - 1], lo, hi))
    if bad and not allow_outside:
        msg = ("目标越出「固件 ∩ URDF」安全包络，拒跑：\n"
               + "\n".join(f"  J{i}: 目标 {t:+.2f}° 不在 [{lo:+.2f}, {hi:+.2f}]"
                           for i, t, lo, hi in bad))
        raise Abort(msg + "\n  确实要越界请加 --allow-outside-limits（不建议："
                          "撞软限位会触发保护，且可能丢步而读数不报警）")
    return bad


def run_pose(args, env, piper, q0_rad, base_env=None):
    """S2-A：慢速走到绝对目标角并停住（供 S2-0 / S2-1 采静态电流）。"""
    target = [float(v) for v in args.target_deg]
    q0_deg = [math.degrees(v) for v in q0_rad]
    hz = args.hz

    # ---- 守卫 F：越界目标直接拒跑，不「限幅后照发」----
    # 限幅后照发最坏：脚本打印「成功」，人以为到了目标，实际停在限位上，
    # 静态电流对应的是另一个姿态 —— 这条数据会污染整条 K 拟合，且查不出来。
    bad = check_pose_target(target, env, args.allow_outside_limits)
    if bad:
        print("[pose][WARN] 已用 --allow-outside-limits 越过包络检查：\n"
              + "\n".join(f"  J{i}: 目标 {t:+.2f}° 不在 [{lo:+.2f}, {hi:+.2f}]"
                          for i, t, lo, hi in bad))

    delta = [t - s for s, t in zip(q0_deg, target)]
    max_d = max(abs(d) for d in delta)
    rate = max(abs(args.rate_deg_s), 1e-6)
    ramp_s = max_d / rate
    total_s = ramp_s + args.hold_s
    print(f"[pose] 起点 (deg) = {[round(v, 2) for v in q0_deg]}")
    print(f"[pose] 目标 (deg) = {[round(v, 2) for v in target]}")
    print(f"[pose] 行程 (deg) = {[round(v, 2) for v in delta]}")
    print(f"[pose] 最大单轴行程 {max_d:.2f}°，速率 {rate:g}°/s → 斜坡 {ramp_s:.1f} s，"
          f"保持 {args.hold_s:g} s，理论总长 {total_s:.1f} s")
    if max_d < 1e-6:
        print("[pose][NOTE] 目标与当前姿态相同，斜坡段只有 1 拍，直接进到位判定。")

    # ---- 通道 ----
    n_ka = [0]

    def _keepalive():
        """重发 0x151（运动模式帧）。

        2026-10-20 10:07 实测：没发 0x151 时，1063 帧关节指令**全部发送成功但臂一动没动**；
        之后任何人发一帧 0x151，臂立刻执行积压的目标跳过去。所以 0x151 不是可选装饰。

        走 SDK 的 `MotionCtrl_2` 而不是自己拼字节：速度百分比是安全相关量，而 SDK 文档
        里连电流字段的符号都写错了（写 uint16，实测 int16），所以宁可借它验证过的编码器。
        每 1 s 一帧，相对 100~300 帧/s 的关节指令可忽略，不构成 R30 那种「两套收发抢总线」。
        """
        if piper is None:
            return
        try:
            piper.MotionCtrl_2(0x01, 0x01, int(args.speed_pct), 0x00)
            n_ka[0] += 1
        except Exception as exc:
            print(f"[pose][WARN] 0x151 发送失败：{exc!r}")

    if args.dry_run:
        _cmd = {"q": list(q0_deg)}
        tx = FakeTx(track=_cmd)                  # 模拟完美跟随，只为跑通全流程

        def read_q_deg():
            return list(_cmd["q"])

        read_currents = None
        read_z_mm = None
    else:
        tx = Tx(args.iface or "can0")
        print(f"[pose] 下发通道已开：{args.iface or 'can0'}（自建 socket，逐帧判定成败）")
        print(f"[pose] 0x151：ctrl=CAN_CTRL move=MOVE_J speed={args.speed_pct:g}%（每 1 s 重发）")

        def read_q_deg():
            return [math.degrees(v) for v in read_q_rad(piper)]

        def read_currents():
            """六轴电流 A（0x251 的 current 字段，单位 0.001 A）。"""
            ms = piper.GetArmHighSpdInfoMsgs()
            return [getattr(ms, f"motor_{i}").current * 0.001 for i in range(1, 7)]

        def read_z_mm():
            """末端法兰 Z，mm。固件给 0.001 mm。"""
            return piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000.0

    _tot = (args.max_total_s if args.max_total_s > 0 else ramp_s * 3.0 + args.hold_s + 10.0)
    print(f"[pose] 守卫：电流>{args.max_current_a:g} A / 下降>{args.max_drop_mm:g} mm / "
          f"卡死窗口 {args.stall_win_s:g} s（阈值 {args.stall_eps_deg:g}°）/ "
          f"总时长上限 {_tot:.1f} s"
          + ("" if args.max_total_s > 0 else "（自动：理论×3+保持+10）"))

    try:
        res = pose_core(q0_deg, target, rate, hz, args.hold_s, tx, read_q_deg,
                        arrive_tol_deg=args.arrive_tol_deg,
                        arrive_cycles=max(1, int(round(args.arrive_cycles))),
                        settle_s=args.settle_s,
                        keepalive=(None if args.dry_run else _keepalive),
                        keepalive_every=max(1, int(round(hz))),
                        read_currents=read_currents,
                        cur_max_a=(args.max_current_a or None),
                        read_z_mm=read_z_mm,
                        z_drop_max_mm=(args.max_drop_mm or None),
                        stall_win_s=args.stall_win_s,
                        stall_eps_deg=args.stall_eps_deg,
                        hard_env=base_env,   # 硬包络（固件∩URDF，不外扩）；None=不夹

                        launch_s=args.launch_s,
                        launch_eps_deg=args.launch_eps_deg,
                        lead_max_deg=args.lead_max_deg,
                        max_cycles=(int(args.max_total_s * hz) if args.max_total_s > 0 else None))
    finally:
        if not args.dry_run:
            tx.close()

    # ---- 落盘：只写真的发出去的行（与 run() 同规矩）----
    out_csv = Path(args.out).with_suffix(".csv")
    out_meta = Path(args.out).with_suffix(".meta.json")
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{t:.6f}," + ",".join(f"{v * RAD_PER_DEG:.6f}" for v in q)
             for t, q in res["rows"]]
    out_csv.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
    meta = {
        "v": CMD_VERSION,
        "kind": "pose",
        "iface": args.iface,
        "dry_run": bool(args.dry_run),
        "hz": hz,
        "source_hz": SOURCE_HZ,
        "rate_deg_s": rate,
        "hold_s": args.hold_s,
        "ramp_s": res["ramp_s"],
        "t0_wall": (res["rows"][0][0] if res["rows"] else None),
        "n": len(res["rows"]),
        "n_ramp": res["n_ramp"],
        "n_hold": res["n_hold"],
        "hold_start_idx": res["hold_start"],
        "n_send_failed": res["n_fail"],
        "n_tx_frames_ok": getattr(tx, "n_ok", None),
        "n_tx_frames_fail": getattr(tx, "n_fail", None),
        "last_tx_error": getattr(tx, "last_err", None),
        "tx_ids": [f"0x{c:03X}" for c in JOINT_CTRL_IDS],
        "envelope_deg": {f"joint_{i}": list(env[i]) for i in range(1, 7)},
        "target_deg": target,
        "q0_deg": q0_deg,
        "q_end_deg": res["q_end_deg"],
        "residual_deg": res["residual_deg"],
        "max_delta_deg": res["max_delta_deg"],
        "arrive_tol_deg": args.arrive_tol_deg,
        "reached": res["reached"],
        "not_executed": res["not_executed"],
        "aborted": res["aborted"],
        "abort_reason": res["abort_reason"],
        "guards_fired": res.get("guards"),
        "guard_cfg": {
            "stall_win_s": args.stall_win_s,
            "stall_eps_deg": args.stall_eps_deg,
            "max_current_a": args.max_current_a,
            "max_drop_mm": args.max_drop_mm,
            "max_total_s": args.max_total_s,
        },
        "speed_pct": args.speed_pct,
        "n_motion_frames_151": n_ka[0],
        "csv_columns": ["t_wall", "j1", "j2", "j3", "j4", "j5", "j6"],
        "unit": "rad",
        "note": "pose：伺服步进（q_cmd = q_meas + clamp(Δ, ±rate*dt)），指令跟实测走；"
                "reached 只看实测 q；任一守卫触发即冻结在实测姿态并退出",
    }
    out_meta.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[pose] 写出 {out_csv}（{len(res['rows'])} 行）与 {out_meta.name}")
    if res["n_hold"]:
        print(f"[pose] 保持段 = CSV 第 {res['hold_start'] + 1}~{len(res['rows'])} 行"
              f"（{res['n_hold']} 拍，约 {res['n_hold'] / hz:.1f} s）")

    resid = res["residual_deg"]
    print(f"[pose] 末端实测 (deg) = {[round(v, 3) for v in res['q_end_deg']]}")
    print(f"[pose] 末端残差 (deg) = {[round(v, 3) for v in resid]}")
    if res["n_fail"]:
        print(f"[pose][WARN] {res['n_fail']} 拍发送失败，**没有写进 CSV** —— 轨迹有缺口")
    if res["aborted"]:
        print(f"[pose][ABORT] 守卫触发：{res['abort_reason']}", file=sys.stderr)
        print("[pose][ABORT] 已把目标改写成当前实测姿态并原地抱住；臂没有失能、没有掉落。",
              file=sys.stderr)
        return 4
    if not res["reached"]:
        worst = max(range(6), key=lambda i: abs(resid[i]))
        print(f"[pose][FAIL] 未到位：最大残差 J{worst + 1} = {resid[worst]:+.2f}° "
              f"（容差 {args.arrive_tol_deg:g}°）。可能被挡住、卡涩，或速率过大引跟踪滞后。",
              file=sys.stderr)
        return 3
    print(f"[pose] 到位（残差 ≤ {args.arrive_tol_deg:g}° 且连续 {args.arrive_cycles:g} 拍）")
    return 0


# ------------------------------------------------------------------ 主流程


def run(args):
    out_csv = Path(args.out).with_suffix(".csv")
    out_meta = Path(args.out).with_suffix(".meta.json")
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    # --allow-disabled 的解析放在最前面：dry-run 也要能走到（下游 meta 要写它），
    # 而且它跟机器人无关，没必要等连上 CAN 才解析。
    allow_dis = set()
    if getattr(args, "allow_disabled", ""):
        for tok in str(args.allow_disabled).replace("，", ",").split(","):
            tok = tok.strip()
            if not tok:
                continue
            if not tok.isdigit() or not (1 <= int(tok) <= 7):
                raise Abort(f"--allow-disabled 只接受 1~7 的轴号，收到 {tok!r}")
            allow_dis.add(int(tok))

    limits = None
    if args.limits:
        limits = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    base_env = safe_envelope(limits, tol_deg=0.0)
    env = safe_envelope(limits, tol_deg=args.limit_tol_deg)
    print("[cmd] 实际安全包络（固件 ∩ URDF"
          + (f"，外扩 {args.limit_tol_deg:g}°" if args.limit_tol_deg else "") + "，deg）："
          + "  ".join(f"J{i}[{env[i][0]:+.1f},{env[i][1]:+.1f}]" for i in range(1, 7)))
    if limits:
        for i in range(1, 7):
            d = limits.get(f"joint_{i}") or {}
            fw = (d.get("min_deg"), d.get("max_deg"))
            # 与「外扩前的标称包络」比，否则外扩 3° 会让每根轴都误报一次「被 URDF 收紧」
            if fw[0] is not None and (abs(base_env[i][0] - fw[0]) > 1.0
                                      or abs(base_env[i][1] - fw[1]) > 1.0):
                print(f"[cmd][NOTE] J{i} 被 URDF 收紧：固件 [{fw[0]:+.2f},{fw[1]:+.2f}] → "
                      f"标称 [{base_env[i][0]:+.2f},{base_env[i][1]:+.2f}]，"
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
        # 默认六轴必须全使能（FIRST_ARM 红线）。唯一的例外是**显式**用
        # --allow-disabled 点名某个轴失能：那种情况下该轴的目标会被钉死在实测值上，
        # 幅值强制为 0，脚本对它是「只发原地保持帧」，不会试图驱动它。
        # 2026-10-20 14:0x 就是缺这个开关：J6 驱动被保护锁死，连「把臂合拢到安全位」
        # 都做不到，因为 pre-pose 一律零帧 Abort。
        en = read_enable(piper)
        if en is not None:
            bad = [i + 1 for i, v in enumerate(en) if not v and (i + 1) not in allow_dis]
            if bad:
                raise Abort(f"六轴未全使能：{en}（失能且未在 --allow-disabled 里点名的是 {bad}）。"
                            f"使能不是本脚本的职责（FIRST_ARM 红线），"
                            f"先在厂家上位机或按现场规程使能。")
            if allow_dis:
                print(f"[cmd][WARN] --allow-disabled={sorted(allow_dis)}："
                      f"这些轴即使失能也继续，但它们的目标会被钉在实测值上、幅值=0，"
                      f"相当于「只原地保持、不驱动」。实测使能 {en}")

        q0 = read_q_rad(piper)
        print(f"[cmd] 当前姿态 (deg) = {[round(math.degrees(v), 2) for v in q0]}")

        if max(abs(v) for v in q0) < 1e-6:
            raise Abort("六个关节读数全为 0 —— 这是 SDK 没收到帧时的默认值，不是真实姿态"
                        "（R18）。先确认总线有反馈帧，再跑这个脚本。")

        # 把 --allow-disabled 点名的轴「焊死」在实测值上：
        #   * sine/step：幅值置 0（轨迹就退化成该轴原地不动）
        #   * pose：目标 = 当前实测（误差恒为 0，到位判定不会因它失败）
        # 这样即便驱动器没使能、根本不跟指令，也不会让守卫误判「卡死」。
        if allow_dis:
            for i in sorted(allow_dis):
                if i > 6:
                    continue
                args.amp_deg[i - 1] = 0.0
                args.target_deg[i - 1] = round(math.degrees(q0[i - 1]), 6)
            print(f"[cmd] 轴 {sorted(i for i in allow_dis if i <= 6)} 已钉在实测："
                  f"{[round(math.degrees(q0[i - 1]), 2) for i in sorted(allow_dis) if i <= 6]} deg")

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
            elif args.kind == "pose":
                _d = [t - math.degrees(s) for t, s in zip(args.target_deg, q0)]
                _mx = max(abs(v) for v in _d)
                print(f"\n即将下发 pose：{args.out}，@ {args.hz:g} Hz，速率 {args.rate_deg_s:g}°/s，"
                      f"到位后保持 {args.hold_s:g} s")
                print(f"起点 = {[round(math.degrees(v), 2) for v in q0]} deg")
                print(f"目标 = {[round(v, 2) for v in args.target_deg]} deg")
                print(f"行程 = {[round(v, 2) for v in _d]} deg，"
                      f"最大单轴 {_mx:.2f}° → 斜坡约 {_mx / max(args.rate_deg_s, 1e-6):.1f} s")
            else:
                print(f"\n即将下发 {args.kind} 轨迹：{args.duration:g} s @ {args.hz:g} Hz，"
                      f"幅值 {args.amp_deg} deg，单周期上限 {args.max_step_deg:g} deg")
            print("确认臂周围无人、无遮挡、手能碰到急停。输入 yes 继续：", end=" ", flush=True)
            if sys.stdin.readline().strip().lower() not in ("yes", "y"):
                raise Abort("操作者未确认，已中止。")

    if args.kind == "pose":
        return run_pose(args, env, piper, q0, base_env)

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

    # ---- 运行时守卫（sine / step / hold 这条路径）----
    # 为什么必须补：2026-10-20 13:5x，J6 在 ±15° 正弦里负向顶死，电流 12.8 A
    # **持续了 58.7 s**，驱动被打成「过流+驱动错误+堵转」并锁死，
    # 而这段脚本一路跑到 duration 结束、退出码 0，把废数据当成功收下。
    # 根因就是这条路径此前只有「单周期限幅」和「发送失败计数」，没有任何反馈侧的守卫
    # （`pose` 走的是 pose_core，那边三道守卫齐全）。
    cur_fn = None
    if not args.dry_run and args.max_current_a:
        def cur_fn():                                  # noqa: F811
            ms = piper.GetArmHighSpdInfoMsgs()
            return [getattr(ms, f"motor_{i}").current * 0.001 for i in range(1, 7)]

    guard_every = max(1, int(round(hz / 50.0)))        # 50 Hz 查一次，别把 100 Hz 节拍压垮
    guard = TrajGuard(cur_limit_a=args.max_current_a,
                      cur_grace_s=args.cur_grace_s,
                      stall_win_ticks=max(4, int(args.stall_win_s * hz)),
                      stall_eps_deg=args.stall_eps_deg,
                      stall_lead_deg=args.stall_lead_deg,
                      q0=q0, every=guard_every)
    guard_reason = None

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

            # ---- 守卫：电流 + 卡死（sine/step/hold 路径，2026-10-20 补）----
            if k % guard_every == 0:
                _cur = None
                if cur_fn is not None:
                    try:
                        _cur = cur_fn()
                    except Exception:
                        _cur = None
                _qm = None
                if piper is not None:
                    try:
                        _qm = read_q_rad(piper)
                    except Exception:
                        _qm = None
                guard_reason = guard.feed(k, time.time(), _cur, _qm, q_want)
                if guard_reason:
                    break

        if guard_reason:
            aborted = True
            print(f"[cmd][ABORT] {guard_reason}", file=sys.stderr)
            # 原地抱住：重发**实测姿态**若干拍。绝不失能（失能即自由下落，手册 §10.2）。
            try:
                if tx is not None and piper is not None:
                    q_hold = read_q_rad(piper)
                    for _ in range(5):
                        tx.send_joint(q_hold)
                        time.sleep(0.01)
                    print(f"[cmd] 已原地抱住（重发实测姿态 5 拍）："
                          f"{[round(math.degrees(v), 2) for v in q_hold]} deg")
            except Exception as exc:
                print(f"[cmd][WARN] 抱住帧发送失败：{exc!r}", file=sys.stderr)

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
        "guard_reason": guard_reason,
        "allow_disabled": sorted(allow_dis),
        "max_current_a": args.max_current_a,
        "cur_grace_s": args.cur_grace_s,
        "stall_win_s": args.stall_win_s,
        "stall_eps_deg": args.stall_eps_deg,
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
    if guard_reason:
        return 4                    # 与 pose 的「运行时守卫触发」用同一个退出码
    return 0


def selftest() -> int:
    """离线自测（任何机器可跑，不碰 CAN）。plan §S2-A 要求的两条都在里面。"""
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

    env = safe_envelope({"joint_1": {"min_deg": -150.0, "max_deg": 150.0},
                         "joint_2": {"min_deg": 0.0, "max_deg": 180.0},
                         "joint_3": {"min_deg": -170.0, "max_deg": 0.0}},
                        tol_deg=0.0)
    # 1. 包络外扩：J2 下界从 0 放到 −3（实测自然零位 −2.003 必须落在里面）
    env_tol = safe_envelope({"joint_2": {"min_deg": 0.0, "max_deg": 180.0}}, tol_deg=3.0)
    chk("F0 外扩 3° 后 J2 下界 = −3", abs(env_tol[2][0] + 3.0) < 1e-9, str(env_tol[2]))
    chk("F0 外扩后自然零位 −2.003 在包络内",
        env_tol[2][0] <= -2.003 <= env_tol[2][1], str(env_tol[2]))

    # 2. F_pose_limits：越界目标必须被拒（不是「限幅后照发」）
    try:
        check_pose_target([0, 200, -60, 0, 45, 0], env)
        chk("F1 越界目标被拒（J2=200）", False, "没有抛 Abort")
    except Abort:
        chk("F1 越界目标被拒（J2=200）", True)
    try:
        check_pose_target([0, 45, -60, 0, 45, 0], env)
        chk("F2 合法目标放行", True)
    except Abort as exc:
        chk("F2 合法目标放行", False, str(exc))

    # 3. R42 核心：卡住时「指令跟实测走」，并且连续进度守卫必须把动作停掉
    q0 = [0.0, 45.0, -60.0, 0.0, 45.0, 0.0]
    tgt = [10.0, 60.0, -40.0, 5.0, 50.0, 10.0]
    res = pose_core(q0, tgt, 50.0, 200.0, 0.0, FakeTx(), lambda: list(q0),
                    arrive_tol_deg=0.5, arrive_cycles=20, settle_s=0.1,
                    stall_win_s=0.1, stall_eps_deg=0.05, log=lambda *a: None)
    chk("G1 卡住 → 卡死守卫触发", res["aborted"] is True and "卡死" in (res["abort_reason"] or ""),
        str(res["abort_reason"]))
    chk("G1 卡住 → 不假装到位", res["reached"] is False)
    chk("G1 卡住仍写下了真实下发行", len(res["rows"]) > 0, f'{len(res["rows"])}')
    chk("G1 卡住 → 残差非零", max(abs(v) for v in res["residual_deg"]) > 5.0)
    # 关键：卡住时最后发出的指令不许离实测超过一步（旧版会一路顶到目标）
    last_cmd = res["rows"][-1][1]
    lead_max = 2.0
    chk("G1 卡住 → 指令没顶向目标（领先不超过 lead_max=2°）",
        max(abs(a - b) for a, b in zip(last_cmd, q0)) <= lead_max + 1e-6,
        f"领先 {max(abs(a - b) for a, b in zip(last_cmd, q0)):.3f}° > {lead_max:.3f}°")

    # 3b. 电流守卫：顶住障碍时电流飙升，必须比「位移为 0」更早停
    res_c = pose_core(q0, tgt, 50.0, 200.0, 0.0, FakeTx(), lambda: list(q0),
                      arrive_tol_deg=0.5, arrive_cycles=20, settle_s=0.1,
                      read_currents=lambda: [0.1, 5.2, 0.1, 0.1, 0.1, 0.1],
                      cur_max_a=3.0, cur_grace_s=0.05,
                      stall_eps_deg=1e9,  # 关掉卡死守卫，单测电流
                      log=lambda *a: None)
    chk("G4 电流超限 → ABORT", res_c["aborted"] is True and "电流" in (res_c["abort_reason"] or ""),
        str(res_c["abort_reason"]))
    chk("G4 电流守卫比卡死守卫更早（拍数更少）",
        len(res_c["rows"]) <= max(3, len(res["rows"])), f'{len(res_c["rows"])} vs {len(res["rows"])}')

    # 3c. 高度守卫：末端 Z 一路往下掉，即使没卡住也要停
    _z = {"v": 400.0}

    def _zread():
        _z["v"] -= 4.0          # 每拍掉 4 mm
        return _z["v"]
    res_z = pose_core(q0, tgt, 50.0, 200.0, 0.0, FakeTx(), lambda: list(q0),
                      arrive_tol_deg=0.5, arrive_cycles=20, settle_s=0.1,
                      read_z_mm=_zread, z_drop_max_mm=100.0,
                      stall_eps_deg=1e9, log=lambda *a: None)
    chk("G5 末端下降超限 → ABORT", res_z["aborted"] is True and "高度" in (res_z["abort_reason"] or ""),
        str(res_z["abort_reason"]))
    chk("G5 触发时下降量已超过阈值", "100" in (res_z["abort_reason"] or ""), str(res_z["abort_reason"]))

    # 4. 既写指令又跟随时，必须到位；且最后一行就是目标
    cmd = {"q": list(q0)}

    def track():
        return list(cmd["q"])
    tx2 = FakeTx()

    class _T(FakeTx):
        def send_joint(self, q_rad):
            cmd["q"] = [math.degrees(v) for v in q_rad]
            return FakeTx.send_joint(self, q_rad)

    res2 = pose_core(q0, tgt, 200.0, 200.0, 0.05, _T(), track,
                     arrive_tol_deg=0.5, arrive_cycles=5, settle_s=0.5,
                     log=lambda *a: None)
    chk("G2 跟随 → reached=True", res2["reached"] is True)
    chk("G2 末行 = 目标", max(abs(a - b) for a, b in zip(res2["rows"][-1][1], tgt)) < 1e-9)
    chk("G2 保持了 hold 段", res2["n_hold"] > 0, str(res2["n_hold"]))

    # 5. 速率限制真的生效：每拍变化 ≤ rate*dt（这是唯一的无斜率保护）
    dt = 1.0 / 200.0
    prev = list(q0)
    worst = 0.0
    for _, q in res2["rows"]:
        worst = max(worst, max(abs(a - b) for a, b in zip(q, prev)))
        prev = q
    chk("G3 每拍变化 ≤ 速率×dt", worst <= 200.0 * dt + 1e-6, f"{worst:.4f} deg")

    # 6. 发送失败的行不许进 CSV（2.0 的第一条硬规矩，别在 pose 上退化）
    #    速率 25°/s ⇒ 斜坡 1 s（200 拍），足够触发「连续 30 拍失败即 ABORT」
    tx3 = FakeTx(ok=False)
    res3 = pose_core(q0, tgt, 25.0, 200.0, 0.0, tx3, lambda: list(q0),
                     settle_s=0.05, log=lambda *a: None)
    chk("H1 全失败 → rows 为空", len(res3["rows"]) == 0, str(len(res3["rows"])))
    chk("H1 全失败 → aborted=True", res3["aborted"] is True)
    chk("H1 全失败 → 记录了失败拍数", res3["n_fail"] >= 30, str(res3["n_fail"]))

    # 7. 目标解析：逗号与空格两种写法都要能吃
    chk("I1 逗号解析", _parse_target_deg(["0,45,-60,0,45,0"]) == [0.0, 45.0, -60.0, 0.0, 45.0, 0.0])
    chk("I2 空格解析", _parse_target_deg(["0", "45", "-60", "0", "45", "0"])
        == [0.0, 45.0, -60.0, 0.0, 45.0, 0.0])

    # 8. 2026-10-20 10:53 的「100.9 s 一动不动」：起点越限 + 指令只领先 0.06° ⇒ 什么都不发生
    #    模拟固件：六轴同步插值，只要有一轴目标越界就整帧不执行（实测就是这么表现的：
    #    连合法的 J1 也一格没动）。每拍最多追 0.5°，够快到能跑完。
    LIM = {i: (-180.0, 180.0) for i in range(1, 7)}
    LIM[2] = (0.0, 180.0)
    LIM[3] = (-170.0, 0.0)

    class _Fw:
        """假固件：目标里有越界轴 ⇒ 整帧丢弃；否则向目标靠近（每拍 ≤0.5°）。"""

        def __init__(self, q):
            self.q = list(q)
            self.accept = 0
            self.reject = 0
            self.n_ok = 0
            self.n_fail = 0
            self.last_err = None

        def send_joint(self, q_rad):
            q = [math.degrees(v) for v in q_rad]
            self.n_ok += 1
            if any(q[i - 1] < LIM[i][0] - 1e-9 or q[i - 1] > LIM[i][1] + 1e-9
                   for i in range(1, 7)):
                self.reject += 1
                return True
            self.accept += 1
            self.q = [a + max(-0.5, min(0.5, b - a)) for a, b in zip(self.q, q)]
            return True

    q_rest = [0.0, -1.998, 1.818, 0.0, 21.748, 0.0]      # 实测停放姿态（J2/J3 越限）
    tgt_p1 = [0.0, 45.0, -60.0, 0.0, 45.0, 0.0]

    fw_a = _Fw(q_rest)
    res_a = pose_core(q_rest, tgt_p1, 50.0, 200.0, 0.0, fw_a, lambda: list(fw_a.q),
                      settle_s=0.05, launch_s=1e9, log=lambda *a: None)
    chk("J1 起点越限也不再死锁（虚拟目标会跨过边界后被固件接受）",
        res_a["reached"] is True, str(res_a["abort_reason"]))
    chk("J1 但起步阶段确实有帧被固件丢弃（说明越限判断是真的）",
        fw_a.reject > 0 and fw_a.accept > 0, f"accept={fw_a.accept} reject={fw_a.reject}")
    first_lead = max(abs(a - b) for a, b in zip(res_a["rows"][0][1], q_rest))
    chk("J1 首拍领先不超过 lead_max（卡住场景由 G1 守住）",
        first_lead <= 2.0 + 1e-6, f"{first_lead:.3f}°")

    fw_b = _Fw(q_rest)
    res_b = pose_core(q_rest, tgt_p1, 50.0, 200.0, 0.0, fw_b, lambda: list(fw_b.q),
                      settle_s=0.05, hard_env=LIM, log=lambda *a: None)
    chk("J2 给硬包络 → 能走起来并到位", res_b["reached"] is True, str(res_b["abort_reason"]))
    bad_rows = [r for _, r in res_b["rows"]
                if any(r[i - 1] < LIM[i][0] - 1e-6 or r[i - 1] > LIM[i][1] + 1e-6
                       for i in range(1, 7))]
    chk("J2 硬包络 → 下发的指令没有一条越界", len(bad_rows) == 0, f"{len(bad_rows)} 行越界")
    chk("J2 首拍指令就把越限轴夹到边界（J2=0, J3=0）",
        abs(res_b["rows"][0][1][1] - 0.0) < 1e-6 and abs(res_b["rows"][0][1][2] - 0.0) < 1e-6,
        str(res_b["rows"][0][1]))

    # 9. 起步守卫：指令发了但臂完全不动时，3 s 内就报「起步失败」，不白等到 deadline
    res_c2 = pose_core(q_rest, tgt_p1, 6.0, 100.0, 0.0, FakeTx(), lambda: list(q_rest),
                       settle_s=0.05, hard_env=LIM, launch_s=0.3, launch_eps_deg=0.05,
                       log=lambda *a: None)
    chk("J3 完全不动 → 起步守卫触发", res_c2["aborted"] is True
        and "起步失败" in (res_c2["abort_reason"] or ""), str(res_c2["abort_reason"]))
    chk("J3 起步守卫比总时长上限早得多（<50 拍）", len(res_c2["rows"]) < 50, str(len(res_c2["rows"])))

    # ---- G 组：轨迹路径（sine/step/hold）的电流 + 卡死守卫 ----
    # 这一组就是 2026-10-20 13:5x 那次事故的回归测试：J6 12.8 A 顶了 58.7 s，
    # 脚本却跑完退出码 0。现在要求「超限持续 grace 就停」。
    g = TrajGuard(cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                  stall_eps_deg=0.08, q0=[0.0] * 6, every=2)
    over = [0.1, 0.1, 0.1, 0.1, 0.1, 12.8]
    ok_cur = [0.1, 0.1, 0.1, 0.1, 0.1, 0.2]
    r = None
    trip_tick = None
    for tick in range(0, 200, 2):
        r = g.feed(tick, tick * 0.01, over, [0.0] * 6, [0.0] * 6)
        if r:
            trip_tick = tick
            break
    chk("G1 12.8 A 持续超限 → 触发电流守卫", r is not None and "电流守卫" in r, str(r))
    chk("G2 触发时刻在 grace 之后（0.3 s = 30 拍）", trip_tick is not None and trip_tick >= 30,
        str(trip_tick))
    chk("G3 守卫认的是 J6（不是别的轴）", r is not None and "J6" in r, str(r))

    g2 = TrajGuard(cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                   stall_eps_deg=0.08, q0=[0.0] * 6, every=2)
    r2 = None
    for tick in range(0, 400, 2):
        r2 = g2.feed(tick, tick * 0.01, ok_cur, [0.0] * 6, [0.0] * 6)
        if r2:
            break
    chk("G4 电流正常时绝不误报", r2 is None, str(r2))

    g3 = TrajGuard(cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                   stall_eps_deg=0.08, q0=[0.0] * 6, every=2)
    r3 = None
    trip3 = None
    for tick in range(0, 400, 2):
        # 指令每拍推进 0.01 rad，实测纹丝不动 = 卡死
        cmd = [0.01 * tick / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        r3 = g3.feed(tick, tick * 0.01, ok_cur, [0.0] * 6, cmd)
        if r3:
            trip3 = tick
            break
    chk("G5 指令走、实测不走 → 触发卡死守卫", r3 is not None and "卡死" in r3, str(r3))
    chk("G6 卡死判定在窗口内（≤ 110 拍）", trip3 is not None and trip3 <= 110, str(trip3))

    g4 = TrajGuard(cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                   stall_eps_deg=0.08, q0=[0.0] * 6, every=2)
    r4 = None
    for tick in range(0, 400, 2):
        cmd = [0.01 * tick / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        r4 = g4.feed(tick, tick * 0.01, ok_cur, list(cmd), cmd)   # 完美跟随
        if r4:
            break
    chk("G7 完美跟随时不误报卡死", r4 is None, str(r4))

    # G8 是 14:2x S1_6_mid 那次**误杀**的逐字复现：正弦峰顶附近，指令净走 0.168°、
    # 实测净走 0.031°、滞后只有 0.14°。旧判据（只要 cmd_prog≥0.08 且 meas_prog<0.08）
    # 会在这里报「卡死」，把一条完全正常的轨迹打断。加了 lead 条件后必须不报。
    g5 = TrajGuard(cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                   stall_eps_deg=0.08, stall_lead_deg=2.0, q0=[0.0] * 6, every=2)
    r5 = None
    for tick in range(0, 400, 2):
        f = tick / 400.0
        cmd5 = [0.168 * RAD_PER_DEG * f, 0, 0, 0, 0, 0]
        meas5 = [0.031 * RAD_PER_DEG * f, 0, 0, 0, 0, 0]
        r5 = g5.feed(tick, tick * 0.01, ok_cur, meas5, cmd5)
        if r5:
            break
    chk("G8 正弦峰顶（指令0.168°/实测0.031°/滞后0.14°）不误报卡死", r5 is None, str(r5))

    # 反向：滞后够大时，同样「实测不走」必须照样报（否则 G8 就是把守卫改废了）
    g6 = TrajGuard(cur_limit_a=3.0, cur_grace_s=0.3, stall_win_ticks=100,
                   stall_eps_deg=0.08, stall_lead_deg=2.0, q0=[0.0] * 6, every=2)
    r6 = None
    for tick in range(0, 400, 2):
        f = tick / 400.0
        cmd6 = [10.0 * RAD_PER_DEG * f, 0, 0, 0, 0, 0]     # 指令走 10°，实测 0.031°
        meas6 = [0.031 * RAD_PER_DEG * f, 0, 0, 0, 0, 0]
        r6 = g6.feed(tick, tick * 0.01, ok_cur, meas6, cmd6)
        if r6:
            break
    chk("G9 滞后 10° 且实测不走 → 仍然报卡死", r6 is not None and "卡死" in r6, str(r6))
    chk("G9 卡死原因里带滞后量", r6 is not None and "滞后" in r6, str(r6))

    print(f"\n自测：{'全部通过' if bad == 0 else f'{bad} 条失败'}（{ok + bad} 条）")
    return 0 if bad == 0 else 1


def _parse_target_deg(raw):
    vals = []
    for chunk in raw:
        vals.extend(str(chunk).replace(",", " ").split())
    return [float(v) for v in vals]


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="L0-b 指令层：下发并记录 q_cmd（与 stage1_l0.py parse --qcmd 配套）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="CSV 无表头：t_wall,j1..j6，单位 rad，epoch 秒。")
    ap.add_argument("kind", choices=list(TRAJ) + ["selftest"],
                    help="hold 保持 / sine 正弦 / step 阶跃 / pose 走到绝对目标角 / selftest 离线自测")
    ap.add_argument("--iface", default="can0", help="CAN 接口名，默认 can0")
    ap.add_argument("--out", default=None, help="输出前缀（会生成 .csv 与 .meta.json）")
    ap.add_argument("--duration", type=float, default=10.0, help="时长 s，默认 10（pose 不用它）")
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
    ap.add_argument("--limit-tol-deg", type=float, default=3.0,
                    help="包络向外放宽（度），默认 3.0。实测自然零位 J2=−2.0°/J3=+1.8° 越过标称，"
                         "不放宽的话限幅会把臂从零位硬拽 2°（限幅自己造出一次移动）")
    ap.add_argument("--limits", default=None,
                    help="joint_limits.json（由 stage0_probe --query-limits 产出）")
    ap.add_argument("--dry-run", action="store_true", help="不连 CAN、不下发，只生成 CSV")
    ap.add_argument("--force-mode", action="store_true",
                    help="[危险] 越过 ctrl_mode != CAN_CTRL 的检查")
    ap.add_argument("--yes", action="store_true", help="跳过人工确认（脚本化时用）")
    ap.add_argument("--t0", type=float, default=None,
                    help="起始 epoch 秒；测试时可固定，默认取当前时间")
    # ---- pose 专用 ----
    ap.add_argument("--target-deg", nargs="+", default=None,
                    help="pose 目标六轴角（deg）：`--target-deg 0,45,-60,0,45,0` 或 6 个空格分隔的数")
    ap.add_argument("--rate-deg-s", type=float, default=8.0,
                    help="pose 斜坡角速度（deg/s），默认 8。慢一点，静态采集时振铃小")
    ap.add_argument("--hold-s", type=float, default=30.0,
                    help="pose 到位后保持的时长（s），默认 30")
    ap.add_argument("--arrive-tol-deg", type=float, default=0.5,
                    help="pose 到位容差（deg，按实测 q），默认 0.5")
    ap.add_argument("--arrive-cycles", type=int, default=20,
                    help="pose 连续满足容差的拍数，默认 20（100 Hz 下 0.2 s）")
    ap.add_argument("--settle-s", type=float, default=3.0,
                    help="pose 到位观察窗口（s），默认 3；窗口内连续满足容差才算到位")
    ap.add_argument("--speed-pct", type=float, default=30.0,
                    help="pose 下发 0x151 时的速度百分比（0~100），默认 30。这是臂自身的上限，"
                         "真正决定快慢的是 --rate-deg-s")
    ap.add_argument("--stall-win-s", type=float, default=1.0,
                    help="卡死判据的窗口长度 s，默认 1.0：窗口内指令走了、实测没走就 ABORT")
    ap.add_argument("--stall-eps-deg", type=float, default=0.08,
                    help="上面窗口内「算走了」的最小角度，默认 0.08°")
    ap.add_argument("--stall-lead-deg", type=float, default=2.0,
                    help="卡死判定的必要条件：指令超前实测的比例超过此值才算卡死，默认 2.0°。"
                         "少了这一条，正弦轨迹在峰顶（净位移天然≈0）会被误判成卡死"
                         "（2026-10-20 14:2x S1_6_mid 实测：指令净走 0.168°、实测 0.031°）")
    ap.add_argument("--max-current-a", type=float, default=3.0,
                    help="任一轴电流超过此值（持续 --cur-grace-s）即 ABORT 并原地抱住，默认 3.0 A。"
                         "空载静态实测 0.1~0.2 A，顶住障碍会飙到数 A。给 0 关闭")
    ap.add_argument("--cur-grace-s", type=float, default=0.3,
                    help="电流超限需要持续多久才判 ABORT，默认 0.3 s。给 0 表示一超就停")
    ap.add_argument("--allow-disabled", type=str, default="",
                    help="允许失能的轴，如 '6' 或 '4,6'。这些轴的目标会被钉在实测值上、"
                         "幅值强制为 0，全程不发运动（只发原地保持）。用于「某轴驱动故障、"
                         "其余轴需要合拢到安全位」的场景。默认空=六轴必须全使能（FIRST_ARM 红线）")
    ap.add_argument("--max-drop-mm", type=float, default=150.0,
                    help="末端 Z 相对起始位置最多允许下降多少 mm，默认 150。"
                         "掉进底盘前 Z 一定先掉，给 0 关闭")
    ap.add_argument("--max-total-s", type=float, default=0.0,
                    help="总时长硬上限 s，默认 0=按理论时长×3 自动算")
    ap.add_argument("--launch-s", type=float, default=3.0,
                    help="起步观察窗 s，默认 3。窗口内实测位移 < --launch-eps-deg 即判「起步失败」"
                         "（指令根本没被执行），不再白等到总时长上限")
    ap.add_argument("--launch-eps-deg", type=float, default=0.05,
                    help="起步窗口内「算走起来了」的最小实测位移，默认 0.05°")
    ap.add_argument("--lead-max-deg", type=float, default=2.0,
                    help="指令最多允许领先实测多少度，默认 2.0。这是「永不往障碍里顶」的"
                         "唯一定量保证：卡住时最多领先 2°，不会再像旧斜坡那样一路顶到目标。"
                         "太小（<0.5°）固件不产生速度，臂会原地不动——0.06° 已经在 2026-10-20 实测过")
    ap.add_argument("--allow-outside-limits", action="store_true",
                    help="[危险] 允许 pose 目标越出包络")
    args = ap.parse_args(argv)

    if args.kind == "selftest":
        return selftest()

    if not args.out:
        ap.error("--out 必填（selftest 除外）")

    if args.kind == "pose":
        if not args.target_deg:
            ap.error("pose 必须给 --target-deg 六个角度")
        vals = _parse_target_deg(args.target_deg)
        if len(vals) != 6:
            ap.error(f"--target-deg 需要 6 个数，收到 {len(vals)} 个")
        args.target_deg = vals

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
