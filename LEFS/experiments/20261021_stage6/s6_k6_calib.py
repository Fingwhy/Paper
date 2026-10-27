#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s6_k6_calib.py 1.0 — 补标 J6 力矩常数 K6（阶段 6 的前置标定）

为什么要单独做这一步
--------------------
阶段 2 的 K 表里 J6 是 null（20261019_stage2/calibration/k_table_free.json）：

    "J6": { "state": "未标定", "K": null,
            "reason": "水瓶质心几乎在工具轴（=J6 转轴）上 ⇒ τ6≈0；需偏心横杆 80 mm" }

定量地看：水瓶质心横偏只有 5.3 mm ⇒ τ6 全程跨度 0.054 N·m，
而电流噪声地板是 0.111 N·m ⇒ 信噪比 < 1。s21_group_fit 强行拟合能出数，
但 σ_K/K = 21%（对比 J2 5%、J3 3.6%），不能用于绝对扭矩。

**根因不是方法错，是没有扭矩源。** 这个脚本给它造一个：挂砝码。

原理
----
    τ = K6·I6 + b          b = 常数偏置（摩擦 + 横杆自重），不可省
    τ_i = m_i · g · L      L = 悬挂点到 J6 轴的距离（沿杆，水平）

用 N ≥ 3 个质量点做最小二乘，斜率就是 K6。

几何前提（三条，缺一条数据就废）
--------------------------------
1. **J6 轴水平**：|u_z| < 0.17（≈10°）。轴竖直时 r×F 退化成零，力矩恒 0。
2. **横杆水平**：轴水平 ⇒ 径向平面是竖直平面，其中唯一的水平方向就是切向；
   横杆水平时自动垂直于轴，力臂不衰减。
3. **砝码自然下垂**：力恒为竖直向下，不需要弹簧秤、不需要人手稳住、不需要读力值。

**最容易踩的坑**：横杆竖直朝下、砝码挂在末端 —— 力沿杆方向，r×F = 0，力矩为零，
但电流读数看起来完全正常。所以必须先跑 `probe` 看倾角，不能直接开测。

电流口径（这个坑 stage3 踩过）
------------------------------
必须用高速帧 `GetArmHighSpdInfoMsgs().motor_6.current`（0x251，单位 0.001 A）。
**不能用** `GetArmLowSpdInfoMsgs().motor_6.bus_current`（0x261）：
官方明确「1.5 kg 关节电机无采样，默认发 0」。s3_j6_live.py 读的就是后者。

用法（VM，~/stage0/20261021_stage6/ 下）
----------------------------------------
    # 0) 断电重启后是 TEACHING，必须先切 CAN_CTRL（缺 --allow-write 会静默不执行）
    python3 ../20261019_stage1_l0/stage1_cmd.py pose \\
        --phase sdk --allow-write --to-can-ctrl

    # 1) 只读检查姿态（不发任何帧）
    python3 s6_k6_calib.py probe

    # 2) 交互式测量（持续发保持帧，臂刚性顶住砝码）
    python3 s6_k6_calib.py run --arm 0.100

    # 3) 只想重算拟合
    python3 s6_k6_calib.py fit --csv data/k6_20261022_101500.csv

    # 4) 离线自测（不连 CAN）
    python3 s6_k6_calib.py selftest

安全
----
- 脚本**不做任何主动运动**：把启动瞬间的实测 q 锁成固定目标反复重发，臂原地顶住。
  刻意不用「q_cmd = q_meas + Δ」那种跟实测走的写法 —— 那样砝码会把臂一点点拽下来。
- 双守卫：任一关节电流 > 3 A，或 J6 位置漂移 > 5° ⇒ 立刻停止发帧并报警，**不失能**。
- 退出后臂仍在 CAN_CTRL 保持。**必须先卸砝码，再断电**：
  实测六轴失能会自由落体（J2 曾 60°→34.9°），届时砝码会把它加速砸下来。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
URDF = HERE.parent / "_vendor" / "urdf" / "piper_description.urdf"
STAGE2 = HERE.parent / "20261019_stage2"

DT_READ = 0.005        # 200 Hz 读
DT_SEND = 0.020        # 50 Hz 发保持帧（与 200 Hz 反馈同跑时不抢总线）
MAX_CUR_A = 3.0        # 任一轴电流硬上限
MAX_DRIFT_DEG = 5.0    # J6 被拉偏超过这个角度，说明位置环顶不住，该点作废
SETTLE_S = 2.0         # 挂上配重后先硬等这么久
STABLE_WIN = 0.5       # 判定「稳住了」的统计窗口，秒
STABLE_STD = 0.010     # 窗口内 I6 标准差低于这个值（A）才算稳
STABLE_MAX_WAIT = 15.0 # 最多等这么久，等不到也强行采（并在报告里标注）
G = 9.80665
WATER_DENSITY = 0.9982  # kg/L @ 20 ℃ —— 路线 2「量水替代砝码」用这个换算
J6_LEVEL = 140.0        # 夹爪竖直位（用户实测）。此时钢尺水平，力矩才全部投影到 J6 轴

JOINT_IDS = (0x155, 0x156, 0x157)   # 每帧装两个关节，int32，0.001 度，大端有符号
CTRL_CAN = 1


# ---------------------------------------------------------------- 小工具

def load_arm():
    """复用阶段 2 的 URDF 正运动学（FK 已与固件末端位姿核验到亚毫米）。"""
    sys.path.insert(0, str(STAGE2))
    import stage2_design as S
    urdf = S.Urdf(URDF)
    return S.Arm(urdf, end_mass=0.5, end_com=urdf.gripper_com())


def axis_report(arm, q_deg):
    """返回 (J6 轴单位向量, link6 原点, x 轴, y 轴)。J6 轴 = URDF 里 joint6 的 axis [0,0,1]。"""
    q = [math.radians(v) for v in q_deg]
    T = arm.transforms(q)
    R, p = T["link6"]
    col = lambda c: [R[0][c], R[1][c], R[2][c]]          # noqa: E731
    return col(2), p, col(0), col(1)


def tilt_deg(v):
    """向量与水平面的夹角（度）。0 = 完全水平。"""
    return math.degrees(math.asin(max(-1.0, min(1.0, abs(v[2])))))


def fit_line(xs, ys):
    """最小二乘 y = K·x + b，不依赖 numpy（VM 上没装）。"""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0:
        return None
    K = sxy / sxx
    b = my - K * mx
    resid = [y - (K * x + b) for x, y in zip(xs, ys)]
    rss = sum(r * r for r in resid)
    dof = n - 2
    s2 = rss / dof if dof > 0 else 0.0
    se_K = math.sqrt(s2 / sxx)
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - rss / ss_tot if ss_tot > 0 else 0.0
    return dict(K=K, b=b, se_K=se_K, r2=r2, rss=rss, n=n,
                rel_se=(se_K / abs(K) * 100.0 if K else float("inf")))


def connect(iface):
    from piper_sdk import C_PiperInterface
    import piper_sdk
    cls = (getattr(piper_sdk, "C_PiperInterface_V3", None)
           or getattr(piper_sdk, "C_PiperInterface_V2", None)
           or C_PiperInterface)
    p = cls(iface)
    # 0.6.1 的 ConnectPort() 方法体没有 return，返回 None；判据必须是 `is False`
    if p.ConnectPort() is False:
        raise SystemExit(f"[ABORT] 连不上 {iface}：can0 没起来？")
    time.sleep(1.0)
    return p


def read_q_deg(p):
    """关节角，度。joint_x 的单位已经是 0.001 度（不是弧度，别再乘 57.29）。"""
    js = p.GetArmJointMsgs().joint_state
    return [getattr(js, f"joint_{i}") * 0.001 for i in range(1, 7)]


def read_cur(p):
    """六轴电流 A，走高速帧 0x251 的 current 字段。"""
    ms = p.GetArmHighSpdInfoMsgs()
    return [getattr(ms, f"motor_{i}").current * 0.001 for i in range(1, 7)]


def read_eff(p):
    ms = p.GetArmHighSpdInfoMsgs()
    return [getattr(ms, f"motor_{i}").effort * 0.001 for i in range(1, 7)]


def read_enable(p):
    en = p.GetArmEnableStatus()
    return en if isinstance(en, (list, tuple)) else [en] * 6


def read_ctrl_mode(p):
    """ctrl_mode 码。0.6.1 的打印形式不一定是 int，取不到就返回 None（不阻断，只警告）。"""
    try:
        st = p.GetArmStatus()
        a = getattr(st, "arm_status", st)
        v = getattr(a, "ctrl_mode", None)
        return int(v) if isinstance(v, (int, float)) else None
    except Exception:                                     # noqa: BLE001
        return None


class Hold:
    """固定目标关节保持。自己建 socket 是为了逐帧判定发送成败
    （SDK 的 JointCtrl 返回值恒为 None，发失败只有 logger 里一行）。"""

    def __init__(self, iface):
        import can
        self._can = can
        self.bus = can.Bus(interface="socketcan", channel=iface)
        self.n_ok = 0
        self.n_fail = 0

    def send(self, q_deg):
        vals = [int(round(v * 1000.0)) for v in q_deg]
        ok = True
        for i, cid in enumerate(JOINT_IDS):
            data = (vals[2 * i].to_bytes(4, "big", signed=True)
                    + vals[2 * i + 1].to_bytes(4, "big", signed=True))
            try:
                self.bus.send(self._can.Message(arbitration_id=cid, data=data,
                                                is_extended_id=False), timeout=0.005)
                self.n_ok += 1
            except Exception:                             # noqa: BLE001
                ok = False
                self.n_fail += 1
        return ok

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:                                 # noqa: BLE001
            pass


# ---------------------------------------------------------------- probe

def cmd_sample(args):
    """单点采样，结果追加到 CSV。**给远程/分步操作设计**。

    为什么要它：`run` 是交互式的（每点都要 input），而通过 SSH 远程驱动时没法
    逐点回话 —— 挂瓶子、换水本来就得人动手。拆成「一次一个点」后，
    操作节奏由人决定，采样只负责把这一次的量准准地记下来。

    CSV 是**追加**的：同一个 --tag 下连跑 5 次就凑成完整的 5 点。
    """
    if args.ml is None and args.mass is None:
        raise SystemExit("[ABORT] 给 --ml（水量）或 --mass（质量 kg）之一")
    if args.ml is not None and args.mass is not None:
        raise SystemExit("[ABORT] --ml 和 --mass 只给一个")
    m = (args.ml * WATER_DENSITY / 1000.0) if args.ml is not None else args.mass
    if m <= 0:
        raise SystemExit("[ABORT] 质量必须为正")
    tau = m * G * args.arm

    p = connect(args.iface)
    en = read_enable(p)
    if not all(en):
        raise SystemExit(f"[ABORT] 有轴未使能 {list(en)}")
    mode = read_ctrl_mode(p)
    if mode is not None and mode != CTRL_CAN:
        raise SystemExit(f"[ABORT] ctrl_mode={mode} 不是 CAN_CTRL；先跑 weigh_init.py --no-app")

    q0 = read_q_deg(p)
    print(f"[sample] q0 = [" + ", ".join(f"{v:+.2f}" for v in q0) + "]")
    src = f"{args.ml:g} ml 水" if args.ml is not None else f"{args.mass:g} kg"
    print(f"[sample] {src} → m={m:.4f} kg，L={args.arm*1000:.1f} mm → τ={tau:.4f} N·m")
    if tau < 0.111:
        print(f"[WARN] τ 低于噪声地板 0.111（SNR {tau/0.111:.2f}），这点会拉低拟合质量")

    hold = Hold(args.iface)
    aborted = None
    try:
        p.MotionCtrl_2(0x01, 0x01, int(args.speed_pct), 0x00)
        hold.send(q0)
        time.sleep(SETTLE_S)
        waited = wait_stable(p, hold, q0, q0[5])
        print("  已稳定（额外等 %.1f s）" % waited if waited >= 0
              else f"  [WARN] {STABLE_MAX_WAIT:.0f}s 内没等到稳定，仍继续", flush=True)
        recs = sample(p, hold, q0, args.sec, q0[5])
        s = summarize(recs)
        print(f"  I6 = {s['i6_mean']:+.4f} ± {s['i6_std']:.4f} A   effort6={s['eff6_mean']:+.4f}"
              f"   漂移 {s['drift_max']:.2f}°   六轴峰值 {s['cur_max']:.2f} A")

        DATA.mkdir(exist_ok=True)
        tag = args.tag or time.strftime("%Y%m%d")
        out = DATA / f"k6_{tag}.csv"
        row = dict(tag=tag, mass_kg=m, arm_m=args.arm, tau_nm=tau,
                   i6_mean=s["i6_mean"], i6_std=s["i6_std"], q6_mean=s["q6_mean"],
                   drift_max=s["drift_max"], eff6_mean=s["eff6_mean"], n=s["n"],
                   q_lock=";".join(f"{v:.3f}" for v in q0))
        new = not out.exists()
        with out.open("a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if new:
                w.writeheader()
            w.writerow(row)
        n_row = sum(1 for _ in out.open(encoding="utf-8")) - 1
        print(f"  已{'新建' if new else '追加'} {out}（现 {n_row} 点）")
    except RuntimeError as exc:
        aborted = exc
        print(f"\n[ABORT] {exc}", file=sys.stderr)
        print("[ABORT] **立刻把配重取下来**，臂仍在原位保持但空闲约 10 min 会回落 TEACHING。",
              file=sys.stderr)
    finally:
        print(f"[sample] 发送统计：ok={hold.n_ok} fail={hold.n_fail}")
        hold.close()
    return 3 if aborted else 0


def _drive_q6(p, hold, q_base, a, b, rate):
    """只转 J6，从 a 到 b，返回匀速段的 I6 均值。"""
    travel = b - a
    if abs(travel) < 0.5:
        return None
    dur = abs(travel) / rate
    n = max(2, int(dur / DT_SEND))
    vals = []
    for k in range(n + 1):
        f = k / n
        qc = list(q_base)
        qc[5] = a + travel * f
        hold.send(qc)
        cur = read_cur(p)
        vals.append((f, cur[5]))
        worst = max(range(6), key=lambda j: abs(cur[j]))
        if abs(cur[worst]) > MAX_CUR_A:
            raise RuntimeError(
                f"J{worst+1} 电流 {cur[worst]:+.2f} A 超过 {MAX_CUR_A} A，已停发。"
                f" 转动时摆动会加剧，减配重或减 --amp。")
        if k % 10 == 0:
            lead = abs(qc[5] - read_q_deg(p)[5])
            if lead > 3.0:
                raise RuntimeError(f"J6 lead {lead:.2f}° > 3°，跟不上。")
        time.sleep(DT_SEND)
    mid = [v for f, v in vals if 0.25 <= f <= 0.75]
    return statistics.mean(mid) if mid else None


def cmd_sweep(args):
    """正反转动差分测 K6 —— 静态法被静摩擦吃掉时用这个。

    为什么必须改成动态
    ------------------
    静态保持时 J6 只需「顶住」，静摩擦就能吃掉零点几 N·m 的负载：
    实测 100 ml → 400 ml 力矩涨 4 倍，而 I6 从 +0.054 掉到 +0.019 A
    （变化量小于标准差本身）。力矩根本没传到电机上。

    转动时摩擦变成动摩擦，而且**方向随运动反向**：

        正转  I_f = (τ_f + τ_load) / K6
        反转  I_r = (τ_f − τ_load) / K6
        ΔI = I_f − I_r = 2·τ_load / K6   ⇒   K6 = 2·τ_load / ΔI

    τ_f 正负相反、相减即消 —— 和阶段 2「空载−带载差分」是同一套思路的另一种用法。
    好处：**不依赖 τ 有多大**，小配重也能出信号；坏处：多花 ~15 s 转动。

    两个几何注记
    ------------
    * J6 轴水平时绕它转 = 配重上下摆（钟摆），τ_load 在 q0±amp 内按 cos 变化，
      amp=15° 时约 3.4%。这里取正反两段的**对称中段**统计，一阶抵消。
    * 转动会让配重摆动，J5 电流会升高。amp 和 rate 别贪大。

    用法：
        python3 s6_k6_calib.py sweep --ml 100 --arm 0.185
        python3 s6_k6_calib.py sweep --mass 0 --arm 0.185   # 空载自检，ΔI 应 ≈ 0
    """
    if args.ml is None and args.mass is None:
        raise SystemExit("[ABORT] 给 --ml 或 --mass（空载自检用 --mass 0）")
    m = (args.ml * WATER_DENSITY / 1000.0) if args.ml is not None else args.mass
    tau = m * G * args.arm
    if args.amp > 30:
        raise SystemExit("[ABORT] --amp 上限 30°（摆动会失控）")
    if args.rate > 18:
        raise SystemExit("[ABORT] --rate 上限 6°/s")

    p = connect(args.iface)
    en = read_enable(p)
    if not all(en):
        raise SystemExit(f"[ABORT] 有轴未使能 {list(en)}")
    mode = read_ctrl_mode(p)
    if mode is not None and mode != CTRL_CAN:
        raise SystemExit(f"[ABORT] ctrl_mode={mode} 不是 CAN_CTRL；先跑 weigh_init.py --no-app")

    q0 = read_q_deg(p)
    c = q0[5]
    lo, hi = c - args.amp, c + args.amp
    if lo < 2.0 or hi > 340.0:
        raise SystemExit(f"[ABORT] q6={c:.1f}±{args.amp:g} 越出 0~343 机械行程")
    print(f"[sweep] q6 = {c:.2f}°，摆幅 ±{args.amp:g}° → [{lo:.1f}, {hi:.1f}]，{args.rate:g}°/s")
    if abs(c - J6_LEVEL) > 5.0:
        print(f"[WARN] q6 偏离水平位 {J6_LEVEL:g}° 有 {abs(c-J6_LEVEL):.1f}°，"
              f"力矩投影要乘 cos={math.cos(math.radians(c-J6_LEVEL)):.4f}。"
              f"先 goto 到 {J6_LEVEL:g}° 再测更干净。")
    print(f"[sweep] m={m:.4f} kg，L={args.arm*1000:.1f} mm → τ_load={tau:.4f} N·m"
          + ("（空载自检，ΔI 应≈0）" if m == 0 else ""))

    hold = Hold(args.iface)
    aborted = None
    try:
        p.MotionCtrl_2(0x01, 0x01, int(args.speed_pct), 0x00)
        hold.send(q0)
        time.sleep(SETTLE_S)
        wait_stable(p, hold, q0, q0[5])

        # 正反两段必须扫**同一个角度区间**的相反方向，否则两段的平均倾斜角不同，
        # 重力矩的 cos 分量差就跑到 ΔI 里去了（空载自检第一次就是这么偏了 0.155 A）。
        # 所以先空跑到 lo（不记录），再 lo→hi 与 hi→lo。
        print("  预处理 %g→%g（不记录）…" % (c, lo), flush=True)
        _drive_q6(p, hold, q0, c, lo, args.rate)
        time.sleep(0.8)
        print("  正向 %g→%g …" % (lo, hi), flush=True)
        i_f = _drive_q6(p, hold, q0, lo, hi, args.rate)
        time.sleep(0.8)
        print("  反向 %g→%g …" % (hi, lo), flush=True)
        i_r = _drive_q6(p, hold, q0, hi, lo, args.rate)
        time.sleep(0.5)
        print("  回中 %g→%g …" % (lo, c), flush=True)
        _drive_q6(p, hold, q0, lo, c, args.rate)

        if i_f is None or i_r is None:
            raise SystemExit("[ABORT] 采样段无效")
        d = i_f - i_r
        print(f"  I6 正向 = {i_f:+.4f} A")
        print(f"  I6 反向 = {i_r:+.4f} A")
        print(f"  ΔI      = {d:+.4f} A")
        if m == 0:
            print("  [空载锚点] ΔI = 2·τ_摩擦/K6，**本来就不为零**，这是摩擦的度量，正常。")
            print("             带载后 ΔI 会从这条基线继续长，长出来的部分才是配重的贡献。")
            print("             多测几个质量点，拟合 ΔI = (2/K6)·τ + const，斜率才是答案。")
        else:
            if abs(d) < 1e-4:
                print("  [FAIL] ΔI 太小，无法定 K6")
                return 1
            print(f"  单点参考 2τ/|ΔI| = {2*tau/abs(d):.4f} N·m/A"
                  f"（**偏高**：分母里还含着摩擦，必须扣掉空载基线）")
        DATA.mkdir(exist_ok=True)
        tag = args.tag or time.strftime("%Y%m%d")
        out = DATA / f"k6sweep_{tag}.csv"
        row = dict(tag=tag, mass_kg=m, arm_m=args.arm, tau_nm=tau,
                   i_fwd=i_f, i_rev=i_r, di=d, q6_center=c, amp=args.amp,
                   rate=args.rate, q_lock=";".join(f"{v:.3f}" for v in q0))
        new = not out.exists()
        with out.open("a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if new:
                w.writeheader()
            w.writerow(row)
        n_row = sum(1 for _ in out.open(encoding="utf-8")) - 1
        print(f"  已{'新建' if new else '追加'} {out}（现 {n_row} 点）")
    except RuntimeError as exc:
        aborted = exc
        print(f"\n[ABORT] {exc}", file=sys.stderr)
    finally:
        print(f"[sweep] 发送统计：ok={hold.n_ok} fail={hold.n_fail}")
        hold.close()
    return 3 if aborted else 0


def cmd_sweepfit(args):
    """把多个 ΔI-τ 点拟合成 K6。

    ΔI = 2(τ_摩擦 + τ_load)/K6 = (2/K6)·τ_load + const
    ⇒ 斜率 = 2/K6 ⇒ **K6 = 2 / 斜率**。摩擦进 const，正是它该待的地方。
    """
    path = Path(args.csv) if args.csv else None
    if path is None:
        cands = sorted(DATA.glob("k6sweep_*.csv"))
        if not cands:
            raise SystemExit(f"[ABORT] {DATA} 下没有 k6sweep_*.csv")
        path = cands[-1]
    rows = []
    with path.open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(dict(mass_kg=float(r["mass_kg"]), arm_m=float(r["arm_m"]),
                             tau_nm=float(r["tau_nm"]), di=float(r["di"])))
    rows.sort(key=lambda r: r["tau_nm"])
    print(f"[sweepfit] 读 {path}（{len(rows)} 点）")
    print("| τ (N·m) | ΔI (A) |")
    print("| --- | --- |")
    for r in rows:
        print(f"| {r['tau_nm']:+.4f} | {r['di']:+.4f} |")
    if len(rows) < 3:
        raise SystemExit("[ABORT] 至少 3 点（含空载锚点）")
    xs = [r["tau_nm"] for r in rows]
    ys = [r["di"] for r in rows]
    f = fit_line(xs, ys)
    if f is None or abs(f["K"]) < 1e-9:
        raise SystemExit("[ABORT] 拟合失败（ΔI 对 τ 不敏感，可能杆在夹爪里打滑了）")
    K6 = 2.0 / f["K"]
    se_K6 = 2.0 * f["se_K"] / (f["K"] ** 2)
    print()
    print("## 拟合  ΔI = (2/K6)·τ + const")
    print(f"  斜率   = {f['K']:+.4f} A/(N·m)   ± {f['se_K']:.4f}")
    print(f"  **K6   = {K6:+.4f} N·m/A**   ± {se_K6:.4f}  (σ/K = {abs(se_K6/K6)*100:.1f}%)")
    print(f"  const  = {f['b']:+.4f} A   ← = 2τ_摩擦/K6，与配重无关")
    print(f"  R²     = {f['r2']:.4f}")
    print(f"  SDK 分组(J4–J6)=0.95844 ⇒ 本次/厂商 = {K6/0.95844:.2f}×")
    print()
    ok = abs(se_K6 / K6) * 100 < 5.0 and f["r2"] > 0.98
    print(f"  判据 σ/K<5% 且 R²>0.98 → {'通过 ✓' if ok else '未通过 ✗（补点或加大量程）'}")
    DATA.mkdir(exist_ok=True)
    out = DATA / "k6_result.json"
    out.write_text(json.dumps(dict(
        version="sweepfit 1.0", source_csv=str(path), method="正反转动差分 ΔI=(2/K6)τ+const",
        K6=K6, se_K6=se_K6, rel_se_pct=abs(se_K6 / K6) * 100, const=f["b"],
        r2=f["r2"], n=f["n"], passed=ok,
        taus=xs, dis=ys,
        note="const = 2τ_摩擦/K6，含钢尺/瓶/绳；换配重不影响 const"), ensure_ascii=False, indent=1),
        encoding="utf-8")
    print(f"  结果已写 {out}")
    return 0


def cmd_twist(args):
    """拧瓶盖：J6 从当前角连续转到目标角，200 Hz 记录全过程。

    为什么要专门做：拧盖是连续接触过程，判据全在**电流的时间曲线上**
    （阻力上升 → 突降 = 盖脱开）。用 goto 只给残差，看不到过程。
    夹爪指令必须**每步重发**：实测夹爪不响应单帧指令，不重发会在过程中松掉。
    """
    p = connect(args.iface)
    en = read_enable(p)
    if not all(en):
        raise SystemExit(f"[ABORT] 有轴未使能 {list(en)}")
    mode = read_ctrl_mode(p)
    if mode is not None and mode != CTRL_CAN:
        raise SystemExit(f"[ABORT] ctrl_mode={mode} 不是 CAN_CTRL；先跑 weigh_init.py --no-app")

    q0 = read_q_deg(p)
    g = p.GetArmGripperMsgs().gripper_state
    a0, e0 = int(g.grippers_angle), int(g.grippers_effort)
    if a0 < 1000:
        raise SystemExit(f"[ABORT] 夹爪开度只有 {a0/1000:.1f} mm，先夹住瓶盖")
    target = float(args.target)
    if abs(target - q0[5]) < 0.5:
        raise SystemExit("[ABORT] 目标与当前 J6 相同")
    print(f"[twist] J6 {q0[5]:.2f}° → {target:.2f}°  @ {args.rate:g}°/s")
    print(f"[twist] 夹爪 {a0/1000:.1f} mm / effort {e0}，每步重发（per {args.reissue} 拍）")

    hold = Hold(args.iface)
    aborted = None
    recs = []
    try:
        p.MotionCtrl_2(0x01, 0x01, int(args.speed_pct), 0x00)
        hold.send(q0)
        time.sleep(0.4)
        span = target - q0[5]
        n = max(2, int(round(abs(span) / args.rate / DT_SEND)))
        t0 = time.time()
        for k in range(n + 1):
            f = k / n
            qc = list(q0)
            qc[5] = q0[5] + span * f
            hold.send(qc)
            if k % args.reissue == 0:
                # 关键：每帧都发**满力**，而不是复用读到的 effort。
                # 复用会导致"目标追着实际跑"——瓶盖把爪顶开后，目标也跟着变大，
                # 夹持力建立不起来（实测 effort 从 −509 掉到 −128，传递扭矩减半）。
                try:
                    p.GripperCtrl(max(0, a0 - args.grip_squeeze), args.grip_effort, 0x01, 0)
                except Exception:                             # noqa: BLE001
                    pass
            cur = read_cur(p)
            qm = read_q_deg(p)
            gs = p.GetArmGripperMsgs().gripper_state
            recs.append((time.time() - t0, qc[5], qm[5], cur[5], cur,
                         int(gs.grippers_angle), int(gs.grippers_effort)))
            worst = max(range(6), key=lambda j: abs(cur[j]))
            if abs(cur[worst]) > MAX_CUR_A:
                raise RuntimeError(f"J{worst+1} 电流 {cur[worst]:+.2f} A 超限，已停发。")
            lead = abs(qc[5] - qm[5])
            if lead > 5.0:
                raise RuntimeError(f"J6 lead {lead:.2f}° > 5°，转不动了（阻矩超过出力）")
            if k % 50 == 0:
                print(f"  {f*100:5.1f}%  cmd={qc[5]:+7.2f}  meas={qm[5]:+7.2f}  "
                      f"I6={cur[5]:+7.4f}  grip={int(gs.grippers_angle)/1000:.1f}mm", flush=True)
            time.sleep(DT_SEND)
        for _ in range(50):
            hold.send([q0[0], q0[1], q0[2], q0[3], q0[4], target])
            p.GripperCtrl(max(0, a0 - args.grip_squeeze), args.grip_effort, 0x01, 0)
            time.sleep(DT_SEND)

        DATA.mkdir(exist_ok=True)
        tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
        out = DATA / f"twist_{tag}.csv"
        with out.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "cmd_q6", "meas_q6", "i6", "i1", "i2", "i3", "i4", "i5",
                        "i6_dup", "grip_angle", "grip_effort"])
            for r in recs:
                w.writerow([f"{r[0]:.4f}", f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.6f}"]
                           + [f"{v:.6f}" for v in r[4]]
                           + [r[5], r[6]])
        qm = read_q_deg(p)
        print(f"[twist] 实测 J6 = {qm[5]:+.2f}°（残差 {qm[5]-target:+.2f}°）")
        print(f"[twist] {len(recs)} 帧已写 {out}")
        # 电流趋势：脱盖时 I6 会突降
        if len(recs) > 40:
            head = [abs(r[3]) for r in recs[:len(recs)//5]]
            tail = [abs(r[3]) for r in recs[-len(recs)//5:]]
            print(f"[twist] |I6| 前 1/5 = {sum(head)/len(head):.4f} A，"
                  f"后 1/5 = {sum(tail)/len(tail):.4f} A，"
                  f"变化 {(sum(tail)/len(tail) - sum(head)/len(head)):+.4f} A")
    except RuntimeError as exc:
        aborted = exc
        print(f"\n[ABORT] {exc}", file=sys.stderr)
    finally:
        print(f"[twist] 发送统计：ok={hold.n_ok} fail={hold.n_fail}")
        hold.close()
    return 3 if aborted else 0


def cmd_goto(args):
    """线性插值到目标关节角（**不查限位表**，但速率/电流/lead 三道守卫全在）。

    为什么不直接用 `stage1_cmd.py pose`
    ------------------------------------
    stage1_cmd 的 `safe_envelope()` 口径是「固件 ∩ URDF」，而 URDF 把 J6 写成 ±120，
    固件回读是 ±180，机械真实行程是 **0 ~ +343（手转实测，0 是硬端点、负向转不动）**。
    三者不一致 ⇒ J6 任何 > 123° 的目标都会被挡下来，包括本任务需要的 150°。
    （`--limits` 也救不了：`safe_envelope` 仍要和 URDF 取交集。）

    所以这里不查限位，但**安全边界一点不松**：
      * 速率硬锁 6°/s（可用 --rate 调小，不能调大）
      * 任一轴电流 > 3 A → 立即停发并报警
      * 指令与实测的 lead > 3° → 判定跟不上/卡住，停发
    这只是给 J6 这一个已知错误的限位开个口子，别拿去跑别的大行程目标。

    用法：
        python3 s6_k6_calib.py goto --target-deg 0,70,-50,0,-15,150 --yes
    """
    target = [float(v) for v in args.target_deg.split(",")]
    if len(target) != 6:
        raise SystemExit("[ABORT] --target-deg 需要 6 个数")
    if args.rate > 15.0:
        raise SystemExit("[ABORT] --rate 不允许超过 6°/s（这是本项目的速率铁律）")

    p = connect(args.iface)
    en = read_enable(p)
    if not all(en):
        raise SystemExit(f"[ABORT] 有轴未使能 {list(en)}，先使能六轴。")

    q0 = read_q_deg(p)
    travel = [t - c for t, c in zip(target, q0)]
    tmax = max(abs(v) for v in travel)
    dur = tmax / args.rate
    print("[goto] 起点 = [" + ", ".join(f"{v:+8.2f}" for v in q0) + "]")
    print("[goto] 目标 = [" + ", ".join(f"{v:+8.2f}" for v in target) + "]")
    print("[goto] 行程 = [" + ", ".join(f"{v:+8.2f}" for v in travel) + "]")
    print(f"[goto] 最大 {tmax:.1f}° @ {args.rate:g}°/s → {dur:.1f} s")
    if tmax < 0.05:
        print("[goto] 已经在目标位，不动作。")
        return 0
    if not args.yes:
        raise SystemExit("[ABORT] 加 --yes 确认（本命令不查限位表）")

    hold = Hold(args.iface)
    aborted = None
    try:
        p.MotionCtrl_2(0x01, 0x01, int(args.speed_pct), 0x00)
        hold.send(q0)
        time.sleep(0.3)
        n = max(1, int(round(dur / DT_SEND)))
        for k in range(n + 1):
            f = k / n
            q = [c + d * f for c, d in zip(q0, travel)]
            hold.send(q)
            cur = read_cur(p)
            worst = max(range(6), key=lambda i: abs(cur[i]))
            if abs(cur[worst]) > MAX_CUR_A:
                raise RuntimeError(f"J{worst+1} 电流 {cur[worst]:+.2f} A 超限，已停发。")
            if k % 10 == 0:
                lead = max(abs(a - b) for a, b in zip(q, read_q_deg(p)))
                if lead > 3.0:
                    raise RuntimeError(f"指令-实测 lead {lead:.2f}° > 3°，跟不上或卡住，已停发。")
            if k % 25 == 0:
                print(f"  {f*100:5.1f}%  " + " ".join(f"{v:+8.2f}" for v in q), flush=True)
            time.sleep(DT_SEND)
        for _ in range(100):
            hold.send(target)
            time.sleep(DT_SEND)
        qm = read_q_deg(p)
        res = [a - b for a, b in zip(qm, target)]
        print("[goto] 实测 = [" + ", ".join(f"{v:+8.2f}" for v in qm) + "]")
        print("[goto] 残差 = [" + ", ".join(f"{v:+.2f}" for v in res) + "]")
    except RuntimeError as exc:
        aborted = exc
        print(f"\n[ABORT] {exc}", file=sys.stderr)
    finally:
        print(f"[goto] 发送统计：ok={hold.n_ok} fail={hold.n_fail}")
        hold.close()
    return 3 if aborted else 0


def cmd_probe(args):
    p = connect(args.iface)
    q = read_q_deg(p)
    en = read_enable(p)
    cur = read_cur(p)
    eff = read_eff(p)
    mode = read_ctrl_mode(p)

    print(f"ctrl_mode   : {mode if mode is not None else '读不到'} "
          f"({'CAN_CTRL ✓' if mode == CTRL_CAN else '不是 CAN_CTRL(1)，先跑 --to-can-ctrl'})")
    print(f"使能        : {all(en)}  逐轴={list(en)}")
    print(f"q (deg)     : [" + ", ".join(f"{v:+8.2f}" for v in q) + "]")
    print(f"电流 (A)    : [" + ", ".join(f"{v:+8.3f}" for v in cur) + "]")
    print(f"effort(N·m) : [" + ", ".join(f"{v:+8.3f}" for v in eff) + "]")
    print()

    if not all(en):
        print("[WARN] 有轴未使能。位置保持无效，砝码会把臂拽下来 —— 先使能六轴。")

    arm = load_arm()
    u, p6, ax, ay = axis_report(arm, q)
    t_u, t_x, t_y = tilt_deg(u), tilt_deg(ax), tilt_deg(ay)

    print("## J6 轴姿态（URDF 口径，joint6 axis = [0,0,1] ⇒ 法兰 z 轴就是 J6 轴）")
    print(f"  J6 轴向量 u = [{u[0]:+.3f}, {u[1]:+.3f}, {u[2]:+.3f}]")
    print(f"  J6 轴倾角        = {t_u:5.1f}°   {'✓ 水平' if t_u < 10 else '✗ 太竖，力矩会被吃掉'}")
    print(f"  link6 x 轴倾角   = {t_x:5.1f}°   ← 若横杆沿 x 方向，这行要接近 0")
    print(f"  link6 y 轴倾角   = {t_y:5.1f}°   ← 若横杆沿 y 方向，这行要接近 0")
    print(f"  link6 原点 (mm)  = [{p6[0]*1000:+.1f}, {p6[1]*1000:+.1f}, {p6[2]*1000:+.1f}]")
    print()

    ok = True
    if t_u >= 10:
        ok = False
        print("[✗] J6 轴不够水平。轴竖直时 r×F ≈ 0，挂多少砝码都是零力矩。")
        print("    调整 J5（腕俯仰）把它压平，目标 |u_z| < 0.17。")
    if min(t_x, t_y) > 10:
        ok = False
        print("[✗] link6 的两个横轴都不水平 ⇒ 横杆必然是斜的，力臂要打折 cos(倾角)。")
        print("    转 J6 找横杆水平的位置，再跑一次 probe。")
    elif min(t_x, t_y) > 3:
        print(f"[!] 横杆倾角 {min(t_x, t_y):.1f}°，力臂衰减 cos = "
              f"{math.cos(math.radians(min(t_x, t_y))):.4f}（{100*(1-math.cos(math.radians(min(t_x, t_y)))):.1f}% 误差）")

    if ok:
        print("[✓] 姿态可用。横杆水平那一侧就是力臂方向，砝码挂在杆上、自然垂下来。")
        print(f"    记得先用 pose_guard 查净空：python3 ../20261019_stage2/"
              f"stage2_pose_guard.py --start-from-can --target-deg "
              + ",".join(f"{v:.1f}" for v in q))
    return 0 if ok else 2


# ---------------------------------------------------------------- 采样

def wait_stable(p, hold, q_lock, q6_0):
    """等 I6 稳定下来再正式采样。

    为什么需要：挂上去的是**装水的瓶子**，水会在瓶里晃，周期约 1 s、衰减要好几秒。
    固定 sleep 2 s 会直接采到晃动段 —— 表现为 i6_std 很大、拟合残差飙升，
    而且不会报错，只会悄悄变差。这里用 0.5 s 窗口的电流标准差当判据。

    返回实际等待秒数；-1 表示超时（仍会继续，但报告里会标注）。
    """
    n_win = max(8, int(STABLE_WIN / DT_READ))
    buf = []
    t0 = time.time()
    next_send = 0.0
    while time.time() - t0 < STABLE_MAX_WAIT:
        now = time.time()
        if now >= next_send:
            hold.send(q_lock)
            next_send = now + DT_SEND
        buf.append(read_cur(p)[5])
        if len(buf) > n_win:
            buf.pop(0)
        if len(buf) >= n_win and statistics.pstdev(buf) < STABLE_STD:
            return now - t0
        if abs(read_q_deg(p)[5] - q6_0) > MAX_DRIFT_DEG:
            raise RuntimeError(f"等待稳定时 J6 已漂移超过 {MAX_DRIFT_DEG}°，位置环顶不住。")
        time.sleep(DT_READ)
    return -1


def sample(p, hold, q_lock, sec, q6_0):
    """只读采样 + 50 Hz 保持帧。返回记录列表；越界抛 RuntimeError。"""
    t0 = time.time()
    recs = []
    next_send = 0.0
    while True:
        now = time.time()
        if now - t0 >= sec:
            break
        if now >= next_send:
            hold.send(q_lock)
            next_send = now + DT_SEND
        q = read_q_deg(p)
        cur = read_cur(p)
        eff = read_eff(p)
        drift = abs(q[5] - q6_0)
        recs.append((now - t0, q, cur, eff, drift))
        worst = max(range(6), key=lambda j: abs(cur[j]))
        if abs(cur[worst]) > MAX_CUR_A:
            raise RuntimeError(
                f"J{worst+1} 电流 {cur[worst]:+.2f} A 超过 {MAX_CUR_A} A 上限，已停止发帧。"
                f" 先卸配重，再减小力臂或减量。")
        if drift > MAX_DRIFT_DEG:
            raise RuntimeError(
                f"J6 被拉偏 {drift:.2f}° (>{MAX_DRIFT_DEG}°)，位置环顶不住这个力臂，"
                f"该点作废。卸砝码、减小力臂或减重后重来。")
        time.sleep(DT_READ)
    return recs


def summarize(recs):
    i6 = [r[2][5] for r in recs]
    q6 = [r[1][5] for r in recs]
    eff6 = [r[3][5] for r in recs]
    drift = [r[4] for r in recs]
    return dict(
        n=len(recs),
        i6_mean=statistics.mean(i6),
        i6_std=statistics.pstdev(i6),
        q6_mean=statistics.mean(q6),
        eff6_mean=statistics.mean(eff6),
        drift_max=max(drift),
        cur_max=max(abs(r[2][i]) for r in recs for i in range(6)),
    )


def cmd_run(args):
    masses = [float(v) for v in args.masses.split(",") if v.strip()] if args.masses else []
    if masses and len(masses) < 3:
        raise SystemExit("[ABORT] --masses 至少给 3 个点；或者留空，改成每点现场称重输入")
    if args.arm <= 0:
        raise SystemExit("[ABORT] --arm 必须是正数（米）")

    p = connect(args.iface)
    en = read_enable(p)
    if not all(en):
        raise SystemExit(f"[ABORT] 有轴未使能 {list(en)}：位置保持无效，先使能六轴。")
    mode = read_ctrl_mode(p)
    if mode is not None and mode != CTRL_CAN:
        raise SystemExit(
            f"[ABORT] ctrl_mode={mode}，不是 CAN_CTRL(1)。\n"
            f"  先跑：python3 ../20261019_stage1_l0/stage1_cmd.py pose "
            f"--phase sdk --allow-write --to-can-ctrl")

    q0 = read_q_deg(p)
    u, _, ax, ay = axis_report(load_arm(), q0)
    t_u = tilt_deg(u)
    print(f"[run] 锁定姿态 q = [" + ", ".join(f"{v:+.2f}" for v in q0) + "]")
    print(f"[run] J6 轴倾角 {t_u:.1f}°" + ("  ✓" if t_u < 10 else "  [WARN] 偏竖，力矩会打折"))
    print(f"[run] 力臂 L = {args.arm*1000:.1f} mm")
    if masses:
        print(f"[run] 质量点 = {masses} kg → 预测力矩 "
              + ", ".join(f"{m*G*args.arm:.3f}" for m in masses) + " N·m")
    else:
        print(f"[run] 质量点 = 现场输入，共 {args.points} 点（回车沿用上一点，q 结束）")
        if args.by_volume:
            print(f"[run] 量水模式：水密度 {WATER_DENSITY} kg/L @20℃。"
                  f"建议 200/300/400/500/600 ml → 0.20~0.60 kg")
            print(f"[run] **只填水的体积，空瓶和绳子的重量不用管** —— 它们是常数，"
                  f"会全被截距 b 吸收，不影响斜率 K6。")
            print(f"[run] 同一个瓶子反复加水即可，不必准备多个配重。")
        else:
            print(f"[run] 没有标准砝码没关系：装水的瓶子 / 米袋 / 哑铃 / 书本都行，"
                  f"用厨房秤称一次即可。")
    print()

    hold = Hold(args.iface)
    aborted = None
    try:
        p.MotionCtrl_2(0x01, 0x01, int(args.speed_pct), 0x00)   # 0x151：CAN_CTRL + MOVE_J
        hold.send(q0)
        time.sleep(0.5)

        DATA.mkdir(exist_ok=True)
        tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
        out = DATA / f"k6_{tag}.csv"
        rows = []
        plan = masses if masses else [None] * args.points
        for idx, m_fixed in enumerate(plan, 1):
            print("=" * 62)
            if m_fixed is None:
                unit = ("水量 ml（1 ml 水 ≈ 0.998 g，瓶重不用填）" if args.by_volume
                        else "质量 kg（厨房秤读数）")
                raw = input(f"  第 {idx}/{len(plan)} 点：挂上重物，输入{unit}"
                            f"（回车=沿用上一点，q=结束）：").strip()
                if raw.lower() in ("q", "quit"):
                    break
                if raw == "":
                    if not rows:
                        print("  [skip] 第一点必须给量")
                        continue
                    m = rows[-1]["mass_kg"]
                    print(f"  [沿用] m = {m:.3f} kg")
                else:
                    try:
                        val = float(raw)
                    except ValueError:
                        print("  [skip] 不是数字")
                        continue
                    if val <= 0:
                        print("  [skip] 必须为正")
                        continue
                    if args.by_volume:
                        m = val * WATER_DENSITY / 1000.0
                        print(f"  [换算] {val:g} ml 水 → m = {m:.4f} kg"
                              f"（只算水，空瓶和绳子的重量由 b 吸收）")
                    else:
                        m = val
            else:
                m = m_fixed
                input(f"  请挂 {m:.3f} kg（理论 τ = {m*G*args.arm:.4f} N·m），挂稳后回车：")
            tau = m * G * args.arm
            print(f"  目标 τ = {tau:.4f} N·m（m={m:.3f} kg × L={args.arm*1000:.0f} mm）")
            if tau < 0.111:
                print(f"  [WARN] τ 低于噪声地板 0.111 N·m（信噪比 {tau/0.111:.2f}），"
                      f"这一点基本是噪声，建议换更重的水量，或事后用 fit --drop 剔掉。")
            print(f"  先等 {SETTLE_S:.1f} s，再看电流是否稳定 …", flush=True)
            time.sleep(SETTLE_S)
            waited = wait_stable(p, hold, q0, q0[5])
            if waited < 0:
                print(f"  [WARN] {STABLE_MAX_WAIT:.0f} s 内没等到稳定（水还在晃？），"
                      f"仍继续采样，但该点可信度下降", flush=True)
            else:
                print(f"  已稳定（额外等 {waited:.1f} s）", flush=True)
            print(f"  采样 {args.sec:.1f} s …", flush=True)
            recs = sample(p, hold, q0, args.sec, q0[5])
            s = summarize(recs)
            print(f"  I6 = {s['i6_mean']:+.4f} ± {s['i6_std']:.4f} A  "
                  f"(n={s['n']}, effort6={s['eff6_mean']:+.4f} N·m, "
                  f"漂移 {s['drift_max']:.2f}°, 六轴峰值 {s['cur_max']:.2f} A)")
            rows.append(dict(tag=tag, mass_kg=m, arm_m=args.arm, tau_nm=tau,
                             i6_mean=s["i6_mean"], i6_std=s["i6_std"],
                             q6_mean=s["q6_mean"], drift_max=s["drift_max"],
                             eff6_mean=s["eff6_mean"], n=s["n"],
                             q_lock=";".join(f"{v:.3f}" for v in q0)))
            print("  >>> 现在把重物取下来 <<<")
            input("  >>> 取下后回车：")

        if not rows:
            print("\n[run] 一个点都没采到，没写文件。")
            return 1
        with out.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\n[run] 明细已写 {out}")
        if len(rows) < 3:
            print(f"[run] 只有 {len(rows)} 个点，不足以拟合（至少要 3 个）。"
                  f"CSV 已保存，补点后可以跑 fit。")
            return 1

        res = do_fit(rows)
        save_result(res, out)
    except RuntimeError as exc:
        aborted = exc
        print(f"\n[ABORT] {exc}", file=sys.stderr)
        print("[ABORT] **立刻卸砝码**。臂仍在原位保持，但空闲约 10 min 会回落 TEACHING，"
              "那时砝码会把臂拽下来。", file=sys.stderr)
    finally:
        print("\n[run] 臂仍在 CAN_CTRL 保持。**先卸砝码，再断电/退出**。")
        print(f"[run] 发送统计：ok={hold.n_ok} fail={hold.n_fail}")
        hold.close()
    return 3 if aborted else 0


def do_fit(rows):
    xs = [r["i6_mean"] for r in rows]
    ys = [r["tau_nm"] for r in rows]
    f = fit_line(xs, ys)
    print()
    print("## 拟合  τ = K6·I6 + b")
    if f is None:
        print("  点数不足或 I 无变化，拟合失败")
        return None
    print(f"  K6     = {f['K']:+.4f} N·m/A   ± {f['se_K']:.4f}  (σ/K = {f['rel_se']:.1f}%)")
    print(f"  b      = {f['b']:+.4f} N·m  ← 常数偏置，含横杆自重 + 空瓶/绳重 + 摩擦")
    print(f"           （不是误差。空瓶没称正是因为它全进了 b，斜率 K6 不受影响）")
    print(f"  R²     = {f['r2']:.4f}")
    print(f"  残差RMS= {math.sqrt(f['rss']/f['n']):.4f} N·m")
    print(f"  SDK 换算(分组 J4–J6) = 0.95844 N·m/A ⇒ 本次/厂商 = "
          f"{f['K']/0.95844:.2f}×")
    print()
    ok = f["rel_se"] < 5.0 and f["r2"] > 0.98
    print(f"  判据 σ/K<5% 且 R²>0.98 → {'通过 ✓' if ok else '未通过 ✗'}")
    if not ok:
        print("  加力臂（--arm 加大到 0.15）或加重点，把 τ 跨度拉开；")
        print("  也要确认横杆没打滑（杆在夹爪里转动会让 I 对 τ 完全不敏感）。")
    return dict(K=f["K"], se_K=f["se_K"], b=f["b"], r2=f["r2"], n=f["n"],
                rel_se_pct=f["rel_se"], passed=ok,
                arm_m=rows[0]["arm_m"],
                masses=[r["mass_kg"] for r in rows],
                taus=[r["tau_nm"] for r in rows],
                currents=[r["i6_mean"] for r in rows])


def save_result(res, src):
    if res is None:
        return
    DATA.mkdir(exist_ok=True)
    out = DATA / "k6_result.json"
    payload = dict(version="s6_k6_calib 1.0", source_csv=str(src), **res,
                   method="悬挂砝码产生已知绕轴扭矩，最小二乘拟合 τ=K6·I+b",
                   note="b 含横杆自重与摩擦偏置；K6 仅对 J6，别套到其他轴")
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"  结果已写 {out}")


def cmd_fit(args):
    path = Path(args.csv) if args.csv else None
    if path is None:
        cands = sorted(DATA.glob("k6_*.csv"))
        if not cands:
            raise SystemExit(f"[ABORT] {DATA} 下没有 k6_*.csv，先跑 run")
        path = cands[-1]
    rows = []
    with path.open(encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(dict(mass_kg=float(r["mass_kg"]), arm_m=float(r["arm_m"]),
                             tau_nm=float(r["tau_nm"]), i6_mean=float(r["i6_mean"])))
    print(f"[fit] 读 {path}（{len(rows)} 点）")
    drops = [float(v) for v in args.drop.split(",") if v.strip()] if args.drop else []
    if drops:
        before = len(rows)
        rows = [r for r in rows
                if not any(abs(r["mass_kg"] - d) < 0.02 for d in drops)]
        print(f"[fit] --drop 剔除 {before - len(rows)} 个点：{drops} kg，剩 {len(rows)} 点")
        if len(rows) < 3:
            raise SystemExit("[ABORT] 剔除后不足 3 点")
    res = do_fit(rows)
    save_result(res, path)
    return 0


def cmd_selftest(args):
    """离线验证拟合器：给一个已知 K=0.62、b=−0.03、带 1% 噪声的数据集。"""
    true_K, true_b = 0.62, -0.03
    taus = [m * G * 0.10 for m in (0.2, 0.35, 0.5, 0.65, 0.8)]
    xs = [(t - true_b) / true_K for t in taus]
    xs = [x * (1 + 0.01 * ((i % 3) - 1)) for i, x in enumerate(xs)]  # 人为 1% 扰动
    f = fit_line(xs, taus)
    print(f"[selftest] 真值 K={true_K} b={true_b}")
    print(f"[selftest] 拟合 K={f['K']:.4f} b={f['b']:+.4f} R²={f['r2']:.4f} "
          f"σ/K={f['rel_se']:.2f}%")
    ok = abs(f["K"] - true_K) / true_K < 0.02 and f["r2"] > 0.99
    print("[selftest] " + ("PASS" if ok else "FAIL"))
    if not ok:
        return 1

    print("[selftest] 量水换算（路线 2：1 ml 水 ≈ 0.9982 g @20℃，空瓶重量由 b 吸收）")
    for ml in (200, 300, 400, 500, 600):
        m_water = ml * WATER_DENSITY / 1000.0
        print(f"  {ml:>3d} ml → {m_water:.4f} kg → τ@L=100mm = {m_water*G*0.100:.4f} N·m")

    arm = load_arm()
    print("[selftest] URDF 姿态检查（J6 轴倾角应 ≈0 才能用；注意 J4=0 是腕部中立位）")
    for q in ([0, 45, -60, 0, 45, 0], [0, 70, -50, 0, -15, 0],
              [0, 70, -50, 0, -15, 90], [0, 100, -100, 0, 5, 0]):
        u, _, ax, ay = axis_report(arm, q)
        print(f"  q={q} → J6 轴倾角 {tilt_deg(u):5.1f}°, x {tilt_deg(ax):5.1f}°, "
              f"y {tilt_deg(ay):5.1f}°")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="J6 力矩常数 K6 标定")
    ap.add_argument("--iface", default="can0")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("probe", help="只读检查姿态（不发帧）")
    sp.set_defaults(func=cmd_probe)

    sg = sub.add_parser("goto", help="[不查限位] 线性插值到目标关节角，速率/电流/lead 守卫")
    sg.add_argument("--target-deg", required=True, help="逗号分隔 6 个角度，如 0,70,-50,0,-15,150")
    sg.add_argument("--rate", type=float, default=6.0, help="速率 °/s，上限 6")
    sg.add_argument("--speed-pct", type=int, default=20)
    sg.add_argument("--yes", action="store_true", help="必填，确认知道本命令不查限位表")
    sg.set_defaults(func=cmd_goto)

    ss = sub.add_parser("sample", help="采单点并追加 CSV（远程/分步操作用）")
    ss.add_argument("--ml", type=float, default=None, help="水量 ml，自动换算质量")
    ss.add_argument("--mass", type=float, default=None, help="直接给质量 kg")
    ss.add_argument("--arm", type=float, default=0.100, help="悬挂点到 J6 轴的距离，米")
    ss.add_argument("--sec", type=float, default=4.0)
    ss.add_argument("--tag", default=None, help="同一 tag 的多个点会追加到同一个 CSV")
    ss.add_argument("--speed-pct", type=int, default=20)
    ss.set_defaults(func=cmd_sample)

    sw = sub.add_parser("sweep", help="正反转动差分测 K6（破静摩擦，小配重也能出信号）")
    sw.add_argument("--ml", type=float, default=None, help="水量 ml")
    sw.add_argument("--mass", type=float, default=None, help="质量 kg（空载自检用 0）")
    sw.add_argument("--arm", type=float, default=0.100, help="力臂，米")
    sw.add_argument("--amp", type=float, default=15.0, help="摆幅 ±°，上限 30")
    sw.add_argument("--rate", type=float, default=2.0, help="转动速率 °/s，默认 2")
    sw.add_argument("--tag", default=None)
    sw.add_argument("--speed-pct", type=int, default=15)
    sw.set_defaults(func=cmd_sweep)

    swf = sub.add_parser("sweepfit", help="拟合多个 ΔI-τ 点得 K6")
    swf.add_argument("--csv", default=None)
    swf.set_defaults(func=cmd_sweepfit)

    tw = sub.add_parser("twist", help="拧瓶盖并 200Hz 记录（判据看 I6 突降）")
    tw.add_argument("--target", type=float, required=True, help="目标 J6 角度，deg")
    tw.add_argument("--rate", type=float, default=3.0, help="速率 °/s，上限 6")
    tw.add_argument("--reissue", type=int, default=5, help="每几拍重发一次夹爪指令")
    tw.add_argument("--grip-effort", type=int, default=5000,
                    help="重发夹爪时用的 effort（必须给满力，否则目标追实际会导致夹持力衰减）")
    tw.add_argument("--grip-squeeze", type=int, default=300,
                    help="重发夹爪时相对实测开度再压进的量（0.001mm）。不给压进量则位置误差约 0、出力建立不起来")
    tw.add_argument("--tag", default=None)
    tw.add_argument("--speed-pct", type=int, default=20)
    tw.set_defaults(func=cmd_twist)

    sr = sub.add_parser("run", help="交互式逐点加载测量")
    sr.add_argument("--arm", type=float, default=0.100, help="悬挂点到 J6 轴的距离，米（量准！）")
    sr.add_argument("--masses", default="",
                    help="逗号分隔的 kg 列表；留空则每点现场称重输入（推荐）")
    sr.add_argument("--points", type=int, default=5, help="交互模式下打算测几个点")
    sr.add_argument("--by-volume", action="store_true",
                    help="按水量 ml 输入（路线 2：用量水代替砝码），自动换算成 kg")
    sr.add_argument("--sec", type=float, default=4.0, help="每点采样秒数")
    sr.add_argument("--tag", default=None, help="输出文件名标签")
    sr.add_argument("--speed-pct", type=int, default=20, help="0x151 速度上限百分比")
    sr.set_defaults(func=cmd_run)

    sf = sub.add_parser("fit", help="对已有 CSV 重算拟合")
    sf.add_argument("--csv", default=None)
    sf.add_argument("--drop", default="",
                    help="拟合时剔除这些质量点，逗号分隔 kg（如 0.1 剔除 100ml 小点）")
    sf.set_defaults(func=cmd_fit)

    st = sub.add_parser("selftest", help="离线自测")
    st.set_defaults(func=cmd_selftest)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main() or 0)
