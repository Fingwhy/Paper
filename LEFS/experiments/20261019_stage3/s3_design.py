#!/usr/bin/env python3
"""阶段 3（自由运动数据集）—— 轨迹矩阵设计 + 几何闸门 + 激励量预估

这个脚本回答三个问题，一个都不能省：

1. **几何安全吗**（发帧之前）：正弦轨迹的包络是「各轴独立摆动」，所以最危险的点是
   `center ± amp` 这个轴对齐盒子的角点（2^6 = 64 个）。把 64 个角点全过一遍闸门，
   比只查两个端点严格，又比逐帧采样便宜。
2. **激励够吗**（采完之后判据）：指南 §6.3 要求「每关节都有明显的位置/速度激励」。
   位置峰峰 = 2·amp，速度峰值 = 2πf·amp。速度必须卡在固件 0.3 rad/s 之下。
3. **覆盖全吗**：六个关节 × 三档速度、多关节联动、重复路径、静止保持、极端姿态。

为什么不复用 `s21_design.py` 的 `path_min` 做正弦闸门
----------------------------------------------------
`path_min` 是**两点之间**的插值路径。正弦在一段之内来回走很多次，起终点都是 center，
两点路径退化成「原地不动」，闸门等于没查。必须换成**包络角点**判据。

为什么口径是夹爪而不是水瓶
--------------------------
阶段 3 必须空载（自由空间标签不允许末端负载，0.525 kg 在高速往复里还会增加碰撞风险）。
所以用 `set_pack('gripper')`：11 cm 夹爪包络、半径 45 mm。

用法（VM，~/stage0/20261019_stage3 下）：
    python3 s3_design.py --selftest      # 自测，不动硬件
    python3 s3_design.py --list          # 打印轨迹矩阵表
    python3 s3_design.py --check         # 全矩阵几何 + 激励 + 限位判定
    python3 s3_design.py --seq           # 打印执行顺序（供 run_s3.sh 读）
    python3 s3_design.py --show S1_2_lo  # 打印单段的完整命令行
"""

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
# stage2 与本目录在 VM 上是同级（~/stage0/20261019_stage2），本机也是同级
# （LEFS/experiments/20261019_stage2），所以这一个相对路径两边都对。
sys.path.insert(0, str(HERE.parent / "20261019_stage2"))

import s21_design as G  # noqa: E402

RAD2DEG = 180.0 / math.pi

# 固件限位（stage0 门 8 实测 0x473 回读，见 env/joint_limits.json）
# 注意 J2 下限是 0、J3 上限是 0 —— 这两条不是笔误，是这台机器的真实约束。
LIMITS = [(-150.0, 150.0), (0.0, 180.0), (-170.0, 0.0),
          (-100.0, 100.0), (-70.0, 70.0), (-180.0, 180.0)]

# 固件 vmax = 300 raw → 0.300 rad/s = 17.19 °/s（R29：未用实测定档，按文档取 0.001）。
# 设计上限取 85%，留出「固件插补 + 上位机抖动」的余量。
VMAX_DEG_S = 0.300 * RAD2DEG
V_DESIGN = 0.85 * VMAX_DEG_S

# 激励判据（比 L0-b 的「峰峰 < 0.5° 才算未激励」严格得多）
# 0.5° 那张表只用来区分「有没有动」，不足以支撑 L1 学动力学。
MIN_PP_DEG = 8.0      # 位置峰峰
MIN_V_PEAK = 3.0      # 速度峰值 °/s

# 三个中心姿态：收拢 / 中伸 / 前伸，覆盖常用工作空间。
# 都在 2026-10-20 用夹爪包络验过（法兰 z 475 / 322 / 349 mm，净空 325 / 191 / 283 mm）。
# 「前伸」原来是 J2=90、J3=−70，工具轴下倾 50°、末端只剩 208 mm —— 加到 ±7° 的包络角点
# 就压到 109 mm 被闸门判死。降到 J2=80 后下倾 40°、末端 279 mm，包络才够。
NC_A = (0.0, 30.0, -60.0, 0.0, 25.0, 0.0)    # 收拢高位
NC_B = (0.0, 60.0, -50.0, 0.0, 40.0, 0.0)    # 中伸（= S2-1 的 K2）
NC_C = (0.0, 80.0, -70.0, 0.0, 35.0, 0.0)    # 前伸

# 三档频率。amp 6° 时峰值速度 3.77 / 8.29 / 13.19 °/s，最高档占上限 77%。
# J6 用 15° 幅值（绕工具轴转不改几何，可以给大激励），频率相应降下来保持同一速度档。
F_LO, F_MID, F_HI = 0.10, 0.22, 0.35
F6_LO, F6_MID, F6_HI = 0.05, 0.09, 0.14

A6 = 6.0     # 通用幅值
AJ6 = 15.0   # J6 幅值


def _amp(i, a):
    v = [0.0] * 6
    v[i] = a
    return tuple(v)


def _sine(name, center, amp, freq, dur, split, note):
    return dict(name=name, kind="sine", center=center, amp=tuple(amp),
                freq=freq, dur=dur, split=split, note=note)


def _pose(name, target, hold, split, note):
    return dict(name=name, kind="pose", center=tuple(target), amp=(0.0,) * 6,
                freq=0.0, dur=hold, split=split, note=note)


# ---------------------------------------------------------------------------
# 轨迹矩阵
# ---------------------------------------------------------------------------
TRACKS = []

# 1) 单关节 × 6 × 三档速度 —— 激励摩擦与方向切换
for i, jn in enumerate(["J1", "J2", "J3", "J4", "J5", "J6"]):
    c = NC_A if i in (0, 1, 2) else NC_B
    amp = AJ6 if i == 5 else A6
    fs = (F6_LO, F6_MID, F6_HI) if i == 5 else (F_LO, F_MID, F_HI)
    durs = (40, 22, 21) if i == 5 else (20, 14, 12)
    for tag, f, d in zip(("lo", "mid", "hi"), fs, durs):
        TRACKS.append(_sine(f"S1_{i+1}_{tag}", c, _amp(i, amp), f, d, "train",
                            f"单关节 {jn} {tag} 档：amp {amp:g}° @ {f} Hz"))

# 2) 多关节联动 2~3 轴 —— 激励耦合项
TRACKS += [
    _sine("M1_J2J3", NC_A, (0, 5, 5, 0, 0, 0), 0.15, 20, "train", "J2+J3 同相（肩肘耦合）"),
    _sine("M2_J2J3J5", NC_A, (0, 5, 5, 0, 4, 0), 0.12, 24, "train", "J2+J3+J5 同相"),
    _sine("M3_J1J4", NC_B, (8, 0, 0, 8, 0, 0), 0.12, 24, "train", "J1+J4 同相（基座+前臂）"),
    _sine("M4_J4J5J6", NC_B, (0, 0, 0, 5, 5, 8), 0.15, 20, "train", "J4+J5+J6 三轴腕部"),
    _sine("M5_J1J2J6", NC_A, (6, 4, 0, 0, 0, 10), 0.12, 24, "train", "J1+J2+J6（跨段耦合）"),
    _sine("M6_J1J5", NC_A, (8, 0, 0, 0, 8, 0), 0.10, 24, "train", "J1+J5（基座与腕俯仰正交）"),
    _sine("M7_J2J6", NC_B, (0, 5, 0, 0, 0, 10), 0.13, 22, "train", "J2+J6（肩与工具轴）"),
]

# 3) 重复路径 × 5 —— 检查重复性与温漂（同参数、独立 episode）
for k in range(1, 6):
    TRACKS.append(_sine(f"R{k}_rep", NC_A, (0, 5, 5, 0, 4, 0), 0.20, 12, "train",
                        f"重复路径第 {k}/5 次（与 M2 同结构，更快的 0.20 Hz）"))

# 4) 静止保持 —— 检查重力项与静态偏置（阶段 2 的 K 就是在这个状态下定的）
TRACKS += [
    _pose("H1_hold_A", NC_A, 8.0, "train", "静止保持 收拢位"),
    _pose("H2_hold_B", NC_B, 8.0, "train", "静止保持 中伸位"),
    _pose("H3_hold_C", NC_C, 8.0, "train", "静止保持 前伸位"),
]

# 5) 极端姿态 —— 接近但不超过限位，检查外推表现
TRACKS += [
    _pose("E1_J2high", (0, 120, -110, 0, 35, 0), 6.0, "train", "J2=120° 大角度（J3 同步折起，否则末端压到 88 mm）"),
    _pose("E2_J3deep", (0, 60, -140, 0, 35, 0), 6.0, "train", "J3=−140° 深折叠"),
    _pose("E3_wrist", (0, 90, -60, 60, 50, 60), 6.0, "train", "J4/J5/J6 三轴同向"),
]

# 6) 验证集与测试集 —— **另外采**，不从训练批里硬切（指南 §6.1）
#    频率/幅值刻意与训练批错开，保证「训练集与测试集不含同一轨迹」。
TRACKS += [
    _sine("V1_J2J3", NC_C, (0, 6, 6, 0, 0, 0), 0.18, 16, "val", "验证：J2+J3 @ 前伸位"),
    _sine("V2_J5J6", NC_C, (0, 0, 0, 0, 6, 12), 0.11, 20, "val", "验证：腕部 J5+J6"),
    _sine("V3_J1", NC_B, (10, 0, 0, 0, 0, 0), 0.09, 24, "val", "验证：J1 大角基座"),
    _sine("T1_J2J3J5", NC_C, (0, 7, 7, 0, 5, 0), 0.26, 14, "test", "测试：三轴联动 高速"),
    _sine("T2_J1J6", NC_C, (0, 0, 0, 0, 0, 16), 0.07, 30, "test", "测试：J6 大角慢速"),
    _sine("T3_J4", NC_B, (0, 0, 0, 9, 0, 0), 0.25, 12, "test", "测试：J4 单轴 高速"),
]

# ---------------------------------------------------------------------------
# J6 单侧偏置（2026-10-20 14:0x 实测后加）
# ---------------------------------------------------------------------------
# 事实：J6 的机械行程是**单侧**的 —— 0° 是硬端点，只能往正方向转，手转实测
# 0 → +342.87°，负向「用手都转不动」。而原设计按「J6 中心 0、±15° 对称正弦」
# 来配，等于有一半行程在墙里：
#   S1_6_lo 让 J6 从 0 → +15°（通）→ 回 0 → 往 −15° 走，在 **−0.52°** 顶死，
#   电流冲到 **−12.8 A** 并持续 58.7 s，驱动打出「过流+驱动错误+堵转」锁死 J6，
#   整批采集停在 17/42。
#
# 所以所有段的 J6 中心统一抬到 J6_HOME，并把 J6 幅值限制在 ≤ 20°：
#   最激动的一段 T2（amp 16）→ 行程 +14 ~ +46，离 0 那个端点和固件上限 +123 都远。
# J6 绕工具轴转，不改变任何连杆位置，因此这次平移**不影响几何闸门**。
J6_HOME = 30.0
J6_AMP_MAX = 20.0

for _t in TRACKS:
    _c = list(_t["center"])
    _c[5] = _c[5] + J6_HOME
    _t["center"] = tuple(_c)
    if _t["amp"][5] > J6_AMP_MAX:
        _a = list(_t["amp"])
        _a[5] = J6_AMP_MAX
        _t["amp"] = tuple(_a)
    # 硬性下限：J6 最低点必须离 0° 端点至少 2°，否则又是「一半行程在墙里」
    _lo = _t["center"][5] - _t["amp"][5]
    if _lo < 2.0:
        raise SystemExit(
            f"[FATAL] 段 {_t['name']} 的 J6 最低点 {_lo:+.1f}° 太靠近硬端点 0°。"
            f"J6 是单侧行程（0 是硬端点，实测只能 0→+343），见 J6_HOME 注释。"
            f"把 center J6 抬到 ≥ {2.0 + _t['amp'][5]:.1f}°")

BY_NAME = {t["name"]: t for t in TRACKS}
ORDER = [t["name"] for t in TRACKS]

# ---------------------------------------------------------------------------
# 几何与激励
# ---------------------------------------------------------------------------
def load(urdf=None):
    """load(urdf) 返回 (urdf, arm)，口径固定为**夹爪包络**（阶段 3 空载）。"""
    p = urdf or str(HERE.parent / "_vendor" / "urdf" / "piper_description.urdf")
    u, a = G.load(p)
    G.set_pack("gripper")
    return u, a


def corners(center, amp):
    """轴对齐盒子的全部角点（2^6 = 64），比逐帧采样便宜且更保守。"""
    out = [[]]
    for c, a in zip(center, amp):
        nxt = []
        for base in out:
            if a == 0.0:
                nxt.append(base + [c])
            else:
                nxt.append(base + [c - a])
                nxt.append(base + [c + a])
        out = nxt
    return out


def limit_margin(q):
    """每轴到固件限位的余量（°），取最小值。负数 = 越界。"""
    return min(min(q[i] - LIMITS[i][0], LIMITS[i][1] - q[i]) for i in range(6))


def artic(t):
    """每轴（位置峰峰 °、速度峰值 °/s）。"""
    pp = [2.0 * a for a in t["amp"]]
    vp = [2 * math.pi * t["freq"] * a for a in t["amp"]]
    return pp, vp


def track_verdict(arm, t):
    """单段判定：几何（全部角点）+ 限位 + 速度 + 激励。"""
    bad = []
    worst = (1e9, None)
    if t["kind"] == "sine":
        for q in corners(t["center"], t["amp"]):
            v = G.pose_verdict(arm, G.deg(*q))
            if v["bad"]:
                bad.append(f"角点 {tuple(round(x, 1) for x in q)}: {v['bad']}")
                break
            c = min(v["zmin"] - G.pack()["zmin"], v["clr"] - G.pack()["clr"])
            if c < worst[0]:
                worst = (c, q)
        lm = min(limit_margin(q) for q in corners(t["center"], t["amp"]))
    else:
        q = t["center"]
        v = G.pose_verdict(arm, G.deg(*q))
        if v["bad"]:
            bad.append(f"目标: {v['bad']}")
        worst = (min(v["zmin"] - G.pack()["zmin"], v["clr"] - G.pack()["clr"]), q)
        lm = limit_margin(q)

    pp, vp = artic(t)
    vmax = max(vp)
    if vmax > V_DESIGN:
        bad.append(f"速度 {vmax:.1f} > 设计上限 {V_DESIGN:.1f} °/s")
    if lm < 0:
        bad.append(f"限位越界 {lm:+.1f}°")
    # 激励判据只对**正弦段**成立。pose 段（静止保持 / 极端姿态）本来就是「不动」，
    # 它们查的是静态重力偏置与外推位形，拿「峰峰 ≥ 8°」去卡等于要求它们必须动起来。
    if t["kind"] == "sine":
        if max(pp) < MIN_PP_DEG:
            bad.append(f"位置峰峰 {max(pp):.1f} < {MIN_PP_DEG}")
        if max(vp) < MIN_V_PEAK:
            bad.append(f"速度峰值 {max(vp):.1f} < {MIN_V_PEAK}")

    return dict(bad=bad, margin=worst[0], at=worst[1], lim=lm,
                pp=pp, vp=vp, vmax=vmax)


def cmdline(t, limits_path, out_dir, rate_deg_s):
    """生成该段的实际命令行（供现场核对，也供 run_s3.sh 对照）。"""
    c = ",".join(f"{x:g}" for x in t["center"])
    pre = ("( cd $STAGE1 && python3 stage1_cmd.py pose --iface $IFACE --limits %s "
           "--target-deg %s --rate-deg-s %g --hold-s 3 --stall-win-s 1.0 "
           "--stall-eps-deg 0.08 --max-current-a 4.0 --max-drop-mm auto "
           "--lead-max-deg 2.0 --out %s/%s_pre --yes )"
           % (limits_path, c, rate_deg_s, out_dir, t["name"]))
    if t["kind"] == "pose":
        main = ("( cd $STAGE1 && python3 stage1_cmd.py pose --iface $IFACE --limits %s "
                "--target-deg %s --rate-deg-s %g --hold-s %g --stall-win-s 1.0 "
                "--stall-eps-deg 0.08 --max-current-a 4.0 --max-drop-mm auto "
                "--lead-max-deg 2.0 --out %s/%s --yes )"
                % (limits_path, c, rate_deg_s, t["dur"], out_dir, t["name"]))
        return [main]
    amps = " ".join(f"{x:g}" for x in t["amp"])
    main = ("( cd $STAGE1 && python3 stage1_cmd.py sine --iface $IFACE --limits %s "
            "--amp-deg %s --freq-hz %g --duration %g --phase-spread 0 "
            "--out %s/%s --yes )"
            % (limits_path, amps, t["freq"], t["dur"], out_dir, t["name"]))
    return [pre, main]


def rec_secs(t):
    """录制时长：到位余量 + 轨迹时长 + 收尾余量。"""
    if t["kind"] == "pose":
        return t["dur"] + 60
    return 25 + t["dur"] + 12


# ---------------------------------------------------------------------------
# 命令
# ---------------------------------------------------------------------------
def cmd_list(a):
    urdf, arm = load(a.urdf)
    print(f"末端口径：{G.pack()['label']}（阶段 3 空载；法兰 z≥{G.MIN_TCP_Z_MM:.0f}、"
          f"末端最低 z≥{G.pack()['zmin']:.0f}、净空≥{G.pack()['clr']:.0f} mm）")
    print(f"速度设计上限 {V_DESIGN:.2f} °/s（固件 {VMAX_DEG_S:.2f} °/s 的 85%）")
    print()
    print("| # | 段名 | 类型 | 中心/目标 q(°) | amp(°) | f(Hz) | 时长s | split | 说明 |")
    print("|" + " --- |" * 9)
    for i, t in enumerate(TRACKS, 1):
        print("| %d | %s | %s | %s | %s | %s | %g | %s | %s |"
              % (i, t["name"], t["kind"],
                 ", ".join("%g" % x for x in t["center"]),
                 ", ".join("%g" % x for x in t["amp"]),
                 ("%.2f" % t["freq"]) if t["freq"] else "—",
                 t["dur"], t["split"], t["note"]))
    tot = sum(t["dur"] for t in TRACKS)
    tr = sum(t["dur"] for t in TRACKS if t["split"] == "train")
    print()
    print(f"段数 {len(TRACKS)}（train {sum(1 for t in TRACKS if t['split']=='train')} / "
          f"val {sum(1 for t in TRACKS if t['split']=='val')} / "
          f"test {sum(1 for t in TRACKS if t['split']=='test')}）")
    print(f"轨迹时长合计 {tot:.0f} s（train {tr:.0f} s = {tr/60:.1f} min）；"
          f"加到位开销预计 {tot*1.6/60:.0f} min")


def cmd_check(a):
    urdf, arm = load(a.urdf)
    print(f"{'段名':<14} {'法兰z':>7} {'最低z':>7} {'净空':>7} {'限位余':>7} "
          f"{'峰峰°':>6} {'v峰°/s':>8} {'判':<4} 说明")
    n_bad = 0
    for t in TRACKS:
        v = track_verdict(arm, t)
        p6 = G.pose_verdict(arm, G.deg(*t["center"]))["p6"]
        ok = not v["bad"]
        n_bad += 0 if ok else 1
        print("%-14s %7.0f %7.0f %7.0f %+7.0f %6.0f %8.1f %-4s %s"
              % (t["name"], p6[2] * 1000, v["margin"] + G.pack()["zmin"],
                 v["margin"] + G.pack()["clr"], v["lim"], max(v["pp"]), v["vmax"],
                 "✓" if ok else "✗", "" if ok else "; ".join(v["bad"])))
    print()
    if n_bad:
        print(f"总判： {n_bad} 段不放行 —— **不许起批**。")
        return 1
    print(f"总判： {len(TRACKS)} 段全部放行")
    return 0


def cmd_seq(a):
    names = list(ORDER)
    if a.seq not in ("", "ROUTE"):
        names = [x.strip() for x in a.seq.split(",") if x.strip()]
        miss = [n for n in names if n not in BY_NAME]
        if miss:
            print(f"[ABORT] 未知段 {miss}", file=sys.stderr)
            return 2
    print(",".join(names))
    return 0


def cmd_route(a):
    """段间移动的路径闸门。

    为什么单独做这一层：段内几何由 `--check` 的包络角点保证，但**段与段之间**是从
    上一段的终点走到下一段的中心，这条路径是两个点的插值（各轴按**同一角速率**走，
    见 R49）。它可能扫过比两端都低的位置 —— S2-1 的 K8→K5 就是这么被判死的
    （旧口径 +82 mm，真实 −87 mm）。

    起点默认取当前实测姿态（`--start-from-can`），或显式给 `--start-deg`。
    """
    urdf, arm = load(a.urdf)
    names = list(ORDER)
    if a.route not in ("", "ROUTE"):
        names = [x.strip() for x in a.route.split(",") if x.strip()]
        miss = [n for n in names if n not in BY_NAME]
        if miss:
            print(f"[ABORT] 未知段 {miss}", file=sys.stderr)
            return 2

    if a.start_deg:
        q0 = G.deg(*[float(x) for x in a.start_deg.split(",")])
    else:
        s = G._read_start(a.iface)          # 返回**度数**
        if s is None:
            print("[ABORT] 读不到当前姿态，用 --start-deg 显式给", file=sys.stderr)
            return 2
        q0 = G.deg(*s)
        print("起点（实测）= %s" % ", ".join("%+.1f" % x for x in s))

    def at(t):
        # 正弦一圈不一定回到中心（时长不是周期的整数倍），但偏差 ≤ 振幅，
        # 用 center 做端点已经落在闸门的保守侧。
        return G.deg(*t["center"])

    worst = (1e9, "")
    print()
    print("%-34s %8s %8s" % ("段间移动", "最差t", "余量mm"))
    prev, pname = q0, "起点(实测)"
    for n in names:
        t = BY_NAME[n]
        q = at(t)
        c, tt = G.path_min(arm, prev, q)
        flag = "" if c >= 0 else "  ✗"
        print("%-34s %8.2f %+8.0f%s" % (f"{pname} → {n}", tt, c, flag))
        if c < worst[0]:
            worst = (c, f"{pname}→{n}")
        prev, pname = q, n
    print()
    if worst[0] < 0:
        print(f"总判： 最紧一段 {worst[1]} 余量 {worst[0]:+.0f} mm —— **不放行**，"
              f"要加中转点或换顺序")
        return 1
    print(f"总判： 全段放行（最紧 {worst[1]} {worst[0]:+.0f} mm）")
    return 0


def cmd_emit(a):
    """把单段的执行参数输出成 shell 赋值，供 run_s3.sh `eval`。

    为什么用这个而不是让 run_s3.sh 自己维护第二张表
    ------------------------------------------------
    S2-1 那次就吃过「run_s21.sh 里另写一份 TARGET 表」的亏：改姿态时两边不同步，
    闸门验的是 A、采的是 B。现在唯一真相源就是本文件的 TRACKS。
    """
    t = BY_NAME.get(a.emit)
    if not t:
        print(f"[ABORT] 未知段 {a.emit}", file=sys.stderr)
        return 2
    print(f"KIND={t['kind']}")
    print(f"CENTER={','.join('%g' % x for x in t['center'])}")
    print(f"AMP='{' '.join('%g' % x for x in t['amp'])}'")
    print(f"FREQ={t['freq']:g}")
    print(f"DUR={t['dur']:g}")
    print(f"REC={rec_secs(t):g}")
    print(f"SPLIT={t['split']}")
    print(f"NOTE='{t['note']}'")
    return 0


def cmd_gate(a):
    """单段发帧前的最终闸门：段内包络角点 + 从当前姿态到中心的移动路径。

    `run_s3.sh` 每段调一次。分三档结论：
      0 = 放行
      1 = 段内几何不合格（这段本身就不能走）
      2 = 移动路径不合格（这段可以走，但不能从当前位置直达）
    """
    t = BY_NAME.get(a.gate)
    if not t:
        print(f"[ABORT] 未知段 {a.gate}", file=sys.stderr)
        return 2
    urdf, arm = load(a.urdf)
    v = track_verdict(arm, t)
    print(f"[gate] 段 {t['name']}  {t['note']}")
    print(f"[gate] 包络：峰峰 {max(v['pp']):.1f}°  速度峰 {v['vmax']:.1f} °/s  "
          f"限位余 {v['lim']:+.0f}°  几何最差 {v['margin']:+.0f} mm")
    if v["bad"]:
        print(f"[gate] ✗ 段内不合格：{'; '.join(v['bad'])}")
        return 1
    print("[gate] ✓ 段内合格")
    if a.skip_route:
        print("[gate] （跳过移动路径检查，SKIP_ROUTE=1）")
        return 0

    if a.start_deg:
        q0 = G.deg(*[float(x) for x in a.start_deg.split(",")])
    else:
        s = G._read_start(a.iface)
        if s is None:
            print("[gate] ✗ 读不到当前姿态（can0 未 UP / 未使能）")
            return 3
        q0 = G.deg(*s)
        print("[gate] 起点（实测）= %s" % ", ".join("%+.1f" % x for x in s))
    q1 = G.deg(*t["center"])
    c, f = G.path_min(arm, q0, q1)
    R0, p0, _ = G.tcp_and_axis(arm, q0)
    z0, c0 = G.bottle_metrics(R0, p0)
    # 首段豁免：起点自身可能贴着门槛（实测落点净空 18 mm），那是事实不是选择。
    # 豁免只能放宽、不能抬高 —— 起点宽敞时仍按 0 判。
    base = min(0.0, min(z0 - G.pack()["zmin"], c0 - G.pack()["clr"]) - 5.0)
    print(f"[gate] 移动路径 最差 f={f:.2f}  余量 {c:+.0f} mm  "
          f"（起点自有余量 {base + 5.0:+.0f}，豁免到 {base:+.0f}）")
    if c < base:
        print("[gate] ✗ 移动路径不合格")
        return 2
    print("[gate] ✓ 移动路径合格 —— 放行")
    return 0


def cmd_show(a):
    t = BY_NAME.get(a.show)
    if not t:
        print(f"[ABORT] 未知段 {a.show}；可用：{', '.join(ORDER)}", file=sys.stderr)
        return 2
    urdf, arm = load(a.urdf)
    v = track_verdict(arm, t)
    print(f"段 {t['name']}  （{t['note']}）")
    print(f"  kind={t['kind']}  split={t['split']}  dur={t['dur']}s  "
          f"录制建议 {rec_secs(t)}s")
    print(f"  峰峰 {['%.1f' % x for x in v['pp']]}")
    print(f"  速度峰值 {['%.2f' % x for x in v['vp']]}")
    print(f"  限位余量 {v['lim']:+.1f}°   几何最差余量 {v['margin']:+.0f} mm")
    print(f"  判定 {'✓ 放行' if not v['bad'] else '✗ ' + '; '.join(v['bad'])}")
    print()
    for line in cmdline(t, "$LIMITS", "$S3/cmd", 6.0):
        print("  " + line)
    return 0


# ---------------------------------------------------------------------------
# 自测
# ---------------------------------------------------------------------------
def _selftest(a):
    urdf, arm = load(a.urdf)
    n_ok = n_bad = 0

    def chk(title, cond, detail=""):
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print(f"  [OK] {title}")
        else:
            n_bad += 1
            print(f"  [XX] {title}  {detail}")

    print("自测 A 矩阵结构")
    chk("A 段数 ≥ 30（够 10 分钟）", len(TRACKS) >= 30, str(len(TRACKS)))
    chk("A 名字唯一", len(set(ORDER)) == len(ORDER))
    tr = sum(t["dur"] for t in TRACKS if t["split"] == "train")
    chk("A 训练批 ≥ 8 min（指南要 10 min 量级）", tr >= 480, f"{tr/60:.1f} min")
    chk("A split 只有 train/val/test",
        all(t["split"] in ("train", "val", "test") for t in TRACKS))

    print("自测 B 速度不越固件上限")
    over = [(t["name"], max(artic(t)[1])) for t in TRACKS
            if max(artic(t)[1]) > V_DESIGN]
    chk("B 全部段速度 ≤ 设计上限", not over, str(over))

    print("自测 C 每段都有足够激励")
    # pose 段不参与激励判据（它们本来就是「不动」的段，查的是静态偏置与外推位形）
    weak = [(t["name"], max(artic(t)[0]), max(artic(t)[1])) for t in TRACKS
            if t["kind"] == "sine"
            and (max(artic(t)[0]) < MIN_PP_DEG or max(artic(t)[1]) < MIN_V_PEAK)]
    chk("C 每段正弦峰峰 ≥ 8° 且速度 ≥ 3 °/s", not weak, str(weak))

    print("自测 D 几何全过（夹爪包络）")
    bads = [(t["name"], track_verdict(arm, t)["bad"]) for t in TRACKS
            if track_verdict(arm, t)["bad"]]
    chk("D 所有段在夹爪包络下放行", not bads, str(bads[:3]))

    print("自测 E 六轴逐个都被单独激励过")
    for i, jn in enumerate(["J1", "J2", "J3", "J4", "J5", "J6"]):
        solo = [t for t in TRACKS if t["kind"] == "sine"
                and t["amp"][i] > 0 and sum(1 for x in t["amp"] if x > 0) == 1]
        chk(f"E {jn} 有单轴正弦段（≥3 档）", len(solo) >= 3, f"{len(solo)} 段")
    # J6 尤其要单独查：L0-b 单轴响应比 0.8672 是六轴里最弱的，六轴同相会掩盖它
    j6 = [t for t in TRACKS if t["kind"] == "sine" and t["amp"][5] > 0
          and sum(1 for x in t["amp"] if x > 0) == 1]
    chk("E J6 单轴段幅值 ≥ 15°", all(t["amp"][5] >= 15 for t in j6) and len(j6) >= 3,
        str([t["amp"][5] for t in j6]))

    print("自测 F 重复路径")
    reps = [t for t in TRACKS if t["name"].startswith("R")]
    chk("F 重复路径正好 5 段", len(reps) == 5, str(len(reps)))
    chk("F 5 段参数完全相同（只名字不同）",
        len({(t["center"], t["amp"], t["freq"], t["dur"]) for t in reps}) == 1)

    print("自测 G 包络角点判据真的比端点严")
    # 构造一个「端点安全、中间角点危险」的假段：J2 大幅值把夹爪压向立柱
    fake = _sine("FAKE", (0.0, 20.0, -20.0, 0.0, 25.0, 0.0),
                 (0, 60, 60, 0, 0, 0), 0.1, 20, "train", "假段")
    vf = track_verdict(arm, fake)
    vc = G.pose_verdict(arm, G.deg(*fake["center"]))
    chk("G 假段自身中心合格但角点被拦（说明角点判据有必要）",
        (not vc["bad"]) and bool(vf["bad"]), f"中心 bad={vc['bad']} 段 bad={vf['bad']}")

    print("自测 H 超速必须被判死")
    fast = _sine("FAST", NC_A, (0, 20, 0, 0, 0, 0), 0.50, 10, "train", "假段")
    vfa = track_verdict(arm, fast)
    chk("H amp20° @0.5Hz 被判速度越限",
        any("速度" in b for b in vfa["bad"]), str(vfa["bad"]))

    print("自测 I 中心姿态都在限位内且留足幅值")
    for t in TRACKS:
        if t["kind"] != "sine":
            continue
        lm = min(limit_margin(q) for q in corners(t["center"], t["amp"]))
        if lm < 0:
            chk(f"I {t['name']} 限位", False, f"{lm:+.1f}°")
            break
    else:
        chk("I 全部正弦段的包络都在固件限位内", True)

    print("自测 J 三档速度确实分档")
    for i in range(6):
        vs = [max(artic(t)[1]) for t in TRACKS
              if t["kind"] == "sine" and t["amp"][i] > 0
              and sum(1 for x in t["amp"] if x > 0) == 1]
        chk(f"J J{i+1} 三档速度递增（{len(vs)} 档）",
            len(vs) >= 3 and min(vs) < max(vs) / 2.5, str([round(v, 1) for v in vs]))

    print("自测 K 验证集/测试集与训练集参数不重复")
    key = lambda t: (round(t["freq"], 4), t["amp"], t["center"])
    trk = {key(t) for t in TRACKS if t["split"] == "train"}
    dup = [t["name"] for t in TRACKS if t["split"] != "train" and key(t) in trk]
    chk("K 无训练/非训练同参数轨迹", not dup, str(dup))

    print(f"\n自测：{n_ok} 过 / {n_bad} 败")
    return 0 if n_bad == 0 else 1


def main():
    ap = argparse.ArgumentParser(description="阶段 3 轨迹矩阵设计")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--list", action="store_true", help="打印轨迹矩阵表")
    ap.add_argument("--check", action="store_true", help="全矩阵几何+激励判定")
    ap.add_argument("--seq", nargs="?", const="ROUTE", default=None,
                    help="打印执行顺序（逗号分隔），默认全部")
    ap.add_argument("--route", nargs="?", const="ROUTE", default=None,
                    help="段间移动路径闸门（rate 口径）；默认全部段")
    ap.add_argument("--start-deg", default=None,
                    help="段间路径的起点（度，逗号分隔）；不给则从 can 读实测姿态")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--show", default=None, help="打印单段详情与命令")
    ap.add_argument("--emit", default=None, help="输出单段执行参数（shell 赋值，供 eval）")
    ap.add_argument("--gate", default=None, help="单段发帧前闸门（段内 + 移动路径）")
    ap.add_argument("--skip-route", action="store_true", help="只查段内，不查移动路径")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return _selftest(a)
    if a.gate:
        return cmd_gate(a)
    if a.emit:
        return cmd_emit(a)
    if a.show:
        return cmd_show(a)
    if a.route is not None:
        return cmd_route(a)
    if a.seq is not None:
        return cmd_seq(a)
    if a.check:
        return cmd_check(a)
    return cmd_list(a)


if __name__ == "__main__":
    sys.exit(main())
