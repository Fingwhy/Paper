#!/usr/bin/env python3
"""s21_design.py —— S2-1 多姿态最小二乘：姿态选型 + **带载**几何闸门

为什么单独写一个
----------------
S2-0 只有 2 个空载姿态 + 1 个带载姿态，够判 H1/H2，但**不够拟合 K**：
`I = τ/K + b` 每轴两个未知数，两个姿态只能让残差恒为 0（零自由度，没法检验）。

S2-1 要多姿态，而多姿态带载会引入两个新问题，这正是本脚本解决的事：

  1. **载荷力矩按「刚性固定在 link6」算，不按吊挂算。**
     现场把 0.525 kg 水瓶用胶带绑在夹爪上 —— 它**跟着夹爪一起倾**（J6 零点归零后
     夹爪本来就不竖直），不是靠重力垂下来的摆。
     代码上只能写成 `tau_point_force(q, link6, c6, (0,0,-mg))`：
     质心 c6 是 link6 系里的**常量**，力永远是世界 −Z。
     （注：这与「吊挂」在数值上等价——力沿自身方向平移不改变力矩；但代码路径必须只有一条，
       否则两个模型混用会再现 2026-10-20 那次「三轴全反号」的假警报。）

  2. **水瓶 22 cm 长，而原闸门只看夹爪法兰。**
     工具轴朝下时瓶底会比法兰低 20 cm，`--min-tcp-z-mm 150` 完全管不到它。
     实测：P4′ 空载是安全的（法兰 240 mm），**带载却会把瓶底压到 23 mm**，
     P2 压到 30 mm —— 这两个 S2-0 用过的姿态，带载后一个都不能再用。
     本脚本把水瓶当成沿工具轴 −40→+220 mm 的胶囊（半径 35 mm），逐点算净空与最低 z。

用法（本机 Windows 或 VM，LEFS/experiments/20261019_stage2）
------------------------------------------------------------
  python s21_design.py --list                     # 候选姿态的力矩与几何指标
  python s21_design.py --seq                      # 校验 S2-1 的整条路线（含逐段路径）
  python s21_design.py --seq --start-from-can     # 用固件实测姿态当起点（VM 上跑）
  python s21_design.py --selftest

判据（写死，避免每次手工挑姿态时忘掉）
--------------------------------------
  * 法兰 z ≥ 150 mm（沿用 S2-0）
  * 水瓶最低点 z ≥ 120 mm（台面 z=0，留 12 cm）
  * 水瓶胶囊（半径 35 mm）到基座立柱净空 ≥ 30 mm
  * 相邻两姿态之间：joint 空间直线上、**f≥1/40 之后**的每一拍都要满足上两条
    （起点豁免，与 stage2_pose_guard 同口径：起点是事实，不是选择）
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage2_design as S
from stage2_pose_guard import _d_hazard, HAZARD_R, HAZARD_Z

# ---------------------------------------------------------------- 现场参数
GRIPPER_MASS = 0.500          # URDF 口径（gripper_base 0.45 + link7/8 各 0.025）
BOTTLE_MASS = 0.525           # 用户实测：连瓶带水带胶带
BOTTLE_COM_D = 0.090          # 质心距法兰沿工具轴 —— 选型用的名义值，拟合时自由
BOTTLE_LEN = 0.220            # 瓶总长（550 ml 瓶约 21~22 cm），保守取 0.22
BOTTLE_R = 0.035              # 瓶半径（Ø64），保守取 35 mm
BOTTLE_BACK = 0.040           # 胶带缠绕段向法兰后方回缩

# 载荷几何包络 —— 两套，必须跟着「臂上到底挂着什么」切换。
#
# 为什么不能一套通用：带载轮的判据是「水瓶 22 cm 胶囊」，拆瓶后这个胶囊不存在了，
# 仍按它判会把**本来安全**的姿态判死。2026-10-20 12:45 实测：断电落点
# (0.08,−0.16,−0.21,3.13,19.74,0) 法兰 z=183.5、到立柱顶角 66 mm，空载完全安全，
# 但胶囊口径给出「瓶净空 −16 mm ✗」—— 差别全在那 18 cm 不存在的瓶身上。
#
# 空载包络取「法兰 → 指尖 110 mm、横向 45 mm」：比实际夹爪略胖，宁保守。
# 空载包络取「法兰 → 指尖 110 mm」，**横向不膨胀**（radius=0）。
# 为什么不膨胀：一开始取 45 mm 半宽把夹爪当胶囊，竖直收拢时端部半球外凸，
# 对断电落点 (法兰 z=183.5、到立柱顶角 66 mm) 判出「净空 −27 mm」——而臂就物理地
# 待在那儿、没碰到任何东西。原因是胶囊端部球把「点距立柱 33.5 mm」再减 45 mm，
# 重复扣了轴向那一份。夹爪是圆柱不是胶囊，端面平，轴向不膨胀才对。
# 横向半宽放弃膨胀的代价：夹爪擦立柱时闸门抓不到 —— 这由运行时电流守卫兜底
# （>3 A 持续 0.3 s 立即抱住），比离线几何更直接。
PACKS = {
    "bottle": dict(back=0.040, length=0.220, radius=0.035,
                   zmin=120.0, clr=30.0, label="水瓶 22 cm 胶囊"),
    "gripper": dict(back=0.000, length=0.110, radius=0.000,
                    zmin=120.0, clr=30.0, label="夹爪 11 cm 轴线"),
}
_PACK_NAME = "bottle"


def set_pack(name):
    """切换载荷包络。所有判据（静态 / 路径 / 首段豁免）都跟着它走。"""
    global _PACK_NAME
    if name not in PACKS:
        raise KeyError(f"未知包络 {name}，可选 {list(PACKS)}")
    _PACK_NAME = name
    return PACKS[name]


def pack():
    return PACKS[_PACK_NAME]


MIN_TCP_Z_MM = 150.0


def deg(*a):
    return [math.radians(x) for x in a]


# ---------------------------------------------------------------- 姿态表
# S2-1 采集集。τ 是「臂自带 + 水瓶」的总保持力矩（N·m，URDF 口径，c6=(0,0,0.09)）。
S21 = [
    # (名字, q_deg, τJ2, τJ3, τJ4, τJ5, 说明)
    ("LIFT", (0, 0, -20, 0, 25, 0),    +4.42, +1.27, -0.01, +1.32,
     "抬臂中转：只把肘弯上去（J3 0→−20），瓶底 93→287 mm。"
     "停放姿态正好在立柱顶角上，必须先抬再走"),
    ("K3",  (0, 30, -110, 0, -40, 0),  +1.84, -0.72, -0.00, -0.76,
     "高抬折叠：J3/J5 取负，瓶底高 591 mm"),
    ("K10", (-45, 30, -80, 0, 50, 0),  +3.87, +1.31, -0.01, +1.32,
     "J2 最大正向激励（+3.87 N·m），J1 转 −45° 排除 J1 影响"),
    ("K15", (0, 45, -60, 0, 45, 30),   +3.13, +1.14, -0.01, +1.20,
     "J6=30°：q6 阶梯第一档"),
    ("K14", (0, 45, -60, 0, 45, 90),   +3.13, +1.14, -0.01, +1.20,
     "J6=90°：q6 阶梯第三档"),
    ("K9",  (0, 60, -50, 90, 40, 90),  +2.21, +0.92, -0.85, -0.07,
     "J4=90°/J6=90°：J4 最大负向激励，J5 也接近另一侧"),
    ("K8",  (0, 60, -50, 60, 40, 0),   +2.18, +0.89, -0.74, +0.43,
     "J4=60°：J4 第二档（与 K9 配成 J4 的两点）"),
    ("K5",  (0, 75, -30, 0, -40, 0),   +1.73, +1.23, -0.01, +1.32,
     "J3/J5 正向台柱，同时压低 J2"),
    ("K2",  (0, 60, -50, 0, 40, 0),    +2.14, +0.85, -0.01, +0.94,
     "与 K8 只差 J4，可直接量出 J4 的载荷贡献"),
    ("P1",  (0, 45, -60, 0, 45, 0),    +3.13, +1.14, -0.01, +1.20,
     "S2-0 主标定点：与 30 min 前的 s20_p1_load 构成重复性对照"),
    ("K12", (0, 120, -90, 90, 40, 0),  -1.04, +0.83, -0.77, -0.36,
     "J2 唯一负向激励（−1.04 N·m），同时 J4/J5 双负"),
    ("K9R", (0, 60, -50, 90, 40, 0),   +2.21, +0.92, -0.85, -0.07,
     "中转点：与 K9 只差 J6（K9 的 J6=90°，这里 0°）。存在的理由不是 τ 跨度，"
     "而是 K2→K12 必须先把 J4 转到 90° 再走 J2/J3 —— 顺序反过来（J2/J3 先到位、"
     "J4 后转）中途瓶底会戳到台面（实测 rate 口径 −49 mm）。"
     "顺带作为 K9 的重复性对照。"),
]

# 采集路线（= 实际下发顺序）。
#
# 2026-10-20 12:5x 改版：**原路线去掉 K5 与 P1，把 K9R 插在 K2 之后**。
#   * K5：τ_pay 四轴全被覆盖（J2 1.93 落在 K2 1.50 与 K12 2.98 之间、
#         J5 0.97 与 LIFT/K10 相同），贡献 +0.000；而且 K8→K5 按真实速率轨迹
#         算出来瓶底只有 **−108 mm**（旧闸门按关节等比插值误报 +82）。
#   * P1：与 K15/K14 同位形，τ_pay 逐轴完全相同；重复性已由
#         s20_p1_load / s21_K15 / s21_K14 三者（J5 −2.400/−2.373/−2.360 A）证明。
#   * K9R：K2→K12 不能直接走。J4 从 0 转到 90 的过程会把瓶子扫到台面
#         （rate 口径 −49 mm）；必须**先在臂收着时把 J4 转到位**，再走 J2/J3。
#         顺序：K8(J4=60) → K2(J4=0) → K9R(J4=90) → K12(J2/J3 最后走)。
ROUTE = ["LIFT", "K3", "K10", "K15", "K14", "K9", "K8", "K2", "K9R", "K12"]

BY_NAME = {n: deg(*q) for n, q, *_ in S21}


# ---------------------------------------------------------------- 运动学小工具
def tcp_and_axis(arm, q):
    T = arm.transforms(q)
    R6, p6 = T[S.END_LINK]
    return R6, p6, [R6[0][2], R6[1][2], R6[2][2]]


def bottle_points(R6, p6, n=32):
    """沿工具轴取一串点，覆盖当前载荷包络（带载=瓶身，空载=夹爪）。"""
    pk = pack()
    ax = [R6[0][2], R6[1][2], R6[2][2]]
    pts = []
    for i in range(n + 1):
        s = -pk["back"] + (pk["length"] + pk["back"]) * i / n
        pts.append([p6[k] + ax[k] * s for k in range(3)])
    return pts, ax


def bottle_metrics(R6, p6):
    pk = pack()
    pts, _ = bottle_points(R6, p6)
    zmin = min(p[2] for p in pts)
    dmin = min(_d_hazard(p) for p in pts) - pk["radius"]
    return zmin * 1000.0, dmin * 1000.0


def tau_all(arm, q, d=BOTTLE_COM_D, m=BOTTLE_MASS):
    ta = arm.tau_gravity(q)
    tp = arm.tau_point_force(q, S.END_LINK, [0.0, 0.0, d], [0.0, 0.0, -m * S.G])
    return ta, tp, [ta[j] + tp[j] for j in range(6)]


def pose_verdict(arm, q):
    pk = pack()
    R6, p6, ax = tcp_and_axis(arm, q)
    ta, tp, tt = tau_all(arm, q)
    zmin, clr = bottle_metrics(R6, p6)
    bad = []
    if p6[2] * 1000.0 < MIN_TCP_Z_MM:
        bad.append(f"法兰z={p6[2]*1000:.0f}<{MIN_TCP_Z_MM:.0f}")
    if zmin < pk["zmin"]:
        bad.append(f"末端最低z={zmin:.0f}<{pk['zmin']:.0f}")
    if clr < pk["clr"]:
        bad.append(f"末端净空={clr:.0f}<{pk['clr']:.0f}")
    return dict(p6=p6, ax=ax, ta=ta, tp=tp, tt=tt, zmin=zmin, clr=clr, bad=bad)


def _interp(qa, qb, t, mode):
    """按 f=t 给出中间关节角。

    mode="rate"  —— **各轴同一角速率**（固件 `MOVE_J` 的真实行为）：
                    行程小的轴先到位，行程大的轴后到位。默认。
    mode="ratio" —— 各轴按行程比例同时到位（旧口径，**与实际运动不符**）。
    """
    d = [b - a for a, b in zip(qa, qb)]
    if mode == "ratio":
        return [a + dd * t for a, dd in zip(qa, d)]
    vmax = max((abs(x) for x in d), default=0.0)
    if vmax < 1e-12:
        return list(qa)
    out = []
    for a, dd in zip(qa, d):
        s = 1.0 if abs(dd) < 1e-12 else min(1.0, t * vmax / abs(dd))
        out.append(a + dd * s)
    return out


def path_min(arm, qa, qb, n=81, mode="rate"):
    """沿真实运动轨迹算最差余量。t 从 1/n 开始（起点豁免）。

    为什么必须用 mode="rate"
    ------------------------
    2026-10-20 12:5x 实际撞到：K8→K5 这一段，闸门（当时用 ratio 口径）判 +126 mm
    全段放行，实机跑到一半触发高度守卫原地抱住 —— 那时的真实位形
    (0,75.0,−33.1,42.9,22.9,0) **瓶底只有 30 mm**。

    原因是两条路径根本不是同一条：
      * ratio：q = qa + (qb−qa)·f，六轴同时按比例到；
      * rate ：各轴 2.5°/s 匀速，J2/J3（行程 15/20°）6~8 s 就到，
               J4/J5（行程 60/80°）要 24~32 s，中间那段臂是「伸出去、
               腕还没转回来」，瓶口朝下戳台面。
    """
    d = [b - a for a, b in zip(qa, qb)]
    pk = pack()
    vmax = max((abs(x) for x in d), default=0.0)
    worst = (1e9, 0.0)
    for i in range(1, n + 1):
        t = i / n
        q = _interp(qa, qb, t, mode)
        R6, p6, _ = tcp_and_axis(arm, q)
        zmin, clr = bottle_metrics(R6, p6)
        c = min(zmin - pk["zmin"], clr - pk["clr"])
        if c < worst[0]:
            worst = (c, t)
    return worst


# ---------------------------------------------------------------- 子命令
def load(urdf_path):
    urdf = S.Urdf(Path(urdf_path))
    arm = S.Arm(urdf, end_mass=GRIPPER_MASS, end_com=urdf.gripper_com())
    return urdf, arm


def cmd_list(a):
    urdf, arm = load(a.urdf)
    pk = set_pack("gripper" if a.no_bottle else "bottle")
    print(f"末端口径：夹爪 {GRIPPER_MASS:.3f} kg（质心 {urdf.gripper_com()[2]*1000:.1f} mm）"
          f" + 载荷包络 = {pk['label']}"
          f"（沿工具轴 {(-pk['back'])*1000:.0f}→{pk['length']*1000:.0f} mm，"
          f"半径 {pk['radius']*1000:.0f} mm）")
    print()
    print("| 姿态 | q(°) | 法兰z | 末端最低z | 末端净空 | τJ2 | τJ3 | τJ4 | τJ5 | 判 |")
    print("|" + " --- |" * 10)
    for name, qd in [(n, q) for n, q, *_ in S21]:
        q = deg(*qd)
        v = pose_verdict(arm, q)
        flag = "OK" if not v["bad"] else "✗ " + ";".join(v["bad"])
        print(f"| {name} | {','.join(map(str, qd))} | {v['p6'][2]*1000:.0f} | {v['zmin']:.0f} | "
              f"{v['clr']:.0f} | " + " ".join(f"{v['tt'][j]:+.2f}" for j in (1, 2, 3, 4))
              + f" | {flag} |")
    print()
    print("被带载判死的 S2-0 姿态（空载安全 ≠ 带载安全）：")
    for title, qd in [("P2 前伸下垂", (0, 75, -40, 0, 30, 0)),
                      ("P4′ 收拢", (0, 100, -75, 0, 60, 0))]:
        v = pose_verdict(arm, deg(*qd))
        print(f"  {title:12s} 法兰z={v['p6'][2]*1000:.0f} 瓶底z={v['zmin']:.0f}  "
              f"瓶净空={v['clr']:.0f}  →  {'✗ ' + ';'.join(v['bad']) if v['bad'] else 'OK'}")


def cmd_seq(a):
    urdf, arm = load(a.urdf)
    set_pack("gripper" if a.no_bottle else "bottle")
    names = list(ROUTE) if a.seq in ("", "ROUTE") else [x.strip() for x in a.seq.split(",") if x.strip()]
    bad = [n for n in names if n not in BY_NAME]
    if bad:
        print(f"[ABORT] 未知姿态 {bad}，可选 {list(BY_NAME)}", file=sys.stderr)
        return 2
    if a.start_from_can:
        start = _read_start(a.iface)
        if start is None:
            print("[ABORT] 读不到当前姿态（can0 未 UP 或没使能）", file=sys.stderr)
            return 2
        print(f"起点（固件实测）：{','.join(f'{x:+.1f}' for x in start)}")
    else:
        start = None

    print(f"路线校验：{' → '.join(names)}   （速率 {a.rate:g}°/s）")
    ok = True
    first_seg = True
    prev_name, prev_q = ("起始", [math.radians(x) for x in start] if start else None)
    for n in names:
        q = BY_NAME[n]
        v = pose_verdict(arm, q)
        print(f"[{n:3s}] q={','.join(f'{math.degrees(x):g}' for x in q)}  "
              f"法兰z={v['p6'][2]*1000:.0f} 瓶底z={v['zmin']:.0f} 瓶净空={v['clr']:.0f}")
        if v["bad"]:
            ok = False
            print(f"       ✗ {v['bad']}")
        else:
            print("       ✓ 静态合格")
        if prev_q is not None:
            c, f = path_min(arm, prev_q, q)
            base = 0.0
            if first_seg:
                # 首段豁免：起点本身可能在立柱上方的边缘位置（实测 −31 mm），
                # 那是臂现在的停放事实，不是我让它走的。规则降级为「不许比起点更糟」。
                R0, p0, _ = tcp_and_axis(arm, prev_q)
                z0, c0 = bottle_metrics(R0, p0)
                # 豁免只能**放宽**（起点本身不合格时），不能**抬高**门槛：
                # 起点若本来就宽敞（余量 +259 mm），首段仍应按 0 判，
                # 否则「从 A 到 B 只要 72 mm」会被判成不合格（12:3x 实际踩到）。
                base = min(0.0, min(z0 - pack()["zmin"],
                                    c0 - pack()["clr"]) - 5.0)
                print(f"       起点自有余量 {base + 5.0:+.0f} mm（豁免到 {base:+.0f} mm）")
                first_seg = False
            print(f"       路径 {prev_name}→{n}: 最差 f={f:.2f} 余量 {c:+.0f} mm "
                  f"{'✓' if c >= base else '✗'}")
            if c < base:
                ok = False
        prev_name, prev_q = n, q
    print()
    print("总判：", "全段放行" if ok else "有不合格项，不许上机")
    return 0 if ok else 2


def _read_start(iface):
    try:
        import time
        import piper_sdk
    except Exception as e:                       # noqa: BLE001
        print(f"[warn] 无法导入 piper_sdk: {e}", file=sys.stderr)
        return None
    p = piper_sdk.C_PiperInterface_V2(iface)
    p.ConnectPort()
    for _ in range(40):
        try:
            js = p.GetArmJointMsgs()
            q = [getattr(js.joint_state, f"joint_{i+1}") * 0.001 for i in range(6)]
            if any(abs(v) > 1e-9 for v in q):
                return q
        except Exception:                        # noqa: BLE001
            pass
        time.sleep(0.1)
    return None


def _selftest(arm, urdf):
    set_pack("bottle")          # 自测默认按带载口径跑，末尾单独验空载口径
    n_ok = n_bad = 0

    def chk(name, cond, detail=""):
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print(f"  ✓ {name}")
        else:
            n_bad += 1
            print(f"  ✗ {name}  {detail}")

    # A. J1 对重力恒零矩（结构事实）
    chk("A J1 自重力矩恒 0（所以 J1 必须横向拉）",
        max(abs(arm.tau_gravity(deg(*qs))[0]) for qs in
            [(0, 45, -60, 0, 45, 0), (-45, 30, -80, 0, 50, 0), (60, 100, -75, 0, 60, 0)]) < 1e-9)

    # B. 质心在工具轴上 ⇒ J6 恒零矩
    tp = arm.tau_point_force(deg(0, 45, -60, 0, 45, 90), S.END_LINK,
                             [0.0, 0.0, 0.09], [0.0, 0.0, -BOTTLE_MASS * S.G])
    chk("B 质心在工具轴上 ⇒ J6 载荷力矩 ≈ 0", abs(tp[5]) < 1e-6, f"{tp[5]:.2e}")

    # C. 载荷力矩对 d 线性
    q = deg(0, 45, -60, 0, 45, 0)
    t = [arm.tau_point_force(q, S.END_LINK, [0, 0, d], [0, 0, -BOTTLE_MASS * S.G])
         for d in (0.05, 0.10, 0.15)]
    chk("C 载荷力矩对 d 线性",
        all(abs((t[2][j] - t[1][j]) - (t[1][j] - t[0][j])) < 1e-9 for j in range(6)))

    # D. 事故姿态（0,100,-20）必须被判死
    chk("D 事故姿态 (0,100,-20) 被判不合格",
        bool(pose_verdict(arm, deg(0, 100, -20, 0, 60, 0))["bad"]))

    # E. 带载后 P4′/P2 必须被判死（空载安全 ≠ 带载安全）
    v4 = pose_verdict(arm, deg(0, 100, -75, 0, 60, 0))
    chk("E 带载 P4′ 被判死（瓶底压到台面）", bool(v4["bad"]), str(v4["bad"]))
    chk("E 带载 P4′ 的瓶底 < 60 mm", v4["zmin"] < 60.0, f"{v4['zmin']:.0f}")
    chk("E 带载 P2 被判死", bool(pose_verdict(arm, deg(0, 75, -40, 0, 30, 0))["bad"]))

    # F. S2-1 全部姿态必须合格
    bads = [n for n, q, *_ in S21 if pose_verdict(arm, deg(*q))["bad"]]
    chk(f"F S2-1 全部 {len(S21)} 个姿态静态合格", not bads, str(bads))

    # G. 路线每一段路径都合格
    qs = [BY_NAME[n] for n in ROUTE]
    worst = min(path_min(arm, a, b)[0] for a, b in zip(qs, qs[1:]))
    chk("G 路线各段路径余量 ≥ 0", worst >= 0, f"{worst:+.0f} mm")

    # H. 力矩跨度够拟合（J2 必须跨零，J6 必然为零）
    j2 = [tau_all(arm, BY_NAME[n])[2][1] for n in BY_NAME]
    chk("H J2 力矩跨零且跨度 > 4 N·m", min(j2) < 0 < max(j2) and max(j2) - min(j2) > 4.0,
        f"{min(j2):.2f}..{max(j2):.2f}")
    j6 = [tau_all(arm, BY_NAME[n])[2][5] for n in BY_NAME]
    # 不是「精确 0」而是「1e-4 量级」：臂自身腕部质心离工具轴有一点点偏，
    # 但相对电流噪声（0.014 A × K 3.7 ≈ 0.05 N·m）是 1/500，结构上标不出来。
    chk("H J6 名义力矩 ≈ 0（质心在轴上 ⇒ 结构上标不出来）",
        max(abs(v) for v in j6) < 2e-3, f"max|τ6|={max(abs(v) for v in j6):.2e}")

    # I. 路径函数确实豁免起点
    same = path_min(arm, BY_NAME["K12"], BY_NAME["K12"])
    chk("I 同点路径不报错且余量 = 静态余量", same[0] > 0, str(same))

    # J. 回归锚点：K8→K5 必须被 rate 口径判死，而 ratio 口径会漏判。
    #    这是 2026-10-20 12:5x 真实撞到的段（实机中途瓶底 30 mm 触发守卫）。
    #    以后谁把 path_min 改回等比插值，这条会立刻红。
    rr, _ = path_min(arm, BY_NAME["K8"], BY_NAME["K5"], mode="ratio")
    rt, _ = path_min(arm, BY_NAME["K8"], BY_NAME["K5"], mode="rate")
    chk("J K8→K5：rate 判死而 ratio 漏判（插值口径不可退回）",
        rt < 0 < rr, f"ratio {rr:+.0f} / rate {rt:+.0f}")

    # K. 两套载荷包络必须真的不同，且各自判对。
    #    锚点是 2026-10-20 12:45 断电后臂的真实落点：竖直收拢、法兰 z=183.5、
    #    夹爪向前下 14° 掠过立柱顶角、最近 18.7 mm。当时臂上**没有水瓶**，
    #    这个位置物理上确实没碰到任何东西。
    #    注意断言的是「首段豁免后能否走通」，不是「pose_verdict 放不放行」——
    #    pose_verdict 只报事实，起点是既成事实，闸门管的是「不许比起点更糟」。
    DANGLING = deg(0.08, -0.16, -0.21, 3.13, 19.74, 0.0)
    set_pack("bottle")
    bad_bt = pose_verdict(arm, DANGLING)["bad"]
    set_pack("gripper")
    bads_gr = [n for n, q, *_ in S21 if pose_verdict(arm, deg(*q))["bad"]]
    chk("K 空载口径：S2-1 全部目标姿态静态合格", not bads_gr, str(bads_gr))
    z0, c0 = bottle_metrics(*tcp_and_axis(arm, DANGLING)[:2])
    base = min(0.0, min(z0 - pack()["zmin"], c0 - pack()["clr"]) - 5.0)
    c_gr, _ = path_min(arm, DANGLING, BY_NAME["K9R"])
    chk(f"K 空载口径：断电落点→K9R 首段经豁免可走（起点自身 {base + 5.0:+.0f} mm 是事实）",
        c_gr >= base, f"段余量 {c_gr:+.0f} / 豁免到 {base:+.0f}")
    chk("K 带载口径判死同一落点（18 cm 瓶身不存在也不许假设它不存在）",
        bool(bad_bt), str(bad_bt))
    set_pack("bottle")

    print(f"\n自测：{n_ok} 过 / {n_bad} 败")
    return 0 if n_bad == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="S2-1 姿态选型与带载几何闸门")
    ap.add_argument("--urdf", default=str(Path(__file__).resolve().parent.parent /
                                         "_vendor" / "urdf" / "piper_description.urdf"))
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--seq", nargs="?", const="ROUTE", default="",
                    help="逗号分隔的姿态序列；不带值 = 内置 ROUTE")
    ap.add_argument("--start-from-can", action="store_true")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--rate", type=float, default=3.0)
    ap.add_argument("--no-bottle", action="store_true",
                    help="空载轮：载荷包络只算夹爪（法兰→指尖 110 mm），不再按 22 cm 水瓶判")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)

    if a.selftest:
        urdf, arm = load(a.urdf)
        return _selftest(arm, urdf)
    if a.seq or a.start_from_can:
        return cmd_seq(a)
    cmd_list(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
