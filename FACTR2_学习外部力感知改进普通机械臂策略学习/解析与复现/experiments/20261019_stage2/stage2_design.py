#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stage2_design.py 1.0 — 阶段 2（力矩尺度核验）的离线设计与空载对照

两个子命令，都**不动机械臂**：

  sweep   用 URDF 的重力模型，算出「加多大砝码 / 拉多大横向力，各关节的 Δτ 是多少」，
          并给出可辨识性矩阵：哪些姿态能激励哪些关节，哪些关节静态砝码**结构上等于 0**。
          直接用来决定买什么砝码、要不要做滑轮工装。

  check   用一次 `stage1_cmd.py hold` 的静态录制，对照 URDF 重力预测与实测电流，
          反推「隐含的 K」，顺带给出电流噪声底与零点。这是不花钱就能做的第一步。

为什么需要它（指南 §5 的两个硬约束）：
  1. `delta_tau_ref = J^T @ wrench`。静态砝码的 wrench 是竖直重力，绕**竖直轴**的关节
     （J1）恒等于 0；若载荷质心落在 J4/J6 的轴上，那两个关节的 Δτ 也≈0。
     不先把这件事算出来，就会买一堆砝码、做完一轮、才发现有三个关节标不出来。
  2. 可辨识性必须在**设计阶段**定，不能等拟合时才发现某轴 Δτ 只有噪声的 2 倍。

口径与边界（读结论前先看这三条）：
  - **只给幅值与可辨识性，不给符号**。URDF 的 q 正方向与固件 `0x251/0x2A5` 的符号
    没有逐关节核验过；符号必须由实测（正反方向加载）确定，指南 §4.2 坑二。
  - 重力取 g = 9.81，方向世界 -z。URDF 的 base_link 是竖直向上的基座坐标系。
  - 惯性参数取自 `piper_description.urdf`，与固件限位无关；本工具不碰限位判据（R38）。

用法：
    python stage2_design.py sweep
    python stage2_design.py sweep --mass 1.0 --pull 10 --json out.json
    python stage2_design.py --gripper none sweep    # 对照：假想拆掉夹爪
    python stage2_design.py --gripper-mass 0.52 sweep   # 称重后按实测夹爪质量刷新
    python stage2_design.py check --samples episodes/l0a_hold/samples.jsonl
    python stage2_design.py check --samples episodes/x/samples.jsonl --payload-kg 0.5
    python stage2_design.py selftest
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

VERSION = "1.2-20261019"

G = 9.81

# SDK 内置的固定换算系数（piper_sdk 0.6.1
# piper_msgs/msg_v2/feedback/arm_feedback_high_spd.py:84 `cal_effort`）
# effort(0.001 N·m) = current_raw * coefficient  ⇒  K_SDK [N·m/A] = coefficient
K_SDK = {1: 1.18125, 2: 1.18125, 3: 1.18125, 4: 0.95844, 5: 0.95844, 6: 0.95844}

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
END_LINK = "link6"

# URDF 里的夹爪三件（质量 0.45 + 0.025 + 0.025 = 0.50 kg）。
# 本复现已定案「不装夹爪」，所以重力模型的默认口径是**裸臂**（这三件置 0）。
GRIPPER_LINKS = ("gripper_base", "link7", "link8")

# 电流量化：0x251 字节2-3，uint16，单位 0.001 A ⇒ 1 count = 0.001 A
CUR_LSB_A = 0.001

# 判据：ΔI 至少是量化底的多少倍才算「可辨识」。
# 20 倍 = 0.02 A ≈ 0.024 N·m。取 20 而不是 3，是因为静态差分的残差主要不是量化，
# 而是摩擦滞回与温漂（指南 §5.2 第 4 步要求卸载复测观察回差）。
IDENT_MIN_COUNTS = 20


# ---------------------------------------------------------------- 线代小工具

def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rpy_to_mat(r, p, y):
    """URDF 的 rpy 是固定轴 roll-pitch-yaw ⇒ R = Rz(y) @ Ry(p) @ Rx(r)。"""
    return mat_mul(mat_mul(rot_z(y), rot_y(p)), rot_x(r))


def axis_angle_to_mat(axis, theta):
    """罗德里格斯公式。axis 已归一化。"""
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    C = 1 - c
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def normalize(v):
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-12:
        raise ValueError("零向量不能当转轴")
    return [x / n for x in v]


# ---------------------------------------------------------------- URDF

class Urdf:
    """只提取本工具要用的三样：关节链、连杆质心、连杆质量。"""

    def __init__(self, path: Path):
        root = ET.parse(str(path)).getroot()
        self.mass = {}
        self.com = {}
        for link in root.findall("link"):
            name = link.get("name")
            ine = link.find("inertial")
            if ine is None:
                continue
            m = ine.find("mass")
            o = ine.find("origin")
            if m is None:
                continue
            xyz = [0.0, 0.0, 0.0] if o is None else [float(v) for v in o.get("xyz").split()]
            self.mass[name] = float(m.get("value"))
            self.com[name] = xyz

        # URDF 自带夹爪模型：gripper_base 0.45 + 两指 0.025×2 = 0.50 kg。
        # 这一点很容易看错——三件的 <mass 是跨行写的（`<mass\n value=...`），
        # 用单行正则扫会全部漏掉，从而误以为 URDF 是「裸臂」。
        # 实测校核：4.66 − 0.50 = 4.16 kg ≈ 官方本体 4.2 kg ⇒ URDF = 臂 + 夹爪，不是裸臂。
        self.gripper_mass = sum(self.mass.get(lk, 0.0) for lk in GRIPPER_LINKS)

        self.joint = {}
        for j in root.findall("joint"):
            name = j.get("name")
            o = j.find("origin")
            ax = j.find("axis")
            xyz = [0.0, 0.0, 0.0] if o is None else [float(v) for v in o.get("xyz").split()]
            rpy = [0.0, 0.0, 0.0] if o is None else [float(v) for v in o.get("rpy").split()]
            axis = [0.0, 0.0, 1.0] if ax is None else [float(v) for v in ax.get("xyz").split()]
            lim = j.find("limit")
            self.joint[name] = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": xyz,
                "rpy": rpy,
                "axis": normalize(axis),
                "lower": None if lim is None else float(lim.get("lower", "nan")),
                "upper": None if lim is None else float(lim.get("upper", "nan")),
            }

    def gripper_com(self):
        """夹爪三件的组合质心，坐标在 link6 帧。

        `joint6_to_gripper_base` 的 origin 是 0，所以 gripper_base 帧 = link6 帧；
        `joint7/joint8` 是 prismatic，位移取 0（闭合位）。两指对称 ⇒ 横向分量抵消，
        结果落在工具轴上，距法兰约 37.6 mm —— 不能当「法兰原点上的 0.5 kg」。
        """
        T = {"link6": ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])}
        tot, acc = 0.0, [0.0, 0.0, 0.0]
        for jn in ("joint6_to_gripper_base", "joint7", "joint8"):
            j = self.joint[jn]
            Rp, pp = T[j["parent"]]
            Rj = mat_mul(Rp, rpy_to_mat(*j["rpy"]))
            pj = [pp[k] + mat_vec(Rp, j["xyz"])[k] for k in range(3)]
            T[j["child"]] = (Rj, pj)
            lk = j["child"]
            m = self.mass.get(lk, 0.0)
            if m > 0:
                c = [pj[k] + mat_vec(Rj, self.com[lk])[k] for k in range(3)]
                tot += m
                acc = [acc[k] + m * c[k] for k in range(3)]
        return None if tot <= 0 else [v / tot for v in acc]

    def chain_links(self):
        """沿 joint1..joint6 走一遍，返回 [(joint_name, link_name)]，用于 FK 与质量累加。"""
        out = []
        link = self.joint["joint1"]["parent"]
        for jn in JOINTS:
            j = self.joint[jn]
            if j["parent"] != link:
                raise ValueError(f"URDF 链断了：{jn} 的 parent={j['parent']}，但上一个是 {link}")
            link = j["child"]
            out.append((jn, link))
        return out


# ---------------------------------------------------------------- 运动学

class Arm:
    def __init__(self, urdf: Urdf, end_mass: float = 0.0, end_com=None):
        self.u = urdf
        self.chain = urdf.chain_links()
        # 参与重力矩的连杆：从 link1 到 link6（base_link 固定，不产生关节力矩）。
        # 注意：chain 在 link6 就结束了，所以 URDF 里的 gripper_base/link7/link8
        # （合计 0.5 kg）**天然不计入**。法兰上的东西只能靠 end_mass 显式加，
        # 不让它变成「看代码才知道」的隐式行为。
        self.links = [lk for (_, lk) in self.chain if lk in urdf.mass]
        self.end_mass = end_mass
        # 末端质量的质心（link6 帧）。默认原点；带夹爪时用 URDF 算出的 37.6 mm。
        self.end_com = [0.0, 0.0, 0.0] if end_com is None else list(end_com)

    def transforms(self, q):
        """q: 6 个关节角（rad，URDF 口径）。返回 {link: (R, p)}。"""
        T = {"base_link": ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0])}
        for idx, (jn, child) in enumerate(self.chain):
            j = self.u.joint[jn]
            Rp, pp = T[j["parent"]]
            # 关节坐标系 = parent * Trans(xyz) * RPY(rpy)
            Rj = mat_mul(Rp, rpy_to_mat(*j["rpy"]))
            pj = [pp[k] + mat_vec(Rp, j["xyz"])[k] for k in range(3)]
            # 再绕 axis 转 q
            Rc = mat_mul(Rj, axis_angle_to_mat(j["axis"], q[idx]))
            T[child] = (Rc, pj)
        return T

    def point_world(self, q, link, local):
        T = self.transforms(q)
        R, p = T[link]
        w = mat_vec(R, local) if local is not None else None
        if w is None:
            return p
        return [p[k] + w[k] for k in range(3)]

    def _z_of_link_coms(self, q):
        T = self.transforms(q)
        z = {}
        for lk in self.links:
            R, p = T[lk]
            z[lk] = p[2] + mat_vec(R, self.u.com[lk])[2] if lk != "link1" else p[2]
            # link1 的质心在 link1 帧，与上面同式；显式分支只为可读
            if lk != "link1":
                pass
        return z

    def tau_gravity(self, q):
        """机械臂自身重力矩（保持所需关节力矩），N·m，符号为 URDF q 的正方向。

        tau_j = d/dq_j ( sum_i m_i * g * z_i )，用中心差分求，避免逐关节写解析式。
        """
        h = 1e-6
        out = [0.0] * 6
        for j in range(6):
            qp = list(q)
            qm = list(q)
            qp[j] += h
            qm[j] -= h
            zp = self._z_of_link_coms(qp)
            zm = self._z_of_link_coms(qm)
            s = 0.0
            for lk in self.links:
                s += self.u.mass[lk] * G * (zp[lk] - zm[lk]) / (2 * h)
            out[j] = s
        if self.end_mass:
            # 末端质量按「质心落在法兰原点」处理（[0,0,0]）：它挂在 link6 上，
            # 位置随 q 变，所以必须走 J^T 而不是当常数。
            tq = self.tau_point_force(q, END_LINK, self.end_com,
                                      [0.0, 0.0, -self.end_mass * G])
            out = [out[k] + tq[k] for k in range(6)]
        return out

    def tau_point_force(self, q, link, local, force_world):
        """外部点力对关节的力矩 J^T F，N·m。force_world 是世界系下的力向量。"""
        h = 1e-6
        out = [0.0] * 6
        for j in range(6):
            qp = list(q)
            qm = list(q)
            qp[j] += h
            qm[j] -= h
            pp = self.point_world(qp, link, local)
            pm = self.point_world(qm, link, local)
            s = 0.0
            for k in range(3):
                s += force_world[k] * (pp[k] - pm[k]) / (2 * h)
            out[j] = s
        return out


# ---------------------------------------------------------------- 姿态表

def deg(*a):
    return [math.radians(x) for x in a]


# 固件实测限位（0x473，13:24）：J1 ±150 / J2 0~180 / J3 -170~0 / J4 ±100 / J5 ±70 / J6 ±180
# 这里每个姿态都留 15° 以上余量，且远离奇异（J5≠0、肘不完全伸直）。
POSES = [
    ("P1 前伸水平", deg(0, 45, -60, 0, 45, 0), "经典标定姿态：力臂最长，J2/J3 激励最强"),
    ("P2 前伸下垂", deg(0, 75, -40, 0, 30, 0), "J2 大角度，重力臂换成 J3 主导"),
    ("P3 侧向水平", deg(60, 45, -60, 0, 45, 0), "J1 转出 60°，改变重力在各轴的分配"),
    ("P4 收拢竖直", deg(0, 100, -20, 0, 60, 0), "接近竖直，J3 力矩趋近 0（反向激励）"),
    ("P5 上举", deg(-45, 30, -80, 0, 50, 0), "J2 负向激励（重力矩反号）"),
    ("P6 腕部扭转", deg(0, 60, -50, 60, 40, 0), "J4 转 60°，检验 J4 是否被激励"),
    ("P7 腕自转90", deg(0, 45, -60, 0, 45, 90), "J6 自转 90°，把偏心质量转到能产生力矩的方位"),
    ("P8 腕双转", deg(0, 60, -50, 90, 40, 90), "J4 与 J6 同时转开，检验两轴是否解耦"),
    # J6 的力矩 ∝ sin(q6)：同一个 0.5 kg 砝码，靠自转角就能造出连续的 Δτ 阶梯，
    # 不必再买第二档砝码。30°/60° 是「几何变载」的两个中间档（P7=90° 是满量程）。
    ("P9 腕自转30", deg(0, 45, -60, 0, 45, 30), "J6 变载档：sin30° ⇒ 约满量程的 1/2"),
    ("P10 腕自转60", deg(0, 45, -60, 0, 45, 60), "J6 变载档：sin60° ⇒ 约满量程的 6/7"),
]


# ---------------------------------------------------------------- sweep

def fmt(v, nd=3):
    return "—" if v is None else f"{v:+.{nd}f}"


def load_arm(a):
    """按 `--gripper` 口径装载重力模型。

    none（默认）= 裸臂，法兰上什么都不挂 —— 与「本次复现不用夹爪」的定案一致；
    urdf = 把 URDF 描述的 0.5 kg 夹爪当作法兰载荷，仅用于对照。
    """
    urdf = Urdf(Path(a.urdf))
    if getattr(a, "gripper", "none") == "urdf":
        mass = getattr(a, "gripper_mass", None) or urdf.gripper_mass
        return urdf, Arm(urdf, end_mass=mass, end_com=urdf.gripper_com())
    return urdf, Arm(urdf, end_mass=0.0)


def endpoint_note(a, urdf):
    """口径必须打在报告最前面 —— 差 0.5 kg 会让 J5 的自重力矩反号。"""
    if a.gripper == "urdf":
        m = getattr(a, "gripper_mass", None) or urdf.gripper_mass
        c = urdf.gripper_com()
        tag = "" if abs(m - urdf.gripper_mass) < 1e-9 else "（质量用实测值覆盖）"
        return (f"**带夹爪 {m:.3f} kg**{tag}，质心在工具轴上 {c[2]*1000:.1f} mm "
                f"—— 与现场一致")
    return (f"**裸臂**（法兰空载，已排除 URDF 自带的 {urdf.gripper_mass:.2f} kg 夹爪）"
            f" —— 仅对照，与现场不符")


def cmd_sweep(a):
    urdf, arm = load_arm(a)
    # 载荷悬挂点在 link6 帧：z 是工具轴（= J6 转轴），x 是偏心横杆方向。
    # 只给 z 外伸时，载荷质心正好落在 J6 的转轴上 ⇒ J6 的 Δτ 恒为 0（自测 T9 验这条）。
    flange_local = [a.lever, 0.0, a.flange]

    payload_force = [0.0, 0.0, -a.mass * G]

    rows = []
    cover_g = {j + 1: [] for j in range(6)}
    cover_p = {j + 1: [] for j in range(6)}

    for label, q, note in POSES:
        tg = arm.tau_gravity(q)
        dta_g = arm.tau_point_force(q, END_LINK, flange_local, payload_force)
        # 滑轮方向取「绕 J1 力矩最大」的切向：t = ẑ × r_水平。
        # 顺着臂拉（径向）对 J1 的力臂恰好为 0 —— 这是自测 T6a 验的那条几何事实。
        p = arm.point_world(q, END_LINK, flange_local)
        rh = [p[0], p[1], 0.0]
        nrh = math.sqrt(rh[0] ** 2 + rh[1] ** 2)
        if nrh > 1e-6:
            tan = [-rh[1] / nrh, rh[0] / nrh, 0.0]
        else:
            tan = [1.0, 0.0, 0.0]
        pull_force = [a.pull * tan[0], a.pull * tan[1], 0.0]
        dta_p = arm.tau_point_force(q, END_LINK, flange_local, pull_force)
        row = {"pose": label, "note": note, "q_deg": [round(math.degrees(v), 1) for v in q],
               "tau_gravity_own": [round(v, 3) for v in tg], "payload": {}, "pull": {}}
        for j in range(6):
            kg = K_SDK[j + 1]
            di_g = dta_g[j] / kg
            di_p = dta_p[j] / kg
            ok_g = abs(di_g) / CUR_LSB_A >= IDENT_MIN_COUNTS
            ok_p = abs(di_p) / CUR_LSB_A >= IDENT_MIN_COUNTS
            row["payload"][f"J{j+1}"] = {"dtau_nm": round(dta_g[j], 4),
                                         "di_a": round(di_g, 5), "identifiable": ok_g}
            row["pull"][f"J{j+1}"] = {"dtau_nm": round(dta_p[j], 4),
                                      "di_a": round(di_p, 5), "identifiable": ok_p}
            if ok_g:
                cover_g[j + 1].append(label.split()[0])
            if ok_p:
                cover_p[j + 1].append(label.split()[0])
        rows.append(row)

    # ---- 报告
    print(f"# 阶段 2 工装设计表（stage2_design {VERSION}）")
    print()
    print(f"- 载荷 **{a.mass:.2f} kg**，作用点 = 法兰沿工具轴外伸 **{a.flange*1000:.0f} mm**"
          + (f"，偏心横杆 **{a.lever*1000:.0f} mm**" if a.lever else "，**无偏心横杆**"))
    print(f"- 横向拉力 **{a.pull:.1f} N**，方向取绕 J1 力矩最大的**切向**（模拟滑轮/弹簧秤）")
    print(f"- 重力模型末端口径：{endpoint_note(a, urdf)}")
    print(f"- K 初值取 SDK 固定系数：J1–J3 **{K_SDK[1]}**、J4–J6 **{K_SDK[4]}** N·m/A")
    print(f"- 可辨识判据 |ΔI| ≥ {IDENT_MIN_COUNTS} counts（= {IDENT_MIN_COUNTS*CUR_LSB_A:.3f} A ≈ "
          f"{IDENT_MIN_COUNTS*CUR_LSB_A*K_SDK[1]:.3f} N·m）")
    print()
    print("## 1 静态砝码（竖直重力）的 Δτ / ΔI")
    print()
    print("| 姿态 | q(°) | " + " | ".join(f"J{j} Δτ/ΔI" for j in range(1, 7)) + " |")
    print("| --- | --- | " + " | ".join("---" for _ in range(6)) + " |")
    for r in rows:
        cells = []
        for j in range(1, 7):
            d = r["payload"][f"J{j}"]
            mark = "" if d["identifiable"] else "·"
            cells.append(f"{d['dtau_nm']:+.2f}/{d['di_a']:+.3f}{mark}")
        print(f"| {r['pose']} | {r['q_deg']} | " + " | ".join(cells) + " |")
    print()
    print("单位：Δτ 为 N·m，ΔI 为 A。**·** = 低于可辨识判据（不是小，是结构上接近 0 或力臂太短）。")
    print()
    print("## 2 横向拉力（滑轮）的 Δτ / ΔI")
    print()
    print("| 姿态 | " + " | ".join(f"J{j}" for j in range(1, 7)) + " |")
    print("| --- | " + " | ".join("---" for _ in range(6)) + " |")
    for r in rows:
        cells = []
        for j in range(1, 7):
            d = r["pull"][f"J{j}"]
            mark = "" if d["identifiable"] else "·"
            cells.append(f"{d['dtau_nm']:+.2f}{mark}")
        print(f"| {r['pose']} | " + " | ".join(cells) + " |")
    print()
    print("## 3 可辨识性覆盖（只有打勾的关节能在该姿态下标出来）")
    print()
    print("| 关节 | K_SDK | 静态砝码可激励的姿态 | 横向拉力可激励的姿态 | 结论 |")
    print("| --- | --- | --- | --- | --- |")
    for j in range(1, 7):
        g = cover_g[j]
        p = cover_p[j]
        if g and p:
            verdict = "两种都行，优先砝码（力值可溯源）"
        elif g:
            verdict = "**只能用砝码**"
        elif p:
            verdict = "**必须横向拉**（重力绕该轴不产生力矩）"
        else:
            verdict = "⚠️ 本表两种都激励不到 → 需另设姿态"
        print(f"| J{j} | {K_SDK[j]} | {', '.join(g) if g else '—'} | "
              f"{', '.join(p) if p else '—'} | {verdict} |")
    print()
    print("## 4 机械臂自身重力矩（旁证：K 量级是否合理）")
    print()
    print("| 姿态 | " + " | ".join(f"J{j}" for j in range(1, 7)) + " |")
    print("| --- | " + " | ".join("---" for _ in range(6)) + " |")
    for r in rows:
        print(f"| {r['pose']} | " + " | ".join(f"{v:+.2f}" for v in r["tau_gravity_own"]) + " |")
    print()
    print("这一栏是**空载保持臂本身**所需的关节力矩（N·m）。`check` 子命令就是拿它和实测")
    print("电流比，若隐含 K 落在 SDK 系数附近（±30%），说明 K 初值可信、符号自洽；")
    print("差一个数量级则先查电流量纲（0.001 A）与零点，不要急着上砝码。")
    print()
    print("## 5 结论与工装清单")
    print()
    must_pull = [j for j in range(1, 7) if not cover_g[j] and cover_p[j]]
    none_at_all = [j for j in range(1, 7) if not cover_g[j] and not cover_p[j]]
    for j in must_pull:
        print(f"- **J{j} 静态砝码标不出来**（重力绕该轴不产生力矩），必须横向拉。"
              f"可用 {', '.join(cover_p[j])}。")
    for j in none_at_all:
        print(f"- **J{j} 本表两种都激励不到**：载荷质心落在该关节的转轴上。"
              f"需要偏心横杆（`--lever`）把质心推离轴线，或改施力方向。")
    need_weights = [j for j in range(1, 7) if cover_g[j]]
    if need_weights:
        print(f"- 静态砝码可覆盖 J{', J'.join(str(j) for j in need_weights)}；"
              f"建议用 **{a.mass:.1f} kg 与 {a.mass/2:.2f} kg 两档**做差分，"
              f"力臂取实测值，不要用设计值。")
    # 每轴的最强姿态：同一条判据下，力臂能差 10 倍以上（J1 的 P1 1.50 vs P4 0.17），
    # 选错姿态会让信噪比白掉一个数量级。
    best = []
    for j in range(1, 7):
        cg = max((abs(r["payload"][f"J{j}"]["dtau_nm"]), r["pose"].split()[0]) for r in rows)
        cp = max((abs(r["pull"][f"J{j}"]["dtau_nm"]), r["pose"].split()[0]) for r in rows)
        if cg >= cp:
            best.append(f"J{j}→{cg[1]}(砝码 {cg[0]:.2f})")
        else:
            best.append(f"J{j}→{cp[1]}(拉力 {cp[0]:.2f})")
    print(f"- **最强姿态**（括号内是该方式下的 |Δτ| N·m）：{'、'.join(best)}。"
          f"标定时优先用这些姿态，别随便挑 —— 同一关节不同姿态的力臂能差 10 倍。")
    print("- 每个姿态都要有**卸载复测**（指南 §5.2 第 4 步）：Δτ 相同、ΔI 不同 ⇒ 那是回差，先记下来。")
    print("- 姿态表里的 q 是 **URDF 口径**，与固件 `0x2A5` 的符号关系尚未逐轴核验；")
    print("  上机时以实测姿态为准，本表只用来保证「力臂够长、关节被激励」。")

    if a.json:
        Path(a.json).write_text(json.dumps({"version": VERSION, "mass_kg": a.mass,
                                            "pull_n": a.pull, "flange_m": a.flange,
                                            "k_sdk": K_SDK, "poses": rows},
                                           ensure_ascii=False, indent=2) + "\n",
                                encoding="utf-8")
        print(f"\n[json] {a.json}")
    return 0


# ---------------------------------------------------------------- check

def cmd_check(a):
    """静态录制的空载对照：q → URDF 重力矩；cur → 隐含 K。"""
    path = Path(a.samples)
    if not path.exists():
        print(f"[ABORT] 找不到 {path}", file=sys.stderr)
        return 2

    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        q = r.get("q")
        if not q or any(v is None for v in q):
            continue
        rows.append((r.get("t"), q, r.get("cur") or [None] * 6))
    n_all = len(rows)
    if n_all == 0:
        print("[ABORT] 没有完整六轴 q 的样本", file=sys.stderr)
        return 2

    # ---- 只看静止段 ----
    # 录的是「斜坡 + 保持」整段：把 120 s 全平均进去，姿态是斜坡中点的、电流是
    # 「边动边保持」的混合值 —— 2026-10-20 11:4x 第一版就吃了这个亏（J2 反推出 K=18，
    # 因为斜坡段 J2 一直在动）。这里先按时间窗截尾，再按「离中位数有多远」筛静止点。
    n_window = n_all
    last_s = getattr(a, "last_s", 0.0) or 0.0        # selftest 的 Namespace 没有这两项
    stable_deg = getattr(a, "stable_deg", 0.3) or 0.3
    if last_s > 0:
        ts = [r[0] for r in rows if r[0] is not None]
        if ts:
            t_end = max(ts)
            rows = [r for r in rows if r[0] is not None and r[0] >= t_end - last_s]
            n_window = len(rows)
    med = [sorted(r[1][j] for r in rows)[len(rows) // 2] for j in range(6)]
    tol = math.radians(stable_deg)
    stable = [r for r in rows
              if all(abs(r[1][j] - med[j]) <= tol for j in range(6))]
    n_drop = len(rows) - len(stable)
    if len(stable) < 20:
        print(f"[ABORT] 静止样本只有 {len(stable)} 条（窗口 {len(rows)} 条，"
              f"容差 {stable_deg:g}°）——加长保持段时间或放宽容差", file=sys.stderr)
        return 2
    rows = stable

    qs = [r[1] for r in rows]
    curs = [[r[2][j] for r in rows if r[2][j] is not None] for j in range(6)]
    n = len(rows)

    q_mean = [sum(v[j] for v in qs) / len(qs) for j in range(6)]
    spread = [max(v[j] for v in qs) - min(v[j] for v in qs) for j in range(6)]

    urdf, arm = load_arm(a)
    tg = arm.tau_gravity(q_mean)

    payload = None
    if a.payload_kg:
        payload = arm.tau_point_force(q_mean, END_LINK, [0.0, 0.0, a.flange],
                                      [0.0, 0.0, -a.payload_kg * G])

    print(f"# 空载静态对照（stage2_design {VERSION}）")
    print()
    print(f"- 原始样本 {n_all} 条；取静止段 {n} 条"
          + (f"（尾窗 {last_s:g} s 内 {n_window} 条，剔除离群 {n_drop} 条，"
             f"容差 {stable_deg:g}°）" if last_s > 0 else
             f"（剔除离群 {n_drop} 条，容差 {stable_deg:g}°）"))
    print("- 静止段姿态稳定性：各轴 q 峰峰 "
          + " ".join(f"J{j+1}={math.degrees(spread[j]):.3f}°" for j in range(6)))
    print(f"- 姿态 q = " + " ".join(f"{math.degrees(v):+.1f}°" for v in q_mean))
    if payload:
        print(f"- 已计入载荷 {a.payload_kg:.2f} kg（外伸 {a.flange*1000:.0f} mm）")
    print()
    print("| 关节 | 实测 cur 均值 A | cur 噪声(σ) A | 噪声 counts | URDF 重力矩 N·m | 隐含 K N·m/A | K_SDK | 比值 |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    bad = 0
    sign_bad = 0
    for j in range(6):
        c = curs[j]
        if not c:
            print(f"| J{j+1} | 无 cur | — | — | — | — | {K_SDK[j+1]} | — |")
            continue
        mu = sum(c) / len(c)
        var = sum((x - mu) ** 2 for x in c) / max(1, len(c) - 1)
        sd = math.sqrt(var)
        tau = tg[j] + (payload[j] if payload else 0.0)
        # |τ| 太小的关节在这里没有信息量：比值 = 噪声/噪声，看起来能算其实无意义。
        if abs(tau) < 0.1:
            print(f"| J{j+1} | {mu:+.4f} | {sd:.4f} | {sd/CUR_LSB_A:.1f} | {tau:+.3f} | "
                  f"— | {K_SDK[j+1]} | 无激励 |")
            continue
        k_impl = tau / mu if abs(mu) > 1e-6 else None
        ratio = (k_impl / K_SDK[j + 1]) if k_impl else None
        # 负比值 = cur 与预测力矩**符号相反**。这不是精度问题，是符号约定错了，
        # 后面所有 Δτ = K·ΔI 都会整体翻号，必须立刻停下。
        if ratio is not None and ratio < 0:
            sign_bad += 1
            verdict = "⚠️ 符号相反"
        elif ratio is None:
            verdict = "—"
        else:
            verdict = f"{ratio:+.2f}"
            if not (0.5 <= ratio <= 2.0):
                bad += 1
        print(f"| J{j+1} | {mu:+.4f} | {sd:.4f} | {sd/CUR_LSB_A:.1f} | {tau:+.3f} | "
              f"{'—' if k_impl is None else f'{k_impl:+.3f}'} | {K_SDK[j+1]} | {verdict} |")

    print()
    print(f"**噪声底**：σ 的 counts 列 = 电流量化之上的真实抖动。静态差分的 ΔI 至少要 >10×σ 才值得信任。")
    print()
    if sign_bad:
        print(f"❌ {sign_bad} 个关节的 `cur` 与预测力矩**符号相反**。"
              f"先停下来确认电流是否按无符号解（uint16）读的，别继续。")
    elif bad:
        print(f"⚠️ {bad} 个关节的隐含 K 与 SDK 系数差 2 倍以上。**先查电流量纲与零点**，"
              f"不要直接用它做标定初值。")
    else:
        print("✅ 有激励的关节，隐含 K 与 SDK 系数在 2 倍以内且同号 —— "
              "K 初值方向正确，可以上砝码做正式标定。")
    print()
    print("注意：静态保持时电流 = 重力矩 + 静摩擦 + 零点偏置，隐含 K 因此**系统性偏高/偏低**。")
    print("这一步只回答「量级对不对、符号自不自洽」，不产出可用的 K（那必须靠差分）。")
    return 0


# ---------------------------------------------------------------- selftest

def cmd_selftest(a):
    ok = True

    def chk(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")

    urdf, arm = load_arm(a)
    chk("T1 链完整", [lk for (_, lk) in arm.chain] == ["link1", "link2", "link3",
                                                      "link4", "link5", "link6"])
    chk("T2 六轴质量齐全", all(lk in urdf.mass for lk in arm.links))

    # 零位重力矩：J1 必须恒 0（重力绕竖直轴不产生力矩）
    tg0 = arm.tau_gravity([0.0] * 6)
    chk("T3 零位 J1 重力矩=0", abs(tg0[0]) < 1e-9, f"{tg0[0]:.3e}")

    # J1 任意角度重力矩仍为 0（结构性质，与 q1 无关）
    tgs = [arm.tau_gravity([math.radians(30), 0.0, 0.0, 0.0, 0.0, 0.0])[0] for _ in range(1)]
    chk("T4 J1 转角后仍为 0", abs(tgs[0]) < 1e-9, f"{tgs[0]:.3e}")

    # 竖直向下的点力对 J1 不产生力矩：与 T3 互为独立验证
    f = arm.tau_point_force([0.0] * 6, END_LINK, [0.0, 0.0, 0.1], [0.0, 0.0, -9.81])
    chk("T5 竖直载荷对 J1 无矩", abs(f[0]) < 1e-9, f"{f[0]:.3e}")

    # 横向力对 J1 的力矩只在**切向**才有：零位时臂落在 xz 平面，+x 方向的力力臂为 0，
    # +y 方向才有力矩。这一条同时验证「为什么滑轮要拉切向，不能顺着臂拉」。
    f_rad = arm.tau_point_force([0.0] * 6, END_LINK, [0.0, 0.0, 0.1], [9.81, 0.0, 0.0])
    # 容差 1e-3：中心差分在 h=1e-6 下的残差约 1e-5 N·m，相对 1.5 N·m 是 1e-5 量级，
    # 与「真有力矩」差三个数量级以上，不会混淆。
    chk("T6a 径向拉力对 J1 无矩", abs(f_rad[0]) < 1e-3, f"{f_rad[0]:.2e}")
    f_tan = arm.tau_point_force([0.0] * 6, END_LINK, [0.0, 0.0, 0.1], [0.0, 9.81, 0.0])
    chk("T6b 切向 1 kgf 对 J1 有矩", abs(f_tan[0]) > 0.01, f"{f_tan[0]:+.3f} N·m")

    # 数值微分自洽：单摆式小角度下 J2 力矩应随臂展增大
    t1 = abs(arm.tau_gravity(deg(0, 20, -90, 0, 0, 0))[1])
    t2 = abs(arm.tau_gravity(deg(0, 90, -90, 0, 0, 0))[1])
    chk("T7 J2 力矩随姿态变化", abs(t1 - t2) > 0.05, f"{t1:.2f} vs {t2:.2f}")

    # FK 一致性：法兰应在基座上方、与臂展同量级
    p = arm.point_world(deg(0, 90, -90, 0, 0, 0), END_LINK, [0.0, 0.0, 0.1358])
    r = math.sqrt(sum(v * v for v in p))
    chk("T8 法兰可达半径 0.2~0.8 m", 0.2 < r < 0.8, f"r={r:.3f} m")

    # J6 的转轴就是工具轴：只要载荷质心在轴线上，绕 J6 的力矩**恒为 0**，与姿态无关。
    # 加了偏心横杆也不够 —— 横杆方向若落在「轴与重力张成的竖直平面」内，力矩仍是 0；
    # 必须靠 J6 自转把偏心质量转到该平面之外（P7 的 J6=90°）。
    # 这四条决定「标 J6 必须既做横杆、又把 J6 转开」，是本工具最重要的结构结论。
    for label, qq, local, want_zero in [
        ("T9a 轴向载荷对 J6 无矩", deg(0, 45, -60, 0, 45, 0), [0.0, 0.0, 0.10], True),
        ("T9b 换姿态轴向仍无矩", deg(0, 45, -60, 0, 45, 90), [0.0, 0.0, 0.10], True),
        ("T9c P1 下横杆仍无矩", deg(0, 45, -60, 0, 45, 0), [0.08, 0.0, 0.10], True),
        ("T9d P7 下横杆有力矩", deg(0, 45, -60, 0, 45, 90), [0.08, 0.0, 0.10], False),
    ]:
        tq = arm.tau_point_force(qq, END_LINK, local, [0.0, 0.0, -9.81])[5]
        cond = abs(tq) < 1e-3 if want_zero else abs(tq) > 0.1
        chk(label, cond, f"{tq:+.3f} N·m")

    # T10：`check` 的符号判据必须真的能抓到「电流整体翻号」。
    # 这类错误（符号约定反了）不会让任何数字变丑，只会让 Δτ 整体反号，
    # 而残差依旧很小 —— 正是本项目最怕的那类「看起来很好、其实是废数据」。
    import contextlib
    import io
    import tempfile
    q1 = deg(0, 45, -60, 0, 45, 0)
    tg = arm.tau_gravity(q1)
    with tempfile.TemporaryDirectory() as td:
        def dump(flip_j3: bool) -> str:
            fp = Path(td) / f"s{flip_j3}.jsonl"
            with fp.open("w", encoding="utf-8") as fh:
                for _ in range(50):
                    cur = [tg[j] / K_SDK[j + 1] for j in range(6)]
                    if flip_j3:
                        cur[2] = -cur[2]
                    fh.write(json.dumps({"q": q1, "cur": cur}) + "\n")
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                cmd_check(argparse.Namespace(samples=str(fp), payload_kg=0.0, flange=0.0,
                                             urdf=a.urdf, gripper=a.gripper))
            return buf.getvalue()

        good = dump(False)
        chk("T10a check 对自洽数据给通过", "✅" in good)
        chk("T10b check 能抓到符号翻转", "符号相反" in dump(True))

    # 末端口径：URDF 自带夹爪 0.5 kg，而本复现已定案「不装夹爪」。
    # 这 0.5 kg 若不扣，J5 的自重力矩会直接反号 —— 判据基准就错了。
    u_full = Urdf(Path(a.urdf))
    a_full = Arm(u_full, end_mass=u_full.gripper_mass, end_com=u_full.gripper_com())
    a_bare = Arm(Urdf(Path(a.urdf)))
    tot = sum(u_full.mass.values())
    chk("T11a URDF 自带夹爪 0.50 kg", abs(u_full.gripper_mass - 0.50) < 1e-6,
        f"{u_full.gripper_mass:.3f} kg；总质量 {tot:.2f} − 夹爪 = "
        f"{tot - u_full.gripper_mass:.2f} ≈ 官方本体 4.2 kg")

    j2a = a_bare.tau_gravity(deg(0, 45, -60, 0, 45, 0))[1]
    j2b = a_bare.tau_gravity(deg(0, 75, -40, 0, 30, 0))[1]
    chk("T11b 裸臂下 J2 自重力矩仍反号", j2a * j2b < 0,
        f"P1 {j2a:+.3f} / P2 {j2b:+.3f} N·m —— 符号判据成立")

    p1 = deg(0, 45, -60, 0, 45, 0)
    r5 = a_full.tau_gravity(p1)[4] / a_bare.tau_gravity(p1)[4]
    gc = u_full.gripper_com()
    chk("T11c 夹爪质心在工具轴上 37.6 mm", gc is not None
        and abs(gc[2] - 0.0376) < 0.002 and abs(gc[0]) < 0.002 and abs(gc[1]) < 0.002,
        f"({gc[0]:+.4f}, {gc[1]:+.4f}, {gc[2]:+.4f}) m —— 两指 25 g 对称 ⇒ 横向抵消")

    j5b = a_bare.tau_gravity(p1)[4]
    j5f = a_full.tau_gravity(p1)[4]
    chk("T11d 含夹爪会让 J5 自重力矩反号", r5 < 0,
        f"含/裸 = {r5:+.2f}（{j5b:+.3f} → {j5f:+.3f} N·m）⇒ 口径必须在报告里声明")

    print(f"\n{'PASS' if ok else 'FAIL'}  selftest" if ok else "\nFAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------- main

def main():
    default_urdf = str(Path(__file__).resolve().parents[1] / "_vendor" / "urdf"
                       / "piper_description.urdf")
    ap = argparse.ArgumentParser(description="阶段 2 工装设计与空载对照（不动机器人）")
    ap.add_argument("--version", action="store_true", help="打印版本")
    ap.add_argument("--urdf", default=default_urdf, help=f"URDF 路径（默认 {default_urdf}）")
    ap.add_argument("--gripper", choices=["urdf", "none"], default="urdf",
                    help="重力模型的末端口径：urdf=带夹爪（默认，与现场一致，"
                         "质心按 URDF 算到 37.6 mm）；none=裸臂（仅对照）")
    ap.add_argument("--gripper-mass", type=float, default=None,
                    help="夹爪实测质量 kg（默认取 URDF 的 0.50）；称过之后填实测值")
    sub = ap.add_subparsers(dest="cmd")

    sp = sub.add_parser("sweep", help="姿态×载荷敏感性表（决定买什么砝码）")
    sp.add_argument("--mass", type=float, default=1.0, help="砝码质量 kg（默认 1.0）")
    sp.add_argument("--pull", type=float, default=10.0, help="横向拉力 N（默认 10）")
    sp.add_argument("--flange", type=float, default=0.05,
                    help="砝码悬挂点沿工具轴的外伸长度 m（默认 0.05）")
    sp.add_argument("--lever", type=float, default=0.08,
                    help="偏心横杆长度 m（垂直于工具轴，默认 0.08；给 0 可复现「标不了 J6」）")
    sp.add_argument("--json", default=None)
    sp.set_defaults(fn=cmd_sweep)

    cp = sub.add_parser("check", help="静态录制的空载对照（反推隐含 K）")
    cp.add_argument("--samples", required=True, help="episodes/<run>/samples.jsonl")
    cp.add_argument("--payload-kg", type=float, default=0.0, help="末端载荷 kg（默认 0）")
    cp.add_argument("--flange", type=float, default=0.0, help="载荷质心外伸 m（默认 0，即法兰处）")
    cp.add_argument("--last-s", type=float, default=0.0,
                    help="只取最后 N 秒（保持段）。0=整段。整段会把斜坡段混进均值，"
                         "反推的 K 直接被运动电流污染，务必给一个小于保持时长的值")
    cp.add_argument("--stable-deg", type=float, default=0.3,
                    help="静止判据：各轴与中位数相差不超过多少度，默认 0.3")
    cp.set_defaults(fn=cmd_check)

    st = sub.add_parser("selftest", help="18 条自测（J1/J6 结构零矩、偏心几何、check 符号判据、末端口径与质心）")
    st.set_defaults(fn=cmd_selftest)

    a = ap.parse_args()
    if a.version:
        print(f"stage2_design {VERSION}")
        return 0
    if not getattr(a, "fn", None):
        ap.print_help()
        return 1
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
