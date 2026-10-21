#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S2-0 载荷差分判别器 —— 区分 H1（URDF 质量模型不对）与 H2（固件在做重力前馈）。

背景（S20_REPORT.md §3）
------------------------
空载两个姿态（P1、P4′）采完以后，跨姿态差分不自洽：
  · J2 的 Δτ/ΔI = +3.52（是 SDK 系数 1.18125 的 3 倍）；
  · J3 = −2.17、J5 = −0.27，**反号**。
「电流不跟 URDF 力矩走」有两种解释，必须分开：
  H1  URDF 的连杆质量/质心没实测过 ⇒ 我算的 τ 本身就错 ⇒ 电流没错，是我的模型错；
  H2  固件按自己的模型做重力前馈 ⇒ 电机电流只是残差 ⇒ 静态电流法整个作废。

判别器
------
**同一个姿态、加 0.5 kg 砝码、再采一遍。** 载荷是「唯一不改变姿态就能改变力矩」的手段。
  · 若 ΔI 显著且与预测 Δτ 同号  ⇒ H1（电流通道可用），K 用 Δτ/ΔI 反推；
  · 若 ΔI 落在噪声里（< 3σ）    ⇒ H2（固件前馈），静态法作废，S2-1 改动态激励。

用法
----
    python s20_load_diff.py --empty episodes/s20_p1/samples.jsonl \\
                            --load  episodes/s20_p1_load/samples.jsonl \\
                            --load-mass 0.5 --load-com-z 0.090 --last-s 45
    python s20_load_diff.py --selftest

不动机器人：只读两个 jsonl 与 URDF。
"""

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import stage2_design as S          # noqa: E402  （复用 Urdf / Arm / END_LINK / G）

VERSION = "1.0-20261020"

# SDK 0.6.1 (arm_feedback_high_spd.py:84) 的两组系数，只作量级参照
SDK_K = {0: 1.18125, 1: 1.18125, 2: 1.18125, 3: 0.95844, 4: 0.95844, 5: 0.95844}


# ---------------------------------------------------------------- 读静态段

def load_static(path, last_s=45.0, stable_deg=0.3):
    """读 samples.jsonl 的静止段，返回 dict(q, cur, sig, n_all, n, n_drop)。

    与 `stage2_design.py check` 同一口径：先截尾窗，再按「离中位数多远」剔离群，
    这样斜坡段（臂在动、电流被运动分量污染）不会混进均值。
    """
    rows = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        q = r.get("q")
        if not q or any(v is None for v in q):
            continue
        rows.append((r.get("t"), list(q), list(r.get("cur") or [None] * 6)))
    n_all = len(rows)
    if n_all == 0:
        raise SystemExit(f"[ABORT] {path} 里没有完整六轴 q 的样本")

    if last_s and last_s > 0:
        ts = [r[0] for r in rows if r[0] is not None]
        if ts:
            t_end = max(ts)
            rows = [r for r in rows if r[0] is not None and r[0] >= t_end - last_s]
    if not rows:
        raise SystemExit(f"[ABORT] {path} 尾窗 {last_s:g} s 内没有样本（保持段太短？）")

    med = [sorted(r[1][j] for r in rows)[len(rows) // 2] for j in range(6)]
    tol = math.radians(stable_deg)
    stable = [r for r in rows if all(abs(r[1][j] - med[j]) <= tol for j in range(6))]
    n_drop = len(rows) - len(stable)
    if len(stable) < 20:
        raise SystemExit(f"[ABORT] {path} 静止样本只有 {len(stable)} 条"
                         f"（容差 {stable_deg:g}°）—— 加长保持段或放宽容差")
    rows = stable

    n = len(rows)
    # 注意：samples.jsonl 的 "q" 存的是**弧度**（录制层直接存固件 int32×0.001 rad）。
    # 这里保持弧度原样返回，只在打印时才转度 —— 2026-10-20 12:1x 第一版把它当度，
    # 结果 P1/P4′ 的 55° 姿态差被当成 0.96「度」，tau 全错。
    q_mean = [sum(r[1][j] for r in rows) / n for j in range(6)]
    cur_mean, cur_sig = [], []
    for j in range(6):
        v = [r[2][j] for r in rows if r[2][j] is not None]
        if len(v) < 10:
            raise SystemExit(f"[ABORT] {path} 的 J{j+1} 电流样本只有 {len(v)} 条")
        m = sum(v) / len(v)
        cur_mean.append(m)
        cur_sig.append(math.sqrt(sum((x - m) ** 2 for x in v) / max(1, len(v) - 1)))
    return {"q": q_mean, "cur": cur_mean, "sig": cur_sig,
            "n_all": n_all, "n": n, "n_drop": n_drop, "path": str(path)}


# ---------------------------------------------------------------- 载荷几何

def _hang_point(arm, q, tip_z, drop):
    """「竖直吊在指尖下方 drop 处」的点在世界系的位置（m）。

    指尖 = 法兰原点沿工具轴前进 tip_z（夹爪长度）；
    瓶子靠重力始终竖直 ⇒ 质心在指尖正下方 drop（世界 −z），**不跟夹爪转**。
    """
    R, p = arm.transforms(q)[S.END_LINK]
    ax = (R[0][2], R[1][2], R[2][2])
    return [p[0] + ax[0] * tip_z,
            p[1] + ax[1] * tip_z,
            p[2] + ax[2] * tip_z - drop]


def tau_hanging(arm, q, tip_z, drop, mass):
    """吊挂点质量对六轴的力矩（J^T F），N·m。

    为什么不能直接用 `Arm.tau_point_force`：那个把点写在 **link6 帧**（刚体绑死，
    跟着夹爪一起转）。吊着的水瓶正相反 —— 它永远竖直，位置是**世界坐标**的函数，
    所以必须对整个世界点做数值微分，否则 J5 一转方向就算反。
    2026-10-20 11:1x 第一次就是按「沿工具轴 90 mm」算的，结果 J2/J3/J5 三轴全反号，
    差点误判成「URDF 模型整体错了」。
    """
    F = (0.0, 0.0, -mass * S.G)
    h = 1e-6
    out = [0.0] * 6
    for j in range(6):
        qp = list(q)
        qm = list(q)
        qp[j] += h
        qm[j] -= h
        pp = _hang_point(arm, qp, tip_z, drop)
        pm = _hang_point(arm, qm, tip_z, drop)
        out[j] = sum(F[k] * (pp[k] - pm[k]) / (2 * h) for k in range(3))
    return out


# ---------------------------------------------------------------- 判定核心

def judge(dtau_pred, dI, sig, tau_eps=0.05, snr_min=3.0):
    """单轴判读。返回 (verdict, K)。

    verdict ∈ insensitive / null / same / opposite
      insensitive : 该轴对「末端加 0.5 kg」本来就不敏感（|Δτ| < tau_eps），无信息
      null        : ΔI 落在噪声里 → 该轴支持 H2
      same        : ΔI 显著且与预测同号 → 该轴支持 H1，K 可算
      opposite    : ΔI 显著但反号 → URDF 对「载荷增量」的方向都算反了，模型有问题
    """
    if abs(dtau_pred) < tau_eps:
        return "insensitive", None
    s = max(sig, 1e-9)
    if abs(dI) / s < snr_min:
        return "null", None
    return ("same" if dtau_pred * dI > 0 else "opposite"), dtau_pred / dI


def verdict_of(details, ratio_tol=2.5):
    """把六轴判读汇总成一个结论。`details = [(verdict, K), ...]`

    2026-10-20 11:2x 修正：原先「只要有轴反号就判 MODEL_WRONG」是错的。
    带 0.525 kg 水瓶实测时，J2/J3/J5 **三轴全反号**，但它们的 |Δτ/ΔI| 是 4.08 / 3.73 / 0.67
    —— 前两个几乎一样。三轴一致地反号不是「模型错」，而是**电流正方向的定义与关节正方向相反**
    （或控制器报的是反作用力矩）。真正的判据是「|K| 互相是否一致」，不是「同不同号」。
    """
    same = [abs(k) for v, k in details if v == "same" and k is not None]
    opp = [abs(k) for v, k in details if v == "opposite" and k is not None]
    n_null = sum(1 for v, _ in details if v == "null")
    allk = same + opp

    if len(allk) >= 2:
        med = sorted(allk)[len(allk) // 2]
        inl = sorted(k for k in allk if med / ratio_tol <= k <= med * ratio_tol)
        if len(inl) >= 2:
            kmid = inl[len(inl) // 2]
            spread = inl[-1] / inl[0]
            neg = len(opp) > len(same)
            msg = (f"有 {len(inl)} 个轴的 |Δτ/ΔI| 互相一致（≈{kmid:.2f} N·m/A，"
                   f"离散 {spread:.2f}×）⇒ **电流确实反映力矩**，静态法可用；"
                   f"只是 URDF 的绝对力矩不准，别拿它当基准。")
            if neg:
                msg += ("符号与 URDF 预测相反 ⇒ 电流正方向的定义与关节正方向相反"
                        "（或控制器报的是反作用力矩）。不影响可用性，K 取负即可。")
            if len(allk) > len(inl):
                msg += (f" 另外 {len(allk) - len(inl)} 个轴的 |K| 离群"
                        f"（J5 常见：内部走线/弹簧带来恒定偏置），要单独标定，不要混进中位数。")
            return "H1", msg
        return "MODEL_WRONG", (f"显著轴的 |Δτ/ΔI| 离散太大（{min(allk):.2f} ~ {max(allk):.2f}"
                               f" N·m/A，比值 {max(allk)/min(allk):.1f}×），凑不出一条载荷→电流的直线。"
                               f"先核对砝码的实际悬挂位置与质量，再考虑把 URDF 质量模型整体作废")
    if n_null >= 2 and not allk:
        return "H2", ("多轴「加了砝码电流也不动」⇒ 固件在按自己的模型做重力前馈，"
                      "电机电流只是残差。**静态电流法作废**，S2-1 必须改成动态激励"
                      "（正弦扫频 / 加减速惯量项），或直接读固件给的 effort 字段做交叉验证")
    return "UNCLEAR", (f"同号轴 {len(same)} 个、无响应轴 {n_null} 个、反号轴 {len(opp)} 个，"
                       f"凑不出统一结论。先检查两次采集的姿态是否真的一致、砝码是否真的挂在末端")


# ---------------------------------------------------------------- 主流程

def run(a):
    urdf = S.Urdf(Path(a.urdf))
    arm = S.Arm(urdf, end_mass=urdf.gripper_mass, end_com=urdf.gripper_com())

    e = load_static(a.empty, a.last_s, a.stable_deg)
    l = load_static(a.load, a.last_s, a.stable_deg)

    print(f"s20_load_diff {VERSION}")
    print(f"URDF        : {a.urdf}")
    if a.load_mode == "hang":
        load_desc = (f"吊挂 {a.load_mass:.3f} kg（指尖=法兰+{a.load_tip_z*1000:.0f} mm，"
                     f"质心在指尖正下方 {a.load_drop*1000:.0f} mm）")
    else:
        load_desc = f"沿工具轴 {a.load_mass:.3f} kg（质心 {a.load_com_z*1000:.0f} mm）"
    print(f"末端口径    : 夹爪 {urdf.gripper_mass:.2f} kg（质心 {urdf.gripper_com()[2]*1000:.1f} mm）"
          f" + {load_desc}")
    print(f"空载        : {a.empty}  静止段 {e['n']} / {e['n_all']} 条（剔离群 {e['n_drop']}）")
    print(f"加载        : {a.load}  静止段 {l['n']} / {l['n_all']} 条（剔离群 {l['n_drop']}）")

    # ---- 姿态一致性：Δτ 的可信度完全取决于「两次是不是同一个姿态」
    dq = [math.degrees(l["q"][j] - e["q"][j]) for j in range(6)]
    worst = max(range(6), key=lambda j: abs(dq[j]))
    print()
    print("## 1. 姿态一致性（两次必须几乎相同，否则 Δτ 里混进了姿态变化）")
    print("  " + "  ".join(f"J{j+1}={dq[j]:+.3f}°" for j in range(6)))
    if abs(dq[worst]) > a.q_tol_deg:
        print(f"  [WARN] J{worst+1} 差了 {dq[worst]:+.3f}° > {a.q_tol_deg:g}°："
              f"Δτ 已按各自实测姿态分别计算，但请确认这不是「没走到位」")
    else:
        print(f"  OK：最大差 {abs(dq[worst]):.3f}° ≤ {a.q_tol_deg:g}°")

    # ---- 力矩：两边各按自己实测姿态算，再加砝码项（q 本来就是弧度，别再转）----
    qe = list(e["q"])
    ql = list(l["q"])
    tau_e = arm.tau_gravity(qe)
    tau_l = arm.tau_gravity(ql)
    if a.load_mode == "hang":
        add_load = tau_hanging(arm, ql, a.load_tip_z, a.load_drop, a.load_mass)
    else:
        add_load = arm.tau_point_force(ql, S.END_LINK, [0.0, 0.0, a.load_com_z],
                                       [0.0, 0.0, -a.load_mass * S.G])
    tau_l = [tau_l[j] + add_load[j] for j in range(6)]
    dtau = [tau_l[j] - tau_e[j] for j in range(6)]

    print()
    print("## 2. 逐轴对照")
    print("| 关节 | τ_空载 | τ_加载 | Δτ 预测 | I_空载 | I_加载 | ΔI | σ_ΔI | ΔI/σ | 判读 | K=Δτ/ΔI |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    details = []
    for j in range(6):
        dI = l["cur"][j] - e["cur"][j]
        sig = math.sqrt(e["sig"][j] ** 2 + l["sig"][j] ** 2)
        v, k = judge(dtau[j], dI, sig, a.tau_eps, a.snr_min)
        details.append((v, k))
        tag = {"insensitive": "不敏感", "null": "无响应→H2",
               "same": "同号→H1", "opposite": "反号"}[v]
        kk = "—" if k is None else f"{k:+.2f}（{abs(k)/SDK_K[j]:.2f}× SDK）"
        print(f"| J{j+1} | {tau_e[j]:+.3f} | {tau_l[j]:+.3f} | {dtau[j]:+.3f} | "
              f"{e['cur'][j]:+.4f} | {l['cur'][j]:+.4f} | {dI:+.4f} | {sig:.4f} | "
              f"{abs(dI)/max(sig,1e-9):5.1f} | {tag} | {kk} |")

    code, text = verdict_of(details)
    print()
    print(f"## 3. 结论：{code}")
    print(f"  {text}")
    ks = [abs(k) for v, k in details if k is not None]
    if ks:
        med = sorted(ks)[len(ks) // 2]
        print(f"  |K| 全轴中位数（含离群轴）：{med:.2f} N·m/A；逐轴 "
              + " ".join(f"J{j+1}={abs(k):.2f}" for j, (v, k) in enumerate(details) if k is not None))
    if code == "H2":
        print("  → 下一步：S2-1 不能再用静态电流。改成「同一姿态加/卸载荷差分 + 正弦激励」，")
        print("     或者用固件自带的 effort 字段（0x251 字节 2-3 经 SDK 系数换算）做交叉验证。")
    elif code == "H1":
        print("  → 下一步：用载荷增量反推每个轴的 K，再回头重算 P1/P4′ 的绝对力矩，")
        print("     检查「URDF 质量模型偏了多少」。别用 URDF 绝对值当基准。")
    return 0 if code in ("H1", "H2", "MODEL_WRONG") else 1


# ---------------------------------------------------------------- 自测

def selftest():
    ok = 0
    bad = 0

    def chk(name, cond, extra=""):
        nonlocal ok, bad
        if cond:
            ok += 1
            print(f"  [PASS] {name}")
        else:
            bad += 1
            print(f"  [FAIL] {name} {extra}")

    print("s20_load_diff selftest")
    tau = [0.0, 0.60, -0.42, 0.0, 0.25, 0.0]
    sig = [0.01, 0.02, 0.02, 0.01, 0.02, 0.01]

    # A. 世界是 H1：ΔI 与 Δτ 同号
    dI = [tau[j] / 1.18 for j in range(6)]
    ds = [judge(tau[j], dI[j], sig[j]) for j in range(6)]
    code, _ = verdict_of(ds)
    chk("A 合成 H1 世界 → 判 H1", code == "H1", code)
    ks = [k for _, k in ds if k is not None]
    chk("A 反推 K≈1.18", all(abs(k - 1.18) < 1e-6 for k in ks),
        str([round(k, 3) for k in ks]))

    # B. 世界是 H2：加砝码电流不动
    dI0 = [0.004, -0.006, 0.003, -0.002, 0.005, 0.001]     # 全在 1σ 内
    ds = [judge(tau[j], dI0[j], sig[j]) for j in range(6)]
    code, _ = verdict_of(ds)
    chk("B 合成 H2 世界 → 判 H2", code == "H2", code)

    # C. 一致地反号（真实数据 2026-10-20 11:2x 就是这样）：这不是模型错，
    #    而是电流正方向与关节正方向相反 ⇒ 仍应判 H1
    dIo = [-tau[j] / 1.18 for j in range(6)]
    ds = [judge(tau[j], dIo[j], sig[j]) for j in range(6)]
    code, msg = verdict_of(ds)
    chk("C 三轴一致反号 → 仍判 H1（符号约定相反）", code == "H1", code)
    chk("C 且说明里点出「符号相反」", "符号" in msg, msg[:40])

    # D. |K| 离散太大（真的凑不出直线）⇒ MODEL_WRONG
    dId = [tau[1] / 1.0, tau[2] / 5.0, tau[4] / 0.2]
    ds = [judge(tau[1], dId[0], sig[1]), judge(tau[2], dId[1], sig[2]),
          judge(tau[4], dId[2], sig[4])]
    code, _ = verdict_of(ds)
    chk("D |K| 离散 25× → 判 MODEL_WRONG", code == "MODEL_WRONG", code)

    # E. 全部不敏感（砝码没碰到末端/挂错地方）
    ds = [judge(0.001, 0.5, sig[j]) for j in range(6)]
    code, _ = verdict_of(ds)
    chk("E 力矩预测全≈0 → UNCLEAR（提示查砝码作用点）", code == "UNCLEAR", code)

    # F. 信噪比门槛：4σ 算显著、2σ 不算
    v4, _ = judge(0.5, 0.04, 0.01)
    v2, _ = judge(0.5, 0.02, 0.01)
    chk("F SNR 门槛 3σ：4σ 判 same", v4 == "same", v4)
    chk("F SNR 门槛 3σ：2σ 判 null", v2 == "null", v2)

    # G. judge 不该在 |Δτ| 很小时乱判
    v, k = judge(0.01, 5.0, 0.001)
    chk("G |Δτ|<tau_eps → insensitive 且 K=None", v == "insensitive" and k is None, v)

    # H. G 就是重力加速度，别被改名
    chk("H G 常数 ≈ 9.81", abs(S.G - 9.81) < 0.05, str(S.G))

    # I. 吊挂模型：J2 加质量后力矩必须同向变大（物理上无可辩驳）
    urdf = S.Urdf(Path(HERE.parent / "_vendor" / "urdf" / "piper_description.urdf"))
    arm = S.Arm(urdf, end_mass=urdf.gripper_mass, end_com=urdf.gripper_com())
    q_p1 = [math.radians(v) for v in (0, 45, -60, 0, 45, 0)]
    t0 = tau_hanging(arm, q_p1, 0.080, 0.060, 0.0)
    t1 = tau_hanging(arm, q_p1, 0.080, 0.060, 0.525)
    d = [t1[j] - t0[j] for j in range(6)]
    chk("I 吊挂 0.525 kg 让 J2 力矩同向增大", d[1] > 0.3, f"{d[1]:+.3f}")
    chk("I 且 J3 也同向增大", d[2] > 0.3, f"{d[2]:+.3f}")
    chk("I 结构零矩轴 J1/J4/J6 基本不动",
        all(abs(d[j]) < 0.02 for j in (0, 3, 5)), str([round(d[j], 3) for j in (0, 3, 5)]))

    print(f"  自测 {ok} 过 / {bad} 挂")
    return 0 if bad == 0 else 1


def main():
    default_urdf = str(HERE.parent / "_vendor" / "urdf" / "piper_description.urdf")
    ap = argparse.ArgumentParser(description="S2-0 载荷差分判别器（H1 vs H2，不动机器人）")
    ap.add_argument("--version", action="store_true", help="打印版本")
    ap.add_argument("--selftest", action="store_true", help="跑内置自测（不需要数据）")
    ap.add_argument("--empty", help="空载那次的 samples.jsonl")
    ap.add_argument("--load", help="加了砝码那次的 samples.jsonl")
    ap.add_argument("--load-mass", type=float, default=0.5, help="砝码质量 kg，默认 0.5")
    ap.add_argument("--load-mode", choices=["hang", "axis"], default="hang",
                    help="hang=靠重力竖直吊在指尖下方（矿泉水瓶、砝码吊块，默认）；"
                         "axis=刚性固定在工具轴上（夹持式砝码）")
    ap.add_argument("--load-tip-z", type=float, default=0.080,
                    help="hang 模式：指尖（=载荷悬挂点）沿工具轴距法兰多远，默认 0.080 m")
    ap.add_argument("--load-drop", type=float, default=0.060,
                    help="hang 模式：载荷质心在指尖正下方多少，默认 0.060 m")
    ap.add_argument("--load-com-z", type=float, default=0.090,
                    help="axis 模式：质心相对法兰原点沿工具轴的外伸 m，默认 0.090。"
                         "这个数只影响 Δτ 的绝对值（进而影响 K），不影响「是否显著」的判定")
    ap.add_argument("--last-s", type=float, default=45.0, help="只取最后 N 秒，默认 45")
    ap.add_argument("--stable-deg", type=float, default=0.3, help="静止判据，默认 0.3°")
    ap.add_argument("--q-tol-deg", type=float, default=1.0,
                    help="两次采集的姿态允许差多少度，默认 1.0（超了只警告不中断）")
    ap.add_argument("--tau-eps", type=float, default=0.05,
                    help="|Δτ| 低于此值视为该轴不敏感，默认 0.05 N·m")
    ap.add_argument("--snr-min", type=float, default=3.0, help="ΔI 显著门槛，默认 3σ")
    ap.add_argument("--urdf", default=default_urdf, help=f"URDF 路径（默认 {default_urdf}）")
    a = ap.parse_args()

    if a.version:
        print(f"s20_load_diff {VERSION}")
        return 0
    if a.selftest:
        return selftest()
    if not a.empty or not a.load:
        ap.error("要么给 --selftest，要么同时给 --empty 和 --load")
    return run(a)


if __name__ == "__main__":
    sys.exit(main())
