#!/usr/bin/env python3
"""s21_fit.py —— S2-1 力矩常数 K 的多姿态最小二乘拟合

模型（每个关节各自独立，但共享一个载荷质心）
-------------------------------------------
    I_kj = ( τ_arm,j(q_k) + m·g·Σ_i A_kji·c_i ) / K_j + b_j

  * `τ_arm,j`  ：URDF 重力模型算的**臂+夹爪**保持力矩（Q4 记忆：末端 0.5 kg 必须显式加）
  * `c`        ：水瓶质心在 **link6 系**里的常量 3 维坐标 —— 全局共享，3 个未知数。
                 这是本脚本的关键：水瓶是用胶带**刚性绑**在夹爪上的，
                 它跟着夹爪一起倾（J6 归零后夹爪本来就不竖直），
                 所以质心是 link6 系的常量，而**力**永远是世界 −Z。
                 不把 c 拟合出来、硬用「质心 90 mm 在轴上」，J4/J5/J6 会系统性偏。
  * `K_j`      ：待求的力矩常数（N·m/A）
  * `b_j`      ：截距（零点/摩擦），**必须保留**（门 4：禁止逐点相除再平均）

为什么 K 与 b 要一起解、不能逐点相除
------------------------------------
逐点相除 (τ/I) 在 I→0 时爆炸，而且把零点塞进 K 里：S2-0 里 J2 在两个姿态上
逐点算出来的 K 差 22 倍，就是零点的锅。带截距的回归才把它分开。

可辨识性判据
------------
一个关节能不能标，只看「τ 的跨度」够不够大：
    span_j ≥ 6 × K_assumed × σ_j
σ_j 用该关节静止段电流标准差（实测 0.003~0.09 A）。跨度不够的写 `未标定`，
**不许填 SDK 系数冒充结果**（门 2 / 指南 §2.3「伪精确比空白更危险」）。

用法（本机 Windows，LEFS/experiments/20261019_stage2）
-----------------------------------------------------
  python s21_fit.py                          # 用内置 episode 清单
  python s21_fit.py --last-s 30 --out calibration
  python s21_fit.py --selftest
"""

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import stage2_design as S
import s20_load_diff as D
import s21_design as G          # 只借用 deg()

EP_DIR = Path(__file__).resolve().parent / "episodes"

# 默认清单：(标签, episode 目录名, 是否带水瓶)
DEFAULT_EPISODES = [
    ("P1_empty",  "s20_p1",      False),
    ("P4p_empty", "s20_p4",      False),
    ("P1_load",   "s20_p1_load", True),
    ("LIFT",      "s21_LIFT",    True),
    ("K3",        "s21_K3",      True),
    ("K10",       "s21_K10",     True),
    ("K15",       "s21_K15",     True),
    ("K14",       "s21_K14",     True),
    ("K9",        "s21_K9",      True),
    ("K8",        "s21_K8",      True),
    ("K5",        "s21_K5",      True),
    ("K2",        "s21_K2",      True),
    ("P1",        "s21_P1",      True),
    ("K12",       "s21_K12",     True),
]

BOTTLE_MASS = 0.525
GRIPPER_MASS = 0.500
K_SDK = [1.18125, 1.18125, 1.18125, 0.95844, 0.95844, 0.95844]
JOINT_NAME = [f"J{i+1}" for i in range(6)]


# ---------------------------------------------------------------- 数据装载
def build_row(arm, path, last_s, stable_deg, loaded):
    """一个 episode → dict(q, cur, sig, tau_arm, A, loaded)。"""
    st = D.load_static(path, last_s, stable_deg)
    q = list(st["q"])
    ta = arm.tau_gravity(q)
    A = [[0.0] * 3 for _ in range(6)]
    p0 = [0.0] * 6
    if loaded:
        # τ_pay(c) = p0 + Σ_i c_i · A[j][i]
        #
        # **p0 不能省**：`tau_point_force(q, link6, local, F)` 里那个 `local` 是
        # 「从法兰原点出发的偏移」，不是「点在 link6 系里的绝对坐标」。
        # 所以 local=0 时力矩并不为零（法兰本身就离各关节轴有距离），
        # 2026-10-20 12:1x 第一版只算了 Σ c_i·A_i 这一项，把 P1 的载荷力矩
        # 从 1.13 算成 0.48 N·m（漏掉的正是 p0=0.71），整轮拟合因此全歪。
        f = [0.0, 0.0, -BOTTLE_MASS * S.G]
        p0 = arm.tau_point_force(q, S.END_LINK, [0.0, 0.0, 0.0], f)
        for i in range(3):
            e = [0.0, 0.0, 0.0]
            e[i] = 1.0
            t = arm.tau_point_force(q, S.END_LINK, e, f)
            for j in range(6):
                A[j][i] = t[j]
    return dict(q=q, cur=list(st["cur"]), sig=list(st["sig"]),
                tau_arm=ta, A=A, p0=p0, loaded=loaded, n=st["n"])


# ---------------------------------------------------------------- 拟合核心
def wls2(T, I, W):
    """加权最小二乘 I = a·T + b。返回 (a, b, χ², cov_a, cov_b, cov_ab)。

    手写 2×2 正规方程 —— 本机没装 numpy，也不值得为 2 个参数引入依赖。
    """
    S0 = sum(W)
    S1 = sum(w * t for w, t in zip(W, T))
    S2 = sum(w * t * t for w, t in zip(W, T))
    R0 = sum(w * i for w, i in zip(W, I))
    R1 = sum(w * t * i for w, t, i in zip(W, T, I))
    det = S0 * S2 - S1 * S1
    if abs(det) < 1e-12:
        return None
    a = (S0 * R1 - S1 * R0) / det
    b = (S2 * R0 - S1 * R1) / det
    chi2 = sum(w * (i - a * t - b) ** 2 for w, t, i in zip(W, T, I))
    cov_a = S0 / det
    cov_b = S2 / det
    cov_ab = -S1 / det
    return a, b, chi2, cov_a, cov_b, cov_ab


def predict_T(row, c):
    p0 = row.get("p0") or [0.0] * 6
    return [row["tau_arm"][j] + p0[j] + sum(row["A"][j][i] * c[i] for i in range(3))
            for j in range(6)]


def fit_for_c(rows, c, joints=(1, 2, 3, 4, 5)):
    """c 固定时，逐关节解 (a,b)。返回 dict。"""
    out = {}
    for j in joints:
        T = [predict_T(r, c)[j] for r in rows]
        I = [r["cur"][j] for r in rows]
        W = [1.0 / max(r["sig"][j], 1e-6) ** 2 for r in rows]
        r = wls2(T, I, W)
        if r is None:
            out[j] = None
            continue
        a, b, chi2, ca, cb, cab = r
        out[j] = dict(a=a, b=b, chi2=chi2, cov_a=ca, cov_b=cb, cov_ab=cab,
                      T=T, I=I, W=W, n=len(rows))
    return out


def total_chi2(rows, c, joints=(1, 2, 3, 4, 5)):
    r = fit_for_c(rows, c, joints)
    return sum(v["chi2"] for v in r.values() if v), r


C_BOX = ((-0.12, 0.12), (-0.12, 0.12), (0.00, 0.26))   # 水瓶质心的物理可能范围 m


def grid_search_c(rows, joints=(1, 2, 3, 4, 5),
                  lo=(-0.12, -0.12, 0.00), hi=(0.12, 0.12, 0.26),
                  steps=(10, 10, 9), rounds=4):
    """水瓶质心 c（link6 系，m）的坐标搜索。

    目标函数是 3 个参数的**双线性**问题（a_j 与 c 相乘），不是凸的，
    但维度只有 3、量级只有 0.1 m，直接「粗网格 → 逐轮缩步长」最稳，
    也不会像梯度法那样在 a·c 的耦合处乱跑。
    """
    lo, hi = list(lo), list(hi)
    best_c, best_chi = None, float("inf")
    for rd in range(rounds):
        axes = []
        for i in range(3):
            n = steps[i] * (2 ** rd) + 1
            n = min(n, 41)
            axes.append([lo[i] + (hi[i] - lo[i]) * k / (n - 1) for k in range(n)])
        for x in axes[0]:
            for y in axes[1]:
                for z in axes[2]:
                    c = [x, y, z]
                    chi, _ = total_chi2(rows, c, joints)
                    if chi < best_chi:
                        best_chi, best_c = chi, c
        span = [(hi[i] - lo[i]) / (2 ** (rd + 1)) for i in range(3)]
        # 每轮收缩后重新夹回物理盒子：不然（a_j 与 c 相乘的）双线性会让解跑到
        # 盒子外面去找「等效方向」——12:1x 第一版就跑到 y=−262 mm，那不是水瓶的位置
        lo = [max(C_BOX[i][0], best_c[i] - span[i]) for i in range(3)]
        hi = [min(C_BOX[i][1], best_c[i] + span[i]) for i in range(3)]
    return best_c, best_chi


# ---------------------------------------------------------------- 报告
def fit_all(rows, last_s, hold_labels=None):
    c, chi = grid_search_c(rows)
    res = fit_for_c(rows, c)
    out = dict(c=c, chi2=chi, joints={})
    for j, v in res.items():
        if v is None:
            out["joints"][JOINT_NAME[j]] = dict(state="failed")
            continue
        a = v["a"]
        K = 1.0 / a if abs(a) > 1e-9 else float("inf")
        sig_a = math.sqrt(v["cov_a"])
        # 回归自带的 σ 尺度：weighted χ²/dof
        dof = max(v["n"] - 2, 1)
        scale = math.sqrt(v["chi2"] / dof)
        sigK = abs(sig_a * scale / (a * a)) if abs(a) > 1e-9 else float("inf")
        T = v["T"]
        span = max(T) - min(T)
        out["joints"][JOINT_NAME[j]] = dict(
            K=K, b=v["b"], sigma_K=sigK, tau_span=span,
            tau_min=min(T), tau_max=max(T), chi2=v["chi2"], dof=dof,
            chi2_red=v["chi2"] / dof, n=v["n"],
            T=T, I=v["I"])
    # 可辨识性判定
    for j in range(1, 5):
        e = out["joints"][JOINT_NAME[j]]
        if e.get("K") is None or not math.isfinite(e.get("K", float("nan"))):
            continue
        sig_cur = max(r["sig"][j] for r in rows)
        need = 6.0 * abs(e["K"]) * sig_cur
        e["span_need"] = need
        e["identified"] = bool(e["tau_span"] >= need)
    # J1 / J6 是结构性的，不看跨度也不许填数
    out["joints"]["J1"] = dict(
        K=None, b=None,
        structural="重力与 J1 轴平行 ⇒ τ1≡0（任何姿态、任何载荷都一样），静态法无解；"
                   "必须用滑轮/弹簧秤做切向拉",
        state="未标定")
    j6 = out["joints"]["J6"]
    j6["structural"] = ("水瓶质心基本在工具轴（=J6 转轴）上 ⇒ τ6≡0；"
                        "需要偏心横杆 80 mm + J6 转 90° 才能标")
    j6["state"] = "未标定"
    return out


def holdout_error(rows, c):
    """留一姿态：拿掉一个 episode 重拟合，预测它自己的电流。"""
    out = []
    for k in range(len(rows)):
        sub = [r for i, r in enumerate(rows) if i != k]
        res = fit_for_c(sub, c)
        pred = []
        for j in range(1, 5):
            v = res[j]
            if v is None:
                continue
            a, b = v["a"], v["b"]
            T = predict_T(rows[k], c)[j]
            pred.append((JOINT_NAME[j], (a * T + b) - rows[k]["cur"][j],
                         rows[k]["sig"][j]))
        if pred:
            out.append((k, pred))
    return out


def fmt(x, nd=3):
    return "—" if x is None or not isinstance(x, float) else f"{x:+.{nd}f}"


def report(out, labels, rows, last_s):
    c = out["c"]
    print("## 1. 载荷质心（link6 系）")
    print(f"  拟合值 c6 = [{c[0]*1000:+.1f}, {c[1]*1000:+.1f}, {c[2]*1000:+.1f}] mm")
    print(f"  沿工具轴外伸 |c6| = {math.sqrt(sum(v*v for v in c))*1000:.1f} mm"
          f"（选型时按 90 mm 估的）")
    print(f"  总 χ² = {out['chi2']:.1f}")
    print()
    print("## 2. 逐关节拟合 I = τ/K + b")
    print("| 关节 | K (N·m/A) | ±1σ | K/K_SDK | b (A) | τ 跨度 | 需要跨度 | 状态 | χ²/dof |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for i in range(6):
        e = out["joints"][JOINT_NAME[i]]
        if i in (0, 5):
            print(f"| {JOINT_NAME[i]} | — | — | — | — | — | — | 未标定（结构） | — |")
            continue
        st = "已标定" if e.get("identified") else "未标定（跨度不足）"
        print(f"| {JOINT_NAME[i]} | {e['K']:+.3f} | {e['sigma_K']:.3f} | "
              f"{abs(e['K'])/K_SDK[i]:.2f}× | {e['b']:+.4f} | {e['tau_span']:.2f} | "
              f"{e['span_need']:.2f} | {st} | {e['chi2_red']:.1f} |")
    print()
    print("## 3. 残差")
    for i in range(1, 5):
        e = out["joints"][JOINT_NAME[i]]
        # χ² 是加权量：除以平均权重换回 A
        wbar = sum(e["W"]) / len(e["W"])
        rms = math.sqrt(e["chi2"] / e["dof"] / wbar)
        print(f"  {JOINT_NAME[i]}: 残差 RMS = {rms:.4f} A "
              f"（χ²/dof={e['chi2_red']:.2f}，权重均值 {wbar:.0f}）")


def selftest(arm):
    n_ok = n_bad = 0

    def chk(name, cond, detail=""):
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print(f"  ✓ {name}")
        else:
            n_bad += 1
            print(f"  ✗ {name}  {detail}")

    # 合成世界：已知 K、b、c，看能不能反解回来
    truth_K = [0.0, 4.00, 3.60, -1.20, 2.80, 0.0]
    truth_b = [0.0, -0.45, -1.10, 0.02, -0.60, 0.0]
    truth_c = [0.030, -0.020, 0.095]
    qs = [G.deg(*v) for v in [
        (0, 45, -60, 0, 45, 0), (0, 30, -110, 0, -40, 0),
        (0, 75, -30, 0, -40, 0), (0, 60, -50, 60, 40, 0),
        (0, 60, -50, 90, 40, 90), (0, 120, -90, 90, 40, 0),
        (-45, 30, -80, 0, 50, 0), (0, 45, -60, 0, 45, 30)]]
    rows = []
    for q in qs:
        r = build_row_fake(arm, q, truth_c, truth_K, truth_b)
        rows.append(r)
    c, chi = grid_search_c(rows, rounds=3)
    res = fit_for_c(rows, c)
    chk("A 合成 c 能反解出来（±3 mm）",
        max(abs(c[i] - truth_c[i]) for i in range(3)) < 0.003, str([round(v*1000,1) for v in c]))
    ok = True
    for j in range(1, 5):
        K = 1.0 / res[j]["a"]
        d = abs(K - truth_K[j]) / abs(truth_K[j])
        if d > 0.02:
            ok = False
            print(f"     J{j+1} K={K:.3f} 真值 {truth_K[j]:.3f} 偏差 {d*100:.1f}%")
    chk("A 合成 K 四轴全部 < 2%", ok)
    chk("A 合成 b 也回来了",
        all(abs(res[j]["b"] - truth_b[j]) < 0.01 for j in range(1, 5)),
        str([round(res[j]["b"], 3) for j in range(1, 5)]))

    # 漏掉 c ⇒ 残差应显著变大（说明 c 必须拟合）
    chi0, _ = total_chi2(rows, [0.0, 0.0, 0.09])
    chk("B 固定 c=(0,0,0.09) 时 χ² 明显劣于拟合 c", chi0 > chi * 5,
        f"{chi0:.1f} vs {chi:.1f}")

    # 只有两个姿态时每轴只有 2 个方程、2 个未知数 ⇒ 残差恒为 0，什么也验证不了。
    # 这正是 S2-0 那三个点不够拟合的原因，写进自测免得以后又拿 2 点出 K。
    rows2 = [rows[0], rows[1]]
    res2 = fit_for_c(rows2, c)
    chk("C 两点拟合自由度 = 0（残差恒 0，不能用来定 K）",
        all(res2[j]["n"] - 2 == 0 for j in range(1, 5)))

    print(f"\n自测：{n_ok} 过 / {n_bad} 败")
    return 0 if n_bad == 0 else 1


def build_row_fake(arm, q, c, K, b, sig=0.01):
    ta = arm.tau_gravity(q)
    f = [0.0, 0.0, -BOTTLE_MASS * S.G]
    A = [[0.0] * 3 for _ in range(6)]
    p0 = arm.tau_point_force(q, S.END_LINK, [0.0, 0.0, 0.0], f)
    for i in range(3):
        e = [0.0, 0.0, 0.0]
        e[i] = 1.0
        t = arm.tau_point_force(q, S.END_LINK, e, f)
        for j in range(6):
            A[j][i] = t[j]
    T = [ta[j] + p0[j] + sum(A[j][i] * c[i] for i in range(3)) for j in range(6)]
    cur = [0.0] * 6
    for j in range(1, 5):
        cur[j] = T[j] / K[j] + b[j]
    return dict(q=list(q), cur=cur, sig=[sig] * 6, tau_arm=ta, A=A, p0=p0,
                loaded=True, n=1000)


def main(argv=None):
    ap = argparse.ArgumentParser(description="S2-1 K 的多姿态最小二乘")
    ap.add_argument("--episodes", default="", help="逗号分隔的 label:dir[:load] 覆盖默认清单")
    ap.add_argument("--last-s", type=float, default=30.0)
    ap.add_argument("--stable-deg", type=float, default=0.3)
    ap.add_argument("--out", default="calibration")
    ap.add_argument("--urdf", default=str(Path(__file__).resolve().parent.parent /
                                         "_vendor" / "urdf" / "piper_description.urdf"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)

    urdf = S.Urdf(Path(a.urdf))
    arm = S.Arm(urdf, end_mass=GRIPPER_MASS, end_com=urdf.gripper_com())

    if a.selftest:
        return selftest(arm)

    eps = DEFAULT_EPISODES
    if a.episodes:
        eps = []
        for item in a.episodes.split(","):
            parts = item.split(":")
            eps.append((parts[0], parts[1], len(parts) < 3 or parts[2] != "0"))

    rows, labels = [], []
    for label, d, loaded in eps:
        p = EP_DIR / d / "samples.jsonl"
        if not p.exists():
            print(f"[skip] {label}: 缺 {p}")
            continue
        r = build_row(arm, p, a.last_s, a.stable_deg, loaded)
        rows.append(r)
        labels.append(label)
        print(f"[load] {label:12s} q={','.join(f'{math.degrees(v):+.1f}' for v in r['q'])}  "
              f"曲={'带载' if loaded else '空载'}  n={r['n']}")
    if len(rows) < 4:
        print(f"[ABORT] 只有 {len(rows)} 个 episode，不够拟合", file=sys.stderr)
        return 2

    out = fit_all(rows, a.last_s)
    out["labels"] = labels
    print()
    report(out, labels, rows, a.last_s)

    # 留一姿态验证
    print()
    print("## 4. 留一姿态验证（拿掉该点重拟合，再预测它自己的电流）")
    ho = holdout_error(rows, out["c"])
    for k, pred in ho:
        dev = " ".join(f"{n}{d:+.4f}/{s:.3f}" for n, d, s in pred)
        print(f"  -{labels[k]:12s} {dev}")
    out["holdout"] = [dict(label=labels[k],
                           dev=[(n, d, s) for n, d, s in pred]) for k, pred in ho]

    od = Path(a.out)
    od.mkdir(parents=True, exist_ok=True)
    kt = dict(
        version="s21_fit 1.0",
        model="I = (tau_arm + m*g*A·c6)/K + b",
        bottleneck_mass_kg=BOTTLE_MASS,
        c6_mm=[round(v * 1000, 2) for v in out["c"]],
        last_s=a.last_s,
        joints={},
    )
    for i in range(6):
        e = out["joints"][JOINT_NAME[i]]
        if i in (0, 5):
            kt["joints"][JOINT_NAME[i]] = dict(
                state="未标定", K=None, b=None, reason=e.get("structural"))
            continue
        kt["joints"][JOINT_NAME[i]] = dict(
            state="已标定" if e["identified"] else "未标定",
            K=round(e["K"], 4), sigma_K=round(e["sigma_K"], 4),
            b=round(e["b"], 5), tau_span=round(e["tau_span"], 3),
            tau_need=round(e["span_need"], 3), chi2_red=round(e["chi2_red"], 3),
            K_over_SDK=round(abs(e["K"]) / K_SDK[i], 3),
            reason="" if e["identified"] else "τ 跨度不足 6σ，K 的不确定度过大")
    (od / "k_table.json").write_text(json.dumps(kt, ensure_ascii=False, indent=2),
                                     encoding="utf-8")
    print(f"\n写出 {od/'k_table.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
