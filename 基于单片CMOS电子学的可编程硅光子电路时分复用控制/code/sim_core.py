#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LPOR 2023 学生版复现 —— 三件事的最小可跑模型
论文: Zanetto et al., "Time-Multiplexed Control of Programmable Silicon Photonic
      Circuits Enabled by Monolithic CMOS Electronics", Laser Photonics Rev. 2023
      DOI: 10.1002/lpor.202300124

不流片、不接硬件，只复现论文里三个能对账的数字:
  (1) 侧栅 nMOS 的开/关比与转移特性       -> 对 Fig.1e
  (2) MUX 建立时间 vs 模拟带宽的自洽性    -> 对 "500 ns / 100 MHz" 这组看似矛盾的数
  (3) S&H 保持时间与电荷注入预算          -> 对 "≈1 s 保持 / 10 Hz 刷新 / 0.03 dB"

用法: python3 sim_core.py            # 打印对照表
      python3 sim_core.py --plot     # 另存 iv_curve.png (需要 matplotlib)
"""
import math
import sys

# ============================================================
# 0. 论文里明确给出的参数（有出处的才填）
# ============================================================
VTH_N = 2.45      # V, nMOS 阈值      [论文 §2]
VTH_P = 1.8       # V, pMOS 阈值      [论文 §2]
S_N = 0.350       # V/dec, 反亚阈值斜率 [论文 §2]
S_P = 0.250       # V/dec
VA_N = 35.0       # V, 厄利电压      [论文 §2]
W_CH = 220e-9     # m, 沟道宽度 = 硅层厚度 [论文 §2]
L_CH = 4e-6       # m, 沟道长度      [论文 §2]
TOX = 200e-9      # m, 栅氧化层厚    [论文 §2]
EPS_OX = 3.45e-11 # F/m, SiO2 介电常数(相对3.9)

# 论文未给出的量，只能自己假设 —— 这些就是你要在报告里写清楚的假设
# μCox 不能随便取: 先由 tox 算出 Cox, 再乘以硅的电子迁移率, 否则反推出来的
# 迁移率会到 8e4 cm^2/Vs 这种非物理值(这是本脚本第一版踩过的坑)
COX = EPS_OX / TOX             # F/m^2 ≈ 0.173 mF/m^2
MU_E = 0.14                    # m^2/Vs, 轻掺杂硅电子迁移率 ≈1400 cm^2/Vs
MU_COX = MU_E * COX            # A/V^2 ≈ 2.4e-5
C_LOAD = 10e-12   # F, 开关负载(PD 结电容 + 焊盘 + 走线)，取 10 pF
C_HOLD = 10e-12   # F, S&H 存储电容 [论文 §4 给出 约 10 pF]
F_DITHER = 2e3    # Hz, 抖动频率   [论文 §5]
F_MUX = 80e3      # Hz, MUX 开关   [论文 §5]
N_PATH = 4        # 每条光路上的监测 PD 数 [论文 §5]

# 论文给出的目标值（用于对账）
TARGET = {
    "settle_ns": 500.0,      # §3: 建立时间 ≈500 ns
    "bw_mhz": 100.0,         # §3: 单开关带宽 ≈100 MHz
    "hold_s": 1.0,           # §4: 保持时间 ≈1 s
    "dither_hz": 2e3,        # §5
    "read_per_path_hz": 20e3,# §5: 每路 4 个传感器 @20 kHz
    "drift_db": 0.03,        # §4: 100 Hz 以上刷新时退化 0.03 dB
    "t_trans_ms": 10.0,      # §5: 整条路径配置 <10 ms
    "p_penalty_db": 0.3,     # §5: 功率代价 0.3 dB
}


# ============================================================
# 1. 侧栅 nMOS: 亚阈值 + 平方律 + 沟道长度调制
# ============================================================
def id_nmos(vgs, vds, vth=VTH_N, s=S_N, mu_cox=MU_COX, va=VA_N):
    """一阶 nMOS 漏极电流(A)。vds 取正值。"""
    nvt = s / math.log(10.0)          # 亚阈值热电压 n*kT/q
    wl = W_CH / L_CH                  # 宽长比 ≈ 0.055
    i_th = 0.5 * mu_cox * wl * nvt ** 2      # vgs=vth 处的电流(连续点)
    if vgs < vth:
        # 亚阈值区: 每 350 mV 掉一个数量级
        return i_th * 10.0 ** ((vgs - vth) / s) * (1.0 - math.exp(-vds / 0.026))
    vov = vgs - vth
    return (0.5 * mu_cox * wl * (2 * vov * vds - vds ** 2) if vds < vov
            else 0.5 * mu_cox * wl * vov ** 2) * (1.0 + vds / va)


def stage1_transistor():
    i_on = id_nmos(3.0, 2.0)
    i_off = id_nmos(0.0, 2.0)
    i_lin = id_nmos(3.0, 0.1)          # 线性区
    return {
        "Vth (V)": (VTH_N, "论文 §2"),
        "反亚阈值斜率 (mV/dec)": (S_N * 1e3, "论文 §2"),
        "厄利电压 (V)": (VA_N, "论文 §2"),
        "W/L": (W_CH / L_CH, "220 nm / 4 µm"),
        "Cox (mF/m^2)": (COX * 1e3, "由 tox=200 nm 算"),
        "μCox (µA/V^2)": (MU_COX * 1e6, "μ=1400 cm^2/Vs × Cox"),
        "Ion @Vgs=3V,Vds=2V (µA)": (i_on * 1e6, "本模型"),
        "Ioff @Vgs=0 (aA)": (i_off * 1e18, "本模型"),
        "开/关比": (i_on / i_off, "本模型, 论文只给曲线(Fig.1e)"),
        "单管线性区 Ron @Vgs=3V (MΩ)": (0.1 / i_lin / 1e6, "本模型"),
        "700 管并联 @Vgs=12V (mA)": (700 * id_nmos(12.0, 3.0) * 1e3, "对论文 §4 的 11 mA"),
    }


# ============================================================
# 2. MUX: 建立时间 与 模拟带宽 能否自洽
# ============================================================
def stage2_mux():
    """若 500 ns 是单极点 RC 建立到 0.1%，反推 Ron*C，再看带宽对不对得上 100 MHz。"""
    ln_acc = math.log(1000.0)                  # 0.1% 精度 ≈ 6.9 τ
    tau = TARGET["settle_ns"] * 1e-9 / ln_acc
    ron = tau / C_LOAD
    bw = 1.0 / (2 * math.pi * tau)
    # 反方向: 若带宽真是 100 MHz, 对应的 tau 和建立时间是多少
    tau_bw = 1.0 / (2 * math.pi * TARGET["bw_mhz"] * 1e6)
    settle_bw = tau_bw * ln_acc * 1e9
    return {
        "由 500 ns 反推的 τ (ns)": (tau * 1e9, "假设单极点"),
        "→ 对应 Ron (kΩ)": (ron / 1e3, "取 C_load=10 pF"),
        "→ 对应带宽 (MHz)": (bw / 1e6, "与论文 100 MHz 差 %.0f 倍" % (TARGET["bw_mhz"] / (bw / 1e6))),
        "若带宽=100 MHz, 建立时间应为 (ns)": (settle_bw, "同样 0.1% 判据"),
        "每路采样率 (kHz)": (F_MUX / N_PATH / 1e3, "论文 §5 给 20 kHz"),
        "抖动频率 / 每路采样率": (F_DITHER / (F_MUX / N_PATH), "需 <0.5 才不混叠"),
        "可复用 PD 数 @100 kHz 读出": (16, "论文 §3"),
    }


# ============================================================
# 3. S&H: 保持时间(漏电) 与 电荷注入(0.03 dB)
# ============================================================
def sh_params():
    """由 12 V -> 11 mA -> 3π 得到移相器的一阶电压-相位灵敏度。"""
    dphi_dv = 3 * math.pi / 12.0        # rad/V [论文 §4: 12 V 给 3π]
    v_bias = 5.5                        # V, 偏置在最小透射点 [论文 §4]
    t_min_db = -26.0                    # dB
    return dphi_dv, v_bias, t_min_db


def stage3_sh(dv_allow=0.1):
    dphi_dv, v_bias, t_min_db = sh_params()
    # (a) 保持时间: 允许栅压跌落 dv_allow(默认 0.1 V) 时的最大漏电
    i_leak = C_HOLD * dv_allow / TARGET["hold_s"]
    # (b) 电荷注入预算: 0.03 dB 允许多大的保持电压误差
    p_rel = 10 ** (t_min_db / 10.0)                    # 最小透射点的相对功率
    dphi = math.pi - 2 * math.asin(math.sqrt(p_rel))   # 偏离 π 的量
    dp_dphi = math.sin(math.pi - dphi) / 2.0           # dP/dφ, 归一化
    d_power = (10 ** (TARGET["drift_db"] / 10.0) - 1.0) * p_rel
    d_phi = d_power / dp_dphi
    d_v = d_phi / dphi_dv
    q_inj = C_HOLD * d_v
    return {
        "移相器灵敏度 dφ/dV (rad/V)": (dphi_dv, "由 3π/12V 一阶外推"),
        "0.03 dB 对应 ΔP/Pmax": (d_power, "在 -26 dB 工作点"),
        "→ 允许的相位漂移 (mrad)": (d_phi * 1e3, "一阶估算"),
        "→ 允许的保持电压误差 (mV)": (d_v * 1e3, "这就是 S&H 的精度指标"),
        "→ 允许的注入电荷 (fC)": (q_inj * 1e15, "Q=C·ΔV, C=10 pF"),
        "保持 1 s 允许的漏电 (pA)": (i_leak * 1e12, "跌落 %.1f V 判据" % dv_allow),
        "→ 对应等效泄放电阻 (GΩ)": (v_bias / i_leak / 1e9, "V/I"),
    }


# ============================================================
# 4. 系统级: 反馈环能否在 10 ms 内收敛
# ============================================================
def stage4_loop(n_iter_need=20, kp=0.3):
    """积分控制器: 每个 MZI 的更新率 = 每路采样率; 看 10 ms 内够迭代几次。"""
    f_upd = F_MUX / N_PATH
    n_iter = f_upd * TARGET["t_trans_ms"] * 1e-3
    # 一阶指数收敛: 误差按 (1-kp)^n 衰减
    err_after = (1 - kp) ** n_iter
    return {
        "每路更新率 (kHz)": (f_upd / 1e3, "= 80 kHz / 4"),
        "10 ms 内迭代次数": (n_iter, "论文 §5"),
        "迭代 %d 次后残差" % n_iter: (err_after, "kp=%.1f 的一阶估算" % kp),
        "能否收敛到 1%": ("能" if err_after < 0.01 else "不能", "判据"),
    }


# ============================================================
def _w(s):
    """按东亚全角宽度计字符显示宽度，用于中英混排对齐。"""
    import unicodedata
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))


def _pad(s, width):
    return s + " " * max(0, width - _w(s))


def show(title, d, unit_fmt="%.4g"):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)
    print(_pad("量", 40) + _pad("值", 16) + "出处")
    print("-" * 78)
    for k, (v, src) in d.items():
        s = v if isinstance(v, str) else unit_fmt % v
        print(_pad(k, 40) + _pad(s, 16) + src)


def plot_iv():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("未安装 matplotlib, 跳过绘图")
        return
    vgs = [i * 0.05 for i in range(0, 81)]
    ids_log = [id_nmos(v, 2.0) for v in vgs]
    fig, ax = plt.subplots(figsize=(6.2, 4.2), dpi=150)
    ax.semilogy(vgs, ids_log, lw=2, color="#8a2b2b")
    ax.axvline(VTH_N, ls="--", c="gray", lw=1)
    ax.text(VTH_N + 0.05, min(ids_log) * 30, "Vth=2.45 V", color="gray", fontsize=9)
    ax.set_xlabel("Vgs (V)"); ax.set_ylabel("Ids (A), Vds=2 V")
    ax.set_title("Side-gate nMOS transfer characteristic (model vs Fig.1e shape)")
    ax.grid(True, which="both", alpha=.25)
    fig.tight_layout(); fig.savefig("iv_curve.png")
    print("\n已保存 iv_curve.png")


if __name__ == "__main__":
    show("阶段 1  侧栅 nMOS 器件级", stage1_transistor())
    show("阶段 2  MUX: 建立时间 vs 带宽", stage2_mux())
    show("阶段 3  S&H: 保持与电荷注入预算", stage3_sh())
    show("阶段 4  抖动锁定反馈环收敛", stage4_loop())
    print("\n" + "=" * 74)
    print("提醒: 带「本模型」字样的都是你自己假设出来的, 不是论文数据。")
    print("      报告里必须把 Δ 归因到假设(μCox、C_load、Ron), 不能只报数。")
    print("=" * 74)
    if "--plot" in sys.argv:
        plot_iv()
