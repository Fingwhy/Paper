#!/usr/bin/env python
"""三方可观测力矩对比：传感器实测 vs 电流推算(K·I) vs NEXT 预测(τ̂_ext)。

运行机器与目录（Windows 本机）
------------------------------
    C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6/

数据从哪来
----------
1. 传感器：`sensor_rec.py`（Windows，COM3，50 Hz，落绝对 epoch 时间）
2. 机械臂：VM 上 `stage1_l0.py record` → `stage1_l0.py parse --hz 50`（落相对时间 t）
   两边**同时启动**，本脚本用
   （a）`--arm-t0` 给的 epoch，或（b）互相关自动找滞后，
   把两路对齐。

三个量
------
    τ_m    = K6 · i6                     电流推算的电机力矩（K6 = 0.3889 N·m/A）
    τ̂_ext  = τ_m − τ̂_f(q, q̇, Δq_d)      NEXT 预测的自由空间力矩扣掉后的外力矩
    τ_sens = 传感器实测

对齐做的第一件事是定符号：不同接线方向会让 τ_sens 整体反号，用 |r| 最大的那个。

用法
----
    # 只要 K·I 对比（不需要 NEXT）
    python compare_torque.py --sensor runs/sync1_sensor.jsonl \
        --samples ~/stage0/20261021_stage6/episodes/SYNC1/samples.jsonl \
        --arm-t0 1792981323.25 --out runs/cmp_sync1.json --svg runs/cmp_sync1.svg

    # 加 NEXT 预测（需要 stage4 的 ckpt）
    python compare_torque.py ... --ckpt ../20261020_stage4/runs/next_v9_h75/model.pt
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE4 = HERE.parent / "20261020_stage4"
sys.path.insert(0, str(STAGE4))
sys.path.insert(0, str(HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"))

K_DEFAULT = dict(J1=4.7185, J2=4.7185, J3=4.7185, J4=0.7446, J5=0.7446, **{"J6": 0.3889})
K6_DEFAULT = 0.3889
FPS = 50.0


# ------------------------------------------------------------------ 读取
def load_sensor(path: Path):
    t, tau = [], []
    hdr = None
    for line in open(path, encoding="utf-8"):
        r = json.loads(line)
        if r.get("type") == "header":
            hdr = r
            continue
        t.append(r["t"])
        tau.append(r["tau"])
    return np.asarray(t), np.asarray(tau), hdr


def load_arm(path: Path):
    """samples.jsonl（stage1_l0.py parse 的输出）。"""
    t, q, dq, cur, qc = [], [], [], [], []
    for line in open(path, encoding="utf-8"):
        r = json.loads(line)
        if not (r.get("q") and r.get("dq") and r.get("cur")):
            continue
        if any(x is None for x in r["q"]) or any(x is None for x in r["dq"]) \
           or any(x is None for x in r["cur"]):
            continue
        t.append(r["t"]); q.append(r["q"]); dq.append(r["dq"]); cur.append(r["cur"])
        qc.append(r["q_cmd"] if r.get("q_cmd") else [None] * 6)
    return (np.asarray(t), np.asarray(q, float), np.asarray(dq, float),
            np.asarray(cur, float), np.asarray(qc, float))


def to_uniform(t, *arrs, fps=FPS):
    """把不等间隔的样本插值到从 t[0] 起的等间隔网格。"""
    n = int(round((t[-1] - t[0]) * fps)) + 1
    tu = t[0] + np.arange(n) / fps
    out = [np.interp(tu, t, a) if a.ndim == 1 else
           np.stack([np.interp(tu, t, a[:, j]) for j in range(a.shape[1])], axis=1)
           for a in arrs]
    return tu, out


# ------------------------------------------------------------------ 对齐
def best_sign(x, y):
    """τ_sens 可能整体反号；按 |r| 最大选符号。"""
    x = x - x.mean(); y = y - y.mean()
    sx, sy = x.std(), y.std()
    if sx < 1e-12 or sy < 1e-12:
        return 1.0, 0.0
    r = float(np.mean(x * y) / (sx * sy))
    return (1.0 if r >= 0 else -1.0), abs(r)


def align(sensor_t, sensor_tau, arm_t, arm_ref, arm_t0=None, auto=True, window=3.0):
    """返回 (offset, lag_s, 相关)。对齐后 t_arm_epoch = offset + arm_t。

    为什么用互相关而不是只靠 --arm-t0：两边的起点差了多久是未知的
    （VM 上开进程、CAN 起来、串口打开各有几百毫秒到几秒的延迟）。
    但如果两边记录的是**同一个物理过程**，|τ| 的时间形状就该重合，
    所以直接搜一个使 |τ_sens| 与 |τ_m| 相关最大的滞后，比相信墙钟可靠。
    """
    offset = arm_t0 if arm_t0 is not None else sensor_t[0]
    if not auto:
        return offset, 0.0, 1.0
    grid = np.arange(-window, window + 1e-9, 0.005)
    ta = offset + arm_t
    ref = np.abs(arm_ref)
    src = np.interp(sensor_t, ta, ref, left=np.nan, right=np.nan)
    ok = ~np.isnan(src)
    best, best_lag = -2.0, 0.0
    for lag in grid:
        shifted = np.interp(sensor_t + lag, ta, ref, left=np.nan, right=np.nan)
        m = ok & ~np.isnan(shifted)
        if m.sum() < 50:
            continue
        a, b = np.abs(sensor_tau[m]), shifted[m]
        if a.std() < 1e-12 or b.std() < 1e-12:
            continue
        r = float(np.corrcoef(a, b)[0, 1])
        if r > best:
            best, best_lag = r, lag
    # 注意符号：上面比的是 interp(sensor_t + lag, ta)，等价于把臂侧整体右移 lag，
    # 所以臂侧的 epoch 起点应当是 offset − lag（不是 offset + lag）。
    # 写错这一处会让重叠区间整体偏 2·lag，症状是相关系数很低、拟合 K 差一大截。
    return offset - best_lag, best_lag, best


# ------------------------------------------------------------------ NEXT
class Next:
    def __init__(self, ckpt: Path, history: int = 75):
        import torch
        from factr2_next.training.models import build_model
        self.torch = torch
        ck = torch.load(ckpt, map_location="cpu", weights_only=False)
        self.norm = {k: np.asarray(v, np.float32) for k, v in ck["norm"].items()}
        self.H = ck.get("history", history)
        self.model = build_model(ck["model"], ck["input_size"], ck["output_size"], self.H)
        self.model.load_state_dict(ck["model_state_dict"])
        self.model.eval()

    def predict(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, np.float32)
        xp = np.vstack([np.repeat(x[:1], self.H - 1, axis=0), x])
        win = np.lib.stride_tricks.sliding_window_view(xp, self.H, axis=0)   # (N,18,H)
        win = np.transpose(win, (0, 2, 1))
        win = (win - self.norm["x_mean"]) / self.norm["x_std"]
        with self.torch.no_grad():
            out = self.model(self.torch.as_tensor(win, dtype=self.torch.float32))
        return np.asarray(out) * self.norm["y_std"] + self.norm["y_mean"]


# ------------------------------------------------------------------ 作图
def write_svg(path: Path, t, curves, title):
    W, H = 980, 420
    L, R, T, B = 78, 130, 56, 54
    pw, ph = W - L - R, H - T - B
    lo = min(np.nanmin(v) for _, v in curves)
    hi = max(np.nanmax(v) for _, v in curves)
    pad = (hi - lo) * 0.12 + 1e-6
    lo, hi = lo - pad, hi + pad

    def X(i):
        return L + pw * i / max(1, len(t) - 1)

    def Y(v):
        return T + ph * (1 - (v - lo) / (hi - lo))

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}" font-family="Segoe UI,Arial,sans-serif">',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
         f'<text x="{L}" y="30" font-size="15" font-weight="600" fill="#1c2833">{title}</text>']
    for k in range(6):
        v = lo + (hi - lo) * k / 5
        yy = Y(v)
        s.append(f'<line x1="{L}" y1="{yy:.1f}" x2="{L+pw}" y2="{yy:.1f}" stroke="#eceff1"/>')
        s.append(f'<text x="{L-8}" y="{yy+4:.1f}" font-size="11" fill="#607d8b" '
                 f'text-anchor="end">{v:+.2f}</text>')
    s.append(f'<text x="26" y="{T+ph/2:.0f}" font-size="12" fill="#455a64" '
             f'transform="rotate(-90 26 {T+ph/2:.0f})" text-anchor="middle">力矩 N·m</text>')
    cols = ["#c0392b", "#2e86de", "#27ae60"]
    for i, (name, v) in enumerate(curves):
        pts = " ".join(f"{X(k):.1f},{Y(float(v[k])):.1f}"
                       for k in range(0, len(v), max(1, len(v) // 1400)))
        s.append(f'<polyline points="{pts}" fill="none" stroke="{cols[i%3]}" '
                 f'stroke-width="1.6" stroke-opacity="0.9"/>')
        ly = T + 6 + i * 20
        s.append(f'<rect x="{L+pw+14}" y="{ly-9}" width="11" height="11" fill="{cols[i%3]}"/>')
        s.append(f'<text x="{L+pw+30}" y="{ly+1}" font-size="11.5" fill="#37474f">{name}</text>')
    s.append(f'<text x="{L+pw}" y="{H-16}" font-size="11" fill="#90a4ae" '
             f'text-anchor="end">时长 {t[-1]-t[0]:.1f} s / {len(t)} 帧</text>')
    s.append("</svg>")
    path.write_text("\n".join(s), encoding="utf-8")


# ------------------------------------------------------------------ 主
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--samples", required=True, help="stage1_l0.py parse 输出的 samples.jsonl")
    ap.add_argument("--arm-t0", type=float, default=None, help="臂侧 t=0 的 epoch 秒")
    ap.add_argument("--no-auto-lag", action="store_true")
    ap.add_argument("--lag-window", type=float, default=3.0, help="互相关搜索半径（秒）")
    ap.add_argument("--ckpt", default="", help="NEXT 权重；不给就只比 K·I")
    ap.add_argument("--k6", type=float, default=K6_DEFAULT)
    ap.add_argument("--joint", type=int, default=6, help="用哪一轴的电流（默认 J6）")
    ap.add_argument("--out", default=str(HERE / "runs" / "cmp.json"))
    ap.add_argument("--svg", default=str(HERE / "runs" / "cmp.svg"))
    a = ap.parse_args()

    st, stau, shdr = load_sensor(Path(a.sensor))
    at, q, dq, cur, qc = load_arm(Path(a.samples))
    if len(q) < 200:
        print("臂侧样本太少（%d）" % len(q))
        return 1

    j = a.joint - 1
    tau_m = a.k6 * cur[:, j]
    offset, lag, xr = align(st, stau, at, tau_m, a.arm_t0,
                            auto=not a.no_auto_lag, window=a.lag_window)
    print("对齐：臂侧起点 epoch = %.3f，互相关滞后修正 %+.3f s（|τ| 相关 %.2f）"
          % (offset, lag, xr))
    print("传感器起点 epoch = %.3f" % st[0])

    # 把臂侧插值到传感器时间轴
    ta_epoch = offset + at
    m = (st >= ta_epoch[0]) & (st <= ta_epoch[-1])
    if m.sum() < 100:
        print("两路记录没有重叠区间：传感器 %s ~ %s，臂 %s ~ %s"
              % (st[0], st[-1], ta_epoch[0], ta_epoch[-1]))
        return 2
    tt = st[m]
    tau_s = stau[m]
    i_j = np.interp(tt, ta_epoch, cur[:, j])
    tmi = a.k6 * i_j
    qi = np.stack([np.interp(tt, ta_epoch, q[:, k]) for k in range(6)], axis=1)
    dqi = np.stack([np.interp(tt, ta_epoch, dq[:, k]) for k in range(6)], axis=1)
    qci = qc.copy()
    qcf = np.stack([np.interp(tt, ta_epoch,
                              np.where(np.isnan(qc[:, k]), 0.0, qc[:, k]))
                    for k in range(6)], axis=1)

    sign, r0 = best_sign(tau_s, tmi)
    tau_s = tau_s * sign
    print("符号：τ_sens × %+.0f（与 K·I 的 |r| = %.3f）" % (sign, r0))
    print("重叠 %.2f s / %d 帧，i%d 范围 [%+.3f, %+.3f] A，τ_sens [%+.3f, %+.3f] N·m\n"
          % (tt[-1] - tt[0], len(tt), a.joint, i_j.min(), i_j.max(),
             tau_s.min(), tau_s.max()))

    curves = [("传感器实测", tau_s), ("K·I", tmi)]
    tau_ext = None

    if a.ckpt:
        try:
            # 单位自检：NEXT 训练用的是弧度，parse 必须带 --pos251-unit mrad。
            # 若 q 的绝对值远超 π，说明拿到的是角度，喂进去预测必错。
            if np.abs(q).max() > 10.0:
                print("！q 的最大值 %.1f 远超 π，看起来是**角度**而不是弧度。" % np.abs(q).max())
                print("  NEXT 输入要求弧度，请用 stage1_l0.py parse --pos251-unit mrad 重新解析。")
            nx = Next(Path(a.ckpt))
            tu, (qu, dqu, qcu) = to_uniform(at, q, dq, np.nan_to_num(qc, nan=0.0))
            x = np.concatenate([qu, dqu, qcu - qu], axis=1)
            tau_f = nx.predict(x)
            tau_m_u = a.k6 * np.interp(tu, at, cur[:, j])
            tau_ext_u = tau_m_u - tau_f[:, j]
            tau_ext = np.interp(tt, offset + tu, tau_ext_u)
            curves.append(("NEXT τ̂_ext", tau_ext))
            print("NEXT 已加载（历史 %d 步）。" % nx.H)
            print("！注意：q 范围 J1 [%.0f, %.0f]° J6 [%.0f, %.0f]°，"
                  "若超出训练分布（J1>120° / J6>90°），τ̂_ext 不可信"
                  % (np.degrees(qu[:, 0]).min(), np.degrees(qu[:, 0]).max(),
                     np.degrees(qu[:, 5]).min(), np.degrees(qu[:, 5]).max()))
        except Exception as e:
            print("NEXT 加载失败，跳过该对比：%s: %s" % (type(e).__name__, e))

    # ---- 统计 ----
    def stat(ref, est, name):
        d = est - ref
        r = float(np.corrcoef(ref, est)[0, 1]) if ref.std() > 1e-9 and est.std() > 1e-9 else float("nan")
        return dict(name=name, r=r, mae=float(np.abs(d).mean()),
                    bias=float(d.mean()), rmse=float(np.sqrt((d ** 2).mean())),
                    ref_range=[float(ref.min()), float(ref.max())])

    rows = [stat(tau_s, tmi, "K·I vs 传感器")]
    if tau_ext is not None:
        rows.append(stat(tau_s, tau_ext, "NEXT τ̂_ext vs 传感器"))

    # 线性拟合：τ_sens = slope·i + intercept ⇒ slope 就是实测 K
    A = np.vstack([i_j, np.ones_like(i_j)]).T
    sol, *_ = np.linalg.lstsq(A, tau_s, rcond=None)
    fit = A @ sol
    print("\n%-24s %8s %8s %8s" % ("对比", "r", "MAE", "bias"))
    for r_ in rows:
        print("%-24s %8.3f %8.4f %8.4f" % (r_["name"], r_["r"], r_["mae"], r_["bias"]))
    print("\n线性拟合 τ_sens = K·i%d + b：K = %.4f N·m/A（现用 %.4f，差 %+.1f%%），"
          "b = %+.4f N·m（摩擦）" % (a.joint, sol[0], a.k6, 100 * (sol[0] / a.k6 - 1), sol[1]))
    print("拟合残差 RMS = %.4f N·m" % float(np.sqrt(np.mean((fit - tau_s) ** 2))))

    out = Path(a.out); out.parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(offset=offset, lag=lag, xcorr=xr, sign=sign,
                   n=int(len(tt)), dur=float(tt[-1] - tt[0]),
                   k6_used=a.k6, k6_fit=float(sol[0]), friction_fit=float(sol[1]),
                   stats=rows), open(out, "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    print("\n已保存", out)

    svg = Path(a.svg); svg.parent.mkdir(parents=True, exist_ok=True)
    step = max(1, len(tt) // 3000)
    write_svg(svg, tt[::step] - tt[0],
              [(n, v[::step]) for n, v in curves],
              "传感器实测 vs 电流推算 vs NEXT 预测（J%d）" % a.joint)
    print("已保存", svg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
