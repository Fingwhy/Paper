#!/usr/bin/env python3
"""实验三·全量：每段单独的「臂侧预测 vs 传感器实测」对比图。

对齐方法
--------
臂侧 twist_log 的 t_s 是段内相对时间（无绝对时间戳），传感器是绝对时间。
两路记录的是**同一个物理过程**（拧瓶盖），力矩随时间形状应高度重合。
因此用**互相关**：把臂侧 i6（折算成 K6·i6）曲线在传感器 τ 曲线上滑动，
找到相关系数最大的滞后，即得两路的时间偏移。

每段输出一张图：同一横轴（传感器时标），红=臂侧 K6·i6，蓝=传感器实测。
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
K6 = 0.3889
K_SENSOR = 0.001

# 每段对应的传感器文件 + 该段在传感器文件中的大致时间窗（秒，用于缩小互相关搜索范围）
SEGMENTS = [
    ("松1", "r1b", "q3", (20, 70)),
    ("松2", "r2", "q3", (65, 110)),
    ("松3", "r3", "q3", (110, 130)),
    ("紧1", "t1", "q3b", (0, 43)),
    ("紧2", "t2", "q3b", (43, 70)),
    ("紧3", "t3", "q3c", (5, 70)),
]


def load_arm(seg):
    rows = list(csv.DictReader(open(HERE / "data" / f"twistlog_q3_{seg}.csv", encoding="utf-8")))
    t = np.array([float(r["t_s"]) for r in rows])
    i6 = np.array([float(r["i6"]) for r in rows])
    q6 = np.array([float(r["q6"]) for r in rows])
    return t, i6, q6


def load_sensor(f):
    rows = [json.loads(l) for l in open(HERE / "runs" / f"sensor_{f}.jsonl", encoding="utf-8")
            if '"tau"' in l]
    t = np.array([r["t"] for r in rows], float)
    tau = np.array([r["raw"] for r in rows], float) * K_SENSOR
    t0 = json.loads(open(HERE / "runs" / f"sensor_{f}.jsonl", encoding="utf-8").readline())["t0_epoch"]
    return t - t0, tau, t0      # 返回相对时间（秒）


def align(arm_t, arm_i6, sen_t, sen_tau, window):
    """在 window（传感器时标范围）内，用互相关找 arm 与 sen 的最佳偏移。

    返回 (偏移 lag，使 sen_t 与 arm_t+lag 对齐，相关系数)。
    """
    lo, hi = window
    sen_m = (sen_t >= lo) & (sen_t <= hi)
    st = sen_t[sen_m]
    sv = sen_tau[sen_m]
    if len(st) < 20:
        return 0.0, float("nan")

    # 臂侧信号（K6·i6），下采样到与传感器可比
    a = K6 * arm_i6
    # 网格：在窗内以 0.05 s 步长扫 lag
    best = (0.0, -2.0)
    for lag in np.arange(-20, 20, 0.1):
        # 传感器时刻对应臂时刻 = sen_t - lag
        pred = np.interp(st - lag, arm_t, a, left=np.nan, right=np.nan)
        ok = ~np.isnan(pred)
        if ok.sum() < 30:
            continue
        if np.std(pred[ok]) < 1e-6 or np.std(sv[ok]) < 1e-6:
            continue
        r = float(np.corrcoef(pred[ok], sv[ok])[0, 1])
        if abs(r) > abs(best[1]):
            best = (lag, r)
    return best


def svg(arm_t, arm_v, sen_t, sen_v, title, colors):
    """两条曲线同一横轴。arm_t/sen_t 都是「对齐后的统一时间轴」。"""
    padL, padR, padT, padB = 64, 14, 30, 36
    W, H = 920, 300
    allv = np.concatenate([arm_v, sen_v])
    lo, hi = float(np.nanmin(allv)), float(np.nanmax(allv))
    if hi - lo < 1e-6:
        hi, lo = lo + 1, lo - 1
    pad = 0.12 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    tmin = min(float(arm_t[0]), float(sen_t[0]))
    tmax = max(float(arm_t[-1]), float(sen_t[-1]))
    sx = lambda t: padL + (t - tmin) / max(1e-9, tmax - tmin) * (W - padL - padR)
    sy = lambda v: H - padB - (v - lo) / (hi - lo) * (H - padT - padB)
    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="19" font-size="14" font-weight="600" fill="#1f2937">{title}</text>']
    for i in range(5):
        v = lo + (hi - lo) * i / 4
        out.append(f'<line x1="{padL}" y1="{sy(v):.1f}" x2="{W-padR}" y2="{sy(v):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{padL-8}" y="{sy(v)+4:.1f}" font-size="10" fill="#6b7280" text-anchor="end">{v:+.1f}</text>')
    for i in range(8):
        t = tmin + (tmax - tmin) * i / 7
        out.append(f'<line x1="{sx(t):.1f}" y1="{padT}" x2="{sx(t):.1f}" y2="{H-padB}" stroke="#f5f7fa"/>')
        out.append(f'<text x="{sx(t):.1f}" y="{H-padB+15}" font-size="10" fill="#6b7280" text-anchor="middle">{t:.0f}s</text>')
    # 传感器（蓝，粗）
    pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(sen_t, sen_v) if np.isfinite(b))
    out.append(f'<polyline points="{pts}" fill="none" stroke="{colors["实测"]}" stroke-width="2.2" stroke-linejoin="round"/>')
    # 臂侧（红，虚线）
    pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(arm_t, arm_v) if np.isfinite(b))
    out.append(f'<polyline points="{pts}" fill="none" stroke="{colors["预测"]}" stroke-width="1.5" '
               f'stroke-dasharray="6,3" stroke-linejoin="round"/>')
    out.append(f'<rect x="{padL+4}" y="{padT+3}" width="16" height="3" fill="{colors["实测"]}"/>')
    out.append(f'<text x="{padL+25}" y="{padT+8}" font-size="11" fill="#374151">传感器实测</text>')
    out.append(f'<rect x="{padL+115}" y="{padT+3}" width="16" height="3" fill="{colors["预测"]}"/>')
    out.append(f'<text x="{padL+136}" y="{padT+8}" font-size="11" fill="#374151">臂侧 K6·i6</text>')
    out.append(f'<text x="{W-padR}" y="{padT+8}" font-size="10" fill="#9ca3af" text-anchor="end">N·m</text>')
    out.append("</svg>")
    return "".join(out)


def main():
    colors = {"实测": "#2563eb", "预测": "#dc2626"}
    html = ["""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>实验三 全量 · 分段对比</title><style>
 body{margin:0;padding:26px 30px 60px;background:#f8fafc;color:#1f2937;
   font-family:-apple-system,Segoe UI,Helvetica,Arial,"Microsoft YaHei",sans-serif;line-height:1.6;font-size:15px}
 h1{font-size:22px;margin:0 0 6px} h2{font-size:17px;margin:24px 0 8px}
 .card{background:#fff;border:1px solid #e5e7eb;border-radius:10px;padding:14px 16px;margin:12px 0}
 .seghead{font-size:15px;font-weight:600;color:#1f2937;margin:0 0 4px}
 .stat{font-size:12.5px;color:#6b7280;margin:0 0 8px}
</style></head><body>
<h1>实验三 · 全量 —— 每段单独对比</h1>
<div style="font-size:13px;color:#6b7280">K6=0.3889 N·m/A（冻结）｜蓝=传感器实测，红=臂侧 K6·i6 预测（互相关对齐）</div>"""]

    lines = ["# 实验三·全量 分段对比", "", "K6 = 0.3889 N·m/A（冻结）；对齐 = 互相关。", ""]

    for label, seg, sf, window in SEGMENTS:
        arm_t, arm_i6, arm_q6 = load_arm(seg)
        sen_t, sen_tau, _ = load_sensor(sf)
        lag, r = align(arm_t, arm_i6, sen_t, sen_tau, window)

        # 对齐：传感器时标 = 臂时标 + lag？定义 arm 对齐到 sen：sen_t ≈ arm_t + lag
        # 统一画图：把臂侧映射到传感器时标
        arm_t_aligned = arm_t + lag
        # 截取公共窗口
        lo, hi = window
        m_sen = (sen_t >= lo) & (sen_t <= hi)
        m_arm = (arm_t_aligned >= lo) & (arm_t_aligned <= hi)

        arm_v = K6 * arm_i6[m_arm]
        sen_v = sen_tau[m_sen]
        at = arm_t_aligned[m_arm]
        st = sen_t[m_sen]
        if len(at) < 5 or len(st) < 5:
            lines.append(f"## {label}（{seg}）—— 该段窗口内数据不足，跳过")
            lines.append("")
            continue

        # 统计（在公共重叠区）
        # 用插值到同一网格
        g = np.arange(max(at[0], st[0]), min(at[-1], st[-1]), 0.05)
        a = np.interp(g, at, arm_v)
        s = np.interp(g, st, sen_v)
        mae = float(np.mean(np.abs(a - s)))
        bias = float(np.mean(a - s))
        ratio = float(a.mean() / s.mean()) if abs(s.mean()) > 0.05 else float("nan")

        title = f"{label}（{seg}）　q6 {arm_q6[0]:.0f}→{arm_q6[-1]:.0f}°"
        html.append(f'<div class="card"><div class="seghead">{title}</div>'
                    f'<div class="stat">对齐 lag={lag:+.2f}s（相关 r={r:+.2f}）｜'
                    f'MAE={mae:.2f} 偏差={bias:+.2f} 臂/实测={ratio:.2f}</div>')
        html.append(svg(at, arm_v, st, sen_v, title, colors))
        html.append("</div>")

        lines.append(f"## {label}（{seg}）")
        lines.append(f"- 对齐 lag={lag:+.2f}s，相关 r={r:+.2f}")
        lines.append(f"- MAE {mae:.3f}，偏差 {bias:+.3f}，臂/实测比值 {ratio:.2f}")
        lines.append("")

    html.append("</body></html>")
    (HERE / "QUANT_EXP3_SEG.md").write_text("\n".join(lines), encoding="utf-8")
    (HERE / "runs" / "exp3_segments.html").write_text("".join(html), encoding="utf-8")
    print("\n".join(lines))
    print("\n已输出 QUANT_EXP3_SEG.md 与 runs/exp3_segments.html")


if __name__ == "__main__":
    main()
