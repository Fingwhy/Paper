#!/usr/bin/env python3
"""实验三·全量：6 段棘轮（松 3 段 + 紧 3 段）的电流推算力矩 vs 传感器实测力矩。

数据组织
--------
臂侧：6 段 twist_log CSV（r1b,r2,r3 = 松；t1,t2,t3 = 紧），每段 t_s 从 1.00 起，
      170 Hz，含 i6/q6/夹爪。
传感器：3 份 jsonl（q3=松方向，q3b=紧方向前两段，q3c=紧方向第3段），各含绝对 epoch。

由于 twist_log 的 t_s 是段内相对时间、无绝对时间戳，无法与传感器精确逐点对齐。
这里采用**各自按段内时间归一**的呈现方式：
- 臂侧：6 段按顺序拼接（段间插 0.5 s 空隙标记棘轮），画 K6·i6 曲线；
- 传感器：3 份按自然时间轴拼接，画 τ 曲线；
- 两者时间轴不同（臂侧无绝对时钟），所以分开两个面板，不硬对齐。

但可以用**各段峰值/均值**做对照表（臂侧峰值 vs 传感器对应段峰值），这是可信的。
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
K6 = 0.3889
K_SENSOR = 0.001
SEGMENTS = [
    ("r1b", "松 1", 0),
    ("r2", "松 2", 0),
    ("r3", "松 3", 0),
    ("t1", "紧 1", 0),
    ("t2", "紧 2", 0),
    ("t3", "紧 3", 0),
]
# 传感器分段：q3 覆盖松方向（前 ~180s），q3b 覆盖紧 1/2（前 ~200s 内），q3c 覆盖紧 3


def load_csv(seg):
    rows = list(csv.DictReader(open(HERE / "data" / f"twistlog_q3_{seg}.csv", encoding="utf-8")))
    t = np.array([float(r["t_s"]) for r in rows])
    i6 = np.array([float(r["i6"]) for r in rows])
    q6 = np.array([float(r["q6"]) for r in rows])
    grip = np.array([float(r["grip_ang_mm"]) for r in rows])
    return t, i6, q6, grip


def load_sensor(f):
    rows = [json.loads(l) for l in open(HERE / "runs" / f"sensor_{f}.jsonl", encoding="utf-8")
            if '"tau"' in l]
    t = np.array([r["t"] for r in rows], float)
    raw = np.array([r["raw"] for r in rows], float)
    return t, raw * K_SENSOR


def svg(curves, ts, title, ylabel, colors, height=260, width=920, marks=None, yfmt="{:+.1f}"):
    padL, padR, padT, padB = 64, 14, 28, 36
    W, H = width, height
    allv = np.concatenate([c[1] for c in curves])
    lo, hi = float(np.nanmin(allv)), float(np.nanmax(allv))
    if hi - lo < 1e-6:
        hi, lo = lo + 1, lo - 1
    pad = 0.1 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    tmin, tmax = float(ts[0]), float(ts[-1])
    sx = lambda t: padL + (t - tmin) / max(1e-9, tmax - tmin) * (W - padL - padR)
    sy = lambda v: H - padB - (v - lo) / (hi - lo) * (H - padT - padB)
    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="18" font-size="13.5" font-weight="600" fill="#1f2937">{title}</text>']
    for i in range(5):
        v = lo + (hi - lo) * i / 4
        out.append(f'<line x1="{padL}" y1="{sy(v):.1f}" x2="{W-padR}" y2="{sy(v):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{padL-8}" y="{sy(v)+4:.1f}" font-size="10" fill="#6b7280" text-anchor="end">{yfmt.format(v)}</text>')
    for i in range(7):
        t = tmin + (tmax - tmin) * i / 6
        out.append(f'<line x1="{sx(t):.1f}" y1="{padT}" x2="{sx(t):.1f}" y2="{H-padB}" stroke="#f5f7fa"/>')
        out.append(f'<text x="{sx(t):.1f}" y="{H-padB+15}" font-size="10" fill="#6b7280" text-anchor="middle">{t:.0f}</text>')
    if marks:
        for x0, x1, lab in marks:
            out.append(f'<rect x="{sx(x0):.1f}" y="{padT}" width="{max(1.0, sx(x1)-sx(x0)):.1f}" '
                       f'height="{H-padT-padB}" fill="#3b82f6" opacity="0.05"/>')
            out.append(f'<text x="{sx(x0)+4:.1f}" y="{padT+11}" font-size="9.5" fill="#3b82f6">{lab}</text>')
    for name, y in curves:
        col = colors.get(name, "#2563eb")
        w = 2.2 if name.startswith("实测") else 1.4
        dash = ' stroke-dasharray="6,3"' if name.startswith("K6") else ""
        pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(ts, y) if np.isfinite(b))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{w}"{dash} stroke-linejoin="round"/>')
    lx = padL + 4
    for name, *_ in curves:
        out.append(f'<rect x="{lx}" y="{padT+2}" width="16" height="3" fill="{colors.get(name, "#2563eb")}"/>')
        out.append(f'<text x="{lx+21}" y="{padT+7}" font-size="10.5" fill="#374151">{name}</text>')
        lx += 21 + 6.4 * len(name) + 16
    out.append(f'<text x="{W-padR}" y="{padT+7}" font-size="10" fill="#9ca3af" text-anchor="end">{ylabel}</text>')
    out.append("</svg>")
    return "".join(out)


def main():
    # ---- 臂侧：6 段拼接 ----
    arm_t, arm_i6, arm_q6 = [], [], []
    marks = []
    base = 0.0
    for seg, label, _ in SEGMENTS:
        t, i6, q6, grip = load_csv(seg)
        t = t - t[0]
        arm_t.append(t + base)
        arm_i6.append(i6)
        arm_q6.append(q6)
        marks.append((base, base + t[-1], label))
        base += t[-1] + 0.8      # 段间留 0.8 s 空隙（棘轮）
    arm_t = np.concatenate(arm_t)
    arm_i6 = np.concatenate(arm_i6)
    arm_q6 = np.concatenate(arm_q6)

    # ---- 传感器：3 份按自然时间拼接 ----
    sen_t, sen_tau = [], []
    smarks = []
    base = 0.0
    for f, segs in (("q3", ["松1", "松2", "松3"]), ("q3b", ["紧1", "紧2"]), ("q3c", ["紧3"])):
        t, tau = load_sensor(f)
        t = t - t[0]
        sen_t.append(t + base)
        sen_tau.append(tau)
        base += t[-1] + 0.5
    sen_t = np.concatenate(sen_t)
    sen_tau = np.concatenate(sen_tau)

    # ---- 每段峰值/均值（臂侧用整段 i6；传感器用该段窗口，窗口同 s6_exp3_aligned）----
    SEG_WIN = {"r1b": ("松", "q3", 20.0, 63.0), "r2": ("松", "q3", 63.0, 124.0),
               "r3": ("松", "q3", 175.0, 239.0), "t1": ("紧", "q3b", 40.0, 70.0),
               "t2": ("紧", "q3b", 142.0, 200.0), "t3": ("紧", "q3c", 0.0, 70.0)}
    seg_rows = []
    for seg, label, _ in SEGMENTS:
        dirc, sf, lo, hi = SEG_WIN[seg]
        t_a, i6_a, q6_a, _g = load_csv(seg)
        a_peak = (i6_a.min() if dirc == "松" else i6_a.max()) * K6
        a_mean = float(i6_a.mean()) * K6
        st_abs, sv = load_sensor(sf)
        t0s = st_abs[0]
        vals = [sv[i] for i in range(len(st_abs)) if lo <= st_abs[i] - t0s <= hi]
        s_peak = (min(vals) if dirc == "松" else max(vals)) if vals else float("nan")
        s_mean = (sum(vals) / len(vals)) if vals else float("nan")
        ratio = abs(a_peak) / abs(s_peak) if abs(s_peak) > 1e-6 else float("nan")
        seg_rows.append((label, dirc, q6_a, a_peak, a_mean, s_peak, s_mean, ratio))

    # ---- 每段峰值/均值对照（臂侧 i6 折算 vs 传感器该段）----
    # 传感器分段：q3 前 3 段各约 46s+回位，粗略按段长切
    # 这里不精确切传感器，只给臂侧各段统计 + 传感器整体峰值
    lines = ["# 实验三 · 全量（松 3 段 + 紧 3 段棘轮）", "",
             f"- K6（冻结）= {K6:.4f} N·m/A；传感器 k = {K_SENSOR} N·m/count", "",
             "## 臂侧各段（K6·i6 统计）", "",
             "| 段 | 方向 | q6 范围(°) | 峰值力矩(N·m) | 均值力矩(N·m) |", "|---|---|---|---|---|"]
    for seg, label, _ in SEGMENTS:
        t, i6, q6, grip = load_csv(seg)
        peak = np.abs(i6).max() * K6
        mean = i6.mean() * K6
        lines.append(f"| {seg} | {label} | {q6[0]:.0f}→{q6[-1]:.0f} | {peak:.2f} | {mean:+.2f} |")

    lines += ["", "## 传感器整体", "",
              f"- 三份记录拼接后：峰值 {np.abs(sen_tau).max():.2f} N·m，"
              f"全程均值 {sen_tau.mean():+.2f} N·m", "",
              "## 说明", "",
              "- 臂侧 twist_log 的 t_s 是**段内相对时间**；本表段峰值用整段（46s）统计。",
              "- 传感器按各段窗口（同 `s6_exp3_aligned.py`）切片统计。",
              "- **2026-10-27 修正**：K6 = 0.3889（旧 10.7882 高估 27.74 倍）、k = 0.001。", "",
              "## 段对段峰值对照", "",
              "| 段 | 方向 | 臂侧峰值(N·m) | 传感器峰值(N·m) | 臂/传感器 |", "|---|---|---|---|---|"]
    for label, dirc, q6_a, a_peak, a_mean, s_peak, s_mean, ratio in seg_rows:
        lines.append(f"| {label} | {dirc} | {a_peak:+.2f} | {s_peak:+.2f} | {ratio:.2f} |")
    lines += ["",
              "- 松方向传感器峰值含**盖子弹开的弹性冲击**（电流法测不到），故峰值比偏低；",
              "  紧方向无此类冲击，峰值比 ≈1（紧1 因信号仅 0.05 N·m 不作数）。",
              "- 稳态均值：松方向臂 "
              f"{seg_rows[0][4]:+.2f}/{seg_rows[1][4]:+.2f}/{seg_rows[2][4]:+.2f} vs 传感器 "
              f"{seg_rows[0][6]:+.2f}/{seg_rows[1][6]:+.2f}/{seg_rows[2][6]:+.2f} N·m；紧方向臂 "
              f"{seg_rows[3][4]:+.2f}/{seg_rows[4][4]:+.2f}/{seg_rows[5][4]:+.2f} vs 传感器 "
              f"{seg_rows[3][6]:+.2f}/{seg_rows[4][6]:+.2f}/{seg_rows[5][6]:+.2f} N·m。"]


    colors = {"K6 预测力矩": "#dc2626", "实测力矩": "#2563eb", "q6 角度": "#059669"}
    arm_svg = svg([("K6 预测力矩", arm_i6 * K6)], arm_t,
                  "实验三·全量 —— 臂侧 K6·i6 预测力矩（6 段棘轮拼接）", "N·m", colors,
                  marks=marks)
    q6_svg = svg([("q6 角度", arm_q6)], arm_t, "实验三·全量 —— J6 角度", "deg", colors,
                 marks=marks, yfmt="{:+.0f}", height=180)
    sen_svg = svg([("实测力矩", sen_tau)], sen_t,
                  "实验三·全量 —— 传感器实测力矩（3 份记录拼接）", "N·m", colors)

    html = f"""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>实验三 全量</title><style>
 body{{margin:0;padding:26px 30px 60px;background:#f8fafc;color:#1f2937;
   font-family:-apple-system,Segoe UI,Helvetica,Arial,"Microsoft YaHei",sans-serif;line-height:1.6;font-size:15px}}
 h1{{font-size:22px;margin:0 0 6px}} h2{{font-size:17px;margin:26px 0 8px}}
 .card{{background:#fff;border:1px solid #e5e7eb;border-radius:10px;padding:16px 18px;margin:14px 0}}
 table{{border-collapse:collapse;width:100%;font-size:13px}}
 th,td{{border-bottom:1px solid #eef2f7;padding:6px 8px;text-align:right}}
 th{{background:#f1f5f9}} td:first-child,th:first-child{{text-align:left}}
 .note{{color:#6b7280;font-size:12.5px}}
</style></head><body>
<h1>实验三 · 全量（松 3 段 + 紧 3 段棘轮）</h1>
<div class="note">K6 = {K6:.4f} N·m/A（冻结）　|　传感器 k = {K_SENSOR} N·m/count</div>
<div class="card"><table>
<tr><th>段</th><th>方向</th><th>q6 范围(°)</th><th>峰值力矩(N·m)</th><th>均值力矩(N·m)</th></tr>"""
    for seg, label, _ in SEGMENTS:
        t, i6, q6, grip = load_csv(seg)
        peak = np.abs(i6).max() * K6
        mean = i6.mean() * K6
        html += f"<tr><td>{seg}</td><td>{label}</td><td>{q6[0]:.0f}→{q6[-1]:.0f}</td>" \
                f"<td>{peak:.2f}</td><td>{mean:+.2f}</td></tr>"
    html += "</table></div>\n<div class=\"card\"><table>\n" \
            "<tr><th>段</th><th>方向</th><th>臂侧峰值(N·m)</th><th>传感器峰值(N·m)</th><th>臂/传感器</th></tr>"
    for label, dirc, q6_a, a_peak, a_mean, s_peak, s_mean, ratio in seg_rows:
        html += (f"<tr><td>{label}</td><td>{dirc}</td><td>{a_peak:+.2f}</td>"
                 f"<td>{s_peak:+.2f}</td><td>{ratio:.2f}</td></tr>")
    html += ("</table>\n<div class=\"note\" style=\"margin-top:8px\">"
             "松方向传感器峰值含「盖子弹开」弹性冲击（电流法测不到），故峰值比偏低；"
             "紧方向无此冲击，峰值比 ≈1（紧1 信号仅 0.05 N·m 不作数）。</div></div>\n"
             f"<div class=\"card\">{arm_svg}</div>")
    html += f"""
<div class="card">{q6_svg}</div>
<div class="card">{sen_svg}
<div class="note">臂侧为 6 段拼接（段间 0.8 s 空隙标记棘轮），传感器为 3 份记录拼接，各自时间轴。
逐点对齐版见 <code>runs/exp3_aligned.html</code>（OFF=−8.2 s）。</div></div>
</body></html>"""


    (HERE / "QUANT_EXP3.md").write_text("\n".join(lines), encoding="utf-8")
    (HERE / "runs" / "exp3_full.html").write_text(html, encoding="utf-8")
    print("\n".join(lines))
    print("已输出 QUANT_EXP3.md 与 runs/exp3_full.html")


if __name__ == "__main__":
    main()
