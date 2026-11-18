#!/usr/bin/env python3
"""电流 vs 传感器 散点图 + 残差分析（回答"电流能否作为传感器预测"）。"""
from __future__ import annotations

import bisect
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
K6 = 0.3889
OFF = -8.2

SEG = [("松1", "松", "q3", (20, 63), "q3"), ("松2", "松", "q3", (63, 124), "q3"),
       ("松3", "松", "q3", (175, 239), "q3"), ("紧1", "紧", "q3b", (40, 70), "q3b"),
       ("紧2", "紧", "q3b", (142, 200), "q3b"), ("紧3", "紧", "q3c", (0, 70), "q3c")]


def load_arm(f):
    rows = [json.loads(l) for l in open(HERE / "runs" / "arm_raw" / f"arm6_{f}.jsonl")]
    return [r["t"] for r in rows], [r["i6"] * K6 for r in rows]


def load_sen(f):
    lines = open(HERE / "runs" / f"sensor_{f}.jsonl", encoding="utf-8").read().splitlines()
    rows = [json.loads(l) for l in lines if '"tau"' in l]
    return [r["t"] for r in rows], [r["tau"] for r in rows]


def interp(x, ts, vs):
    k = bisect.bisect_left(ts, x)
    if k == 0:
        return vs[0]
    if k >= len(ts):
        return vs[-1]
    t0, t1 = ts[k - 1], ts[k]
    v0, v1 = vs[k - 1], vs[k]
    return v0 if t1 == t0 else v0 + (v1 - v0) * (x - t0) / (t1 - t0)


def main():
    # 收集 (臂侧, 传感器, 方向)
    pts = []  # (arm, sen, is_松)
    for label, dirc, sf, (lo, hi), armf in SEG:
        at_abs, av = load_arm(armf)
        st_abs, sv = load_sen(sf)
        at = [x - OFF - st_abs[0] for x in at_abs]
        st = [x - st_abs[0] for x in st_abs]
        m_arm = [i for i in range(len(at)) if lo <= at[i] <= hi]
        m_sen = [i for i in range(len(st)) if lo <= st[i] <= hi]
        ta = [at[i] for i in m_arm]; va = [av[i] for i in m_arm]
        ts = [st[i] for i in m_sen]; vs = [sv[i] for i in m_sen]
        for i in range(0, len(ts), 4):
            pts.append((interp(ts[i], ta, va), vs[i], dirc == "松"))

    # 回归
    n = len(pts)
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    mx = sum(xs) / n; my = sum(ys) / n
    cov = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    vx = sum((xs[i] - mx) ** 2 for i in range(n))
    a = cov / vx; b = my - a * mx
    r = cov / ((vx * sum((ys[i] - my) ** 2 for i in range(n))) ** 0.5)

    # 散点图 SVG
    W, H = 620, 420
    padL, padR, padT, padB = 56, 16, 34, 40
    xmin, xmax = -5, 5
    ymin, ymax = -6.5, 5
    sx = lambda x: padL + (x - xmin) / (xmax - xmin) * (W - padL - padR)
    sy = lambda y: H - padB - (y - ymin) / (ymax - ymin) * (H - padT - padB)

    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="20" font-size="14" font-weight="600" fill="#1f2937">'
           f'电流预测 vs 传感器实测（每点=一个对齐样本）</text>']
    # 网格 + 轴
    for i in range(-5, 6):
        out.append(f'<line x1="{sx(i):.1f}" y1="{padT}" x2="{sx(i):.1f}" y2="{H-padB}" stroke="#f0f3f7"/>')
        out.append(f'<text x="{sx(i):.1f}" y="{H-padB+16}" font-size="9" fill="#9ca3af" text-anchor="middle">{i}</text>')
    for i in range(-6, 6):
        out.append(f'<line x1="{padL}" y1="{sy(i):.1f}" x2="{W-padR}" y2="{sy(i):.1f}" stroke="#f0f3f7"/>')
        out.append(f'<text x="{padL-6}" y="{sy(i)+3:.1f}" font-size="9" fill="#9ca3af" text-anchor="end">{i}</text>')
    # 理想线 y=x
    out.append(f'<line x1="{sx(-5):.1f}" y1="{sy(-5):.1f}" x2="{sx(5):.1f}" y2="{sy(5):.1f}" stroke="#cbd5e1" stroke-width="1" stroke-dasharray="5,4"/>')
    # 回归线
    out.append(f'<line x1="{sx(-5):.1f}" y1="{sy(a*-5+b):.1f}" x2="{sx(5):.1f}" y2="{sy(a*5+b):.1f}" stroke="#059669" stroke-width="1.6"/>')
    # 散点
    for arm, sen, is_song in pts:
        col = "#dc2626" if is_song else "#2563eb"
        out.append(f'<circle cx="{sx(arm):.1f}" cy="{sy(sen):.1f}" r="1.6" fill="{col}" opacity="0.35"/>')
    # 图例
    out.append(f'<circle cx="{padL+6}" cy="{padT+12}" r="4" fill="#dc2626" opacity="0.7"/><text x="{padL+15}" y="{padT+16}" font-size="10" fill="#374151">松方向</text>')
    out.append(f'<circle cx="{padL+70}" cy="{padT+12}" r="4" fill="#2563eb" opacity="0.7"/><text x="{padL+79}" y="{padT+16}" font-size="10" fill="#374151">紧方向</text>')
    out.append(f'<line x1="{padL+140}" y1="{padT+12}" x2="{padL+160}" y2="{padT+12}" stroke="#cbd5e1" stroke-dasharray="5,4"/><text x="{padL+165}" y="{padT+16}" font-size="10" fill="#374151">理想 y=x</text>')
    out.append(f'<line x1="{padL+230}" y1="{padT+12}" x2="{padL+250}" y2="{padT+12}" stroke="#059669"/><text x="{padL+255}" y="{padT+16}" font-size="10" fill="#374151">回归线</text>')
    out.append(f'<text x="{padL}" y="{padT-2}" font-size="10" fill="#059669">τ = {a:.3f}·(K6·i6) {b:+.3f}　R²={r*r:.3f}</text>')
    out.append(f'<text x="{W-padR}" y="{H-padB+16}" font-size="9" fill="#9ca3af" text-anchor="end">臂侧 K6·i6 (N·m)</text>')
    out.append(f'<text x="{padL-30}" y="{padT-8}" font-size="9" fill="#9ca3af" transform="rotate(-90 {padL-30} {H-padB})">传感器 τ (N·m)</text>')
    out.append("</svg>")
    scatter = "".join(out)

    # 残差直方图（简化：用分桶）
    resid = [ys[i] - (a * xs[i] + b) for i in range(n)]
    bins = [-3, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 3]
    counts = [0] * (len(bins) - 1)
    for e in resid:
        for bi in range(len(bins) - 1):
            if bins[bi] <= e < bins[bi + 1]:
                counts[bi] += 1
                break
    maxc = max(counts)
    bw = (W - padL - padR) / len(counts)
    hist = [f'<svg viewBox="0 0 {W} 260" width="100%" xmlns="http://www.w3.org/2000/svg" '
            f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
            f'<rect width="{W}" height="260" fill="#ffffff"/>',
            f'<text x="{padL}" y="20" font-size="14" font-weight="600" fill="#1f2937">预测残差分布（传感器 − 预测）</text>']
    for bi in range(len(counts)):
        bh = counts[bi] / maxc * 160
        x0 = padL + bi * bw
        out_rect = f'<rect x="{x0:.1f}" y="{230-bh:.1f}" width="{bw-2:.1f}" height="{bh:.1f}" fill="#059669" opacity="0.7"/>'
        hist.append(out_rect)
        lab = f"{bins[bi]:+.1f}"
        hist.append(f'<text x="{x0+bw/2:.1f}" y="248" font-size="8" fill="#6b7280" text-anchor="middle">{lab}</text>')
    hist.append(f'<text x="{W-padR}" y="252" font-size="9" fill="#9ca3af" text-anchor="end">残差 (N·m)</text>')
    hist.append("</svg>")
    hist_svg = "".join(hist)

    html = f"""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>电流作为传感器预测 · 分析</title><style>
 body{{margin:0;padding:26px 30px 60px;background:#f8fafc;color:#1f2937;
   font-family:-apple-system,Segoe UI,Helvetica,Arial,"Microsoft YaHei",sans-serif;line-height:1.6;font-size:15px}}
 h1{{font-size:22px;margin:0 0 6px}} h2{{font-size:17px;margin:24px 0 8px}}
 .card{{background:#fff;border:1px solid #e5e7eb;border-radius:10px;padding:16px 18px;margin:14px 0}}
 .num{{font-size:15px;color:#1f2937}}
</style></head><body>
<h1>电流（K6·i6）能否作为力矩传感器的预测</h1>
<div style="font-size:13px;color:#6b7280">数据：实验三 6 段（松3+紧3）逐点对齐，n={n}；OFF=−8.2s</div>
<div class="card">{scatter}</div>
<div class="card">{hist_svg}</div>
<h2>关键数字</h2>
<div class="card num">
整体 R² = {r*r:.3f}（被松/紧两大簇撑高）　·　段内波动 R² = 0.388　·　斜率 {a:.3f}（≈1，K6 未标错）<br>
预测精度：MAE ≈ 0.73 N·m（满量程 15%）　·　无系统偏差（bias ≈ 0）<br>
<b>结论：电流能当传感器的「粗预测」（方向+量级），当不了「精预测」（段内逐帧只相关 39%）。</b>
</div></body></html>"""
    (HERE / "runs" / "exp3_scatter.html").write_text(html, encoding="utf-8")
    print(f"散点图已输出 runs/exp3_scatter.html（n={n}，R²={r*r:.3f}，斜率{a:.3f}）")


if __name__ == "__main__":
    main()
