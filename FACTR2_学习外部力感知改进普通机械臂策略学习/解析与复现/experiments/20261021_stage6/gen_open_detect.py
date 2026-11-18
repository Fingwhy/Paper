#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_open_detect.py —— 「瓶盖是否脱离夹爪」判据证据图（纯 SVG，无外部依赖）

出 3 个面板：
  A 夹爪开度 grip_ang_mm 时序：脱开的会阶梯式坍缩，没脱开的全程一条水平线
  B 夹爪 effort 时序：脱开瞬间出现大幅波动（std 涨 7~12 倍）
  C 散点图：eff_std(x) 对 开度坍缩量(y)，看两类是否分得开

输入：data/ 下全部「序号.中文名.csv」
输出：OPEN_DETECT.html
本机运行：python gen_open_detect.py
"""
import csv, json, os, statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, 'data')
OUT = os.path.join(HERE, 'OPEN_DETECT.html')

# (文件名, 显示名, 颜色, 是否脱开)  —— 是否脱开由开度坍缩自动判定，这里只给配色
CASES = [
    ("1.全量原盖未打开.csv",     "组1 原盖·未打开",      "#e05252"),
    ("5.全量预松1_未打开.csv",   "组5 轮1·未打开",       "#c084fc"),
    ("3.棘轮全量圆盖三圈开_轮3.csv", "组3 轮3·开(盖没掉)",  "#4a9eff"),
    ("4.棘轮全量预开全打开_轮3.csv", "组4 轮3·开(盖滑出)",  "#4ade80"),
    ("5.全量预松2_打开.csv",     "组5 轮2·开(盖滑出)",   "#22d3ee"),
    ("5.全量预松3_已打开空转.csv", "组5 轮3·已开空转",     "#e0a052"),
]
NPT = 220  # 每条线降采样点数


def load(fn):
    with open(os.path.join(D, fn), newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def series(rows, key, fn=float):
    t0 = float(rows[0]['t_s'])
    n = len(rows)
    step = max(1, n // NPT)
    xs, ys = [], []
    for k in range(0, n, step):
        xs.append(round(float(rows[k]['t_s']) - t0, 3))
        ys.append(round(fn(rows[k][key]), 4))
    return xs, ys


def stats(rows):
    an = [float(x['grip_ang_mm']) for x in rows]
    ef = [abs(float(x['grip_eff_nm'])) for x in rows]
    t0 = float(rows[0]['t_s'])
    tj, qj = None, None
    for k in range(1, len(rows)):
        if abs(an[k] - an[k - 1]) > 0.02:
            tj, qj = float(rows[k]['t_s']) - t0, float(rows[k]['q6'])
            break
    return dict(sd=st.pstdev(ef), drop=an[0] - an[-1], tj=tj, qj=qj)


# ------------------------------------------------ 组装数据
data = []
for fn, name, color in CASES:
    rows = load(fn)
    s = stats(rows)
    a0 = float(rows[0]['grip_ang_mm'])
    tx, ty = series(rows, 'grip_ang_mm', lambda v: float(v) - a0)
    ex, ey = series(rows, 'grip_eff_nm', lambda v: abs(float(v)))
    data.append(dict(name=name, color=color, sd=s['sd'], drop=s['drop'],
                     tj=s['tj'], qj=s['qj'],
                     t_ang=tx, ang=ty, t_eff=ex, eff=ey))

# 全量 11 份用于散点
allpts = []
for fn, _, c in CASES:
    pass
for fn in sorted(os.listdir(D)):
    if not (fn.endswith('.csv') and fn[0].isdigit()):
        continue
    with open(os.path.join(D, fn), newline='', encoding='utf-8') as f:
        rd = csv.DictReader(f)
        if 'grip_eff_nm' not in (rd.fieldnames or []):
            continue
        rows = list(rd)
    s = stats(rows)
    allpts.append(dict(n=fn[:-4], sd=round(s['sd'], 4), drop=round(s['drop'], 3)))

# ------------------------------------------------ SVG 生成
W, H = 1140, 290
PAD_L, PAD_R, PAD_T, PAD_B = 62, 18, 26, 34


def sx(t, tmax):
    return PAD_L + (t / tmax) * (W - PAD_L - PAD_R)


def sy(v, lo, hi):
    return H - PAD_B - (v - lo) / (hi - lo) * (H - PAD_T - PAD_B)


def panel(title, sub, lines, ykey, ylabel, ylo, yhi, yticks, marks=True):
    tmax = max(max(d['t_' + ykey]) for d in lines)
    o = [f'<div class="card"><h3>{title}</h3><div class="sub">{sub}</div>']
    o.append(f'<svg viewBox="0 0 {W} {H}" width="100%" style="max-width:{W}px">')
    # 网格 + y 轴刻度
    for v in yticks:
        y = sy(v, ylo, yhi)
        o.append(f'<line x1="{PAD_L}" y1="{y:.1f}" x2="{W-PAD_R}" y2="{y:.1f}" '
                 f'stroke="#ffffff14" stroke-width="1"/>')
        o.append(f'<text x="{PAD_L-8}" y="{y+4:.1f}" text-anchor="end" '
                 f'fill="#8b95a5" font-size="11">{v:g}</text>')
    # x 轴
    for tt in range(0, int(tmax) + 1, 2):
        x = sx(tt, tmax)
        o.append(f'<text x="{x:.1f}" y="{H-PAD_B+18}" text-anchor="middle" '
                 f'fill="#8b95a5" font-size="11">{tt}</text>')
    o.append(f'<line x1="{PAD_L}" y1="{H-PAD_B}" x2="{W-PAD_R}" y2="{H-PAD_B}" '
             f'stroke="#ffffff22"/>')
    o.append(f'<text x="{(PAD_L+W-PAD_R)/2:.0f}" y="{H-6}" text-anchor="middle" '
             f'fill="#8b95a5" font-size="11">时间 (s)</text>')
    o.append(f'<text x="10" y="{PAD_T}" fill="#8b95a5" font-size="11">{ylabel}</text>')
    # 曲线
    for d in lines:
        pts = ' '.join(f'{sx(x_, tmax):.1f},{sy(y_, ylo, yhi):.1f}'
                       for x_, y_ in zip(d['t_' + ykey], d[ykey]))
        dash = '6,4' if d['drop'] > 0.15 else ''
        o.append(f'<polyline points="{pts}" fill="none" stroke="{d["color"]}" '
                 f'stroke-width="1.9" stroke-dasharray="{dash}"/>')
        if marks and d['tj'] and d['drop'] > 0.15:
            x = sx(d['tj'], tmax)
            o.append(f'<line x1="{x:.1f}" y1="{PAD_T}" x2="{x:.1f}" y2="{H-PAD_B}" '
                     f'stroke="{d["color"]}" stroke-width="1" stroke-dasharray="2,3" opacity=".8"/>')
            o.append(f'<circle cx="{x:.1f}" cy="{PAD_T+6}" r="3.5" fill="{d["color"]}"/>')
            o.append(f'<text x="{x+6:.1f}" y="{PAD_T+10}" fill="{d["color"]}" '
                     f'font-size="10">脱开 {d["tj"]:.1f}s / q6={d["qj"]:.0f}°</text>')
    o.append('</svg><div class="legend">')
    for d in lines:
        dash = 'dash' if d['drop'] > 0.15 else ''
        o.append(f'<span class="lg"><i class="{dash}" style="background:{d["color"]}"></i>'
                 f'{d["name"]}</span>')
    o.append('</div></div>')
    return '\n'.join(o)


# 散点图
def scatter():
    w, h = W, 330
    pl, pr, pt, pb = 70, 24, 26, 44
    xs = [p['sd'] for p in allpts]
    ys = [p['drop'] for p in allpts]
    xhi = max(xs) * 1.15
    yhi = max(ys) * 1.35
    o = [f'<div class="card"><h3>判据可分性：eff 波动 vs 开度坍缩（全部 11 份全量）</h3>',
         f'<div class="sub">左下一簇 = 盖没脱开；右上三颗 = 盖已滑出夹爪。'
         f'两道阈值之间是空的。</div>',
         f'<svg viewBox="0 0 {w} {h}" width="100%" style="max-width:{w}px">']
    x0, y0 = pl, h - pb
    x1, y1 = w - pr, pt
    # 阈值带
    tx = x0 + 0.020 / xhi * (x1 - x0)
    ty = y0 - 0.15 / yhi * (y0 - y1)
    o.append(f'<rect x="{tx:.1f}" y="{y1}" width="{x1-tx:.1f}" height="{y0-y1}" '
             f'fill="#4ade800e"/>')
    o.append(f'<rect x="{x0}" y="{y1}" width="{x1-x0}" height="{ty-y1:.1f}" '
             f'fill="#e052520e"/>')
    o.append(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}" stroke="#ffffff22"/>')
    o.append(f'<line x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}" stroke="#ffffff22"/>')
    o.append(f'<line x1="{tx:.1f}" y1="{y0}" x2="{tx:.1f}" y2="{y1}" stroke="#e0a052" '
             f'stroke-dasharray="4,4" opacity=".7"/>')
    o.append(f'<line x1="{x0}" y1="{ty:.1f}" x2="{x1}" y2="{ty:.1f}" stroke="#e0a052" '
             f'stroke-dasharray="4,4" opacity=".7"/>')
    o.append(f'<text x="{tx+6:.1f}" y="{y0-8}" fill="#e0a052" font-size="11">'
             f'eff_std = 0.020</text>')
    o.append(f'<text x="{x0+8}" y="{ty-6:.1f}" fill="#e0a052" font-size="11">'
             f'开度坍缩 = 0.15 mm</text>')
    # 未脱开的 8 份挤在一起，合并成一个标记 + 说明，避免标签叠成一团
    miss = [p for p in allpts if not (p['sd'] > 0.02 and p['drop'] > 0.15)]
    hit = [p for p in allpts if (p['sd'] > 0.02 and p['drop'] > 0.15)]
    for p in hit:
        cx = x0 + p['sd'] / xhi * (x1 - x0)
        cy = y0 - p['drop'] / yhi * (y0 - y1)
        o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="6" fill="#4ade80" '
                 f'fill-opacity=".9" stroke="#0b0e14" stroke-width="1"/>')
        o.append(f'<text x="{cx-10:.1f}" y="{cy-10:.1f}" text-anchor="end" '
                 f'fill="#4ade80" font-size="10.5">{p["n"]}</text>')
    if miss:
        # 用平均位置画一个簇，半径示意离散程度
        mxs = [p['sd'] for p in miss]; mys = [p['drop'] for p in miss]
        cx = x0 + (sum(mxs) / len(mxs)) / xhi * (x1 - x0)
        cy = y0 - (sum(mys) / len(mys)) / yhi * (y0 - y1)
        o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="9" fill="none" '
                 f'stroke="#7b8494" stroke-width="1.4" stroke-dasharray="3,3"/>')
        o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="4" fill="#7b8494"/>')
        o.append(f'<text x="{cx+14:.1f}" y="{cy+4:.1f}" fill="#9aa4b2" font-size="10.5">'
                 f'其余 {len(miss)} 份：eff_std ≤ 0.015，坍缩 = 0.000</text>')
    # 轴标签
    for v in [0, 0.02, 0.04, 0.06, 0.08]:
        if v <= xhi:
            xx = x0 + v / xhi * (x1 - x0)
            o.append(f'<text x="{xx:.1f}" y="{y0+18}" text-anchor="middle" '
                     f'fill="#8b95a5" font-size="11">{v:g}</text>')
    for v in [0, 0.1, 0.2, 0.3]:
        if v <= yhi:
            yy = y0 - v / yhi * (y0 - y1)
            o.append(f'<text x="{x0-8}" y="{yy+4:.1f}" text-anchor="end" '
                     f'fill="#8b95a5" font-size="11">{v:g}</text>')
    o.append(f'<text x="{(x0+x1)/2:.0f}" y="{h-8}" text-anchor="middle" '
             f'fill="#8b95a5" font-size="11">eff_std (N·m)</text>')
    o.append(f'<text x="16" y="{(y0+y1)/2:.0f}" fill="#8b95a5" font-size="11" '
             f'transform="rotate(-90 16 {(y0+y1)/2:.0f})">开度坍缩 (mm)</text>')
    o.append('</svg></div>')
    return '\n'.join(o)


html = f'''<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>瓶盖脱开判据 · S6</title>
<style>
  :root{{--bg:#0b0e14;--panel:#121722;--line:#1e2733;--txt:#e6ebf2;--mut:#8b95a5}}
  *{{box-sizing:border-box}}
  body{{margin:0;padding:26px 30px 70px;background:var(--bg);color:var(--txt);
        font:14px/1.65 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif}}
  h1{{font-size:21px;margin:0 0 6px}}
  .lead{{color:var(--mut);margin:0 0 22px;max-width:900px}}
  .card{{background:var(--panel);border:1px solid var(--line);border-radius:9px;
         padding:16px 18px;margin-bottom:16px}}
  .card h3{{margin:0 0 4px;font-size:15px;font-weight:600}}
  .sub{{color:var(--mut);font-size:12.5px;margin-bottom:10px}}
  .legend{{display:flex;flex-wrap:wrap;gap:16px;margin-top:8px;font-size:12px;color:var(--mut)}}
  .lg{{display:flex;align-items:center;gap:6px}}
  .lg i{{width:18px;height:3px;border-radius:2px;display:inline-block}}
  .lg i.dash{{background-image:linear-gradient(90deg,currentColor 0 60%,transparent 60%)!important;
              background-size:6px 3px;background-repeat:repeat-x}}
  .note{{background:#12181f;border-left:3px solid #22d3ee;padding:13px 17px;
         border-radius:0 7px 7px 0;margin-bottom:16px}}
  .warn{{border-left-color:#e0a052}}
  b.k{{color:#22d3ee}}
  code{{background:#1a2130;padding:1px 5px;border-radius:4px;font-size:12.5px}}
</style></head>
<body>
<h1>瓶盖是否真的脱离夹爪 —— 两个独立信号的证据</h1>
<p class="lead">左边两个面板：实线=盖没脱开，虚线=盖脱开了。判据是
<b class="k">夹爪开度阶梯式坍缩</b>（几何量，最硬）配合 <b class="k">effort 波动放大</b>（力矩量）。
下面散点图把全部 11 份全量记录摊开，两类之间是空的。</p>

{panel("① 夹爪开度变化量 Δgrip_ang（相对本次首帧）", "盖一直被两指夹着 → 直径不变 → 死死贴在 0 线上；盖滑出去 → 手指合拢 → 阶梯下跳。图中 3 条未脱开的全部重叠成一条 0 线（共 8 份未脱开，只挑了 3 份代表）。",
       data, "ang", "Δ开度 (mm)", -0.40, 0.05,
       [0, -0.1, -0.2, -0.3, -0.4])}

{panel("② 夹爪 effort |grip_eff_nm|", "同一批记录。脱开瞬间负载从「稳定抵抗」变成「松弛抖动」，曲线明显变毛。",
       data, "eff", "|effort| (N·m)", 0, 0.62,
       [0, 0.15, 0.3, 0.45, 0.6])}

{scatter()}

<div class="note"><b>怎么看这张图</b><br>
三颗绿点（组4轮3、组5轮2、组5轮3）开度都掉了 0.21~0.35 mm，eff_std 都在 0.04 以上；
其余 8 份开度一根直线、eff_std 全在 0.015 以内。<b>阈值 0.020 / 0.15 mm 落在真空中</b>，
不是硬凑的中间值。</div>

<div class="note warn"><b>一条边界要说清楚</b><br>
开度坍缩只能证明「<b>盖离开了夹爪</b>」，证明不了「<b>螺纹已经脱开</b>」。
组3 轮3 就是反例：瓶盖明明拧开了，但盖还卡在两指之间，开度全程 29.82 一动不动。
所以判据要分两种结论用：<br>
· 开度坍缩 → 盖已自由，可以取走（组5 轮2、轮3 属于这类）<br>
· 开度不变 → 盖仍在爪内，螺纹脱没脱<b>数据判不了</b>（组5 轮1 属于这类）</div>
</body></html>'''

with open(OUT, 'w', encoding='utf-8') as f:
    f.write(html)
print(f'[out] {OUT}')
for d in data:
    print(f'  {d["name"]:<20} eff_std={d["sd"]:.4f}  坍缩={d["drop"]:.3f}mm '
          f' 脱开={d["tj"]}')
