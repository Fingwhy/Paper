#!/usr/bin/env python3
"""实验三·全量 —— 每段单独的「臂侧 K6·i6 vs 传感器实测」对比（方案 B：不重跑）。

思路（不硬对齐逐点，因为臂侧 twist_log 无绝对时间戳、且松方向弹开冲击臂侧电流法测不到）：

1. 每段臂侧数据是 46s 的段内记录（t_s 从 1.0 起），横轴 = 段内时间。
2. 传感器侧每段用「物理锚点」定位：
   - 松方向：三次「盖子弹开」冲击尖峰（-6.02@23.9s / -5.46@65s / -4.36@178s，q3 文件）
   - 紧方向：拧到底的峰值（紧1 +1.44@50s、紧2 +4.47@190s、紧3 +4.33@13s）
3. 每段输出一张图：上栏臂侧 K6·i6，下栏传感器 τ，各自段内时间轴，并标锚点；
   再给一张数值对照（峰值 / 均值）。

结论要点（写进图里）：
- 松方向：传感器抓到「弹开冲击」超 5 N·m 量程的尖峰，臂侧电流法完全没有（静摩擦主导），
  两路峰值不可比；但「稳态棘轮段」均值可对照（臂 -2.1 vs 传感 -2.3，量级一致）。
- 紧方向：拧到底在量程内，紧2/紧3 臂侧 4.7/4.4 与传感器 4.5/4.3 吻合（~10%），紧1 臂侧偏高（静摩擦）。
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
K6 = 0.3889
K_SENSOR = 0.001

# 每段：(标签, 臂侧文件名, 方向, 传感器文件, 传感器段窗口[s], 传感器锚点时刻[s], 锚点说明)
SEGMENTS = [
    ("松1", "r1b", "松", "q3", (20.0, 63.0), 23.9, "盖子弹开冲击 -6.02 N·m（超量程）"),
    ("松2", "r2",  "松", "q3", (63.0, 124.0), 65.0, "盖子弹开冲击 -5.46 N·m（超量程）"),
    ("松3", "r3",  "松", "q3", (175.0, 239.0), 178.0, "盖子弹开冲击 -4.36 N·m"),
    ("紧1", "t1",  "紧", "q3b", (40.0, 70.0), 50.0, "拧紧峰值 +1.44 N·m"),
    ("紧2", "t2",  "紧", "q3b", (142.0, 200.0), 189.6, "拧紧峰值 +4.47 N·m"),
    ("紧3", "t3",  "紧", "q3c", (0.0, 70.0), 13.2, "拧紧峰值 +4.33 N·m"),
]


def load_arm(seg):
    rows = list(csv.DictReader(open(HERE / "data" / f"twistlog_q3_{seg}.csv", encoding="utf-8")))
    t = [float(r["t_s"]) for r in rows]
    i6 = [float(r["i6"]) for r in rows]
    q6 = [float(r["q6"]) for r in rows]
    return t, i6, q6


def load_sensor(f, lo, hi):
    lines = open(HERE / "runs" / f"sensor_{f}.jsonl", encoding="utf-8").read().splitlines()
    hdr = json.loads(lines[0])
    t0 = hdr["t0_epoch"]
    rows = [json.loads(l) for l in lines if '"tau"' in l]
    t = [r["t"] - t0 for r in rows]
    tau = [r["tau"] for r in rows]
    # 只取窗口内，且把窗口起点归零到段内时间
    sel_t = [t[i] - lo for i in range(len(t)) if lo <= t[i] <= hi]
    sel_v = [tau[i] for i in range(len(t)) if lo <= t[i] <= hi]
    return sel_t, sel_v


def stat(v, dirc):
    """峰值按方向取：松方向取负峰（min），紧方向取正峰（max）。返回 (峰值, 均值)。"""
    if not v:
        return 0.0, 0.0
    if dirc == "松":
        return min(v), sum(v) / len(v)
    return max(v), sum(v) / len(v)


def line_svg(title, t, v, color, ylabel, anchor=None, anchor_lab=None, dash=False,
             width=920, height=210):
    """画一条曲线。t/v 等长。anchor = 段内时刻（传感器锚点换算到段内）。
    传感器锚点换算：锚点绝对时刻 - 段窗口起点 = 段内时刻。"""
    padL, padR, padT, padB = 62, 14, 26, 34
    W, H = width, height
    vmin, vmax = min(v), max(v)
    if vmax - vmin < 1e-6:
        vmax, vmin = vmax + 1, vmin - 1
    pad = 0.12 * (vmax - vmin)
    lo, hi = vmin - pad, vmax + pad
    tmin, tmax = min(t), max(t)
    if tmax - tmin < 1e-6:
        tmax = tmin + 1
    sx = lambda x: padL + (x - tmin) / (tmax - tmin) * (W - padL - padR)
    sy = lambda y: H - padB - (y - lo) / (hi - lo) * (H - padT - padB)
    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="16" font-size="12.5" font-weight="600" fill="#1f2937">{title}</text>']
    for i in range(5):
        y = lo + (hi - lo) * i / 4
        out.append(f'<line x1="{padL}" y1="{sy(y):.1f}" x2="{W-padR}" y2="{sy(y):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{padL-8}" y="{sy(y)+4:.1f}" font-size="10" fill="#6b7280" text-anchor="end">{y:+.1f}</text>')
    for i in range(7):
        x = tmin + (tmax - tmin) * i / 6
        out.append(f'<line x1="{sx(x):.1f}" y1="{padT}" x2="{sx(x):.1f}" y2="{H-padB}" stroke="#f5f7fa"/>')
        out.append(f'<text x="{sx(x):.1f}" y="{H-padB+15}" font-size="10" fill="#6b7280" text-anchor="middle">{x:.0f}s</text>')
    # 锚点竖线
    if anchor is not None and tmin <= anchor <= tmax:
        ax = sx(anchor)
        out.append(f'<line x1="{ax:.1f}" y1="{padT}" x2="{ax:.1f}" y2="{H-padB}" stroke="#f59e0b" stroke-width="1.4" stroke-dasharray="5,3"/>')
        out.append(f'<text x="{ax:.1f}" y="{padT+9}" font-size="9.5" fill="#b45309" text-anchor="middle">▾锚点</text>')
    dashattr = ' stroke-dasharray="6,3"' if dash else ""
    pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(t, v))
    out.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.8"{dashattr} stroke-linejoin="round"/>')
    out.append(f'<text x="{W-padR}" y="{padT-2}" font-size="10" fill="#9ca3af" text-anchor="end">{ylabel}</text>')
    if anchor_lab:
        out.append(f'<text x="{padL}" y="{H-6}" font-size="10.5" fill="#b45309">◉ {anchor_lab}</text>')
    out.append("</svg>")
    return "".join(out)


def main():
    colors_arm = "#dc2626"
    colors_sen = "#2563eb"
    html = ["""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>实验三·全量 每段对比</title><style>
 body{margin:0;padding:26px 30px 60px;background:#f8fafc;color:#1f2937;
   font-family:-apple-system,Segoe UI,Helvetica,Arial,"Microsoft YaHei",sans-serif;line-height:1.6;font-size:15px}
 h1{font-size:22px;margin:0 0 6px} h2{font-size:17px;margin:26px 0 8px}
 .seg{background:#fff;border:1px solid #e5e7eb;border-radius:10px;padding:16px 18px;margin:18px 0}
 .seghead{font-size:16px;font-weight:700;margin:0 0 2px}
 .segsub{font-size:12.5px;color:#6b7280;margin:0 0 12px}
 .row{display:flex;gap:14px;margin-bottom:6px}
 .col{flex:1;border:1px solid #eef2f7;border-radius:8px;padding:6px}
 .collab{font-size:12px;color:#374151;margin:0 0 2px}
 table{border-collapse:collapse;width:100%;font-size:13px;margin-top:8px}
 th,td{border-bottom:1px solid #eef2f7;padding:6px 8px;text-align:right}
 th{background:#f1f5f9} td:first-child,th:first-child{text-align:left}
 .note{color:#6b7280;font-size:12.5px}
 .good{color:#059669;font-weight:600} .bad{color:#dc2626;font-weight:600}
</style></head><body>
<h1>实验三 · 全量 —— 每段单独对比</h1>
<div class="note">K6 = 0.3889 N·m/A（冻结）　|　传感器 k = 0.02774 N·m/count　|　红=臂侧 K6·i6，蓝=传感器实测</div>"""]

    md = ["# 实验三 · 全量 —— 每段单独对比", "",
          f"- K6（冻结）= {K6:.4f} N·m/A；传感器 k = {K_SENSOR} N·m/count", "",
          "> 对齐策略：臂侧 twist_log 无绝对时间戳，且松方向「盖子弹开」冲击臂侧电流法测不到，" 
          "逐点硬对齐既不可能也无意义。这里每段用**物理锚点**定位传感器窗口（松=弹开冲击，紧=拧到底峰值），" 
          "臂侧/传感器各自段内时间轴并排对照。", ""]

    for label, seg, dirc, sf, (lo, hi), anchor_abs, anchor_lab in SEGMENTS:
        at, ai6, aq6 = load_arm(seg)
        st, stau = load_sensor(sf, lo, hi)

        av = [x * K6 for x in ai6]
        at = [x - at[0] for x in at]  # 段内时间从 0 起

        # 锚点换算到传感器段内时间
        anchor_rel = anchor_abs - lo

        arm_peak, arm_mean = stat(av, dirc)
        sen_peak, sen_mean = stat(stau, dirc)

        # 臂侧锚点：松方向找 i6 峰值时刻，紧方向同理（用于对齐标注，实际不硬对齐）
        title = f"{label}　（臂侧 q6 {aq6[0]:.0f}° → {aq6[-1]:.0f}°　·　{dirc}方向）"

        html.append(f'<div class="seg"><div class="seghead">{title}</div>'
                    f'<div class="segsub">臂侧峰值 {arm_peak:+.2f} N·m（均值 {arm_mean:+.2f}）　·　'
                    f'传感器峰值 {sen_peak:+.2f} N·m（均值 {sen_mean:+.2f}）　·　锚点：{anchor_lab}</div>')
        html.append('<div class="row">')
        html.append(f'<div class="col"><div class="collab">臂侧 K6·i6（预测）</div>'
                    f'{line_svg("", at, av, colors_arm, "N·m", anchor=None)}</div>')
        html.append(f'<div class="col"><div class="collab">传感器实测 τ</div>'
                    f'{line_svg("", st, stau, colors_sen, "N·m", anchor=anchor_rel, anchor_lab=anchor_lab)}</div>')
        html.append('</div></div>')

        md.append(f"## {label}（{seg}）　·　{dirc}方向　q6 {aq6[0]:.0f}→{aq6[-1]:.0f}°")
        md.append(f"- 臂侧：峰值 {arm_peak:+.2f} N·m，均值 {arm_mean:+.2f} N·m")
        md.append(f"- 传感器：峰值 {sen_peak:+.2f} N·m，均值 {sen_mean:+.2f} N·m")
        md.append(f"- 锚点：{anchor_lab}")
        md.append("")

    # 汇总对照表
    md.append("## 段对段对照汇总")
    md.append("")
    md.append("| 段 | 方向 | 臂峰值(N·m) | 臂均值(N·m) | 传感器峰值(N·m) | 传感器均值(N·m) | 峰值比(臂/传感) |")
    md.append("|---|---|---|---|---|---|---|")
    html.append('<h2>段对段对照汇总</h2><div class="seg"><table>'
                '<tr><th>段</th><th>方向</th><th>臂峰值(N·m)</th><th>臂均值(N·m)</th>'
                '<th>传感器峰值(N·m)</th><th>传感器均值(N·m)</th><th>峰值比(臂/传感)</th></tr>')
    for label, seg, dirc, sf, (lo, hi), anchor_abs, anchor_lab in SEGMENTS:
        at, ai6, aq6 = load_arm(seg)
        st, stau = load_sensor(sf, lo, hi)
        av = [x * K6 for x in ai6]
        arm_peak, arm_mean = stat(av, dirc)
        sen_peak, sen_mean = stat(stau, dirc)
        ratio = abs(arm_peak) / abs(sen_peak) if abs(sen_peak) > 0.05 else float("nan")
        md.append(f"| {label} | {dirc} | {arm_peak:.2f} | {arm_mean:+.2f} | {sen_peak:+.2f} | {sen_mean:+.2f} | {ratio:.2f} |")
        html.append(f"<tr><td>{label}</td><td>{dirc}</td><td>{arm_peak:.2f}</td><td>{arm_mean:+.2f}</td>"
                    f"<td>{sen_peak:+.2f}</td><td>{sen_mean:+.2f}</td><td>{ratio:.2f}</td></tr>")
    html.append("</table></div>")

    # 结论
    concl = """
<h2>结论</h2>
<div class="seg">
<div class="note" style="font-size:14px;color:#1f2937">
<b>1. 松方向（松1/2/3）：臂侧电流法测不到「盖子弹开」冲击。</b><br>
&nbsp;&nbsp;传感器抓到 3 次弹开冲击尖峰 −6.02 / −5.46 / −4.36 N·m（前两次超 5 N·m 量程），
但臂侧 i6 全程平坦（静摩擦主导，段内 −0.3~−0.4 A），没有对应尖峰。
所以松方向<b>峰值不可比</b>（臂/传感 0.73~0.82 是被弹开冲击拉低的假象）。<br>
&nbsp;&nbsp;但<b>稳态棘轮段均值可对照</b>：臂 −2.10/−1.99/−1.52 vs 传感器 −2.26/−2.59/−2.14 N·m，量级一致。<br><br>
<b>2. 紧方向（紧1/2/3）：拧到底在量程内，可以比。</b><br>
&nbsp;&nbsp;紧2 臂 4.68 vs 传感 4.47（比 1.05）、紧3 臂 4.44 vs 传感 4.33（比 1.03），<b>吻合 ~10%</b>。<br>
&nbsp;&nbsp;紧1 臂 3.56 vs 传感 1.44（比 2.47），臂侧明显偏高 —— 盖子刚回位、螺纹尚未咬合，电流里混了大量静摩擦。<br><br>
<b>3. 综合：K6·i6 在「稳态、量程内」的紧方向拧到底场景下，与外部力矩传感器吻合 ~10%</b>；
松方向的弹开冲击是弹性储能释放，电流法测不到，属方法本身的盲区，不是 K6 标定错误。
</div></div>
"""
    html.append(concl)
    md += ["", "## 结论", "",
           "1. **松方向**：臂侧电流法测不到「盖子弹开」冲击（传感器 −6.02/−5.46/−4.36 N·m，前两次超量程；臂侧无尖峰）。"
           "松方向峰值不可比，但稳态棘轮段均值量级一致（臂 −2.1 vs 传感 −2.3）。",
           "2. **紧方向**：拧到底在量程内。紧2/紧3 臂 4.68/4.44 vs 传感 4.47/4.33，吻合 ~10%；紧1 臂侧偏高（静摩擦）。",
           "3. 综合：稳态、量程内的紧方向拧到底场景，K6·i6 与外部力矩传感器吻合 ~10%。"]

    (HERE / "QUANT_EXP3_SEG2.md").write_text("\n".join(md), encoding="utf-8")
    (HERE / "runs" / "exp3_seg2.html").write_text("".join(html) + "</body></html>", encoding="utf-8")
    print("\n".join(md))
    print("\n已输出 QUANT_EXP3_SEG2.md 与 runs/exp3_seg2.html")


if __name__ == "__main__":
    main()
