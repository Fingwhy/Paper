#!/usr/bin/env python
"""把 FIRST 消融结果画成 SVG 柱状图（本机 Windows，与 s6_first_policy.py 同目录）。

    python gen_first_chart.py
    → S6_FIRST_DELTA.svg
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE / "runs" / "first_policy.json"
OUT = HERE / "S6_FIRST_DELTA.svg"

COL = {"PreContact": "#c0392b", "Contact": "#2e86de", "Free": "#7f8c8d"}
PH_CN = {"PreContact": "Pre-Contact", "Contact": "Contact", "Free": "Free Space"}


def main():
    d = json.load(open(SRC, encoding="utf-8"))
    runs = d["runs"]
    base = runs["base_F1_PC1_C1"]
    keys = [k for k in runs if k != "base_F1_PC1_C1"]
    order = []
    for arm in ("first", "wregr"):
        for k in keys:
            if k.startswith(arm):
                order.append(k)
    order.sort(key=lambda k: (0 if k.startswith("first") else 1,
                              float(k.split("_PC")[1].split("_")[0])))

    data = {}
    for k in order:
        data[k] = {}
        for ph in ("PreContact", "Contact", "Free"):
            dif = np.array([r["fm"][ph] - b["fm"][ph] for r, b in zip(runs[k], base)])
            ref = np.mean([b["fm"][ph] for b in base])
            se = dif.std(ddof=1) / np.sqrt(len(dif))
            data[k][ph] = (100 * dif.mean() / ref, dif.mean() / se)

    W, H = 900, 460
    L, R, T, B = 74, 24, 66, 116
    pw, ph_ = W - L - R, H - T - B
    ymax = 14.0
    n = len(order)
    slot = pw / n

    def y(v):
        return T + ph_ * (1 - (v + 2.0) / (ymax + 2.0))

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}" font-family="Segoe UI,Helvetica,Arial,sans-serif">',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
         f'<text x="{L}" y="30" font-size="17" font-weight="600" fill="#1c2833">'
         'FIRST 消融：各相位验证损失相对基线的变化（同一折同一种子配对，n=10）</text>',
         f'<text x="{L}" y="50" font-size="12" fill="#5d6d7e">'
         '负值＝比普通行为克隆更好；误差棒为配对差值的标准误</text>']

    # 网格与 y 轴
    for tick in (-2, 0, 2, 4, 6, 8, 10, 12, 14):
        yy = y(tick)
        s.append(f'<line x1="{L}" y1="{yy:.1f}" x2="{L+pw}" y2="{yy:.1f}" '
                 f'stroke="{"#34495e" if tick == 0 else "#e5e8ea"}" '
                 f'stroke-width="{1.4 if tick == 0 else 1}"/>')
        s.append(f'<text x="{L-10}" y="{yy+4:.1f}" font-size="11" fill="#5d6d7e" '
                 f'text-anchor="end">{tick:+d}%</text>' if tick else
                 f'<text x="{L-10}" y="{yy+4:.1f}" font-size="11" fill="#34495e" '
                 f'text-anchor="end">0</text>')

    # 柱
    bw = slot * 0.24
    for i, k in enumerate(order):
        cx = L + slot * (i + 0.5)
        for j, ph in enumerate(("PreContact", "Contact", "Free")):
            v, t = data[k][ph]
            x = cx + (j - 1) * bw
            y0, y1 = y(0), y(v)
            top, hgt = min(y0, y1), abs(y1 - y0)
            s.append(f'<rect x="{x-bw/2:.1f}" y="{top:.1f}" width="{bw:.1f}" '
                     f'height="{max(hgt,0.6):.1f}" fill="{COL[ph]}" fill-opacity="0.88"/>')
            if abs(t) > 2.0:
                s.append(f'<text x="{x:.1f}" y="{top-4:.1f}" font-size="9.5" '
                         f'fill="{COL[ph]}" text-anchor="middle">{"*" if abs(t)<3 else "**"}</text>')
        lab = k.split("_")[0].upper() if False else ("FIRST" if k.startswith("first") else "加权回归")
        wv = k.split("_PC")[1].split("_")[0]
        s.append(f'<text x="{cx:.1f}" y="{T+ph_+18:.1f}" font-size="11.5" '
                 f'text-anchor="middle" fill="#1c2833">{lab}</text>')
        s.append(f'<text x="{cx:.1f}" y="{T+ph_+33:.1f}" font-size="10.5" '
                 f'text-anchor="middle" fill="#7f8c8d">w_PC={wv}</text>')

    # 图例
    lx = L
    for ph in ("PreContact", "Contact", "Free"):
        s.append(f'<rect x="{lx}" y="{H-34}" width="12" height="12" fill="{COL[ph]}"/>')
        s.append(f'<text x="{lx+17}" y="{H-24}" font-size="12" fill="#34495e">'
                 f'{PH_CN[ph]}</text>')
        lx += 24 + 8 * len(PH_CN[ph]) + 26
    s.append(f'<text x="{L+pw}" y="{H-24}" font-size="11" fill="#95a5a6" '
             'text-anchor="end">*  |t|&gt;2   **  |t|&gt;3</text>')

    # 分隔线：FIRST / 加权回归
    xs = L + slot * sum(1 for k in order if k.startswith("first"))
    s.append(f'<line x1="{xs:.1f}" y1="{T}" x2="{xs:.1f}" y2="{T+ph_}" '
             'stroke="#bdc3c7" stroke-dasharray="4 3"/>')
    s.append("</svg>")
    OUT.write_text("\n".join(s), encoding="utf-8")
    print("已生成", OUT)


if __name__ == "__main__":
    main()
