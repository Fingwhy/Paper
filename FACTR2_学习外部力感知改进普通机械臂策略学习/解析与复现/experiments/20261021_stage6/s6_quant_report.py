#!/usr/bin/env python3
"""定量实验通用分析：电流推算力矩 vs 传感器实测力矩。

输入
----
* `--sensor`   传感器 jsonl（sensor_rec.py 产出，含绝对时刻）
* `--samples`  臂侧 50 Hz 样本（stage1_l0.py parse 后转出的 jsonl）
* `--arm-start` 臂侧 raw 文件头里的 "started"（VM 本地时间，如 2026-10-26T16:30:52+0800）
* `--skew`     两机时钟偏差（VM − Windows），默认 7.97，正数表示 VM 慢

输出
----
* `<prefix>.md`     文字版统计表
* `<prefix>.html`   折线图 + 逐点表（自包含 SVG）

对齐
----
传感器在 Windows、机械臂在 VM，两机时钟差约 8 s；用「臂侧实际起始时刻 + 时钟偏差」
换算到 Windows 时基后求偏移 L，映射为 `臂时间 = 传感器时间 + L`。**不靠互相关猜。**
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SKEW_DEFAULT = 7.97


def load_sensor(p: Path):
    rows = [json.loads(l) for l in open(p, encoding="utf-8") if '"tau"' in l]
    t = np.array([r["t"] for r in rows], float)
    count = np.array([r["raw"] for r in rows], float)
    return t, count


def load_arm(p: Path):
    rows = [json.loads(l) for l in open(p, encoding="utf-8")]
    t = np.array([r["t"] for r in rows], float)
    i6 = np.array([r["cur"][5] for r in rows], float)
    q6 = np.degrees(np.array([r["q"][5] for r in rows], float))
    dq6 = np.array([r["dq"][5] for r in rows], float)
    return t, i6, q6, dq6


def svg(curves, ts, title, ylabel, colors, height=230, width=900, hline=None,
        yfmt="{:+.2f}", marks=None):
    padL, padR, padT, padB = 64, 14, 26, 34
    W, H = width, height
    allv = np.concatenate([c[1] for c in curves])
    lo, hi = float(np.nanmin(allv)), float(np.nanmax(allv))
    if hi - lo < 1e-6:
        hi, lo = lo + 1, lo - 1
    pad = 0.12 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    tmin, tmax = float(ts[0]), float(ts[-1])
    sx = lambda t: padL + (t - tmin) / max(1e-9, tmax - tmin) * (W - padL - padR)
    sy = lambda v: H - padB - (v - lo) / (hi - lo) * (H - padT - padB)
    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="17" font-size="13" font-weight="600" fill="#1f2937">{title}</text>']
    for i in range(5):
        v = lo + (hi - lo) * i / 4
        out.append(f'<line x1="{padL}" y1="{sy(v):.1f}" x2="{W-padR}" y2="{sy(v):.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{padL-8}" y="{sy(v)+4:.1f}" font-size="10" fill="#6b7280" '
                   f'text-anchor="end">{yfmt.format(v)}</text>')
    for i in range(7):
        t = tmin + (tmax - tmin) * i / 6
        out.append(f'<line x1="{sx(t):.1f}" y1="{padT}" x2="{sx(t):.1f}" y2="{H-padB}" stroke="#f5f7fa"/>')
        out.append(f'<text x="{sx(t):.1f}" y="{H-padB+15}" font-size="10" fill="#6b7280" '
                   f'text-anchor="middle">{t:.0f}</text>')
    if hline is not None:
        out.append(f'<line x1="{padL}" y1="{sy(hline):.1f}" x2="{W-padR}" y2="{sy(hline):.1f}" '
                   f'stroke="#9ca3af" stroke-dasharray="5,4"/>')
    if marks:
        for x0, x1, lab in marks:
            out.append(f'<rect x="{sx(x0):.1f}" y="{padT}" width="{max(1.0, sx(x1)-sx(x0)):.1f}" '
                       f'height="{H-padT-padB}" fill="#3b82f6" opacity="0.06"/>')
            out.append(f'<text x="{sx(x0)+4:.1f}" y="{padT+12}" font-size="9.5" fill="#3b82f6">{lab}</text>')
    for name, y in curves:
        col = colors.get(name, "#2563eb")
        pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(ts, y) if np.isfinite(b))
        w = 2.2 if name.startswith("实测") else 1.5
        dash = ' stroke-dasharray="6,3"' if name.startswith("K6 预测") else ""
        out.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{w}"'
                   f'{dash} stroke-linejoin="round"/>')
    lx = padL + 4
    for name, *_ in curves:
        out.append(f'<rect x="{lx}" y="{padT+2}" width="16" height="3" fill="{colors.get(name, "#2563eb")}"/>')
        out.append(f'<text x="{lx+21}" y="{padT+7}" font-size="10.5" fill="#374151">{name}</text>')
        lx += 21 + 6.4 * len(name) + 16
    out.append(f'<text x="{W-padR}" y="{padT+7}" font-size="10" fill="#9ca3af" text-anchor="end">'
               f'{ylabel}  |  横轴 传感器时标 t (s)</text>')
    out.append("</svg>")
    return "".join(out)


def main():
    ap = argparse.ArgumentParser(description="定量实验：预测力矩 vs 实测力矩")
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--samples", required=True)
    ap.add_argument("--arm-start", required=True, help='raw 头的 "started"，如 2026-10-26T16:30:52+0800')
    ap.add_argument("--skew", type=float, default=SKEW_DEFAULT, help="VM − Windows，秒")
    ap.add_argument("--tag", default="q1")
    ap.add_argument("--title", default="")
    ap.add_argument("--k6", type=float, default=None, help="默认读 data/k6_locked.json")
    ap.add_argument("--k-sensor", type=float, default=0.001)
    ap.add_argument("--out", default=None, help="输出前缀，默认 runs/<tag>_quant")
    a = ap.parse_args()
    k6 = a.k6
    if k6 is None:
        try:
            k6 = float(json.load(open(HERE / "data" / "k6_locked.json", encoding="utf-8"))["K6"])
        except Exception:
            k6 = 0.3889

    st, scnt = load_sensor(Path(a.sensor))
    at, ai, aq, adq = load_arm(Path(a.samples))
    a0 = datetime.strptime(a.arm_start, "%Y-%m-%dT%H:%M:%S%z").timestamp() + a.skew
    s0 = json.loads(open(a.sensor, encoding="utf-8").readline())["t0_epoch"]
    L = s0 - a0                     # 臂时间 = 传感器时间 + L

    ts = np.arange(0, st[-1] - st[0], 0.02) + st[0]
    sens = np.interp(ts, st, scnt) * a.k_sensor
    ta = (ts - st[0]) + L
    i6 = np.interp(ta, at, ai)
    q6 = np.interp(ta, at, aq)
    dq6 = np.interp(ta, at, adq)
    pred = k6 * i6

    # 转动窗：起点 = 首次偏离初始角 >0.3°（即转动开始）；
    # 窗口 = [起点 − 2 s, 起点 + 10 s]，共 12 s。用户指定口径。
    gone = np.abs(aq - aq[0]) > 0.3
    if gone.any():
        t_start = float(at[np.argmax(gone)])
        r0 = t_start - 2.0 - L + st[0]
        r1 = t_start + 10.0 - L + st[0]
    else:
        r0, r1 = float(ts[0]), float(ts[-1])
    m = (ts >= r0) & (ts <= r1)

    def stats(x, y):
        d = x - y
        return dict(mae=float(np.mean(np.abs(d))), rmse=float(np.sqrt(np.mean(d ** 2))),
                    bias=float(np.mean(d)), r=float(np.corrcoef(x, y)[0, 1]) if x.std() > 0 and y.std() > 0 else float("nan"))

    s_all = stats(pred, sens)
    s_rot = stats(pred[m], sens[m])
    K_ls = float(np.polyfit(i6[m], sens[m], 1)[0]) if m.sum() > 20 and i6[m].std() > 0 else float("nan")

    lines = [f"# 定量实验 {a.tag}：{a.title}", "",
             f"- K6（冻结值）= **{k6:.4f} N·m/A**；传感器 k = {a.k_sensor} N·m/count",
             f"- 对齐：L = {L:+.2f} s（臂晚开始，时钟偏差 {a.skew} s）",
             f"- 转动窗：{r0 - st[0]:.1f} ~ {r1 - st[0]:.1f} s（传感器时标）", "",
             "| 口径 | MAE | RMSE | 偏差(预测−实测) | 相关 r |", "|---|---|---|---|---|",
             f"| 全程 | {s_all['mae']:.3f} | {s_all['rmse']:.3f} | {s_all['bias']:+.3f} | {s_all['r']:+.3f} |",
             f"| 转动窗 | {s_rot['mae']:.3f} | {s_rot['rmse']:.3f} | {s_rot['bias']:+.3f} | {s_rot['r']:+.3f} |", "",
             f"- 转动窗内：预测均值 {pred[m].mean():+.3f}，实测均值 {sens[m].mean():+.3f}，"
             f"比值 {pred[m].mean() / sens[m].mean() if abs(sens[m].mean()) > .05 else float('nan'):.2f}",
             f"- 峰值：预测 {pred[np.argmax(np.abs(pred))]:+.2f}，实测 {sens[np.argmax(np.abs(sens))]:+.2f} N·m",
             f"- 仅用转动窗反解 K = {K_ls:.2f} N·m/A（相对冻结值 {(K_ls / k6 - 1) * 100:+.0f}%）", ""]

    colors = {"实测（传感器）": "#2563eb", "K6 预测（%.2f·i6）" % k6: "#dc2626",
              "J6 电流 i6": "#7c3aed", "q6": "#059669"}
    title = f"{a.tag}  {a.title}"
    charts = [svg([("实测（传感器）", sens), ("K6 预测（%.2f·i6）" % k6, pred)], ts,
                  f"{title} —— 预测 vs 实测（N·m）", "N·m", colors,
                  marks=[(r0, r1, "J6 转动")]),
              svg([("J6 电流 i6", i6)], ts, f"{title} —— J6 电流（A）", "A", colors,
                  height=150, hline=0.0, yfmt="{:+.3f}")]

    body = []
    for k in range(0, len(ts), 50):
        body.append("<tr><td>%.1f</td><td>%+.2f</td><td>%+.4f</td><td>%+.3f</td><td>%+.3f</td>"
                    "<td>%+.3f</td></tr>"
                    % (ts[k] - st[0], q6[k], i6[k], pred[k], sens[k], pred[k] - sens[k]))

    html = f"""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>{a.tag} 定量实验</title><style>
 body{{margin:0;padding:26px 30px 60px;background:#f8fafc;color:#1f2937;
   font-family:-apple-system,Segoe UI,Helvetica,Arial,"Microsoft YaHei",sans-serif;line-height:1.6;font-size:15px}}
 h1{{font-size:22px;margin:0 0 6px}} h2{{font-size:17px;margin:26px 0 8px}}
 .card{{background:#fff;border:1px solid #e5e7eb;border-radius:10px;padding:16px 18px;margin:14px 0}}
 table{{border-collapse:collapse;width:100%;font-size:13px}}
 th,td{{border-bottom:1px solid #eef2f7;padding:6px 8px;text-align:right}}
 th{{background:#f1f5f9}} td:first-child,th:first-child{{text-align:left}}
 .note{{color:#6b7280;font-size:12.5px}}
 code{{background:#f1f5f9;padding:1px 5px;border-radius:4px}}
</style></head><body>
<h1>定量实验 {a.tag}：{a.title}</h1>
<div class="note">K6 = {k6:.4f} N·m/A（冻结，4.6%）　|　传感器 k = {a.k_sensor} N·m/count　|
对齐 L = {L:+.2f} s</div>
<div class="card"><table>
<tr><th>口径</th><th>MAE</th><th>RMSE</th><th>偏差</th><th>相关 r</th></tr>
<tr><td>全程</td><td>{s_all['mae']:.3f}</td><td>{s_all['rmse']:.3f}</td><td>{s_all['bias']:+.3f}</td><td>{s_all['r']:+.3f}</td></tr>
<tr><td><b>转动窗</b></td><td><b>{s_rot['mae']:.3f}</b></td><td>{s_rot['rmse']:.3f}</td><td>{s_rot['bias']:+.3f}</td><td>{s_rot['r']:+.3f}</td></tr>
</table></div>
<div class="card">{charts[0]}{charts[1]}</div>
<div class="card"><table><tr><th>t (s)</th><th>q6 (°)</th><th>i6 (A)</th>
<th>K6 预测 (N·m)</th><th>实测 (N·m)</th><th>差</th></tr>{''.join(body)}</table></div>
</body></html>"""

    prefix = Path(a.out) if a.out else HERE / "runs" / f"{a.tag}_quant"
    Path(str(prefix) + ".md").write_text("\n".join(lines), encoding="utf-8")
    Path(str(prefix) + ".html").write_text(html, encoding="utf-8")
    print("\n".join(lines))
    print(f"已输出 {prefix}.md / .html")


if __name__ == "__main__":
    main()
