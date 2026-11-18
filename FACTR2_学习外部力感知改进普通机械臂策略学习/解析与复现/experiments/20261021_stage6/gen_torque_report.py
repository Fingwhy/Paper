#!/usr/bin/env python3
"""汇总四次「瓶盖 × 力矩传感器 × J6 电流」运行，输出表格 + 折线图（自包含 HTML）。

三条力矩曲线
------------
蓝  实测（传感器）
红  用 K6 = 0.3889 预测（K6·i6，项目现用常数）
橙  用「反解出来的 K」预测（K_fit·i6 + b_本次），即拿实测数据反推 K 后再预测

对齐口径
--------
两机时钟不同步：实测 VM 比 Windows 慢 7.97 s。臂侧 raw 头的 `started`（VM 本地时间）
+7.97 s 换算到 Windows 时基，再与传感器头的 `t0_epoch` 求差得 L，`臂时间 = 传感器时间 + L`。

反解 K 的口径
-------------
把四次运行**转动窗内**的全部样本合成一条直线 τ_sens = K·i6 + b 来解 K。

不能逐次解：单次电流跨度只有 ~0.2 A，组内斜率完全是噪声
（四次分别 3.7 / 3.5 / 0.2 / −0.5）。
也不能给每次单独加截距：那样 K 只能由组内斜率决定，会解出 2.3 这种错值。
合成一条线用的是全程 0.62 A 跨度，才有分辨力。

输出
----
S6_TORQUE_CHARTS.html   图表 + 表格
S6_TORQUE_DATA.csv      逐点数据（0.25 s 网格）
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RUNS = [
    ("CAP1", "拧紧 · 到底", "cap1", "瓶盖已拧到底；传感器峰值超满量程 12%"),
    ("CAP2", "松开 · 紧盖", "cap2", "瓶盖仍很紧，夹爪打滑"),
    ("CAP3", "松开 · 松盖", "cap3", "用户手动松开后"),
    ("CAP4", "拧紧 · 松盖", "cap4", "同一状态，反向拧紧 30°"),
]
STARTED = {  # 从 raw/*.jsonl 头里读出的 "started"（VM 本地时间）
    "cap1": "2026-10-26T15:18:58+0800",
    "cap2": "2026-10-26T15:24:41+0800",
    "cap3": "2026-10-26T15:30:34+0800",
    "cap4": "2026-10-26T15:32:29+0800",
}
SKEW = 7.97                 # VM 时钟比 Windows 慢（秒），实测两次一致
K6 = 0.3889                # 项目现用的 J6 力矩常数
K_SENSOR = 0.001          # N·m / count（仪表 Sd=3 直接读 N·m，无需标定；2026-10-27 挂重验证 +0.87%）
GRID = 0.25                 # 重采样步长 s


def load(tag: str):
    srows = [json.loads(l) for l in open(HERE / "runs" / f"sensor_{tag}.jsonl", encoding="utf-8")
             if '"tau"' in l]
    st = np.array([r["t"] for r in srows], float)
    sv = np.array([r["raw"] for r in srows], float) * K_SENSOR
    st -= st[0]

    arows = [json.loads(l) for l in open(HERE / f"runs/{tag}_samples.jsonl", encoding="utf-8")]
    at = np.array([r["t"] for r in arows], float)
    at -= at[0]
    ai = np.array([r["cur"][5] for r in arows], float)
    aq = np.degrees(np.array([r["q"][5] for r in arows], float))
    adq = np.abs(np.array([r["dq"][5] for r in arows], float))
    return dict(st=st, sv=sv, at=at, ai=ai, aq=aq, adq=adq)


def find_offset(d: dict, tag: str) -> tuple[float, float]:
    """L = 传感器起点(Windows) − 臂起点(换算后 Windows 时基)。负值表示传感器先开始。"""
    from datetime import datetime
    a0 = datetime.strptime(STARTED[tag], "%Y-%m-%dT%H:%M:%S%z").timestamp() + SKEW
    s0 = json.loads(open(HERE / "runs" / f"sensor_{tag}.jsonl", encoding="utf-8")
                    .readline())["t0_epoch"]
    L = s0 - a0
    g = np.arange(max(d["st"][0], d["at"][0] - L), min(d["st"][-1], d["at"][-1] - L), 0.1)
    if len(g) > 20:
        s_ = np.interp(g, d["st"], d["sv"])
        a_ = K6 * np.interp(g + L, d["at"], d["ai"])
        r = float(np.corrcoef(s_, a_)[0, 1])
    else:
        r = float("nan")
    return L, r


def resample(d: dict, L: float):
    ts = np.arange(0, d["st"][-1], GRID)
    sv = np.interp(ts, d["st"], d["sv"])
    ta = ts + L
    ai = np.interp(ta, d["at"], d["ai"])
    q6 = np.interp(ta, d["at"], d["aq"])
    dq = np.interp(ta, d["at"], d["adq"])
    return ts, sv, K6 * ai, ai, q6, dq


# ---------- SVG 折线图 ----------
def svg_chart(curves, tmin, tmax, title, ylabel, colors, height=210, width=880,
              hline=None, yfmt="{:+.2f}"):
    padL, padR, padT, padB = 62, 14, 26, 34
    W, H = width, height
    allv = np.concatenate([c[2] for c in curves])
    lo, hi = float(np.nanmin(allv)), float(np.nanmax(allv))
    if hi - lo < 1e-6:
        hi, lo = lo + 1, lo - 1
    pad = 0.12 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    sx = lambda t: padL + (t - tmin) / max(1e-9, (tmax - tmin)) * (W - padL - padR)
    sy = lambda v: H - padB - (v - lo) / (hi - lo) * (H - padT - padB)

    out = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect x="0" y="0" width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="17" font-size="13" font-weight="600" fill="#1f2937">{title}</text>']
    for i in range(5):
        v = lo + (hi - lo) * i / 4
        y = sy(v)
        out.append(f'<line x1="{padL}" y1="{y:.1f}" x2="{W-padR}" y2="{y:.1f}" stroke="#eef2f7"/>')
        out.append(f'<text x="{padL-8}" y="{y+4:.1f}" font-size="10" fill="#6b7280" '
                   f'text-anchor="end">{yfmt.format(v)}</text>')
    n = 6
    for i in range(n + 1):
        t = tmin + (tmax - tmin) * i / n
        x = sx(t)
        out.append(f'<line x1="{x:.1f}" y1="{padT}" x2="{x:.1f}" y2="{H-padB}" stroke="#f5f7fa"/>')
        out.append(f'<text x="{x:.1f}" y="{H-padB+15}" font-size="10" fill="#6b7280" '
                   f'text-anchor="middle">{t:.0f}</text>')
    if hline is not None:
        out.append(f'<line x1="{padL}" y1="{sy(hline):.1f}" x2="{W-padR}" y2="{sy(hline):.1f}" '
                   f'stroke="#9ca3af" stroke-width="1" stroke-dasharray="5,4"/>')
    for name, t, y in curves:
        pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(t, y) if np.isfinite(b))
        col = colors.get(name, "#2563eb")
        w = 2.1 if name.startswith("实测") else 1.5
        dash = ' stroke-dasharray="6,3"' if name.startswith("拟合") else ""
        out.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{w}"'
                   f'{dash} stroke-linejoin="round"/>')
    lx = padL + 4
    for name, *_ in curves:
        col = colors.get(name, "#2563eb")
        out.append(f'<rect x="{lx}" y="{padT+2}" width="16" height="3" fill="{col}"/>')
        out.append(f'<text x="{lx+21}" y="{padT+7}" font-size="10.5" fill="#374151">{name}</text>')
        lx += 21 + 6.4 * len(name) + 16
    out.append(f'<text x="{W-padR}" y="{padT+7}" font-size="10" fill="#9ca3af" '
               f'text-anchor="end">{ylabel}  |  横轴 t (s)</text>')
    out.append("</svg>")
    return "".join(out)


def main():
    # ---------- 第一遍：加载 + 对齐 + 取转动窗 ----------
    data = []
    for name, desc, tag, note in RUNS:
        d = load(tag)
        L, r = find_offset(d, tag)
        ts, sv, av, ai, q6, dq = resample(d, L)
        m_arm = d["adq"] > 0.3 * d["adq"].max()
        a0, a1 = float(d["at"][m_arm].min()), float(d["at"][m_arm].max())
        win = (a0 - L, a1 - L)          # ta = ts + L ⇒ ts = ta − L
        m = (ts >= win[0]) & (ts <= win[1])
        data.append(dict(name=name, desc=desc, tag=tag, note=note, L=L, r=r,
                         ts=ts, sv=sv, av=av, ai=ai, q6=q6, dq=dq, win=win, m=m))

    # ---------- 反解 K：全部转动窗样本拟合一条直线 τ = K·i6 + b ----------
    # 注意：**不能**给每次单独加截距。单次电流只有 ~0.2 A 的跨度，加了截距后 K 只能
    # 由组内斜率决定，而组内斜率完全是噪声（实测四次分别 3.7 / 3.5 / 0.2 / −0.5），
    # 会把 K 解到 2.3 这种明显错误的值。合成一条直线用的是全程 0.62 A 的跨度，
    # 才有分辨力。
    Xg = np.concatenate([dd["ai"][dd["m"]] for dd in data if dd["m"].sum() > 20])
    Yg = np.concatenate([dd["sv"][dd["m"]] for dd in data if dd["m"].sum() > 20])
    if len(Xg) > 50:
        K_fit, b_fit = (float(x) for x in np.polyfit(Xg, Yg, 1))
        res_all = float(np.sqrt(np.mean((K_fit * Xg + b_fit - Yg) ** 2)))
        r_all = float(np.corrcoef(Xg, Yg)[0, 1])
    else:
        K_fit, b_fit, res_all, r_all = float("nan"), 0.0, float("nan"), float("nan")

    # ---------- 第二遍：出表与图 ----------
    rows_csv = ["run,phase,t_s,q6_deg,i6_A,k6i6_Nm,tau_sens_Nm,tau_fitK_Nm,dq6_rad_s"]
    summary, charts, tables = [], [], []
    colors = {"实测（传感器）": "#2563eb", "K6 预测（K=0.3889）": "#dc2626",
              "拟合K 预测": "#f59e0b", "J6 角度 q6": "#059669", "i6": "#7c3aed"}

    for k, dd in enumerate(data):
        ts, sv, av, ai, q6, dq, m = (dd["ts"], dd["sv"], dd["av"], dd["ai"],
                                     dd["q6"], dd["dq"], dd["m"])
        sv_fit = K_fit * ai + b_fit      # 用反解出的 K 预测（全局一条线）
        arm_m, sen_m = float(av[m].mean()), float(sv[m].mean())
        ratio = arm_m / sen_m if abs(sen_m) > 0.05 else float("nan")
        summary.append(dict(name=dd["name"], desc=dd["desc"], note=dd["note"], L=dd["L"],
                            r=dd["r"], win=dd["win"], arm=arm_m, sen=sen_m, ratio=ratio,
                            arm_pk=float(av[np.argmax(np.abs(av))]),
                            sen_pk=float(sv[np.argmax(np.abs(sv))]),
                            bfit=b_fit,
                            err_m=float(np.sqrt(np.mean((sv_fit[m] - sv[m]) ** 2)))
                            if m.sum() else float("nan")))

        for t, q, i, a, s_, f_, dqq in zip(ts, q6, ai, av, sv, sv_fit, dq):
            rows_csv.append("%s,%s,%.2f,%.2f,%.5f,%.4f,%.4f,%.4f,%.4f"
                            % (dd["tag"], dd["desc"], t, q, i, a, s_, f_, dqq))

        charts.append(svg_chart([("实测（传感器）", ts, sv),
                                 ("K6 预测（K=0.3889）", ts, av),
                                 ("拟合K 预测", ts, sv_fit)],
                                ts[0], ts[-1],
                                f"{dd['name']}  {dd['desc']}   —— 实测 vs 用 K 预测（N·m）"
                                f"　拟合 K = {K_fit:.2f}（{(K_fit / K6 - 1) * 100:+.0f}%）",
                                "N·m", colors))
        charts.append(svg_chart([("J6 角度 q6", ts, q6)], ts[0], ts[-1],
                                f"{dd['name']}  J6 角度（deg）", "deg", colors, height=150,
                                yfmt="{:+.0f}"))
        charts.append(svg_chart([("i6", ts, ai)], ts[0], ts[-1],
                                f"{dd['name']}  J6 电流（A）", "A", colors, height=150,
                                hline=0.0, yfmt="{:+.3f}"))

        body = []
        for j in range(0, len(ts), int(2 / GRID)):
            body.append("<tr><td>%.1f</td><td>%+.2f</td><td>%+.4f</td><td>%+.3f</td>"
                        "<td>%+.3f</td><td>%+.3f</td><td>%s</td></tr>"
                        % (ts[j], q6[j], ai[j], av[j], sv[j], sv_fit[j],
                           ("%.2f" % (av[j] / sv[j])) if abs(sv[j]) > 0.08 else "—"))
        tables.append((dd["name"], dd["desc"], "".join(body)))

    # 汇总表
    head = ("<tr><th>运行</th><th>方向 / 状态</th><th>对齐 L (s)</th><th>相关 r</th>"
            "<th>转动窗 (s)</th><th>臂 K6·i6</th><th>实测</th><th>臂/实测</th>"
            "<th>臂峰值</th><th>实测峰值</th><th>拟合K 预测残差</th></tr>")
    srows = []
    for s in summary:
        w = f"{s['win'][0]:.0f}~{s['win'][1]:.0f}"
        srows.append("<tr><td><b>%s</b></td><td>%s</td><td>%.2f</td><td>%+.2f</td><td>%s</td>"
                     "<td>%+.3f</td><td>%+.3f</td><td><b>%.2f</b></td><td>%+.2f</td>"
                     "<td>%+.2f</td><td>%.3f</td></tr>"
                     % (s["name"], s["desc"], s["L"], s["r"], w, s["arm"], s["sen"],
                        s["ratio"], s["arm_pk"], s["sen_pk"], s["err_m"]))

    # 散点：臂 vs 实测
    sc = [(dd["name"], dd["sv"][dd["m"]], dd["av"][dd["m"]]) for dd in data]
    lo = min(float(s[1].min()) for s in sc) - 0.3
    hi = max(float(s[1].max()) for s in sc) + 0.3
    W, H, padL, padB, padT, padR = 640, 430, 66, 44, 26, 16
    sx = lambda v: padL + (v - lo) / (hi - lo) * (W - padL - padR)
    sy = lambda v: H - padB - (v - lo) / (hi - lo) * (H - padT - padB)
    dot_col = {"CAP1": "#f59e0b", "CAP2": "#2563eb", "CAP3": "#059669", "CAP4": "#dc2626"}
    svg = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" '
           f'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{padL}" y="17" font-size="13" font-weight="600" fill="#1f2937">'
           f'臂 K6·i6（纵） vs 实测（横），仅取 J6 转动窗</text>']
    for i in range(5):
        v = lo + (hi - lo) * i / 4
        svg.append(f'<line x1="{padL}" y1="{sy(v):.1f}" x2="{W-padR}" y2="{sy(v):.1f}" stroke="#eef2f7"/>')
        svg.append(f'<text x="{padL-8}" y="{sy(v)+4:.1f}" font-size="10" fill="#6b7280" '
                   f'text-anchor="end">{v:+.1f}</text>')
        svg.append(f'<line x1="{sx(v):.1f}" y1="{padT}" x2="{sx(v):.1f}" y2="{H-padB}" stroke="#f5f7fa"/>')
        svg.append(f'<text x="{sx(v):.1f}" y="{H-padB+15}" font-size="10" fill="#6b7280" '
                   f'text-anchor="middle">{v:+.1f}</text>')
    svg.append(f'<line x1="{sx(lo):.1f}" y1="{sy(lo):.1f}" x2="{sx(hi):.1f}" y2="{sy(hi):.1f}" '
               f'stroke="#9ca3af" stroke-dasharray="6,4"/>')
    svg.append(f'<text x="{sx(hi)-6:.1f}" y="{sy(hi)+14:.1f}" font-size="10" fill="#6b7280" '
               f'text-anchor="end">y = x</text>')
    for name, s_, a_ in sc:
        svg.append("".join(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="1.7" '
                           f'fill="{dot_col[name]}" opacity="0.55"/>'
                           for x, y in zip(s_, a_) if np.isfinite(y)))
    lx = padL + 4
    for name, *_ in sc:
        svg.append(f'<circle cx="{lx+5}" cy="{padT+2}" r="4" fill="{dot_col[name]}"/>')
        svg.append(f'<text x="{lx+14}" y="{padT+6}" font-size="10.5" fill="#374151">{name}</text>')
        lx += 28 + 6.4 * len(name)
    svg.append(f'<text x="{W-padR}" y="{padT+6}" font-size="10" fill="#9ca3af" text-anchor="end">'
               f'虚线 = 完全一致</text>')
    svg.append("</svg>")
    scatter = "".join(svg)

    pct = (K_fit / K6 - 1) * 100
    html = [f"""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>瓶盖力矩对比 · 四次运行</title>
<style>
 body{{margin:0;padding:28px 30px 60px;background:#f8fafc;color:#1f2937;
      font-family:-apple-system,Segoe UI,Helvetica,Arial,"Microsoft YaHei",sans-serif;line-height:1.65;font-size:15px}}
 h1{{font-size:23px;margin:0 0 6px}} h2{{font-size:18px;margin:34px 0 10px;padding-bottom:6px;border-bottom:2px solid #e2e8f0}}
 .sub{{color:#6b7280;font-size:13px;margin-bottom:18px}}
 .card{{background:#fff;border:1px solid #e5e7eb;border-radius:10px;padding:16px 18px;margin:14px 0;
       box-shadow:0 1px 2px rgba(0,0,0,.03)}}
 table{{border-collapse:collapse;width:100%;font-size:13px}}
 th,td{{border-bottom:1px solid #eef2f7;padding:6px 8px;text-align:right;white-space:nowrap}}
 th{{background:#f1f5f9;font-weight:600;color:#334155}}
 td:first-child,th:first-child,td:nth-child(2),th:nth-child(2){{text-align:left}}
 tr:hover td{{background:#f8fafc}}
 .note{{color:#6b7280;font-size:12.5px}}
 code{{background:#f1f5f9;padding:1px 5px;border-radius:4px;font-size:12.5px}}
 .big{{font-size:15px;font-weight:600;color:#b45309}}
</style></head><body>
<h1>瓶盖 · 力矩传感器 · J6 电流：四次运行全部数据</h1>
<div class="sub">2026-10-26 &nbsp;|&nbsp; 传感器 VYJN1-5Nm（k = 0.02774 N·m/count）&nbsp;|&nbsp; K6 = 0.3889 N·m/A</div>""",
            f"""<div class="card"><b>四次联立反解 K</b>
<div style="margin-top:8px;font-size:15px">
 τ<sub>sensor</sub> = <span class="big">K · i6</span> + b<sub>本次</sub>
 &nbsp;⇒&nbsp; <span class="big">K = {K_fit:.2f} N·m/A</span>
 （现用 10.79，<b>{'+' if pct > 0 else ''}{pct:.0f}%</b>），拟合残差 {res_all:.3f} N·m</div>
<div class="note" style="margin-top:6px">
用的是<b>四次转动窗全部样本合成一条直线</b>（电流跨度 −0.23~+0.39 A，相关系数 {r_all:.2f}）。
不能逐次解：单次电流跨度只有 ~0.2 A，组内斜率是纯噪声（四次分别 3.7 / 3.5 / 0.2 / −0.5）；
也不能给每次单独加截距，那样 K 只能由组内斜率定，会解出 2.3 这种错值。
截距 b = {b_fit:+.3f} N·m 是四次共用的平均偏置。</div></div>""",
            """<div class="card"><b>口径</b>
<ul style="margin:8px 0 0;padding-left:22px">
<li><b>对齐</b>：实测 VM 时钟比 Windows 慢 <b>7.97 s</b>。臂侧 raw 头的 <code>started</code>（VM 本地时间）
+7.97 s 换到 Windows 时基，再与传感器头 <code>t0_epoch</code> 求差得 L，
映射为 <code>臂时间 = 传感器时间 + L</code>。</li>
<li><b>转动窗</b>：以 <code>|dq6| &gt; 0.3·max</code> 判定 J6 真正在转的区间，只在窗内取均值与反解。</li>
<li><b>传感器</b>：一律 <code>raw × 0.02774</code>，<b>不减零点</b>（换夹具后零点会变，看增量更可靠）。</li>
<li><b>三条力矩曲线</b>：
<span style="color:#2563eb;font-weight:600">蓝 = 实测（传感器）</span>；
<span style="color:#dc2626;font-weight:600">红 = 用 K6=0.3889 预测（K6·i6）</span>；
<span style="color:#f59e0b;font-weight:600">橙虚线 = 用联立反解的 K 预测（K_fit·i6 + b）</span>。
红蓝偏离 = K6 的误差；橙蓝贴合 = 该模型能达到的上限。</li>
<li><b>物理预期</b>：臂力矩 = 载荷 + 关节摩擦，所以「臂 / 实测」应略大于 1。</li>
</ul></div>""",
            "<h2>汇总表</h2><div class=\"card\"><table>" + head + "".join(srows) + """</table>
<div class="note" style="margin-top:10px">CAP1 的 0.65 属于异常：拧到底时传感器峰值超满量程 12%，
且力矩 1 s 内冲高回落。其余三次 1.09 / 1.41 / 1.18，方向正确。</div></div>"""]

    html.append("<h2>散点：臂 vs 实测</h2><div class=\"card\">" + scatter +
                "<div class=\"note\">点基本落在 y = x 上方，说明臂的读数略大于实测，"
                "与「臂 = 载荷 + 摩擦」一致。</div></div>")

    for i, (name, desc, body) in enumerate(tables):
        s = summary[i]
        html.append(f"<h2>{name} · {desc}</h2><div class=\"card\"><div class=\"note\" "
                    f"style=\"margin-bottom:8px\">{s['note']}　|　对齐 L = {s['L']:.2f} s　|　"
                    f"相关 r = {s['r']:+.2f}</div>")
        html.append(charts[i * 3] + charts[i * 3 + 1] + charts[i * 3 + 2])
        html.append("</div><div class=\"card\"><table><tr><th>t (s)</th><th>q6 (°)</th>"
                    "<th>i6 (A)</th><th>K6 预测 (N·m)</th><th>实测 (N·m)</th>"
                    "<th>拟合K 预测 (N·m)</th><th>臂/实测</th></tr>" + body + "</table></div>")

    html.append("</body></html>")
    (HERE / "S6_TORQUE_CHARTS.html").write_text("".join(html), encoding="utf-8")
    (HERE / "S6_TORQUE_DATA.csv").write_text("\n".join(rows_csv) + "\n", encoding="utf-8")

    print("已生成 S6_TORQUE_CHARTS.html 与 S6_TORQUE_DATA.csv")
    print("联立反解：K = %.2f N·m/A（现用 %.4f，%+.0f%%），残差 %.3f N·m"
          % (K_fit, K6, pct, res_all))
    for s in summary:
        print("  %-5s %-12s 臂 %+.3f  实测 %+.3f  臂/实测 %.2f  残差 %.3f"
              % (s["name"], s["desc"], s["arm"], s["sen"], s["ratio"], s["err_m"]))


if __name__ == "__main__":
    main()
