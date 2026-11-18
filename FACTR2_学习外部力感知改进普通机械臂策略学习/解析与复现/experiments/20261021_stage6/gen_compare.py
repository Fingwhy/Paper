#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_compare.py —— 两份拧盖记录的逐项对比（单文件 HTML）

默认对比：
    组1   1.全量原盖未打开.csv         （原盖，未打开）
    组4轮1 4.棘轮全量预开全打开_轮1.csv  （预开后第 1 轮，最终全打开）

图
--
1. |i6| 随转角          两条叠加，看趋势
2. ΔI6 随转角           组B − 组A，看差在哪
3. |effort| 随转角      两条叠加，看夹持力是否一致
4. |i6| 分布直方图      两组叠加，看能不能分开
5. |i6| 时序片段        前 N 帧，看粘滑微观结构

用法
----
    python gen_compare.py
    python gen_compare.py --a "1.全量原盖未打开.csv" --b "2.全量预松未全开.csv" --out X.html
    python gen_compare.py --timewin 900        # 时序图画前 900 帧
"""
import argparse
import csv
import json
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")

DEF_A = "1.全量原盖未打开.csv"
DEF_B = "4.棘轮全量预开全打开_轮1.csv"
COL_A = "#e05252"
COL_B = "#4ade80"


def load(path):
    with open(path, encoding="utf-8") as f:
        rd = list(csv.reader(f))
    hdr, rows = rd[0], rd[1:]
    ix = {h: i for i, h in enumerate(hdr)}
    cols = ["t_s", "q6", "dq6", "grip_ang_mm", "grip_eff_nm"] + [f"i{k}" for k in range(1, 7)]
    return {c: [float(r[ix[c]]) for r in rows] for c in cols}


def qtl(v, p):
    s = sorted(v)
    n = len(s)
    k = (n - 1) * p / 100.0
    a = int(k)
    return s[a] if a + 1 >= n else s[a] + (s[a + 1] - s[a]) * (k - a)


def bin_by_angle(q6, series, step=1.0):
    q0 = q6[0]
    n = int(round(170.0 / step))
    buck = [[] for _ in range(n + 1)]
    for j, v in enumerate(q6):
        a = abs(v - q0)
        k = int(a / step)
        if 0 <= k <= n:
            buck[k].append(series[j])
    xs, ys = [], []
    for k, b in enumerate(buck):
        if len(b) < 3:
            continue
        xs.append(round((k + 0.5) * step, 2))
        ys.append(round(st.mean(b), 5))
    return xs, ys


def hist(vals, lo=0.0, hi=0.5, step=0.02):
    n = int(round((hi - lo) / step))
    out = [0] * n
    for v in vals:
        k = int((v - lo) / step)
        if 0 <= k < n:
            out[k] += 1
    centers = [round(lo + (k + 0.5) * step, 3) for k in range(n)]
    return centers, out


def auc(a, b):
    """AUC = P(a 大于 b)，用秩和法（含并列取平均秩）。0.5=分不开，1=完全分开。"""
    allv = sorted([(v, 0) for v in a] + [(v, 1) for v in b], key=lambda t: t[0])
    n1, n2 = len(a), len(b)
    i, rsum1 = 0, 0.0
    while i < len(allv):
        j = i
        while j + 1 < len(allv) and allv[j + 1][0] == allv[i][0]:
            j += 1
        avg = (i + j) / 2.0 + 1
        for k in range(i, j + 1):
            if allv[k][1] == 0:
                rsum1 += avg
        i = j + 1
    return (rsum1 - n1 * (n1 + 1) / 2.0) / (n1 * n2)


def seg_means(d, series, step=10.0):
    """按转角每 step 度分段的均值（每段代表约 step/15 秒的数据）。"""
    q = d["q6"]
    q0 = q[0]
    n = int(170.0 / step)
    buck = [[] for _ in range(n + 1)]
    for j, v in enumerate(q):
        k = int(abs(v - q0) / step)
        if 0 <= k <= n:
            buck[k].append(series[j])
    return [st.mean(b) for b in buck if len(b) >= 10]


def best_acc(a, b):
    """扫阈值，返回 (最优准确率, 阈值)。"""
    vals = [x / 1000.0 for x in range(30, 500, 5)]
    best = (0.0, 0.0)
    for t in vals:
        acc = (sum(1 for v in a if v >= t) + sum(1 for v in b if v < t)) / (len(a) + len(b))
        if acc > best[0]:
            best = (acc, t)
    return best


def stat_block(d, name):
    i6 = [abs(v) for v in d["i6"]]
    eff = [abs(v) for v in d["grip_eff_nm"]]
    t = d["t_s"]
    n = len(t)
    dur = t[-1] - t[0]
    runs, c = [], 0
    for v in i6:
        if v < 0.20:
            c += 1
        elif c:
            runs.append(c)
            c = 0
    if c:
        runs.append(c)
    return {
        "name": name, "frames": n, "dur": round(dur, 2), "hz": round(n / dur, 1),
        "q0": round(d["q6"][0], 1), "q1": round(d["q6"][-1], 1),
        "i6_mean": round(st.mean(i6), 4), "i6_std": round(st.pstdev(i6), 4),
        "i6_min": round(min(i6), 4), "i6_max": round(max(i6), 4),
        "p10": round(qtl(i6, 10), 4), "p50": round(qtl(i6, 50), 4),
        "p90": round(qtl(i6, 90), 4), "p99": round(qtl(i6, 99), 4),
        "pos": sum(1 for v in d["i6"] if v > 0),
        "idle": round(100 * sum(1 for v in i6 if v < 0.20) / n, 1),
        "runs": len(runs), "run_max": max(runs) if runs else 0,
        "eff_mean": round(st.mean(eff), 4), "eff_std": round(st.pstdev(eff), 4),
        "eff_min": round(min(eff), 4), "eff_max": round(max(eff), 4),
        "grip": round(st.mean(d["grip_ang_mm"]), 3),
        "i1": round(st.mean(d["i1"]), 3), "i2": round(st.mean(d["i2"]), 3),
        "i3": round(st.mean(d["i3"]), 3), "i4": round(st.mean(d["i4"]), 3),
        "i5": round(st.mean(d["i5"]), 3), "i6": round(st.mean(d["i6"]), 3),
        "dq6": round(st.mean([abs(v) for v in d["dq6"]]), 2),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default=DEF_A)
    ap.add_argument("--b", default=DEF_B)
    ap.add_argument("--out", default=os.path.join(HERE, "COMPARE_G1_G4R1.html"))
    ap.add_argument("--step", type=float, default=1.0, help="分箱步长（度）")
    ap.add_argument("--timewin", type=int, default=700, help="时序图画前多少帧")
    a = ap.parse_args()

    pa = os.path.join(DATA, a.a)
    pb = os.path.join(DATA, a.b)
    for p in (pa, pb):
        if not os.path.exists(p):
            raise SystemExit(f"[FAIL] 不存在 {p}")
    da, db = load(pa), load(pb)

    i6a = [abs(v) for v in da["i6"]]
    i6b = [abs(v) for v in db["i6"]]
    ea = [abs(v) for v in da["grip_eff_nm"]]
    eb = [abs(v) for v in db["grip_eff_nm"]]

    xa, ya = bin_by_angle(da["q6"], i6a, a.step)
    xb, yb = bin_by_angle(db["q6"], i6b, a.step)
    # 对齐到共同网格再求差
    mapb = dict(zip(xb, yb))
    dx, dy = [], []
    for x, y in zip(xa, ya):
        if x in mapb:
            dx.append(x)
            dy.append(round(mapb[x] - y, 5))

    ea_x, ea_y = bin_by_angle(da["q6"], ea, a.step)
    eb_x, eb_y = bin_by_angle(db["q6"], eb, a.step)

    hc, ha = hist(i6a)
    _, hb = hist(i6b)

    n = min(len(i6a), len(i6b), a.timewin)
    tser = {
        "t": [round(da["t_s"][j] - da["t_s"][0], 3) for j in range(n)],
        "a": [round(i6a[j], 4) for j in range(n)],
        "b": [round(i6b[j], 4) for j in range(n)],
    }

    sa = stat_block(da, os.path.basename(pa)[:-4])
    sb = stat_block(db, os.path.basename(pb)[:-4])

    # 判别能力：这两组数据到底能不能被分开
    acc_f, thr = best_acc(i6a, i6b)
    sgm_a = seg_means(da, i6a)
    sgm_b = seg_means(db, i6b)
    disc = {
        "auc_frame": round(auc(i6a, i6b), 3),
        "acc_frame": round(acc_f * 100, 1),
        "thr": round(thr, 3),
        "auc_seg": round(auc(sgm_a, sgm_b), 3),
        "n_seg": len(sgm_a),
        "seg_a": round(st.mean(sgm_a), 4) if sgm_a else 0,
        "seg_b": round(st.mean(sgm_b), 4) if sgm_b else 0,
    }

    payload = {
        "a": sa, "b": sb, "disc": disc,
        "curves": {
            "i6_a": {"x": xa, "y": ya}, "i6_b": {"x": xb, "y": yb},
            "d": {"x": dx, "y": dy},
            "eff_a": {"x": ea_x, "y": ea_y}, "eff_b": {"x": eb_x, "y": eb_y},
        },
        "hist": {"c": hc, "a": ha, "b": hb},
        "tser": tser,
        "colors": {"a": COL_A, "b": COL_B},
        "step": a.step,
    }
    title = "%s vs %s" % (os.path.basename(pa)[:-4], os.path.basename(pb)[:-4])
    html = (TEMPLATE
            .replace("__TITLE__", title)
            .replace("__DATA__", json.dumps(payload, ensure_ascii=False,
                                            separators=(",", ":"))))
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[out] {a.out}  {os.path.getsize(a.out)/1024:.1f} KB")
    print(f"  A {sa['name']}: I6 {sa['i6_mean']:.4f}  eff {sa['eff_mean']:.4f}  开度 {sa['grip']}")
    print(f"  B {sb['name']}: I6 {sb['i6_mean']:.4f}  eff {sb['eff_mean']:.4f}  开度 {sb['grip']}")
    print(f"  ΔI6 = {sb['i6_mean']-sa['i6_mean']:+.4f} A   Δeff = {sb['eff_mean']-sa['eff_mean']:+.4f} N·m")
    print(f"  判别能力: 帧级 AUC {disc['auc_frame']}  准确率 {disc['acc_frame']}%  "
          f"| 10° 段级 AUC {disc['auc_seg']}（各 {disc['n_seg']} 段）")
    return a.out


TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>拧盖对比 · __TITLE__</title>
<style>
  :root{--bg:#1e1e1e;--panel:#252526;--line:#3a3a3a;--txt:#d4d4d4;--dim:#8a8a8a}
  *{box-sizing:border-box}
  body{margin:0;padding:24px 28px 60px;background:var(--bg);color:var(--txt);
       font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;font-size:13px;line-height:1.6}
  h1{font-size:19px;font-weight:600;margin:0 0 6px}
  .sub{color:var(--dim);font-size:12px;margin-bottom:20px}
  .cards{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:20px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px 16px}
  .card h3{font-size:13px;font-weight:600;margin:0 0 8px;display:flex;align-items:center;gap:7px}
  .dot{width:9px;height:9px;border-radius:50%;display:inline-block}
  .card dl{margin:0;display:grid;grid-template-columns:auto 1fr;gap:3px 12px;font-size:12px}
  .card dt{color:var(--dim)}
  .card dd{margin:0;text-align:right;font-variant-numeric:tabular-nums}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;
         padding:16px 18px 12px;margin-bottom:20px}
  .panel h2{font-size:14px;font-weight:600;margin:0 0 2px}
  .panel .hint{color:var(--dim);font-size:11.5px;margin-bottom:10px}
  canvas{display:block;width:100%;height:300px}
  .legend{display:flex;gap:20px;margin-top:10px;font-size:11.5px}
  .legend i{display:inline-block;width:22px;border-top-width:2.5px;border-top-style:solid;
            vertical-align:middle;margin-right:6px}
  table{border-collapse:collapse;width:100%;font-size:12px;margin-top:4px}
  th,td{padding:6px 10px;border-bottom:1px solid var(--line);text-align:right;
        font-variant-numeric:tabular-nums;white-space:nowrap}
  th{color:var(--dim);font-weight:500;font-size:11.5px}
  td:first-child,th:first-child{text-align:left}
  tr.hl td{background:#2d2a1e}
  .delta{font-weight:600}
  .pos{color:#4ade80}.neg{color:#e05252}.zero{color:var(--dim)}
  .note{background:#2a2418;border-left:3px solid #e0a052;padding:12px 16px;
        border-radius:5px;font-size:12.5px;margin-bottom:20px}
  .note b{color:#e0a052}
  .cards3{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-top:6px}
  .kpi{background:#2b2b2b;border:1px solid var(--line);border-radius:7px;padding:12px 14px}
  .kpi .k{color:var(--dim);font-size:11.5px;margin-bottom:4px}
  .kpi .v{font-size:23px;font-weight:600;font-variant-numeric:tabular-nums;line-height:1.2}
  .kpi .n{color:var(--dim);font-size:11.5px;margin-top:4px}
  .tip{position:fixed;pointer-events:none;background:#111;border:1px solid #555;
       border-radius:5px;padding:6px 9px;font-size:11.5px;display:none;z-index:99;white-space:nowrap}
</style>
</head>
<body>
<h1>拧盖记录逐项对比</h1>
<div class="sub" id="sub"></div>

<div class="note">
<b>先说结论</b>：I6 均值差 <b>10.2%</b>（0.1899 → 0.1705），但两者分布大面积重叠（图 4）；
<b>夹爪 effort 几乎完全相同</b>（0.4437 vs 0.4436，差 0.0001 N·m），
而夹爪开度却差了 <b>0.28 mm</b>（29.89 vs 29.61）。
<br><br>
<b>顺带修正一个之前的错误结论</b>：10-22 我说过"开度能解释 effort 92.9% 的方差"，
那是按开度分组做 F 检验得出的。现在用全部 8 份数据直接算相关：
<b>r(开度, effort) = +0.099，r² = 1.0%</b>——开度基本解释不了 effort。
开度 29.82 的三次 effort 是 0.4608 / 0.4337 / 0.4584（差 6%），而 29.61 与 29.89 两组的均值反而更低。
<b>结论：effort 的波动与开度无单调关系，主要来自瓶盖椭圆度/接触点等随机因素。</b>
</div>

<div class="cards" id="cards"></div>

<div class="panel">
  <h2>图 1 · |i6| 随转角</h2>
  <div class="hint">每 1° 分箱取均值。看曲线高低与前后的走势</div>
  <canvas id="c1"></canvas>
  <div class="legend" id="lg1"></div>
</div>

<div class="panel">
  <h2>图 2 · ΔI6 随转角（B − A）</h2>
  <div class="hint">零线以上 = B 更大。起伏幅度反映"两份记录的差异随转角怎么变"</div>
  <canvas id="c2"></canvas>
  <div class="legend" id="lg2"></div>
</div>

<div class="panel">
  <h2>图 3 · |effort| 随转角</h2>
  <div class="hint">y 轴跨度固定为 ≥0.08 N·m（不随数据缩放），避免把 0.0001 的差别放大成"看起来差很多"</div>
  <canvas id="c3"></canvas>
  <div class="legend" id="lg3"></div>
</div>

<div class="panel">
  <h2>图 4 · |i6| 分布直方图</h2>
  <div class="hint">档宽 0.02 A。两块大面积重叠 ⇒ 单帧电流无法区分两组</div>
  <canvas id="c4"></canvas>
  <div class="legend" id="lg4"></div>
</div>

<div class="panel">
  <h2>图 5 · |i6| 时序片段（开头约 4 秒）</h2>
  <div class="hint">逐帧原值。能看到每秒一次的"顶住→滑脱"锯齿，这是粘滑，不是两种状态</div>
  <canvas id="c5"></canvas>
  <div class="legend" id="lg5"></div>
</div>

<div class="panel">
  <h2>判别能力：能不能靠 |i6| 判断这条数据属于哪一组</h2>
  <div class="hint">AUC = 随机抽一个样本、A 的值大于 B 的概率。0.5 = 完全分不开，1 = 完全分开；要当判据一般需 &gt;0.95</div>
  <div class="cards3" id="disc"></div>
</div>

<div class="panel">
  <h2>逐项统计</h2>
  <div class="hint">差值 = B − A；百分比以 A 为基准</div>
  <table id="tbl"></table>
</div>

<div class="tip" id="tip"></div>

<script>
const D = __DATA__;

function draw(canvas, series, opts){
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  canvas.width = W*dpr; canvas.height = H*dpr;
  const g = canvas.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,W,H);

  const pad = {l:62,r:16,t:14,b:36};
  const iw = W-pad.l-pad.r, ih = H-pad.t-pad.b;

  let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
  for(const s of series){
    const xs=s.x, ys=s.y;
    for(let j=0;j<xs.length;j++){
      const vx=xs[j], vy=ys[j];
      if(vx<xmin) xmin=vx;
      if(vx>xmax) xmax=vx;
      if(vy<ymin) ymin=vy;
      if(vy>ymax) ymax=vy;
    }
  }
  if(!isFinite(xmin)) return;
  if(opts.xRange){ xmin=opts.xRange[0]; xmax=opts.xRange[1]; }
  if(opts.yRange){ ymin=opts.yRange[0]; ymax=opts.yRange[1]; }
  else{
    let sp=ymax-ymin;
    if(!isFinite(sp) || sp<=0) sp=0.01;
    if(opts.minSpan && sp<opts.minSpan){          // 跨度太小就撑开，别放大噪声
      const mid=(ymin+ymax)/2;
      ymin=mid-opts.minSpan/2;
      ymax=mid+opts.minSpan/2;
    }else{
      ymin=ymin-sp*0.1;
      ymax=ymax+sp*0.1;
    }
  }

  const X=v=>pad.l+(v-xmin)/(xmax-xmin)*iw;
  const Y=v=>pad.t+ih-(v-ymin)/(ymax-ymin)*ih;

  g.font="11px -apple-system,Segoe UI,sans-serif";
  g.strokeStyle="#333"; g.lineWidth=1;
  for(let i=0;i<=6;i++){
    const v=xmin+(xmax-xmin)*i/6, px=X(v);
    g.beginPath(); g.moveTo(px,pad.t); g.lineTo(px,pad.t+ih); g.stroke();
    g.fillStyle="#8a8a8a"; g.textAlign="center";
    g.fillText(v.toFixed(opts.xDec||0), px, pad.t+ih+15);
  }
  for(let i=0;i<=5;i++){
    const v=ymin+(ymax-ymin)*i/5, py=Y(v);
    g.beginPath(); g.moveTo(pad.l,py); g.lineTo(pad.l+iw,py); g.stroke();
    g.fillStyle="#8a8a8a"; g.textAlign="right";
    g.fillText(v.toFixed(opts.yDec===undefined?3:opts.yDec), pad.l-8, py+4);
  }
  // 零线
  if(ymin<0 && ymax>0){
    g.strokeStyle="#666"; g.setLineDash([4,3]);
    g.beginPath(); g.moveTo(pad.l,Y(0)); g.lineTo(pad.l+iw,Y(0)); g.stroke();
    g.setLineDash([]);
  }
  g.fillStyle="#8a8a8a"; g.textAlign="center";
  g.fillText(opts.xLabel||"", pad.l+iw/2, H-5);
  g.save(); g.translate(13,pad.t+ih/2); g.rotate(-Math.PI/2);
  g.fillText(opts.yLabel||"",0,0); g.restore();

  for(const s of series){
    g.beginPath(); g.strokeStyle=s.color; g.lineWidth=s.w||1.9;
    g.globalAlpha = s.alpha===undefined?1:s.alpha;
    g.setLineDash(s.dash? s.dash.split(",").map(Number):[]);
    let started=false;
    for(let j=0;j<s.x.length;j++){
      const px=X(s.x[j]), py=Y(s.y[j]);
      if(!started){g.moveTo(px,py);started=true;} else g.lineTo(px,py);
    }
    g.stroke(); g.setLineDash([]); g.globalAlpha=1;
  }
  canvas._map={X,Y,pad,iw,xmin,xmax,ymin,ymax,series,opts};
}

function drawHist(canvas, hist, colors, opts){
  const dpr=window.devicePixelRatio||1;
  const W=canvas.clientWidth,H=canvas.clientHeight;
  canvas.width=W*dpr; canvas.height=H*dpr;
  const g=canvas.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const pad={l:52,r:16,t:14,b:36};
  const iw=W-pad.l-pad.r, ih=H-pad.t-pad.b;
  const c=hist.c, a=hist.a, b=hist.b;
  const n=c.length;
  let mx=0;
  for(let k=0;k<n;k++){ if(a[k]>mx)mx=a[k]; if(b[k]>mx)mx=b[k]; }
  if(!mx) return;
  const bw=iw/n;
  const Y=v=>pad.t+ih-v/mx*ih;

  g.font="11px -apple-system,Segoe UI,sans-serif";
  g.strokeStyle="#333";
  for(let i=0;i<=5;i++){
    const v=mx*i/5, py=Y(v);
    g.beginPath(); g.moveTo(pad.l,py); g.lineTo(pad.l+iw,py); g.stroke();
    g.fillStyle="#8a8a8a"; g.textAlign="right"; g.fillText(v.toFixed(0),pad.l-8,py+4);
  }
  for(let i=0;i<=5;i++){
    const k=Math.round((n-1)*i/5), px=pad.l+k*bw;
    g.fillStyle="#8a8a8a"; g.textAlign="center";
    g.fillText(c[k].toFixed(1), px, pad.t+ih+15);
  }
  g.fillText(opts.xLabel||"", pad.l+iw/2, H-5);
  g.save(); g.translate(12,pad.t+ih/2); g.rotate(-Math.PI/2);
  g.fillText(opts.yLabel||"",0,0); g.restore();

  // 两组半透明叠加
  for(let k=0;k<n;k++){
    g.fillStyle=colors.a+"99";
    g.fillRect(pad.l+k*bw, Y(a[k]), bw-0.8, ih-Y(a[k])+pad.t);
  }
  for(let k=0;k<n;k++){
    g.fillStyle=colors.b+"99";
    g.fillRect(pad.l+k*bw, Y(b[k]), bw-0.8, ih-Y(b[k])+pad.t);
  }
  canvas._mapnull=true;
}

function bindTip(canvas){
  canvas.addEventListener("mousemove", ev=>{
    const m=canvas._map; if(!m) return;
    const r=canvas.getBoundingClientRect();
    const mx=ev.clientX-r.left;
    if(mx<m.pad.l||mx>m.pad.l+m.iw){tip.style.display="none";return;}
    const xv=m.xmin+(mx-m.pad.l)/m.iw*(m.xmax-m.xmin);
    const rows=[];
    for(const s of m.series){
      let best=-1,bd=1e9;
      for(let j=0;j<s.x.length;j++){
        const d=Math.abs(s.x[j]-xv);
        if(d<bd){bd=d;best=j;}
      }
      if(best>=0&&bd<=4) rows.push([s.label,s.color,s.y[best]]);
    }
    if(!rows.length){tip.style.display="none";return;}
    tip.innerHTML=`<b>x = ${xv.toFixed(1)}</b><br>`+
      rows.map(r=>`<span style="color:${r[1]}">■</span> ${r[0]}：${r[2].toFixed(4)}`).join("<br>");
    tip.style.display="block";
    tip.style.left=(ev.clientX+14)+"px";
    tip.style.top=(ev.clientY+10)+"px";
  });
  canvas.addEventListener("mouseleave",()=>{tip.style.display="none";});
}

const tip=document.getElementById("tip");
const CA=D.colors.a, CB=D.colors.b;

function render(){
  document.getElementById("sub").innerHTML =
    `A = <span style="color:${CA}">${D.a.name}</span> ｜ ` +
    `B = <span style="color:${CB}">${D.b.name}</span> ｜ ` +
    `分箱 ${D.step}°`;

  // 卡片
  const fmt=(v,d=4)=>v.toFixed(d);
  const cardOf=(s,col)=>`
    <div class="card">
      <h3><span class="dot" style="background:${col}"></span>${s.name}</h3>
      <dl>
        <dt>帧数 / 时长</dt><dd>${s.frames} / ${s.dur}s (${s.hz} Hz)</dd>
        <dt>q6 范围</dt><dd>${s.q0}° → ${s.q1}°</dd>
        <dt>|i6| 均值 ± std</dt><dd>${fmt(s.i6_mean)} ± ${fmt(s.i6_std)}</dd>
        <dt>|i6| P10 / P50 / P90</dt><dd>${fmt(s.p10)} / ${fmt(s.p50)} / ${fmt(s.p90)}</dd>
        <dt>空转占比 (&lt;0.20)</dt><dd>${s.idle}%（${s.runs} 段，最长 ${s.run_max} 帧）</dd>
        <dt>|effort| 均值 ± std</dt><dd>${fmt(s.eff_mean)} ± ${fmt(s.eff_std)}</dd>
        <dt>夹爪开度</dt><dd>${s.grip} mm</dd>
      </dl>
    </div>`;
  document.getElementById("cards").innerHTML = cardOf(D.a,CA) + cardOf(D.b,CB);

  const S1=[
    {label:"A "+D.a.name, color:CA, x:D.curves.i6_a.x, y:D.curves.i6_a.y},
    {label:"B "+D.b.name, color:CB, x:D.curves.i6_b.x, y:D.curves.i6_b.y, dash:"6,4"}
  ];
  draw(document.getElementById("c1"), S1,
       {xLabel:"已转过角度 (°)", yLabel:"|i6|  (A)", xRange:[0,170]});
  document.getElementById("lg1").innerHTML = S1.map(s=>
    `<span><i style="border-top-color:${s.color};border-top-style:${s.dash?'dashed':'solid'}"></i>${s.label}</span>`).join("");

  const S2=[{label:"ΔI6 = B − A", color:"#e0a052", x:D.curves.d.x, y:D.curves.d.y}];
  draw(document.getElementById("c2"), S2,
       {xLabel:"已转过角度 (°)", yLabel:"Δ|i6|  (A)", xRange:[0,170], yDec:4});
  document.getElementById("lg2").innerHTML =
    `<span><i style="border-top-color:#e0a052"></i>ΔI6 = B − A</span>`;

  const S3=[
    {label:"A "+D.a.name, color:CA, x:D.curves.eff_a.x, y:D.curves.eff_a.y},
    {label:"B "+D.b.name, color:CB, x:D.curves.eff_b.x, y:D.curves.eff_b.y, dash:"6,4"}
  ];
  draw(document.getElementById("c3"), S3,
       {xLabel:"已转过角度 (°)", yLabel:"|effort|  (N·m)", xRange:[0,170], yDec:3,
        minSpan:0.08});
  document.getElementById("lg3").innerHTML = S3.map(s=>
    `<span><i style="border-top-color:${s.color};border-top-style:${s.dash?'dashed':'solid'}"></i>${s.label}</span>`).join("");

  drawHist(document.getElementById("c4"), D.hist, {a:CA,b:CB},
           {xLabel:"|i6|  (A)", yLabel:"帧数"});
  document.getElementById("lg4").innerHTML =
    `<span><i style="border-top-color:${CA};border-top-width:10px"></i>A ${D.a.name}</span>`+
    `<span><i style="border-top-color:${CB};border-top-width:10px"></i>B ${D.b.name}</span>`;

  const T=D.tser;
  const S5=[
    {label:"A", color:CA, x:T.t, y:T.a, w:1.1},
    {label:"B", color:CB, x:T.t, y:T.b, w:1.1, alpha:0.75}
  ];
  draw(document.getElementById("c5"), S5,
       {xLabel:"时间 (s)", yLabel:"|i6|  (A)", xDec:1});
  document.getElementById("lg5").innerHTML = S5.map(s=>
    `<span><i style="border-top-color:${s.color}"></i>${s.label} ${s.label==="A"?D.a.name:D.b.name}</span>`).join("");

  // 判别能力
  const dc=D.disc;
  const col=v=> v>=0.95?"#4ade80":(v>=0.8?"#e0a052":"#e05252");
  document.getElementById("disc").innerHTML =
    `<div class="kpi">
       <div class="k">帧级 AUC（逐帧判断）</div>
       <div class="v" style="color:${col(dc.auc_frame)}">${dc.auc_frame.toFixed(3)}</div>
       <div class="n">随机抽一帧，A 比 B 大的概率只有 ${(dc.auc_frame*100).toFixed(1)}%</div>
     </div>
     <div class="kpi">
       <div class="k">帧级最优阈值准确率</div>
       <div class="v" style="color:${col((dc.acc_frame/100-0.5)*2+0.5)}">${dc.acc_frame}%</div>
       <div class="n">阈值 ${dc.thr} A；丢硬币是 50%</div>
     </div>
     <div class="kpi">
       <div class="k">10° 段级 AUC（先平均再判断）</div>
       <div class="v" style="color:${col(dc.auc_seg)}">${dc.auc_seg.toFixed(3)}</div>
       <div class="n">各 ${dc.n_seg} 段；段均值 A ${dc.seg_a.toFixed(4)} / B ${dc.seg_b.toFixed(4)}</div>
     </div>`;

  // 表格
  const A=D.a, B=D.b;
  const rows=[
    ["帧数","frames",0,0],
    ["时长 (s)","dur",1,2],
    ["采样率 (Hz)","hz",1,1],
    ["|i6| 均值 (A)","i6_mean",4,4,"hl"],
    ["|i6| std (A)","i6_std",4,4],
    ["|i6| 最小 (A)","i6_min",4,4],
    ["|i6| 最大 (A)","i6_max",4,4],
    ["|i6| P10","p10",4,4],
    ["|i6| P50","p50",4,4],
    ["|i6| P90","p90",4,4],
    ["|i6| P99","p99",4,4],
    ["i6>0 的帧数","pos",0,0],
    ["空转占比 (&lt;0.20) %","idle",1,1],
    ["空转段数","runs",0,0],
    ["最长空转 (帧)","run_max",0,0],
    ["|effort| 均值 (N·m)","eff_mean",4,4,"hl"],
    ["|effort| std (N·m)","eff_std",4,4],
    ["|effort| 最小","eff_min",4,4],
    ["|effort| 最大","eff_max",4,4],
    ["夹爪开度 (mm)","grip",3,3],
    ["i1 均值 (A)","i1",3,3],
    ["i2 均值 (A)","i2",3,3],
    ["i3 均值 (A)","i3",3,3],
    ["i4 均值 (A)","i4",3,3],
    ["i5 均值 (A)","i5",3,3],
    ["i6 均值 (A)","i6",3,3],
    ["|dq6| 均值 (°/s)","dq6",2,2]
  ];
  let h="<thead><tr><th>量</th><th style='color:"+CA+"'>A · "+A.name+
        "</th><th style='color:"+CB+"'>B · "+B.name+"</th><th>差 (B−A)</th><th>相对 A</th></tr></thead><tbody>";
  for(const r of rows){
    const [label,key,dec,dec2,cls]=r;
    const va=A[key], vb=B[key];
    const dv=vb-va;
    const pct=Math.abs(va)>1e-9 ? (dv/va*100) : 0;
    const sig=Math.abs(dv)>Math.abs(va)*0.02;
    const c=sig?(dv>0?"pos":"neg"):"zero";
    h+=`<tr class="${cls||''}"><td>${label}</td>`+
       `<td>${va.toFixed(dec)}</td><td>${vb.toFixed(dec)}</td>`+
       `<td class="delta ${c}">${dv>0?"+":""}${dv.toFixed(dec2)}</td>`+
       `<td class="${c}">${Math.abs(va)>1e-9?(pct>0?"+":"")+pct.toFixed(1)+"%":"—"}</td></tr>`;
  }
  h+="</tbody>";
  document.getElementById("tbl").innerHTML=h;

  for(const id of ["c1","c2","c3","c5"]) bindTip(document.getElementById(id));
}

render();
let rt=null;
window.addEventListener("resize",()=>{clearTimeout(rt);rt=setTimeout(render,150);});
</script>
</body>
</html>
"""

if __name__ == "__main__":
    main()
