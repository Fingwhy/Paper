#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_compare_multi.py —— 任意 2~4 份拧盖记录的折线对比（单文件 HTML）

默认对比四份：
    1.全量原盖未打开.csv      （组1，红）
    5.全量预松1_未打开.csv    （组5轮1，紫实线）
    5.全量预松2_打开.csv      （组5轮2，紫虚线）
    5.全量预松3_已打开空转.csv（组5轮3，紫点划线）

图
--
1. |i6| 随转角
2. |effort| 随转角
3. |i6| 直方图
4. |i6| 时序片段

用法
----
    python gen_compare_multi.py
    python gen_compare_multi.py -f "1.全量原盖未打开.csv" -f "5.全量预松2_打开.csv" --out X.html
    python gen_compare_multi.py --timewin 900
"""
import argparse
import csv
import json
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")

# 默认三份：(文件名, 显示名, 颜色, 线型)
DEFAULT = [
    ("1.全量原盖未打开.csv", "组1 原盖未打开", "#e05252", ""),
    ("5.全量预松1_未打开.csv", "组5 预松·轮1(未打开)", "#c084fc", ""),
    ("5.全量预松2_打开.csv", "组5 预松·轮2(打开)", "#c084fc", "6,4"),
    ("5.全量预松3_已打开空转.csv", "组5 预松·轮3(已开空转)", "#c084fc", "2,4"),
]
PALETTE = ["#e05252", "#c084fc", "#22d3ee", "#e0a052"]


def load(path):
    with open(path, encoding="utf-8") as f:
        rd = list(csv.reader(f))
    hdr, rows = rd[0], rd[1:]
    ix = {h: i for i, h in enumerate(hdr)}
    cols = ["t_s", "q6", "grip_ang_mm", "grip_eff_nm"] + [f"i{k}" for k in range(1, 7)]
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
    return [round(lo + (k + 0.5) * step, 3) for k in range(n)], out


def block(d, name):
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
        "i6_mean": round(st.mean(i6), 4), "i6_std": round(st.pstdev(i6), 4),
        "i6_min": round(min(i6), 4), "i6_max": round(max(i6), 4),
        "p10": round(qtl(i6, 10), 4), "p50": round(qtl(i6, 50), 4),
        "p90": round(qtl(i6, 90), 4),
        "idle": round(100 * sum(1 for v in i6 if v < 0.20) / n, 1),
        "runs": len(runs), "run_max": max(runs) if runs else 0,
        "eff_mean": round(st.mean(eff), 4), "eff_std": round(st.pstdev(eff), 4),
        "eff_min": round(min(eff), 4), "eff_max": round(max(eff), 4),
        "grip": round(st.mean(d["grip_ang_mm"]), 2),
        "i2": round(st.mean(d["i2"]), 3), "i3": round(st.mean(d["i3"]), 3),
        "i5": round(st.mean(d["i5"]), 3),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--file", action="append", default=None,
                    help="可多次给出；不给则用默认三份")
    ap.add_argument("--out", default=os.path.join(HERE, "COMPARE_G1_G5R123.html"))
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--timewin", type=int, default=700, help="时序图前 N 帧")
    a = ap.parse_args()

    specs = []
    if a.file:
        # 同组同色：组号取文件名第一个 "." 之前的部分（"1"/"5"…），
        # 组内第 k 轮用实线/虚线/点线区分 —— 这样一眼能看出"哪条是同一次实验的不同轮"
        gcolor, gcount = {}, {}
        for fn in a.file:
            gid = os.path.basename(fn).split(".")[0] or "?"
            if gid not in gcolor:
                gcolor[gid] = PALETTE[len(gcolor) % len(PALETTE)]
                gcount[gid] = 0
            k = gcount[gid]
            specs.append((fn, os.path.basename(fn)[:-4], gcolor[gid],
                          ["", "6,4", "2,4"][k % 3]))
            gcount[gid] += 1
    else:
        specs = DEFAULT

    loaded = []
    for fn, label, color, dash in specs:
        p = os.path.join(DATA, fn)
        if not os.path.exists(p):
            raise SystemExit(f"[FAIL] 不存在 {p}")
        d = load(p)
        i6 = [abs(v) for v in d["i6"]]
        eff = [abs(v) for v in d["grip_eff_nm"]]
        x, y = bin_by_angle(d["q6"], i6, a.step)
        ex, ey = bin_by_angle(d["q6"], eff, a.step)
        hc, hv = hist(i6)
        loaded.append({
            "label": label, "color": color, "dash": dash,
            "stats": block(d, label),
            "i6": {"x": x, "y": y},
            "eff": {"x": ex, "y": ey},
            "hist": hv,
            "ts": {"t": [round(d["t_s"][j] - d["t_s"][0], 3) for j in range(min(len(i6), a.timewin))],
                   "y": [round(i6[j], 4) for j in range(min(len(i6), a.timewin))]},
        })

    payload = {"series": loaded, "histc": hc, "step": a.step}
    title = " vs ".join(s["label"] for s in loaded)
    html = (TEMPLATE
            .replace("__TITLE__", title)
            .replace("__DATA__", json.dumps(payload, ensure_ascii=False,
                                            separators=(",", ":"))))
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[out] {a.out}  {os.path.getsize(a.out)/1024:.1f} KB")
    for s in loaded:
        b = s["stats"]
        print(f"  {b['name']:<16} I6 {b['i6_mean']:.4f}  std {b['i6_std']:.4f}  "
              f"空转 {b['idle']:.0f}%  |eff| {b['eff_mean']:.4f}  eff_std {b['eff_std']:.4f}")
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
  .cards{display:grid;gap:14px;margin-bottom:20px}
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
  .legend{display:flex;flex-wrap:wrap;gap:8px 20px;margin-top:10px;font-size:11.5px}
  .legend i{display:inline-block;width:22px;border-top-width:2.5px;border-top-style:solid;
            vertical-align:middle;margin-right:6px}
  table{border-collapse:collapse;width:100%;font-size:12px;margin-top:4px}
  th,td{padding:6px 9px;border-bottom:1px solid var(--line);text-align:right;
        font-variant-numeric:tabular-nums;white-space:nowrap}
  th{color:var(--dim);font-weight:500;font-size:11.5px}
  td:first-child,th:first-child{text-align:left}
  .pos{color:#4ade80}.neg{color:#e05252}.zero{color:var(--dim)}
  .note{background:#2a2418;border-left:3px solid #e0a052;padding:12px 16px;
        border-radius:5px;font-size:12.5px;margin-bottom:20px}
  .note b{color:#e0a052}
  .tip{position:fixed;pointer-events:none;background:#111;border:1px solid #555;
       border-radius:5px;padding:6px 9px;font-size:11.5px;display:none;z-index:99;white-space:nowrap}
</style>
</head>
<body>
<h1>拧盖记录折线对比</h1>
<div class="sub" id="sub"></div>

<div class="note">
<b>读图要点</b>：<br>
① 组1 是<b>原盖</b>（未打开）；组5 是<b>预松过的同一颗盖</b>、<b>新基准位姿</b>、夹爪<b>重新标零</b>过。<br>
② 夹爪开度列<b>不可与组1 直接比</b>：标零后量程 69.51 mm（旧 69.58），且这次夹得更深。
</div>

<div class="cards" id="cards"></div>

<div class="panel">
  <h2>图 1 · |i6| 随转角</h2>
  <div class="hint">每 1° 分箱取均值。组5 轮2 整体低一档，且后段更低</div>
  <canvas id="c1"></canvas>
  <div class="legend" id="lg1"></div>
</div>

<div class="panel">
  <h2>图 2 · |effort| 随转角</h2>
  <div class="hint">y 轴跨度固定 ≥0.08 N·m。看波动幅度——组5 轮2 的抖动明显更大</div>
  <canvas id="c2"></canvas>
  <div class="legend" id="lg2"></div>
</div>

<div class="panel">
  <h2>图 3 · |i6| 分布直方图</h2>
  <div class="hint">档宽 0.02 A。三组大面积重叠，靠单帧电流分不开</div>
  <canvas id="c3"></canvas>
  <div class="legend" id="lg3"></div>
</div>

<div class="panel">
  <h2>图 4 · |i6| 时序片段（开头约 4 秒）</h2>
  <div class="hint">逐帧原值。能看到每秒一次的"顶住→滑脱"锯齿</div>
  <canvas id="c4"></canvas>
  <div class="legend" id="lg4"></div>
</div>

<div class="panel">
  <h2>逐项统计</h2>
  <div class="hint">差值以第一份（组1）为基准；超过基准 2% 才着色</div>
  <table id="tbl"></table>
</div>

<div class="tip" id="tip"></div>

<script>
const D = __DATA__;
const tip = document.getElementById("tip");

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
  else if(opts.xMin!==undefined){}
  if(opts.yRange){ ymin=opts.yRange[0]; ymax=opts.yRange[1]; }
  else{
    let sp=ymax-ymin;
    if(!isFinite(sp)||sp<=0) sp=0.01;
    if(opts.minSpan && sp<opts.minSpan){
      const mid=(ymin+ymax)/2;
      ymin=mid-opts.minSpan/2; ymax=mid+opts.minSpan/2;
    }else{
      ymin=ymin-sp*0.1; ymax=ymax+sp*0.1;
    }
  }
  if(opts.yFloor===true && ymin<0) ymin=0;

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

function drawHist(canvas, hc, series, colors){
  const dpr=window.devicePixelRatio||1;
  const W=canvas.clientWidth,H=canvas.clientHeight;
  canvas.width=W*dpr; canvas.height=H*dpr;
  const g=canvas.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const pad={l:52,r:16,t:14,b:36};
  const iw=W-pad.l-pad.r, ih=H-pad.t-pad.b;
  const n=hc.length;
  let mx=0;
  for(const s of series) for(let k=0;k<n;k++) if(s.hist[k]>mx) mx=s.hist[k];
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
    g.fillText(hc[k].toFixed(1), px, pad.t+ih+15);
  }
  g.fillText("|i6|  (A)", pad.l+iw/2, H-5);
  g.save(); g.translate(12,pad.t+ih/2); g.rotate(-Math.PI/2);
  g.fillText("帧数",0,0); g.restore();

  for(const s of series){
    g.fillStyle=s.color+"77";
    for(let k=0;k<n;k++) g.fillRect(pad.l+k*bw, Y(s.hist[k]), bw-0.8, ih-Y(s.hist[k])+pad.t);
  }
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

function legend(el, series){
  el.innerHTML = series.map(s=>
    `<span><i style="border-top-color:${s.color};border-top-style:${s.dash?'dashed':'solid'}"></i>${s.label}</span>`
  ).join("");
}

function render(){
  const S = D.series;
  document.getElementById("sub").innerHTML =
    S.map(s=>`<span style="color:${s.color}">${s.label}</span>`).join(" ｜ ") +
    ` ｜ 分箱 ${D.step}°`;

  document.getElementById("cards").style.gridTemplateColumns =
    `repeat(${S.length}, minmax(0,1fr))`;
  document.getElementById("cards").innerHTML = S.map(s=>{
    const b=s.stats;
    return `<div class="card">
      <h3><span class="dot" style="background:${s.color}"></span>${b.name}</h3>
      <dl>
        <dt>帧 / 时长</dt><dd>${b.frames} / ${b.dur}s (${b.hz} Hz)</dd>
        <dt>|i6| 均值 ± std</dt><dd>${b.i6_mean.toFixed(4)} ± ${b.i6_std.toFixed(4)}</dd>
        <dt>|i6| P10 / P50 / P90</dt><dd>${b.p10.toFixed(3)} / ${b.p50.toFixed(3)} / ${b.p90.toFixed(3)}</dd>
        <dt>空转占比</dt><dd>${b.idle}%（最长 ${b.run_max} 帧）</dd>
        <dt>|effort| 均值 ± std</dt><dd>${b.eff_mean.toFixed(4)} ± <b>${b.eff_std.toFixed(4)}</b></dd>
        <dt>夹爪开度</dt><dd>${b.grip} mm</dd>
      </dl></div>`;
  }).join("");

  const s1 = S.map(s=>({label:s.label,color:s.color,dash:s.dash,x:s.i6.x,y:s.i6.y}));
  draw(document.getElementById("c1"), s1,
       {xLabel:"已转过角度 (°)", yLabel:"|i6|  (A)", xRange:[0,170], yFloor:true});
  legend(document.getElementById("lg1"), s1);

  const s2 = S.map(s=>({label:s.label,color:s.color,dash:s.dash,x:s.eff.x,y:s.eff.y}));
  draw(document.getElementById("c2"), s2,
       {xLabel:"已转过角度 (°)", yLabel:"|effort|  (N·m)", xRange:[0,170], yDec:3, minSpan:0.10});
  legend(document.getElementById("lg2"), s2);

  drawHist(document.getElementById("c3"), D.histc, S, S.map(s=>s.color));
  document.getElementById("lg3").innerHTML = S.map(s=>
    `<span><i style="border-top-color:${s.color};border-top-width:10px"></i>${s.label}</span>`).join("");

  const s4 = S.map(s=>({label:s.label,color:s.color,dash:s.dash,
                        x:s.ts.t,y:s.ts.y,w:1.05,alpha:0.85}));
  draw(document.getElementById("c4"), s4,
       {xLabel:"时间 (s)", yLabel:"|i6|  (A)", xDec:1, yFloor:true});
  legend(document.getElementById("lg4"), s4);

  // 表格：第一份为基准
  const base = S[0].stats;
  const rows=[
    ["帧数","frames",0],["时长 (s)","dur",2],["采样率 (Hz)","hz",1],
    ["|i6| 均值 (A)","i6_mean",4],["|i6| std (A)","i6_std",4],
    ["|i6| 最小","i6_min",4],["|i6| 最大","i6_max",4],
    ["|i6| P10","p10",3],["|i6| P50","p50",3],["|i6| P90","p90",3],
    ["空转占比 %","idle",1],["空转段数","runs",0],["最长空转 (帧)","run_max",0],
    ["|effort| 均值","eff_mean",4],["|effort| std","eff_std",4],
    ["|effort| 最小","eff_min",4],["|effort| 最大","eff_max",4],
    ["夹爪开度 (mm)","grip",2],
    ["i2 均值 (A)","i2",3],["i3 均值 (A)","i3",3],["i5 均值 (A)","i5",3],
  ];
  let h = "<thead><tr><th>量</th>";
  for(const s of S) h += `<th style="color:${s.color}">${s.label}</th>`;
  for(let k=1;k<S.length;k++) h += `<th>Δ ${S[k].label} − ${S[0].label}</th>`;
  h += "</tr></thead><tbody>";

  for(const [label,key,dec] of rows){
    h += `<tr><td>${label}</td>`;
    for(const s of S) h += `<td>${s.stats[key].toFixed(dec)}</td>`;
    const v0 = base[key];
    for(let k=1;k<S.length;k++){
      const dv = S[k].stats[key]-v0;
      const pct = Math.abs(v0)>1e-9 ? dv/v0*100 : 0;
      const sig = Math.abs(pct)>2;
      const c = sig ? (dv>0?"pos":"neg") : "zero";
      h += `<td class="${c}">${dv>0?"+":""}${dv.toFixed(dec)}</td>`;
    }
    h += "</tr>";
  }
  h += "</tbody>";
  document.getElementById("tbl").innerHTML = h;

  for(const id of ["c1","c2","c4"]) bindTip(document.getElementById(id));
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
