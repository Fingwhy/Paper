#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_curves_html.py —— 把 4 组拧盖实验的电流/effort 画成折线图（单文件 HTML）

四组实验（共 8 份记录）：
    组1  1.全量原盖未打开          1 份  结果：未打开
    组2  2.全量预松未全开          1 份  结果：未全开
    组3  3.棘轮全量圆盖三圈开_轮1~3  3 份  结果：三圈后打开
    组4  4.棘轮全量预开全打开_轮1~3  3 份  结果：全打开

口径
----
x 轴用「已转过角度」= |q6 − 起始角|（0→170°），不是时间。
按角度分箱（默认 1°/箱）取均值，因为粘滑让逐帧值抖动 ±0.3 A，
逐帧画出来是一团毛刺，看不出趋势。

用法
----
    python make_curves_html.py            # 生成 TWIST_CURVES.html
    python make_curves_html.py --step 2   # 每 2° 一箱（点更少）
"""
import argparse
import csv
import glob
import json
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")

# (组号, 组名, 结果, 文件 glob, 颜色)
GROUPS = [
    (1, "原盖未打开", "未打开", "1.全量原盖未打开.csv", "#e05252"),
    (2, "预松未全开", "未全开", "2.全量预松未全开.csv", "#e0a052"),
    (3, "棘轮·三圈开", "打开", "3.棘轮全量圆盖三圈开_轮*.csv", "#4a9eff"),
    (4, "棘轮·预开全打开", "全打开", "4.棘轮全量预开全打开_轮*.csv", "#4ade80"),
    (5, "预松(新位姿)", "轮2打开", "5.全量预松*.csv", "#c084fc"),
]
DASHES = ["", "6,4", "2,4"]        # 组内第 1/2/3 轮的线型


def load(path):
    with open(path, encoding="utf-8") as f:
        rd = list(csv.reader(f))
    hdr, rows = rd[0], rd[1:]
    ix = {h: i for i, h in enumerate(hdr)}
    cols = ["t_s", "q6", "dq6", "grip_eff_nm"] + [f"i{k}" for k in range(1, 7)]
    for c in cols:
        if c not in ix:
            raise SystemExit(f"[FAIL] {path} 缺列 {c}")
    return {c: [float(r[ix[c]]) for r in rows] for c in cols}


def bin_by_angle(q6, series, step=1.0, span=(0.0, 170.0)):
    """按「已转过角度」分箱取均值。

    已转角度 = |q6 − q6[0]|。q6 从 170° 递减到 0°，所以转过角度是 0→170。
    """
    q0 = q6[0]
    lo, hi = span
    n = int(round((hi - lo) / step))
    buckets = [[] for _ in range(n + 1)]
    for j, v in enumerate(q6):
        a = abs(v - q0)
        k = int((a - lo) / step)
        if 0 <= k <= n:
            buckets[k].append(series[j])
    xs, ys = [], []
    for k, b in enumerate(buckets):
        if len(b) < 3:                 # 样本太少（首尾）不入图
            continue
        xs.append(round(lo + (k + 0.5) * step, 2))
        ys.append(round(st.mean(b), 5))
    return xs, ys


def summarize(d, name, group, result):
    i6 = [abs(v) for v in d["i6"]]
    eff = [abs(v) for v in d["grip_eff_nm"]]
    t = d["t_s"]
    n = len(t)
    dur = t[-1] - t[0]
    return {
        "name": name, "group": group, "result": result,
        "frames": n, "dur": round(dur, 2), "hz": round(n / dur, 1),
        "i6_mean": round(st.mean(i6), 4),
        "i6_std": round(st.pstdev(i6), 4),
        "i6_max": round(max(i6), 4),
        "i6_min": round(min(i6), 4),
        "eff_mean": round(st.mean(eff), 4),
        "eff_std": round(st.pstdev(eff), 4),
        "eff_min": round(min(eff), 4),
        "eff_max": round(max(eff), 4),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=1.0, help="分箱步长（度）")
    ap.add_argument("--out", default=os.path.join(HERE, "TWIST_CURVES.html"))
    a = ap.parse_args()

    lines = []      # 每条线：{label, group, color, dash, x, y}
    summary = []

    for gid, gname, result, pat, color in GROUPS:
        files = sorted(glob.glob(os.path.join(DATA, pat)))
        if not files:
            print(f"[warn] 组{gid} 没有匹配到文件：{pat}")
            continue
        for k, fp in enumerate(files):
            d = load(fp)
            base = os.path.basename(fp)[:-4]
            label = f"组{gid} {gname}" + (f"·轮{k+1}" if len(files) > 1 else "")
            x, y = bin_by_angle(d["q6"], [abs(v) for v in d["i6"]], a.step)
            lines.append({"label": label, "group": gid, "color": color,
                          "dash": DASHES[k % len(DASHES)], "x": x, "y": y,
                          "metric": "i6"})
            xe, ye = bin_by_angle(d["q6"], [abs(v) for v in d["grip_eff_nm"]], a.step)
            lines.append({"label": label, "group": gid, "color": color,
                          "dash": DASHES[k % len(DASHES)], "x": xe, "y": ye,
                          "metric": "eff"})
            # 其余轴（i2/i3/i5）也存一份，供第二张图
            for ax in ("i2", "i3", "i5"):
                xa, ya = bin_by_angle(d["q6"], [abs(v) for v in d[ax]], a.step)
                lines.append({"label": label, "group": gid, "color": color,
                              "dash": DASHES[k % len(DASHES)], "x": xa, "y": ya,
                              "metric": ax})
            summary.append(summarize(d, base, gid, result))
            print(f"  组{gid} {base}: {len(x)} 箱  I6 均值 {st.mean(abs(v) for v in d['i6']):.4f}")

    payload = {"lines": lines, "summary": summary, "step": a.step}
    js = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    html = TEMPLATE.replace("__DATA__", js)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"\n[out] {a.out}  {os.path.getsize(a.out)/1024:.1f} KB")
    print(f"      线 {len(lines)} 条，汇总 {len(summary)} 行")
    return a.out


TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>拧盖实验电流与夹爪力矩（4 组 / 8 份）</title>
<style>
  :root{
    --bg:#1e1e1e; --panel:#252526; --line:#3a3a3a; --txt:#d4d4d4;
    --dim:#8a8a8a; --accent:#4a9eff;
  }
  *{box-sizing:border-box}
  body{margin:0;padding:24px 28px 60px;background:var(--bg);color:var(--txt);
       font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;
       font-size:13px;line-height:1.6}
  h1{font-size:19px;font-weight:600;margin:0 0 4px}
  .sub{color:var(--dim);font-size:12px;margin-bottom:22px}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;
         padding:16px 18px 12px;margin-bottom:20px}
  .panel h2{font-size:14px;font-weight:600;margin:0 0 2px}
  .panel .hint{color:var(--dim);font-size:11.5px;margin-bottom:10px}
  canvas{display:block;width:100%;height:340px}
  .tall canvas{height:420px}
  .legend{display:flex;flex-wrap:wrap;gap:6px 18px;margin-top:10px;font-size:11.5px}
  .legend i{display:inline-block;width:22px;height:0;border-top-width:2.5px;
            border-top-style:solid;vertical-align:middle;margin-right:6px}
  .legend span{color:var(--dim)}
  table{border-collapse:collapse;width:100%;font-size:12px;margin-top:6px}
  th,td{padding:6px 9px;text-align:right;border-bottom:1px solid var(--line);
        white-space:nowrap}
  th{color:var(--dim);font-weight:500;text-align:right;font-size:11.5px}
  td:first-child,th:first-child{text-align:left}
  tr.g1 td:first-child{border-left:3px solid #e05252;padding-left:9px}
  tr.g2 td:first-child{border-left:3px solid #e0a052;padding-left:9px}
  tr.g3 td:first-child{border-left:3px solid #4a9eff;padding-left:9px}
  tr.g4 td:first-child{border-left:3px solid #4ade80;padding-left:9px}
  .tip{position:fixed;pointer-events:none;background:#111;border:1px solid #555;
       border-radius:5px;padding:6px 9px;font-size:11.5px;color:#eee;
       display:none;z-index:99;white-space:nowrap}
</style>
</head>
<body>
<h1>拧盖实验：关节电流与夹爪力矩</h1>
<div class="sub">
  x 轴 = <b>已转过角度</b>（|q6 − 起始角|，0→170°），不是时间；每 1° 分箱取均值。
  逐帧画会有 ±0.3 A 的粘滑毛刺，分箱后才看得见趋势。
</div>

<div class="panel">
  <h2>图 1 · J6 电流 |i6| 随转角</h2>
  <div class="hint">拧盖的主轴。看两件事：起点高低（阻力大小）、后段是否下降（是否咬住）</div>
  <canvas id="c1"></canvas>
  <div class="legend" id="lg1"></div>
</div>

<div class="panel">
  <h2>图 2 · 夹爪力矩 |effort| 随转角</h2>
  <div class="hint">反映夹持力大小，不是负载。跨组差异主要来自夹得多紧，与打滑无关</div>
  <canvas id="c2"></canvas>
  <div class="legend" id="lg2"></div>
</div>

<div class="panel tall">
  <h2>图 3 · 其他关节电流（J2 / J3 / J5）随转角</h2>
  <div class="hint">这三轴在拧盖时不动，其电流波动只反映整臂姿态与振动——用来判断"负载是否传到臂上"</div>
  <canvas id="c3"></canvas>
  <div class="legend" id="lg3"></div>
</div>

<div class="panel">
  <h2>图 4 · 汇总</h2>
  <div class="hint">I6 与 effort 均为绝对值（电机电流有符号，拧盖方向为负）</div>
  <table id="tbl"></table>
</div>

<div class="tip" id="tip"></div>

<script>
const D = __DATA__;
const COLOR = {1:"#e05252",2:"#e0a052",3:"#4a9eff",4:"#4ade80"};
const tip = document.getElementById("tip");

function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim();}

function draw(canvas, seriesList, opts){
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  canvas.width = W*dpr; canvas.height = H*dpr;
  const g = canvas.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,W,H);

  const pad = {l:58,r:16,t:14,b:38};
  const iw = W-pad.l-pad.r, ih = H-pad.t-pad.b;

  let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
  for(const s of seriesList){
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
  if(opts.xRange){xmin=opts.xRange[0];xmax=opts.xRange[1];}
  if(opts.yRange){ymin=opts.yRange[0];ymax=opts.yRange[1];}
  else{
    const padY=(ymax-ymin)*0.08||0.01;
    ymin=Math.max(0,ymin-padY); ymax=ymax+padY;
  }
  const X=v=>pad.l+(v-xmin)/(xmax-xmin)*iw;
  const Y=v=>pad.t+ih-(v-ymin)/(ymax-ymin)*ih;

  // 网格
  g.strokeStyle="#333"; g.lineWidth=1; g.font="11px -apple-system,Segoe UI,sans-serif";
  g.fillStyle="#8a8a8a";
  const nx=6;
  for(let i=0;i<=nx;i++){
    const v=xmin+(xmax-xmin)*i/nx, px=X(v);
    g.beginPath(); g.moveTo(px,pad.t); g.lineTo(px,pad.t+ih); g.stroke();
    g.textAlign="center"; g.fillText(v.toFixed(0), px, pad.t+ih+16);
  }
  const ny=5;
  for(let i=0;i<=ny;i++){
    const v=ymin+(ymax-ymin)*i/ny, py=Y(v);
    g.beginPath(); g.moveTo(pad.l,py); g.lineTo(pad.l+iw,py); g.stroke();
    g.textAlign="right"; g.fillText(opts.fmt?opts.fmt(v):v.toFixed(3), pad.l-8, py+4);
  }
  // 轴标题
  g.fillStyle="#8a8a8a"; g.textAlign="center";
  g.fillText(opts.xLabel||"", pad.l+iw/2, H-6);
  g.save(); g.translate(13,pad.t+ih/2); g.rotate(-Math.PI/2);
  g.fillText(opts.yLabel||"",0,0); g.restore();

  // 线
  for(const s of seriesList){
    g.beginPath();
    g.strokeStyle=s.color; g.lineWidth=1.8;
    g.setLineDash(s.dash? s.dash.split(",").map(Number):[]);
    g.globalAlpha = s.dim?0.28:1;
    let started=false;
    for(let j=0;j<s.x.length;j++){
      const px=X(s.x[j]), py=Y(s.y[j]);
      if(!started){g.moveTo(px,py);started=true;} else g.lineTo(px,py);
    }
    g.stroke();
    g.setLineDash([]); g.globalAlpha=1;
  }

  canvas._map={X,Y,pad,iw,ih,xmin,xmax,ymin,ymax,seriesList,opts};
}

function bindTip(canvas){
  canvas.addEventListener("mousemove", ev=>{
    const m=canvas._map; if(!m) return;
    const r=canvas.getBoundingClientRect();
    const mx=ev.clientX-r.left;
    if(mx<m.pad.l||mx>m.pad.l+m.iw){tip.style.display="none";return;}
    const xv=m.xmin+(mx-m.pad.l)/m.iw*(m.xmax-m.xmin);
    let rows=[];
    for(const s of m.seriesList){
      if(s.dim) continue;
      let best=-1,bd=1e9;
      for(let j=0;j<s.x.length;j++){
        const d=Math.abs(s.x[j]-xv);
        if(d<bd){bd=d;best=j;}
      }
      if(best>=0 && bd<=3) rows.push([s.label,s.color,s.y[best]]);
    }
    if(!rows.length){tip.style.display="none";return;}
    rows.sort((a,b)=>b[2]-a[2]);
    tip.innerHTML=`<b>已转 ${xv.toFixed(0)}°</b><br>`+
      rows.map(r=>`<span style="color:${r[1]}">■</span> ${r[0]}：${r[2].toFixed(4)}`).join("<br>");
    tip.style.display="block";
    tip.style.left=(ev.clientX+14)+"px";
    tip.style.top=(ev.clientY+10)+"px";
  });
  canvas.addEventListener("mouseleave",()=>{tip.style.display="none";});
}

function legend(el, seriesList){
  el.innerHTML = seriesList.map(s=>
    `<i style="border-top-color:${s.color};border-top-style:${s.dash?'dashed':'solid'}"></i>${s.label}`
  ).join("");
}

function pick(metric){
  return D.lines.filter(l=>l.metric===metric).map(l=>({
    label:l.label,color:l.color,dash:l.dash,x:l.x,y:l.y,dim:false
  }));
}

function render(){
  const s1 = pick("i6");
  const s2 = pick("eff");
  draw(document.getElementById("c1"), s1,
       {xLabel:"已转过角度 (°)", yLabel:"|i6|  (A)", xRange:[0,170]});
  legend(document.getElementById("lg1"), s1);

  draw(document.getElementById("c2"), s2,
       {xLabel:"已转过角度 (°)", yLabel:"|effort|  (N·m)", xRange:[0,170]});
  legend(document.getElementById("lg2"), s2);

  // 图3：只画组1 和组4轮3 作对照（避免 24 条线糊成一团）
  const keep = ["组1 ","组4 "];
  const s3 = [];
  for(const ax of ["i2","i3","i5"]){
    for(const l of D.lines.filter(l=>l.metric===ax)){
      if(!(l.label.startsWith("组1 ")||l.label==="组4 棘轮·预开全打开·轮3")) continue;
      s3.push({label:l.label.replace("棘轮·","")+"/"+ax.toUpperCase(),
               color:l.color,dash:ax==="i2"?"":(ax==="i3"?"6,4":"2,4"),
               x:l.x,y:l.y,dim:false});
    }
  }
  draw(document.getElementById("c3"), s3,
       {xLabel:"已转过角度 (°)", yLabel:"|i|  (A)", xRange:[0,170]});
  legend(document.getElementById("lg3"), s3);

  // 表格
  const th = ["文件","组","结果","帧","时长(s)","Hz","I6 均值","I6 std","I6 峰值","eff 均值","eff std"];
  let h = "<thead><tr>"+th.map(t=>`<th>${t}</th>`).join("")+"</tr></thead><tbody>";
  for(const r of D.summary){
    h += `<tr class="g${r.group}">`+
      `<td>${r.name}</td><td>${r.group}</td><td>${r.result}</td>`+
      `<td>${r.frames}</td><td>${r.dur}</td><td>${r.hz}</td>`+
      `<td>${r.i6_mean.toFixed(4)}</td><td>${r.i6_std.toFixed(4)}</td><td>${r.i6_max.toFixed(3)}</td>`+
      `<td>${r.eff_mean.toFixed(4)}</td><td>${r.eff_std.toFixed(4)}</td></tr>`;
  }
  h += "</tbody>";
  document.getElementById("tbl").innerHTML = h;

  for(const id of ["c1","c2","c3"]) bindTip(document.getElementById(id));
}

render();
let t=null;
window.addEventListener("resize",()=>{clearTimeout(t);t=setTimeout(render,150);});
</script>
</body>
</html>
"""

if __name__ == "__main__":
    main()
