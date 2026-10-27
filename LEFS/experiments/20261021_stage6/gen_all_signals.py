#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_all_signals.py —— 把组1~5 全部 11 份全量记录的「所有独立信号」画成折线图。

出 22 个面板（每个信号一个），横轴统一为时间，11 份记录叠加：
  q1..q6      关节角 (deg)
  dq1..dq6    关节速度 (deg/s)
  i1..i6      电机电流 (A)
  qcmd6       J6 指令角 (deg)
  grip_ang_mm 夹爪开度 (mm)
  grip_eff_nm 夹爪 effort (N·m)
  tau6_ext    J6 外部力矩估计 (N·m)

不重复画的列（在页面底部有说明）：
  tau1_k..tau5_k = K_i * I_i，与 i1..i5 完全线性，画出来是同一形状
  i6_ma          = i6 的滑动平均
  bite_ratio / slip 是整段标量，不是时间序列
  t_s 是横轴本身

本机运行：python gen_all_signals.py
"""
import csv, json, os

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, 'data')
OUT = os.path.join(HERE, 'ALL_SIGNALS.html')

# (文件名, 组号, 组内轮次, 短名)
EXPS = [
    ("1.全量原盖未打开.csv",          1, 1, "组1 原盖"),
    ("2.全量预松未全开.csv",          2, 1, "组2 预松"),
    ("3.棘轮全量圆盖三圈开_轮1.csv",   3, 1, "组3 轮1"),
    ("3.棘轮全量圆盖三圈开_轮2.csv",   3, 2, "组3 轮2"),
    ("3.棘轮全量圆盖三圈开_轮3.csv",   3, 3, "组3 轮3"),
    ("4.棘轮全量预开全打开_轮1.csv",   4, 1, "组4 轮1"),
    ("4.棘轮全量预开全打开_轮2.csv",   4, 2, "组4 轮2"),
    ("4.棘轮全量预开全打开_轮3.csv",   4, 3, "组4 轮3"),
    ("5.全量预松1_未打开.csv",        5, 1, "组5 轮1"),
    ("5.全量预松2_打开.csv",          5, 2, "组5 轮2"),
    ("5.全量预松3_已打开空转.csv",     5, 3, "组5 轮3"),
]

GROUP_COLOR = {1: "#ff6b6b", 2: "#ffa94d", 3: "#4dabf7", 4: "#51cf66", 5: "#cc5de8"}
BACKUP = {1: "#cc5de8", 2: "#ffd43b", 3: "#ffa94d", 4: "#38d9a9", 5: "#ff8787"}
DASH = {1: None, 2: [6, 4], 3: [2, 3]}

SIGNALS = ([f'q{i}' for i in range(1, 7)]
           + [f'dq{i}' for i in range(1, 7)]
           + [f'i{i}' for i in range(1, 7)]
           + ['qcmd6', 'grip_ang_mm', 'grip_eff_nm', 'tau6_ext'])

PANELS = [
    ('q', '关节角 q1~q6', 'deg', [f'q{i}' for i in range(1, 7)],
     ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']),
    ('dq', '关节速度 dq1~dq6', 'deg/s', [f'dq{i}' for i in range(1, 7)],
     ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']),
    ('i', '电机电流 i1~i6', 'A', [f'i{i}' for i in range(1, 7)],
     ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']),
    ('x', 'J6 指令角 qcmd6', 'deg', ['qcmd6'], ['指令']),
    ('x', '夹爪开度 grip_ang_mm', 'mm', ['grip_ang_mm'], ['开度']),
    ('x', '夹爪 effort grip_eff_nm', 'N·m', ['grip_eff_nm'], ['effort']),
    ('x', 'J6 外部力矩 tau6_ext', 'N·m', ['tau6_ext'], ['tau6_ext']),
]

TARGET = 280  # 每条线降采样后的点数上限


def decimate(vals, target=TARGET):
    n = len(vals)
    if n <= target:
        return list(range(n)), vals
    step = n / target
    ix, vy = [], []
    for k in range(target):
        lo = int(k * step)
        ix.append(lo)
        vy.append(vals[lo])
    return ix, vy


def load(fn):
    with open(os.path.join(D, fn), newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


exps = []
tmax = 0.0
for fn, grp, rnd, short in EXPS:
    rows = load(fn)
    t0 = float(rows[0]['t_s'])
    ts = [round(float(r['t_s']) - t0, 3) for r in rows]
    tmax = max(tmax, ts[-1])
    idx, _ = decimate(ts)
    obj = dict(name=short, grp=grp, rnd=rnd,
               color=GROUP_COLOR[grp], dash=DASH[rnd],
               t=[ts[i] for i in idx], s={})
    for sig in SIGNALS:
        vals = [round(float(r[sig]), 3) for r in rows]
        _, vy = decimate(vals)
        obj['s'][sig] = vy
    exps.append(obj)

# 面板数据：每个面板可能有多个子信号（q1~q6 一个面板画 6 条，每条对应不同关节）
# 为控制复杂度，这里改成「每个子信号一个面板」
panels_json = []
for grp_key, gname, unit, sigs, labels in PANELS:
    for sig, lb in zip(sigs, labels):
        panels_json.append(dict(grp=grp_key, sig=sig,
                                title=f'{sig}　{lb}' if grp_key != 'x' else f'{gname}',
                                unit=unit))

data = dict(tmax=round(tmax, 2), exps=exps, panels=panels_json)

html = '''<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>组1~5 全信号折线图</title>
<style>
:root{--bg:#0b0e14;--panel:#121722;--line:#1e2733;--txt:#e6ebf2;--mut:#8b95a5}
*{box-sizing:border-box}
body{margin:0;padding:22px 28px 80px;background:var(--bg);color:var(--txt);
     font:13px/1.6 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
h1{font-size:20px;margin:0 0 5px;font-weight:500}
.lead{color:var(--mut);margin:0 0 16px;max-width:920px}
.bar{position:sticky;top:0;z-index:9;background:var(--bg);padding:10px 0 12px;
     border-bottom:1px solid var(--line);margin-bottom:16px}
.chips{display:flex;flex-wrap:wrap;gap:7px;margin-bottom:9px}
.chip{display:flex;align-items:center;gap:6px;padding:4px 10px;border-radius:14px;
      border:1px solid var(--line);cursor:pointer;font-size:12px;user-select:none;
      background:var(--panel);transition:opacity .12s}
.chip.off{opacity:.32}
.chip i{width:14px;height:3px;border-radius:2px;display:block;flex:none}
.chip small{color:var(--mut);font-size:11px}
nav{display:flex;flex-wrap:wrap;gap:8px}
nav a{color:var(--mut);text-decoration:none;font-size:12px;padding:3px 9px;
      border-radius:5px;background:var(--panel);border:1px solid var(--line)}
nav a:hover{color:var(--txt)}
h2{font-size:15px;font-weight:500;margin:30px 0 10px;padding-left:9px;
   border-left:3px solid #4dabf7}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(520px,1fr));gap:10px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;
       padding:10px 12px 6px}
.ptitle{font-size:12.5px;color:var(--txt);margin-bottom:2px}
.ptitle span{color:var(--mut);font-size:11.5px}
canvas{display:block;width:100%;height:148px}
.foot{margin-top:34px;padding:15px 18px;background:#12181f;border-left:3px solid #ffa94d;
      border-radius:0 7px 7px 0;color:var(--mut);font-size:12.5px;line-height:1.75}
.foot b{color:var(--txt);font-weight:500}
</style></head>
<body>
<h1>组 1~5 全部 11 份记录的信号总图</h1>
<p class="lead">每个信号一个面板，11 份记录全部叠加。组内第 1/2/3 轮用实线 / 长虚线 / 点线区分。
点图例可单独隐藏某一份。横轴统一为本次记录开始的相对时间。</p>

<div class="bar">
  <div class="chips" id="chips"></div>
  <nav>
    <a href="#g-q">关节角</a><a href="#g-dq">速度</a><a href="#g-i">电流</a>
    <a href="#g-x">夹爪 / 指令 / 力矩</a>
    <a href="#" id="allon">全部显示</a>
  </nav>
</div>

<div id="body"></div>

<div class="foot">
  <b>32 列里为什么只画了 22 个</b><br>
  · <b>tau1_k ~ tau5_k</b> = K_i × I_i（K 是常数），与 i1~i5 完全线性，形状一模一样，画出来是重复的。tau6_k 同理。<br>
  · <b>i6_ma</b> 是 i6 的滑动平均，已含在 i6 里。<br>
  · <b>bite_ratio / slip</b> 是整段算出来的一个标量，不是时间序列，没有折线可画。<br>
  · <b>t_s</b> 就是横轴本身。<br>
  · 因此 11 份 × 32 列里，真正随时间变化的独立信号就是这 22 个。<br><br>
  <b>采样</b>：原始每份 2153~2304 帧（~171 Hz），图上按等间隔抽稀到约 280 点，只看趋势够了；
  要看逐帧细节用 <b>q.py &lt;tag&gt;</b>。
</div>

<script>
const DATA = __DATA__;
const TMAX = DATA.tmax;
let on = DATA.exps.map(() => true);

function build() {
  const body = document.getElementById('body');
  let cur = null, sec = null, grid = null;
  const titles = {q:'关节角 q1~q6', dq:'关节速度 dq1~dq6', i:'电机电流 i1~i6',
                  x:'夹爪 / 指令 / 外力矩'};
  DATA.panels.forEach(function(p) {
    if (p.grp !== cur) {
      cur = p.grp;
      sec = document.createElement('section');
      sec.id = 'g-' + p.grp;
      const h = document.createElement('h2');
      h.textContent = titles[p.grp];
      sec.appendChild(h);
      grid = document.createElement('div');
      grid.className = 'grid';
      sec.appendChild(grid);
      body.appendChild(sec);
    }
    const d = document.createElement('div');
    d.className = 'panel';
    d.innerHTML = '<div class="ptitle">' + p.title +
      ' <span>(' + p.unit + ')</span></div>' +
      '<canvas data-sig="' + p.sig + '"></canvas>';
    grid.appendChild(d);
  });
}

function yrange(sig) {
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < DATA.exps.length; i++) {
    if (!on[i]) continue;
    const v = DATA.exps[i].s[sig];
    for (let k = 0; k < v.length; k++) {
      if (v[k] < lo) lo = v[k];
      if (v[k] > hi) hi = v[k];
    }
  }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (hi - lo < 1e-6) { hi = lo + 1; }
  const pad = (hi - lo) * 0.10;
  return [lo - pad, hi + pad];
}

function drawOne(cv, sig) {
  const dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  cv.width = W * dpr; cv.height = H * dpr;
  const c = cv.getContext('2d');
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, W, H);
  const r = yrange(sig);
  const lo = r[0], hi = r[1];
  const span = hi - lo;
  const PL = span < 0.5 ? 78 : (span < 8 ? 68 : 60), PR = 12, PT = 8, PB = 20;
  const x0 = PL, x1 = W - PR, y0 = H - PB, y1 = PT;

  function fmt(v, sp) {
    let d = 0;
    if (sp < 0.02) d = 4;
    else if (sp < 0.2) d = 3;
    else if (sp < 2) d = 2;
    else if (sp < 20) d = 1;
    return v.toFixed(d);
  }

  c.strokeStyle = '#ffffff12'; c.lineWidth = 1;
  c.fillStyle = '#8b95a5'; c.font = '10.5px -apple-system,sans-serif';
  c.textAlign = 'right'; c.textBaseline = 'middle';
  for (let g = 0; g <= 4; g++) {
    const y = y1 + (y0 - y1) * g / 4;
    c.beginPath(); c.moveTo(x0, y); c.lineTo(x1, y); c.stroke();
    c.fillText(fmt(hi - span * g / 4, span), x0 - 7, y);
  }
  c.textAlign = 'center'; c.textBaseline = 'top';
  for (let tt = 0; tt <= TMAX + 0.01; tt += 2) {
    const x = x0 + (x1 - x0) * tt / TMAX;
    c.fillText(tt.toFixed(0), x, y0 + 5);
  }
  c.strokeStyle = '#ffffff22'; c.beginPath();
  c.moveTo(x0, y0); c.lineTo(x1, y0); c.stroke();

  for (let i = 0; i < DATA.exps.length; i++) {
    if (!on[i]) continue;
    const e = DATA.exps[i], ts = e.t, vs = e.s[sig];
    c.strokeStyle = e.color;
    c.globalAlpha = 0.9;
    c.lineWidth = 1.5;
    c.setLineDash(e.dash || []);
    c.beginPath();
    for (let k = 0; k < vs.length; k++) {
      const x = x0 + (x1 - x0) * ts[k] / TMAX;
      const y = y0 - (y0 - y1) * (vs[k] - lo) / (hi - lo);
      if (k === 0) c.moveTo(x, y); else c.lineTo(x, y);
    }
    c.stroke();
  }
  c.setLineDash([]); c.globalAlpha = 1;
}

function drawAll() {
  const list = document.querySelectorAll('canvas');
  for (let i = 0; i < list.length; i++) {
    drawOne(list[i], list[i].getAttribute('data-sig'));
  }
}

function buildChips() {
  const box = document.getElementById('chips');
  DATA.exps.forEach(function(e, i) {
    const s = document.createElement('div');
    s.className = 'chip';
    const dash = e.dash ? 'background-image:linear-gradient(90deg,' + e.color +
      ' 0 55%,transparent 55%);background-size:6px 3px;' : '';
    s.innerHTML = '<i style="background:' + e.color + ';' + dash + '"></i>' + e.name;
    s.onclick = function() {
      on[i] = !on[i];
      s.classList.toggle('off', !on[i]);
      drawAll();
    };
    box.appendChild(s);
  });
}

build();
buildChips();
drawAll();
let rt = null;
window.addEventListener('resize', function() {
  clearTimeout(rt); rt = setTimeout(drawAll, 150);
});
document.getElementById('allon').onclick = function(ev) {
  ev.preventDefault();
  for (let i = 0; i < on.length; i++) on[i] = true;
  const cs = document.querySelectorAll('.chip');
  for (let i = 0; i < cs.length; i++) cs[i].classList.remove('off');
  drawAll();
};
</script>
</body></html>'''

html = html.replace('__DATA__', json.dumps(data, ensure_ascii=False, separators=(',', ':')))
with open(OUT, 'w', encoding='utf-8') as f:
    f.write(html)
print(f'[out] {OUT}  {os.path.getsize(OUT)/1024:.0f} KB')
print(f'  记录 {len(exps)} 份，面板 {len(panels_json)} 个，每条线 ~{TARGET} 点')
