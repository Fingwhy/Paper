#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""twist_diff.py —— 空载基线差分，得到纯负载力矩 tau_load(q6)

对应 FACTR 2 的  tau_ext = tau_m - tau_f：
    这里 tau_m = K6 * i6
    tau_f 用「同轨迹张开夹爪跑一遍」实测代替（论文用神经网络学）

核心思想
--------
单向 I6 的噪声 0.08 A（≈0.88 N·m）里，绝大部分是**共模慢漂**
（温漂 + 固件重力前馈残差 + 整臂振动）。同一 q6 姿态下做差分，
共模项相消，噪声降到 mN·m 量级（K6 标定时实测降 310 倍）。

用法
----
    python3 twist_diff.py --baseline 5.空载基线.csv --loaded 6.夹盖未开.csv

    # 换 K6（默认读 data/k6_result.json）
    python3 twist_diff.py -b A.csv -l B.csv --k6 0.3889

    # 存结果
    python3 twist_diff.py -b A.csv -l B.csv --save diff_out.csv

判据
----
    tau_load 随 q6 近似恒定  -> 打滑（动摩擦 mu_k*F*r，与转角无关）
    tau_load 随 q6 起伏/有台阶 -> 跟转（阻力来自螺纹+密封环几何）
"""
import argparse
import csv
import json
import os
import statistics as st
import sys

K6_DEFAULT = 0.3889        # N·m/A，20261021_stage6 k6_result.json
TAU_F6 = 0.86               # N·m，J6 减速箱摩擦
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


# ------------------------------------------------------------------ 读文件
def load_series(path, want=("q6", "i6", "dq6")):
    """返回 {列名: [float]}，兼容 twistlog_*（32 列）与旧 twist_*（12 列）。"""
    with open(path, encoding="utf-8") as f:
        rd = list(csv.reader(f))
    if len(rd) < 3:
        sys.exit("[FAIL] 文件太短: %s" % path)
    hdr, rows = rd[0], rd[1:]
    ix = {h: i for i, h in enumerate(hdr)}

    full = ("tau6_ext" in ix) or ("grip_ang_mm" in ix)
    if full:                                   # 全量格式，按表头名
        cols = {"q6": "q6", "i6": "i6", "dq6": "dq6"}
    else:                                      # 旧 12 列，按列号（表头对不齐）
        cols = {"q6": 2, "i6": 3, "dq6": None}
    out = {}
    for name, key in cols.items():
        if key is None:
            out[name] = None
            continue
        if isinstance(key, int):
            out[name] = [float(r[key]) for r in rows if len(r) > key]
        else:
            out[name] = [float(r[ix[key]]) for r in rows]
    if not out.get("q6") or not out.get("i6"):
        sys.exit("[FAIL] 读不到 q6/i6: %s" % path)
    if len(out["q6"]) != len(out["i6"]):
        n = min(len(out["q6"]), len(out["i6"]))
        out["q6"], out["i6"] = out["q6"][:n], out["i6"][:n]
    return out


def smooth_by_q6(d, win=15.0, step=0.5):
    """沿 q6 做滑动窗口平均。

    为什么必须沿 q6 平均：粘滑（stick-slip）是**时间上的随机过程**，
    同一个 q6 角度，两次运行可能一次在 stick（I6 高）、一次在 slip（I6 低），
    瞬时值差可达 0.3 A。只有把窗口拉到 ≥1 个粘滑周期（15°/s 下约 15°，
    即 170 帧 ≈ 1 s）才能把它平均掉。实测：窗口 0.5°→20°，std 从
    0.035 降到 0.014 A（降 2.6 倍）。

    返回 [q6], [i6 均值], [窗口内帧数]
    """
    import bisect
    q = d["q6"]
    i = [abs(v) for v in d["i6"]]
    order = sorted(range(len(q)), key=lambda j: q[j])
    qs = [q[j] for j in order]
    is_ = [i[j] for j in order]
    lo, hi = qs[0], qs[-1]
    n = int(round((hi - lo) / step)) + 1
    out_g, out_m, out_n = [], [], []
    half = win / 2.0
    for k in range(n):
        x = lo + k * step
        a = bisect.bisect_left(qs, x - half)
        b = bisect.bisect_right(qs, x + half)
        if b - a < 5:                     # 窗口内样本太少，跳过边缘
            continue
        out_g.append(x)
        out_m.append(st.mean(is_[a:b]))
        out_n.append(b - a)
    return out_g, out_m, out_n


def interp(grid, values, xs):
    """线性插值：把 (grid, values) 采到 xs 上；超出范围返回 None。"""
    out = []
    n = len(grid)
    for x in xs:
        if x < grid[0] - 1e-9 or x > grid[-1] + 1e-9:
            out.append(None)
            continue
        lo, hi = 0, n - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if grid[mid] <= x:
                lo = mid
            else:
                hi = mid
        if grid[hi] == grid[lo]:
            out.append(values[lo])
        else:
            w = (x - grid[lo]) / (grid[hi] - grid[lo])
            out.append(values[lo] + w * (values[hi] - values[lo]))
    return out


# ------------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser(
        description="空载基线差分：tau_load(q6) = (i6_loaded - i6_baseline) * K6")
    ap.add_argument("-b", "--baseline", required=True,
                    help="空载基线 csv（张开夹爪跑同一条 q6 轨迹）")
    ap.add_argument("-l", "--loaded", required=True, help="夹着瓶盖的 csv")
    ap.add_argument("--k6", type=float, default=None,
                    help="J6 力矩常数，默认从 data/k6_result.json 读")
    ap.add_argument("--win", type=float, default=15.0,
                    help="q6 窗口宽度（度）。粘滑周期约 1s，15°/s 下约 15°，"
                         "窗口小于它平均不掉粘滑。默认 15")
    ap.add_argument("--step", type=float, default=0.1, help="q6 网格步长（度）")
    ap.add_argument("--save", default=None, help="把对齐后的结果存 csv")
    a = ap.parse_args()

    # K6
    k6 = a.k6
    if k6 is None:
        p = os.path.join(DATA, "k6_result.json")
        if os.path.exists(p):
            try:
                k6 = float(json.load(open(p, encoding="utf-8"))["K6"])
            except Exception:                              # noqa: BLE001
                k6 = K6_DEFAULT
        else:
            k6 = K6_DEFAULT
    print("[cfg] K6 = %.4f N·m/A   tau_f6 = %.2f N·m   窗口 %.1f°  网格 %.2f°"
          % (k6, TAU_F6, a.win, a.step))

    db = load_series(a.baseline)
    dl = load_series(a.loaded)
    for tag, d in (("baseline", db), ("loaded", dl)):
        q = d["q6"]
        print("[in ] %-9s n=%5d  q6=%7.1f→%6.1f   |i6| 均值=%.4f"
              % (tag, len(q), q[0], q[-1], st.mean(abs(v) for v in d["i6"])))

    gb, mb, nb = smooth_by_q6(db, a.win, a.step)
    gl, ml, nl = smooth_by_q6(dl, a.win, a.step)
    if not gb or not gl:
        sys.exit("[FAIL] 平滑后没有有效点，检查 q6 范围是否重叠")

    lo = max(min(gb), min(gl))
    hi = min(max(gb), max(gl))
    if hi - lo < 5:
        sys.exit("[FAIL] 两段 q6 重叠范围只有 %.1f°，太小。" % (hi - lo))
    xs = [lo + k * a.step for k in range(int(round((hi - lo) / a.step)) + 1)]

    vb = interp(gb, mb, xs)
    vl = interp(gl, ml, xs)
    keep = [(x, b, v) for x, b, v in zip(xs, vb, vl) if b is not None and v is not None]
    if len(keep) < 20:
        sys.exit("[FAIL] 对齐后只剩 %d 点" % len(keep))

    xs2 = [k[0] for k in keep]
    bi = [k[1] for k in keep]
    li = [k[2] for k in keep]
    # 注：smooth_by_q6 里已统一取 |i6|（两次运行转向可能相反，符号不可比）
    di = [l - b for l, b in zip(li, bi)]
    tau = [d * k6 for d in di]

    print("\n[对齐] 重叠 q6 %.1f°→%.1f°，%d 点（步长 %.2f°）"
          % (xs2[0], xs2[-1], len(xs2), a.step))
    print("[基线] 空载 |i6|  %.4f ± %.4f A   （std 即单向噪声）"
          % (st.mean(bi), st.pstdev(bi)))
    print("[实测] 带载 |i6|  %.4f ± %.4f A" % (st.mean(li), st.pstdev(li)))
    print("[差分] ΔI        %.4f ± %.4f A" % (st.mean(di), st.pstdev(di)))
    print("[力矩] τ_load    %+.4f ± %.4f N·m   (峰峰 %.4f)"
          % (st.mean(tau), st.pstdev(tau), max(tau) - min(tau)))

    noise_1 = st.pstdev(li)
    noise_d = st.pstdev(di)
    print("\n[降噪] 单向 %.4f A → 差分 %.4f A   （降 %.0f 倍）"
          % (noise_1, noise_d, (noise_1 / noise_d) if noise_d > 1e-9 else float("inf")))

    # 分段：判打滑的核心
    print("\n[按转角分段] τ_load 的形状才是判据")
    print(f"{'q6 区间':>16}{'点数':>6}{'τ_load 均值':>13}{'τ_load std':>12}{'ΔI 均值':>10}")
    n = len(xs2)
    seg = max(1, n // 10)
    rows_out = []
    for s in range(0, n, seg):
        e = min(n, s + seg)
        if e - s < 3:
            continue
        ts = tau[s:e]
        print(f"{f'{xs2[s]:.1f}~{xs2[e-1]:.1f}':>16}{e-s:>6}"
              f"{st.mean(ts):>+13.4f}{st.pstdev(ts):>12.4f}{st.mean(di[s:e]):>+10.4f}")
        rows_out.append((xs2[s], xs2[e - 1], e - s, st.mean(ts), st.pstdev(ts), st.mean(di[s:e])))

    # 判读
    flat = st.pstdev(tau)
    print("\n[判读] τ_load 全程 std = %.4f N·m，峰峰 %.4f N·m"
          % (flat, max(tau) - min(tau)))
    print("       粗判：std < 0.05 → 像打滑（动摩擦恒定）；"
          "std > 0.10 → 像跟转（随几何起伏）")
    print("       ⚠ 这只是粗判，阈值要先积累几组已知结果才能定")

    # 与单点常数的区别
    t_const = [v * k6 - TAU_F6 for v in li]
    print("\n[对比] 现在的做法（只减常数 %.2f N·m）：std = %.4f N·m"
          % (TAU_F6, st.pstdev(t_const)))
    print("       空载差分后：                          std = %.4f N·m"
          % st.pstdev(tau))

    if a.save:
        with open(a.save, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["q6", "i6_baseline", "i6_loaded", "dI", "tau_load"])
            for x, b, l, d, t in zip(xs2, bi, li, di, tau):
                w.writerow(["%.3f" % x, "%.5f" % b, "%.5f" % l,
                            "%.5f" % d, "%.5f" % t])
        print("\n[out ] 已写 %s" % a.save)


if __name__ == "__main__":
    main()
