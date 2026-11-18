#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拧盖实验数据速查工具（S6）

一句话：忘了第几次是哪个文件时，直接 `python3 q.py list`，再 `python3 q.py 3` 看电流。

用法
----
  python3 q.py list                 全部实验编号表（按采集时间排序）
  python3 q.py 3                    第 3 次：六轴电流统计 + I6 分段
  python3 q.py z177                 按 tag 模糊匹配
  python3 q.py 3 --by-angle         按"已转过角度"分段（拧盖分析的正确口径）
  python3 q.py 3 -n 12              分 12 段（默认 8）
  python3 q.py 3 --raw 20           打印原始前 20 行
  python3 q.py 3 --save seg.csv     把分段结果存成 csv
  python3 q.py list --dir ~/stage0/20261021_stage6/data

说明
----
- 编号 = 按文件修改时间（mtime）升序，即"第几次采集"。VM 上就是原始采集顺序。
- CSV 列固定 12 列（见 COLS）。第 3 列是 i6，第 4~8 列是 i1~i5，第 9 列是重复的 i6。
- 电流单位 A，0.001 A/bit。
- 本工具只输出客观数据，不判断"拧开与否"（那需要现场目视）。
"""
import argparse
import csv
import os
import statistics as st
import sys

COLS = ["t_s", "cmd_q6", "meas_q6", "i6", "i1", "i2", "i3", "i4", "i5",
        "i6_dup", "grip_angle", "grip_effort"]
DEFAULT_DIR = os.path.expanduser("~/stage0/20261021_stage6/data")
K6 = 0.3889          # N·m/A，20261021_stage6 标定结果
TAU_F6 = 0.86         # N·m，J6 减速箱摩擦

# 现场目视记录的"拧开与否"。只填确定的，其余留 "-"。
RESULTS = {
    "r1": "开(人工预松)", "r2": "开", "full1": "未开", "full2": "未开",
    "full3": "未开", "full4": "未开", "full5": "未开", "r3": "转未开",
    "cyc1": "作废", "cyc2": "作废", "cyc3": "作废", "sq2": "未开",
    "fast1": "开", "fast2": "污染", "fast9": "前滑后动", "fast12": "开",
    "fast12b": "未开", "fast15": "未开", "slow2": "未开", "fast15b": "开",
    "sq03_15": "未开", "sq1_15c": "未开", "dn2mm": "开",
    "dn2mm_b": "后段开", "rc1": "未开", "rc2": "未开", "rc3": "动",
    "sq15mm": "未开", "z177": "开",
}


def result_of(tag):
    """结果优先查 RESULTS；查不到就从文件名推断（推荐命名 `1.全量原盖未打开`）。"""
    if tag in RESULTS:
        return RESULTS[tag]
    # 顺序要紧：先判更具体的否定式，否则 "未全开" 会被 "打开" 规则吃掉
    if "未全开" in tag:
        return "未全开"
    if "未打开" in tag:
        return "未打开"
    if "打开" in tag or ("开" in tag and "未" not in tag):
        return "打开"
    return "-"


# ---------------------------------------------------------------- 基础
def find_files(d):
    """返回按 mtime 升序的 csv 路径列表。

    收录规则：`twist_*` / `twistlog_*`，以及**数字开头**的文件
    （如 `1.全量原盖未打开.csv` —— 文件名自带序号和结果，推荐这种命名）。
    `k6sweep_*.csv` / `s6_pose*.csv` 不会被误收。
    """
    if not os.path.isdir(d):
        sys.exit(f"[FAIL] 目录不存在: {d}\n   VM 上是 ~/stage0/20261021_stage6/data，"
                 f"用 --dir 指定")
    fs = []
    for f in os.listdir(d):
        if not f.endswith(".csv"):
            continue
        if f.startswith(("twist_", "twistlog_")) or f[0].isdigit():
            fs.append(os.path.join(d, f))
    fs.sort(key=lambda p: (os.path.getmtime(p), p))
    return fs


def load(path):
    """统一成同一套键：t_s / meas_q6 / i1..i6 / grip_angle(0.001mm) / grip_effort(0.001N·m)。
    twistlog 额外有 q1..q6 / dq6 / tau6_ext / bite_ratio / slip。"""
    with open(path, encoding="utf-8") as f:
        rd = list(csv.reader(f))
    if len(rd) < 2:
        return None
    hdr, rows = rd[0], rd[1:]
    # 按**表头**判断格式，不依赖文件名（文件可能已被改名成 "1.全量..." 这种）
    if "tau6_ext" in hdr and "grip_ang_mm" in hdr:
        kind = "full"
        ix = {h: i for i, h in enumerate(hdr)}
        need = ["t_s", "qcmd6", "i1", "i2", "i3", "i4", "i5", "i6",
                "grip_ang_mm", "grip_eff_nm", "tau6_ext", "bite_ratio", "slip", "dq6"]
        if any(k not in ix for k in need):
            return None
        out = {k: [] for k in need}
        out.update({f"q{i}": [] for i in range(1, 7)})
        for r in rows:
            if len(r) < len(hdr):
                continue
            try:
                tmp = {k: float(r[ix[k]]) for k in need}
                tmq = {f"q{i}": float(r[ix[f"q{i}"]]) for i in range(1, 7)}
            except (ValueError, KeyError):
                continue
            # 全部解析成功才一起追加，避免数组长度错位
            for k, v in tmp.items():
                out[k].append(v)
            for k, v in tmq.items():
                out[k].append(v)
        # 统一口径：mm → 0.001mm，N·m → 0.001N·m
        out["grip_angle"] = [v * 1000.0 for v in out.pop("grip_ang_mm")]
        out["grip_effort"] = [v * 1000.0 for v in out.pop("grip_eff_nm")]
        out["meas_q6"] = out["q6"]
        out["cmd_q6"] = out["qcmd6"]
        out["kind"] = kind
        return out if out["t_s"] else None

    # 旧格式（12 列 / 11 列）：按列号读（11 列文件缺 i6_dup，表头与数据对不齐，不能按名读）
    kind = "legacy"
    ncol = len(rows[0])
    out = {c: [] for c in COLS}
    for r in rows:
        if len(r) < 11:
            continue
        try:
            for k, c in enumerate(COLS):
                if k < len(r) and k < ncol:
                    out[c].append(float(r[k]))
        except ValueError:
            continue
    out["q6"] = out["meas_q6"]
    out["kind"] = kind
    return out if out["t_s"] else None


def tag_of(path):
    b = os.path.basename(path)
    for pre in ("twistlog_", "twist_"):
        if b.startswith(pre) and b.endswith(".csv"):
            return b[len(pre):-len(".csv")]
    return b[:-4] if b.endswith(".csv") else b


def bite_ratio(d):
    """I6 末段/首段。<0.90 = 后半程咬住；>=0.90 = 全程打滑（2026-10-22 实测规律）"""
    i6 = [abs(v) for v in d["i6"]]
    n = len(i6)
    if n < 20:
        return float("nan")
    k = max(5, n // 5)
    head = st.mean(i6[:k])
    tail = st.mean(i6[-k:])
    return (tail / head) if head > 1e-6 else float("nan")


# ---------------------------------------------------------------- 子命令
def cmd_list(fs):
    print(f"{'#':>3} {'tag':<26}{'格式':<8}{'采集时间':<15}{'帧':>6}{'时长':>7}"
          f"{'q6 范围':>13}{'I6':>7}{'末/首':>7}  {'结果'}")
    print("-" * 112)
    for n, p in enumerate(fs, 1):
        d = load(p)
        if not d:
            print(f"{n:>3} {tag_of(p):<26}{'(空/损坏)'}")
            continue
        q0, q1 = d["meas_q6"][0], d["meas_q6"][-1]
        i6 = st.mean(abs(v) for v in d["i6"])
        r = bite_ratio(d)
        res = result_of(tag_of(p))
        ko = "全量" if d.get("kind") == "full" else "旧12列"
        print(f"{n:>3} {tag_of(p):<26}{ko:<8}{_mtime_str(p):<15}"
              f"{len(d['t_s']):>6}{d['t_s'][-1]:>6.1f}s"
              f"{q0:>6.0f}->{q1:<7.0f}{i6:>7.3f}{r:>7.2f}  {res}")
    print(f"\n共 {len(fs)} 个文件（全量=200Hz 32 列，旧12列=50Hz）。"
          f"看详情：python3 q.py <#或tag>")


def _mtime_str(p):
    import time
    return time.strftime("%m-%d %H:%M:%S", time.localtime(os.path.getmtime(p)))


def cmd_detail(path, nseg, by_angle, raw, save, axes):
    d = load(path)
    if not d:
        sys.exit(f"[FAIL] {path} 读不出数据")
    n = len(d["t_s"])
    dur = d["t_s"][-1]
    print(f"文件 : {path}")
    print(f"编号 : #{_num_hint(path)}   tag = {tag_of(path)}")
    print(f"采集 : {_mtime_str(path)}   帧 {n}   时长 {dur:.2f}s   "
          f"约 {n / dur:.0f} Hz")
    print(f"q6   : {d['meas_q6'][0]:.2f}° → {d['meas_q6'][-1]:.2f}°   "
          f"转过 {abs(d['meas_q6'][-1] - d['meas_q6'][0]):.1f}°")
    print(f"夹爪 : {d['grip_angle'][0] / 1000:.2f} mm → {d['grip_angle'][-1] / 1000:.2f} mm"
          f"   effort 均值 {st.mean(d['grip_effort']):+.0f}"
          f"（{st.mean(d['grip_effort']) / 1000:+.3f} N·m）")

    # 六轴电流
    print("\n[六轴电流]  单位 A")
    print(f"  {'轴':<4}{'均值':>10}{'标准差':>10}{'峰峰':>10}{'最小':>10}{'最大':>10}")
    for ax in axes:
        v = d["i" + ax[-1]]
        print(f"  {ax:<4}{st.mean(v):>+10.4f}{st.pstdev(v):>10.4f}"
              f"{max(v) - min(v):>10.4f}{min(v):>+10.4f}{max(v):>+10.4f}")

    # I6 分段
    i6a = [abs(v) for v in d["i6"]]
    print(f"\n[I6 分段]  {'按已转过角度' if by_angle else '按时间'}，{nseg} 段")
    if by_angle:
        q = d["meas_q6"]
        q0 = q[0]
        lo, hi = min(q), max(q)
        span = (hi - lo) / nseg if hi > lo else 1e-9
        buckets = []
        for k in range(nseg):
            a, b = lo + span * k, lo + span * (k + 1)
            idx = [j for j, v in enumerate(q) if a <= v < b]
            if len(idx) < 3:
                continue
            deg = abs(q0 - st.mean(q[j] for j in idx))
            buckets.append((k, deg, idx))
        buckets.sort(key=lambda t: t[1])
        print(f"  {'已转过':>8}{'帧数':>7}{'I6均值':>10}{'std':>9}{'τ_load推定':>12}")
        rows_out = []
        for k, deg, idx in buckets:
            m = st.mean(i6a[j] for j in idx)
            s = st.pstdev(i6a[j] for j in idx)
            tau = m * K6 - TAU_F6
            print(f"  {deg:>7.0f}°{len(idx):>7}{m:>10.4f}{s:>9.4f}{tau:>+11.2f}N·m")
            rows_out.append([f"{deg:.1f}", len(idx), f"{m:.5f}", f"{s:.5f}", f"{tau:.3f}"])
    else:
        print(f"  {'时间窗':>13}{'帧数':>7}{'I6均值':>10}{'std':>9}{'τ_load推定':>12}")
        rows_out = []
        for k in range(nseg):
            a, b = k * n // nseg, (k + 1) * n // nseg
            if b - a < 3:
                continue
            seg = i6a[a:b]
            m, s = st.mean(seg), st.pstdev(seg)
            tau = m * K6 - TAU_F6
            print(f"  {d['t_s'][a]:>5.1f}~{d['t_s'][b - 1]:>5.1f}s{b - a:>7}"
                  f"{m:>10.4f}{s:>9.4f}{tau:>+11.2f}N·m")
            rows_out.append([f"{d['t_s'][a]:.2f}-{d['t_s'][b - 1]:.2f}", b - a,
                             f"{m:.5f}", f"{s:.5f}", f"{tau:.3f}"])

    r = bite_ratio(d)
    verdict = "后半程咬住（I6 下降）" if r < 0.90 else "疑似全程打滑（I6 未降）"
    print(f"\n[咬住判据] 末段/首段 I6 = {r:.2f}  →  {verdict}")
    print(f"           判据阈值 0.90；依据 2026-10-22 实测：I6 高=打滑、I6 低=咬住")

    if raw:
        print(f"\n[原始前 {raw} 行]")
        print("  " + ",".join(f"{c:>10}" for c in COLS))
        for j in range(min(raw, n)):
            print("  " + ",".join(f"{d[c][j]:>10.3f}" for c in COLS))

    if save:
        with open(save, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["segment", "frames", "i6_mean", "i6_std", "tau_load_nm"])
            w.writerows(rows_out)
        print(f"\n[csv] 已写 {save}")


def _num_hint(path):
    fs = find_files(os.path.dirname(os.path.abspath(path)))
    base = os.path.basename(path)
    for n, p in enumerate(fs, 1):
        if os.path.basename(p) == base:
            return n
    return "?"


def resolve(token, fs):
    if not fs:
        sys.exit("[FAIL] 没找到 twist_*.csv / twistlog_*.csv")
    if token.isdigit():
        k = int(token)
        if not 1 <= k <= len(fs):
            sys.exit(f"[FAIL] 编号 {k} 超出范围 1~{len(fs)}")
        return fs[k - 1]
    hit = [p for p in fs if token.lower() in tag_of(p).lower()]
    if len(hit) == 1:
        return hit[0]
    if not hit:
        sys.exit(f"[FAIL] 没有 tag 含 '{token}'。先跑 python3 q.py list")
    sys.exit("[FAIL] 匹配到多个：" + ", ".join(tag_of(p) for p in hit))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="拧盖数据速查：按次数/tag 定位并打印关节电流变化",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("target", nargs="?", default="list",
                    help="编号(1,2,3..) / tag 片段 / list")
    ap.add_argument("--dir", default=DEFAULT_DIR, help="data 目录")
    ap.add_argument("-n", "--nseg", type=int, default=8, help="分段数（默认 8）")
    ap.add_argument("--by-angle", action="store_true", help="按已转过角度分段")
    ap.add_argument("--raw", type=int, default=0, help="打印前 N 行原始数据")
    ap.add_argument("--save", default=None, help="分段结果存 csv")
    ap.add_argument("--axes", default="J1,J2,J3,J4,J5,J6", help="要打印的轴")
    a = ap.parse_args()

    fs = find_files(a.dir)
    if a.target == "list":
        cmd_list(fs)
        return
    cmd_detail(resolve(a.target, fs), a.nseg, a.by_angle, a.raw, a.save,
               [x.strip().upper() for x in a.axes.split(",") if x.strip()])


if __name__ == "__main__":
    main()
