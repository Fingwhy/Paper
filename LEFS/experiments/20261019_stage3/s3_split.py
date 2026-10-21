#!/usr/bin/env python3
"""阶段 3 —— 数据集切分（8:1:1）+ 覆盖验收 + 哈希冻结

三件事，缺一不可：
1. **按 episode 划分**，不是按时间窗划分。指南 §6.4：「先按 episode / 采集批次划分
   训练、验证、测试，再构窗。禁止跨段、跨集合构窗。」一段长录硬切的话，边界处
   前一格属于上一条轨迹、下一格已是下一条，那就是跨界污染。
2. **覆盖验收**（指南 §6.4 的验收门）：10 分钟数据覆盖六关节；每关节都有明显的
   位置/速度激励。
3. **冻结并哈希**：`splits.json` 写完后算 sha256，之后任何改动都会被发现。

为什么实测不判「速度」
----------------------
R39 实测过：50 Hz 层的差分速度被位置量化（0.001 rad / 0.02 s）抬到 **2.87 °/s**
的噪声底，而本阶段的速度激励门槛只有 3 °/s —— 判据和噪声同量级，判了等于没判。

所以速度按**设计值**判（`s3_design.py` 里 `2πf·amp`，那是有意义的），
实测只判位置峰峰：要求实测 ≥ 设计值的 70%（证明轨迹真的执行了，没有被挡/没发出去）。
文件里给的速度是**平滑后**的参考值，标注了不作判据。

用法（VM 或本机，episodes 同级目录下）：
    python3 s3_split.py                 # 扫全部 episode，写 splits.json
    python3 s3_split.py --selftest      # 自测，不需要真数据
    python3 s3_split.py --episodes /path/to/episodes
"""

import argparse
import hashlib
import json
import math
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import s3_design as D  # noqa: E402

RAD2DEG = 180.0 / math.pi

# 实测覆盖率门槛：真的走到的行程不应低于设计的 70%
# （低于它说明被挡、被限幅、或指令没发出去）
MIN_REALIZE = 0.70

# 训练集运动时长下限（指南 §6.1「约 10 分钟」）。取 540 s = 9 min，
# 留一点余量给「10 分钟是量级、不是硬指标」。
MIN_TRAIN_SEC = 540.0

# 切分比例容差
SPLIT_TARGET = {"train": 0.8, "val": 0.1, "test": 0.1}
SPLIT_TOL = 0.06


def read_ep(path):
    """读 samples.jsonl → (t 列表, q 列表[弧度], 是否有 q_cmd 的布尔列表)。"""
    ts, qs, hc = [], [], []
    f = Path(path) / "samples.jsonl"
    if not f.exists():
        return ts, qs, hc
    for line in f.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        q = r.get("q")
        if not q or any(v is None for v in q):
            continue
        ts.append(r.get("t"))
        qs.append([float(v) for v in q])
        hc.append(r.get("q_cmd") is not None)
    return ts, qs, hc


def excitation(ts, qs, hc=None):
    """返回 (每轴位置峰峰 °, 每轴平滑速度峰 °/s 参考值, 全段时长 s, 激励窗时长 s)。

    **只统计激励窗口**，这是必须的：一个 sine 段的 raw 覆盖「pose 到位 + 正弦 + 收尾」，
    而 pose 到位本身就让 J2 走 30°、J3 走 40°。若把整段都算进去，J2/J3 的「峰峰」
    会被到位移动顶到 30~40°，看起来激励很足，实际上正弦那一轴可能只摆了 3° ——
    这正是「数字好看、数据是废的」。

    判据窗口 = `q_cmd` 非空的样本。sine 段的 raw 里，只有正弦那 20~40 s 有 q_cmd
    （parse 的 `--qcmd` 指向 sine 的 CSV），到位段 q_cmd 为 None，天然被排除。
    pose 段的 q_cmd 覆盖全过程，所以它统计的就是全程（pose 段不判激励，无妨）。

    速度先做 5 点滑动平均再差分 —— 不是为了"美化数据"，是因为 20 ms 栅格上的
    0.001 rad 量化会把差分抬到 ~2.9 °/s（R39）。平滑后噪声底降到 ~0.3 °/s。
    它仍然只是参考值，不作判据。
    """
    if not qs:
        return [0.0] * 6, [0.0] * 6, 0.0, 0.0

    def span(tv):
        good = [t for t in tv if t is not None]
        return (max(good) - min(good)) if len(good) >= 2 else 0.0

    dur_all = span(ts)

    idx = [k for k in range(len(qs)) if hc is None or hc[k]]
    if not idx:
        idx = list(range(len(qs)))
    sel_t = [ts[k] for k in idx]
    sel_q = [qs[k] for k in idx]
    dur_win = span(sel_t)

    pp = [(max(q[j] for q in sel_q) - min(q[j] for q in sel_q)) * RAD2DEG
          for j in range(6)]

    sm = []
    W = 5
    for k in range(len(sel_q)):
        lo, hi = max(0, k - W // 2), min(len(sel_q), k + W // 2 + 1)
        sm.append([sum(sel_q[m][j] for m in range(lo, hi)) / (hi - lo) for j in range(6)])

    vmax = [0.0] * 6
    for k in range(1, len(sm)):
        if sel_t[k] is None or sel_t[k - 1] is None:
            continue
        dt = sel_t[k] - sel_t[k - 1]
        if dt <= 0:
            continue
        for j in range(6):
            v = abs(sm[k][j] - sm[k - 1][j]) / dt * RAD2DEG
            if v > vmax[j]:
                vmax[j] = v
    return pp, vmax, dur_all, dur_win


def collect(ep_dir, names=None):
    """扫 episode 目录，只收已生成 samples.jsonl 的段。"""
    out = {}
    for t in D.TRACKS:
        n = t["name"]
        if names and n not in names:
            continue
        p = Path(ep_dir) / n
        if not (p / "samples.jsonl").exists():
            continue
        ts, qs, hc = read_ep(p)
        pp, vmax, dur_all, dur_win = excitation(ts, qs, hc)
        dpp, dvp = D.artic(t)
        out[n] = dict(split=t["split"], kind=t["kind"], n=len(qs),
                      dur_s=round(dur_all, 2), win_s=round(dur_win, 2),
                      pp_deg=[round(x, 2) for x in pp],
                      v_peak_dps=[round(x, 2) for x in vmax],
                      design_pp_deg=[round(x, 2) for x in dpp],
                      design_v_dps=[round(x, 2) for x in dvp],
                      realize=round(max((pp[j] / dpp[j]) if dpp[j] > 0 else 1.0
                                        for j in range(6)), 3),
                      note=t["note"])
    return out


def gate(eps):
    """验收门。返回 (判据 dict, 是否通过)。"""
    g = {}

    for sp in ("train", "val", "test"):
        sel = [v for v in eps.values() if v["split"] == sp]
        g[f"n_{sp}"] = len(sel)
        g[f"dur_{sp}_s"] = round(sum(v.get("win_s", v["dur_s"]) for v in sel), 1)

    # 门 1：训练集**有效激励时长**（不是整段 raw 时长 —— 到位与收尾不算激励）
    g["train_over_9min"] = g["dur_train_s"] >= MIN_TRAIN_SEC

    # 门 2：六关节覆盖 —— 每轴在训练集里至少有一段实测峰峰 ≥ 门槛
    cov = []
    for j in range(6):
        best = max((v["pp_deg"][j] for v in eps.values() if v["split"] == "train"),
                   default=0.0)
        cov.append(round(best, 2))
    g["coverage_pp_deg"] = cov
    g["coverage_6joint"] = all(x >= D.MIN_PP_DEG for x in cov)

    # 门 3：轨迹真的执行了（实测 ≥ 设计 70%）
    low = {n: v["realize"] for n, v in eps.items()
           if v["kind"] == "sine" and v["realize"] < MIN_REALIZE}
    g["under_realized"] = low
    g["all_realized"] = not low

    # 门 4：8:1:1
    tot = max(1, len(eps))
    ratio = {sp: round(g[f"n_{sp}"] / tot, 3) for sp in ("train", "val", "test")}
    g["ratio"] = ratio
    g["ratio_ok"] = all(abs(ratio[sp] - SPLIT_TARGET[sp]) <= SPLIT_TOL for sp in ratio)

    # 门 5：同一轨迹不跨集合
    def key(t):
        return (round(t["freq"], 4), tuple(t["amp"]), tuple(t["center"]))

    seen = {}
    dup = []
    for t in D.TRACKS:
        k = key(t)
        if k in seen and seen[k] != t["split"]:
            dup.append((t["name"], seen[k], t["split"]))
        seen.setdefault(k, t["split"])
    g["cross_split_dup"] = dup
    g["no_cross_split"] = not dup

    ok = all(g[k] for k in ("train_over_9min", "coverage_6joint",
                            "all_realized", "ratio_ok", "no_cross_split"))
    return g, ok


def freeze(eps, g):
    """生成可哈希的切分清单。哈希只覆盖 name→split 与集合成员，不受统计数字影响。"""
    sets = {sp: sorted(n for n, v in eps.items() if v["split"] == sp)
            for sp in ("train", "val", "test")}
    core = {"sets": sets,
            "members": {n: eps[n]["split"] for n in sorted(eps)}}
    blob = json.dumps(core, sort_keys=True, ensure_ascii=False).encode("utf-8")
    h = hashlib.sha256(blob).hexdigest()
    return dict(created=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
                stage="stage3",
                hash_sha256=h,
                hash_scope="name→split 与三集合成员（排序后 JSON 的 sha256）",
                counts={sp: len(sets[sp]) for sp in sets},
                gates=g,
                sets=sets,
                episodes=eps)


# ---------------------------------------------------------------------------
# 自测
# ---------------------------------------------------------------------------
def _selftest():
    n_ok = n_bad = 0

    def chk(title, cond, detail=""):
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print(f"  [OK] {title}")
        else:
            n_bad += 1
            print(f"  [XX] {title}  {detail}")

    print("自测 A 矩阵与切分设计一致")
    sp = {s: [t for t in D.TRACKS if t["split"] == s] for s in ("train", "val", "test")}
    tot = len(D.TRACKS)
    chk("A 三集合都非空", all(sp.values()),
        str({k: len(v) for k, v in sp.items()}))
    chk("A 比例接近 8:1:1（±6%）",
        all(abs(len(sp[s]) / tot - SPLIT_TARGET[s]) <= SPLIT_TOL for s in sp),
        str({k: round(len(v) / tot, 2) for k, v in sp.items()}))
    tr = sum(t["dur"] for t in sp["train"])
    chk("A 训练批设计时长 ≥ 9 min", tr >= MIN_TRAIN_SEC, f"{tr/60:.1f} min")

    print("自测 B 合成样本的峰峰算得对")
    # 合成：J3 上 1 Hz、±10° 的方块波，4 s
    qs, ts = [], []
    for k in range(200):
        t = k * 0.02
        q = [0.0] * 6
        q[2] = math.radians(10.0 if (t % 1.0) < 0.5 else -10.0)
        qs.append(q)
        ts.append(t)
    pp, vmax, dur, win = excitation(ts, qs)
    chk("B 峰峰 = 20°（±10°）", abs(pp[2] - 20.0) < 0.1, f"{pp[2]:.2f}")
    chk("B 时长 ≈ 3.98 s", abs(dur - 3.98) < 0.05, f"{dur:.2f}")
    chk("B 速度峰 >= 15 °/s（方块波约 20 °/s）", vmax[2] >= 15.0, f"{vmax[2]:.1f}")

    print("自测 B2 只统计 q_cmd 窗口（到位段必须被排除）")
    # 前半段：J2 从 0° 走到 30°（模拟 pose 到位，无 q_cmd）
    # 后半段：J1 ±6° 正弦（有 q_cmd）
    qs2, ts2, hc2 = [], [], []
    for k in range(100):
        q = [0.0] * 6
        q[1] = math.radians(30.0 * k / 99)
        qs2.append(q)
        ts2.append(k * 0.02)
        hc2.append(False)
    for k in range(200):
        q = [0.0] * 6
        q[1] = math.radians(30.0)
        q[0] = math.radians(6.0 if (k // 25) % 2 == 0 else -6.0)
        qs2.append(q)
        ts2.append(2.0 + k * 0.02)
        hc2.append(True)
    pp2, v2, d2, w2 = excitation(ts2, qs2, hc2)
    chk("B2 J2 不计到位移动（应恒为 0，而不是 30°）", pp2[1] < 0.1, f"{pp2[1]:.2f}")
    chk("B2 J1 只计正弦窗口 → 12°", abs(pp2[0] - 12.0) < 0.1, f"{pp2[0]:.2f}")
    chk("B2 全段时长 ≈ 5.98 s、窗口 ≈ 3.98 s",
        abs(d2 - 5.98) < 0.05 and abs(w2 - 3.98) < 0.05, f"{d2:.2f} / {w2:.2f}")

    print("自测 C 静止数据的速度不该被量化噪声抬到门槛")
    qs3 = [[math.radians(5.0)] * 6 for _ in range(200)]
    pp3, v3, _, _ = excitation([k * 0.02 for k in range(200)], qs3)
    chk("C 完全静止 → 速度峰 < 0.5 °/s（5 点平滑压住量化噪声）",
        max(v3) < 0.5, f"{max(v3):.2f}")

    print("自测 D 哈希可复现且对改动敏感")
    fake = {t["name"]: dict(split=t["split"], kind=t["kind"], n=1, dur_s=1.0,
                            pp_deg=[0] * 6, v_peak_dps=[0] * 6,
                            design_pp_deg=[0] * 6, design_v_dps=[0] * 6,
                            realize=1.0, note="") for t in D.TRACKS}
    gfake = {k: True for k in ("train_over_9min", "coverage_6joint",
                               "all_realized", "ratio_ok", "no_cross_split")}
    h1 = freeze(fake, gfake)["hash_sha256"]
    h2 = freeze(fake, gfake)["hash_sha256"]
    chk("D 同一输入两次哈希相同", h1 == h2)
    fake2 = json.loads(json.dumps(fake))
    k0 = D.TRACKS[0]["name"]
    fake2[k0]["split"] = "test" if fake2[k0]["split"] != "test" else "train"
    h3 = freeze(fake2, gfake)["hash_sha256"]
    chk("D 改一个段的 split 哈希就变", h1 != h3)

    print("自测 E 验收门能抓住缺段")
    g1, ok1 = gate(fake)
    chk("E 合成数据（时长不足）判不过", not ok1, str({k: v for k, v in g1.items()
                                                     if isinstance(v, bool)}))
    full = json.loads(json.dumps(fake))
    for n in full:
        full[n]["dur_s"] = 30.0
        full[n]["pp_deg"] = [20.0] * 6
    g2, ok2 = gate(full)
    chk("E 补齐时长与峰峰后判通过", ok2,
        str({k: v for k, v in g2.items() if isinstance(v, bool)}))

    print(f"\n自测：{n_ok} 过 / {n_bad} 败")
    return 0 if n_bad == 0 else 1


def main():
    ap = argparse.ArgumentParser(description="阶段 3 数据集切分与验收")
    ap.add_argument("--episodes", default=str(HERE / "episodes"))
    ap.add_argument("--out", default=str(HERE / "splits.json"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return _selftest()

    ep_dir = Path(a.episodes)
    if not ep_dir.exists():
        print(f"[ABORT] 没有 episode 目录 {ep_dir}", file=sys.stderr)
        return 2
    eps = collect(ep_dir)
    if not eps:
        print(f"[ABORT] {ep_dir} 下没有任何 trace 的 samples.jsonl", file=sys.stderr)
        return 2

    missing = [t["name"] for t in D.TRACKS if t["name"] not in eps]
    g, ok = gate(eps)
    out = freeze(eps, g)
    out["missing_tracks"] = missing
    Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"{'段名':<14}{'split':<7}{'n':>6}{'时长s':>8}{'实测峰峰°':>10}{'设计峰峰°':>10}{'达成':>7}")
    for n in sorted(eps):
        v = eps[n]
        print("%-14s%-7s%6d%8.1f%10.1f%10.1f%7.2f"
              % (n, v["split"], v["n"], v["dur_s"],
                 max(v["pp_deg"]), max(v["design_pp_deg"]), v["realize"]))
    print()
    print(f"训练 {g['n_train']} 段 / {g['dur_train_s']:.0f} s；"
          f"验证 {g['n_val']} 段 / {g['dur_val_s']:.0f} s；"
          f"测试 {g['n_test']} 段 / {g['dur_test_s']:.0f} s")
    print(f"比例 {g['ratio']}（目标 8:1:1 ±{SPLIT_TOL}）")
    print(f"六关节覆盖峰峰 {g['coverage_pp_deg']}（门槛 {D.MIN_PP_DEG}）")
    if missing:
        print(f"⚠️ 缺 {len(missing)} 段：{missing}")
    print()
    for k in ("train_over_9min", "coverage_6joint", "all_realized",
              "ratio_ok", "no_cross_split"):
        print(f"  {'✓' if g[k] else '✗'} {k}")
    if g["under_realized"]:
        print(f"  ✗ 达成率不足：{g['under_realized']}")
    print()
    print(f"切分清单 → {a.out}")
    print(f"  sha256 = {out['hash_sha256']}")
    print(f"总判： {'通过' if ok and not missing else '未通过（补齐缺段后再冻结）'}")
    return 0 if (ok and not missing) else 1


if __name__ == "__main__":
    sys.exit(main())
