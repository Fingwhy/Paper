#!/usr/bin/env python3
"""stage1_l0.py —— 阶段 1（L0）：信号链与同步（原始层 + 解析层 + 自测）

对应指南 §4.1 七步：
  1 原始层落盘（只追加，不解析）   → `record` 子命令，写 raw/*.jsonl
  2 解析层（0x251~0x256 / 0x2A1~0x2A8）→ `parse`
  3 统一单位（rad / rad·s⁻¹ / A / N·m）
  4 构造 q_cmd（必须用真正下发的参考位置，不能用 q 代替）
  5 时间对齐（记录字段年龄与跨关节时间跨度）
  6 异常处理（NaN/缺帧/过期/饱和/保护/指令中断 —— 全部带标记，绝不静默填零）
  7 压测（10 分钟静止 + 低速运动）→ `record --iface can0 --seconds 600`

设计约束
--------
* **纯标准库 + python-can**。minimal 安装里没有 numpy，本文件一行都不用它。
* `record` 与 `parse` 分开：原始层一旦写坏就永远无法补救，所以原始层只存
  「CAN ID + 8 字节 + 接收时刻 + 异常位」，**一个字段都不解释**。
* 缺少硬件时用 `--replay` 或 `--synthetic` 走同一条代码路径，先自测通过。
* 越界（oob）值**不进 q 字段**，只进 `q_rejected`：宁可留 null 让人发现，
  也不让一个 37480 rad 的数悄悄流进训练集。

时间基准
--------
原始层的 `t` 是**绝对 epoch 秒**（socketcan 的 `msg.timestamp`）。
`--qcmd` 给的 CSV 必须用同一个时间基准，否则会全部判 `stale:q_cmd`。

用法
----
    python stage1_l0.py selftest                       # 不需要硬件，8 个用例
    python stage1_l0.py record --synthetic normal --seconds 5 --out raw/syn.jsonl
    python stage1_l0.py parse raw/syn.jsonl --hz 50 --out episodes/syn
    python stage1_l0.py record --iface can0 --seconds 600 --out raw/s1.jsonl   # 真机
    python stage1_l0.py parse raw/s1.jsonl --hz 50 --qcmd cmd.csv --out episodes/s1

q_cmd CSV 格式（可选，两列，无表头）：`t_sec,j1,j2,j3,j4,j5,j6`，单位 rad。
没有它也能跑，但样本里 q_cmd 会是 null 并打 `no_q_cmd` 标记——不允许用 q 顶替。

产物
----
    raw/<name>.jsonl            原始层，一行一帧，只追加
    episodes/<name>/samples.jsonl   对齐后的样本（q / dq / cur / q_cmd / age_ms）
    episodes/<name>/sync_report.md  人读的同步报告
    episodes/<name>/metrics.json    机读的判据结果（供后续流水线直接消费）
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import sys
import time
from pathlib import Path

RAW_VERSION = "stage1_l0/1.2"          # 原始层格式版本（改字段含义才动）
PARSER_VERSION = "stage1_l0/1.7-20261019"   # 解析器版本（判据/报告改动都动，便于现场对账）

# ---------------------------------------------------------------- 帧定义（指南 §4.2）

HIGH_SPD_IDS = list(range(0x251, 0x257))      # 关节高速反馈：转速/电流/位置。**实测 5 ms / 200 Hz**
JOINT_ANGLE_IDS = [0x2A5, 0x2A6, 0x2A7]       # 关节角：0x2A5=J1J2, 0x2A6=J3J4, 0x2A7=J5J6
STATUS_ID = 0x2A1                             # 臂状态（0x2A1~0x2A8 同频，均为 200 Hz）
GRIPPER_ID = 0x2A8                            # 夹爪角度，不是第七轴——必须与关节区分
LOW_SPD_IDS = list(range(0x261, 0x267))       # 低速反馈（母线等）。实测 25 ms / 40 Hz，不是 0

# 源帧周期。2026-10-19 12:12 上电实测：0x251~0x256 与 0x2A1~0x2A8 的 median 间隔
# 都是 5.00 ms（p95 5.2 / max 13.5），即 200 Hz。1.1 版写 0.020（50 Hz）是照文档抄的，
# 与实测差 4 倍 —— 这会让「期望帧数」「丢帧判据」全线偏松，真丢一半帧也看不出来。
NOMINAL_DT = 0.005

# 原始层异常位
F_OUT_OF_ORDER = 0x01
F_UNKNOWN_ID = 0x02
F_SHORT_DLC = 0x04
F_ERROR = 0x08

# 提示性标记：出现它不代表数据不可用，只代表「这一列本来就缺」
# 判「干净样本」时必须排除，否则不接 q_cmd 的 L0 压测会 100% 被判脏
BENIGN_FLAGS = frozenset({"no_q_cmd", "qcmd_out_of_window"})

# 与 piper_ros humble URDF 一致，仅作「合理区间」守卫，产品代码不得硬编码。
# 真机跑过 0x473 之后，应把回读值写进 --limits 覆盖本表（TODO: L1 前完成）。
JOINT_MIN = [-2.6180, 0.0, -2.9670, -1.7450, -1.2200, -2.0944]
JOINT_MAX = [2.6180, 3.1416, 0.0, 1.7450, 1.2200, 2.0944]

# 实测自然零位上 J2 = −2.003°、J3 = +1.798°：两个关节都落在 URDF 标称边界之外
# （J2 下界 0、J3 上界 0）。1.7 版把这种情况判成 `oob` 并**把 q 置 null**，
# 结果 S2-0 的样本里 q2/q3 全程为 null —— 六轴缺两轴，静态力矩根本拟合不了。
# 越界 2° 属于已登记的模型差（手册 §10.3 的 DH 偏移），不是异常；放大 3° 后再判。
JOINT_LIMIT_TOL = math.radians(3.0)


def _s16(b: bytes) -> int:
    return int.from_bytes(b, "big", signed=True)


def _u16(b: bytes) -> int:
    return int.from_bytes(b, "big", signed=False)


def _s32(b: bytes) -> int:
    return int.from_bytes(b, "big", signed=True)


# ---------------------------------------------------------------- 解码

def decode_high_spd(joint_idx: int, data: bytes, pos251_unit: str) -> dict:
    """0x251~0x256：字节0-1 转速(int16,0.001 rad/s)、2-3 电流、4-7 位置(int32)。

    **电流字段的符号与量纲是实测纠正过的**（2026-10-20）：
    SDK 文档写 `uint16 / 0.001 A`，但静止读出的原始字节是 `fffe`、`fbc9` 这类值 ——
    按 uint16 就是 65.5 A，物理上不可能。按 **int16** 读则是 −2、−1079，即 −0.002 A /
    −1.079 A，与「静态保持时各轴在零点附近小幅正负」完全吻合。
    因此：`cur` 用 int16（0.001 A，带符号），`cur_raw` 保留无符号原值备查。
    这条不改的话，S2-0 的「电流噪声 σ」和符号判定全建立在一个饱和假值上。

    位置单位仍有两种说法（指南写 rad；实测 0.001 rad，差 1000 倍），由 `pos251_unit`
    显式选择，并在报告里用 0x2A5 交叉核验，不靠猜。
    """
    scale = 1.0 if pos251_unit == "rad" else 0.001
    cur_i = _s16(data[2:4])
    return {
        "joint": joint_idx,
        "dq": _s16(data[0:2]) * 0.001,
        "cur_raw": _u16(data[2:4]),          # 无符号原值，备查
        "cur": cur_i * 0.001,                # 实测口径：带符号
        "pos": _s32(data[4:8]) * scale,
    }


def decode_joint_pair(first_joint: int, data: bytes) -> dict:
    """0x2A5/6/7：两个关节角，各 int32，单位 0.001 度 → rad。"""
    out = {}
    for k in range(2):
        raw = _s32(data[k * 4:(k + 1) * 4])
        out[first_joint + k] = math.radians(raw * 0.001)
    return out


def decode_status(data: bytes) -> dict:
    """0x2A1：byte0 控制模式、byte1 臂状态、byte2 模式反馈、byte3 示教、byte4 运动状态。"""
    return {
        "ctrl_mode": data[0],
        "arm_status": data[1],
        "mode_feed": data[2],
        "teach_status": data[3],
        "motion_status": data[4],
    }


# ---------------------------------------------------------------- 帧源

def frames_synthetic(mode: str, seconds: float):
    """合成帧源：50 Hz，6 路高速反馈 + 3 路关节角 + 1 路状态。

    关键约束：**所有通道都由同一个 q_now 生成**。
    只要有一路用了不同姿态，「0x251 位置 / 0x2A5 角度」的比值校验就会失真，
    从而把工具自己的 bug 报成硬件单位问题——这正是 1.0 版自测假失败的原因。

    mode 只改变「发什么、发多少」，不改变物理量本身。
    """
    n = int(seconds / NOMINAL_DT)
    t0 = 1_000_000.0
    q0 = [0.10, 1.20, -1.30, 0.20, 0.30, -0.40]     # 一个六轴都在量程内的姿态

    drop_joint = 4 if mode == "missing_joint" else None     # 0-based → J5
    sat_joint = 2 if mode == "saturated" else None          # 0-based → J3
    gap_at = 1.0 if mode == "gap" else None                 # 1.0 s 处断 150 ms
    # 关节角是「成对」下发的：J5 落在 0x2A7 的低 4 字节，丢它必然连 J6 一起丢。
    # 所以 drop_joint 不仅跳过 0x255，还要跳过承载它的那一对角度帧。
    drop_pair = None if drop_joint is None else drop_joint // 2

    for k in range(n):
        rel = k * NOMINAL_DT
        t = t0 + rel
        if gap_at is not None and gap_at <= rel < gap_at + 0.150:
            continue

        sway = 0.02 * math.sin(2 * math.pi * 0.25 * rel)
        q_now = [v + sway for v in q0]
        if sat_joint is not None:
            # int32 上限在 0.001 度编码下对应的弧度值
            q_now[sat_joint] = math.radians(0x7FFFFFFF * 0.001)

        yield t, STATUS_ID, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0, 0, 0])

        for j, cid in enumerate(HIGH_SPD_IDS):
            if drop_joint is not None and j == drop_joint:
                continue
            dq_raw = 10                       # 0.010 rad/s，恒定
            cur_raw = 150 + j                 # 0.150~0.155 A，恒定
            pos_raw = int(round(q_now[j] * (1000 if mode == "pos_mrad" else 1)))
            if sat_joint is not None and j == sat_joint:
                pos_raw = 0x7FFFFFFF
            data = (dq_raw.to_bytes(2, "big", signed=True)
                    + cur_raw.to_bytes(2, "big", signed=False)
                    + pos_raw.to_bytes(4, "big", signed=True))
            if mode == "short_dlc" and j == 4:
                data = data[:5]               # 只发 5 字节，验证短帧被标记
            tt = t - 0.040 if (mode == "out_of_order" and k == 30) else t
            yield tt, cid, data

        for pair_idx, cid in enumerate(JOINT_ANGLE_IDS):
            j1 = pair_idx * 2
            if drop_pair is not None and pair_idx == drop_pair:
                continue
            data = b"".join(
                int(round(math.degrees(q_now[j]) * 1000)).to_bytes(4, "big", signed=True)
                for j in (j1, j1 + 1)
            )
            yield t, cid, data


def frames_replay(path: Path):
    """回放已落盘的原始层（同格式），用于重复解析与回归。"""
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("kind"):
                continue
            yield rec["t"], rec["id"], bytes.fromhex(rec["d"])


def frames_can(iface: str, seconds: float, backend: str = "socketcan", rcvbuf_mb: int = 8):
    """真机帧源。总线错误只计数、不中断——录到一半炸掉比丢了 3 帧更糟。"""
    import can
    bus = can.Bus(interface=backend, channel=iface)
    # SO_RCVBUF：Linux 默认只有 208 KB。本总线 25 个 ID × 200 Hz，突发一下就到上限，
    # 溢出时内核**静默丢帧**，进程侧看不出来。这不是"提高性能"，是把一个不可见的
    # 失败模式变成可控的。注意内核上限 net.core.rmem_max 通常就是 208 KB，
    # 请求更大只能拿到上限 —— 所以必须把实际值打出来，别让人以为设了 8 MB 就安全。
    try:
        sk = getattr(bus, "socket", None)
        if sk is not None:
            want = int(rcvbuf_mb) * 1024 * 1024
            sk.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, want)
            got = sk.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            extra = ""
            if got < want:
                extra = (f"（被 net.core.rmem_max 限住：sudo sysctl -w "
                         f"net.core.rmem_max={want} 可放开）")
            print(f"[record] SO_RCVBUF 请求 {want // 1024} KB → 实际 {got // 1024} KB{extra}")
    except Exception as exc:
        print(f"[record][WARN] 设置 SO_RCVBUF 失败：{exc!r}", file=sys.stderr)

    t_end = time.time() + seconds
    n_err = 0
    try:
        while time.time() < t_end:
            try:
                msg = bus.recv(timeout=0.2)
            except can.CanError as exc:          # 控制器错误帧、瞬时 overflow
                n_err += 1
                if n_err <= 5:
                    print(f"[record] 总线错误 #{n_err}: {exc}", file=sys.stderr)
                continue
            if msg is None:
                continue
            yield msg.timestamp, msg.arbitration_id, bytes(msg.data)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- 第 1 步：原始层落盘

def cmd_record(args) -> int:
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    if args.synthetic:
        src = frames_synthetic(args.synthetic, args.seconds)
        desc = f"synthetic:{args.synthetic}"
    elif args.replay:
        src = frames_replay(Path(args.replay))
        desc = f"replay:{args.replay}"
    else:
        src = frames_can(args.iface, args.seconds, args.backend)
        desc = f"can:{args.iface}@{args.backend}"

    known = (set(HIGH_SPD_IDS) | set(JOINT_ANGLE_IDS) | {STATUS_ID, GRIPPER_ID}
             | set(range(0x2A2, 0x2A5)) | set(LOW_SPD_IDS))
    stats = {"n": 0, "unknown": 0, "short": 0, "ooo": 0, "per_id": {},
             "t_first": None, "t_last": None}
    last_t = {}
    t_start = time.time()

    with out.open("w", encoding="utf-8") as f:
        f.write(json.dumps({"v": RAW_VERSION, "kind": "header", "source": desc,
                            "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                            "planned_seconds": (args.seconds if args.seconds else None)},
                           ensure_ascii=False) + "\n")
        f.flush()
        try:
            for t, cid, data in src:
                flags = 0
                if cid not in known:
                    flags |= F_UNKNOWN_ID
                    stats["unknown"] += 1
                if len(data) < 8:
                    flags |= F_SHORT_DLC
                    stats["short"] += 1
                prev = last_t.get(cid)
                if prev is not None and t < prev:
                    flags |= F_OUT_OF_ORDER
                    stats["ooo"] += 1
                last_t[cid] = max(t, prev if prev is not None else t)

                # 只存事实，不存解释
                f.write(json.dumps({"t": round(t, 6), "id": cid,
                                    "d": bytes(data).hex(), "f": flags},
                                   ensure_ascii=False, separators=(",", ":")) + "\n")
                stats["n"] += 1
                stats["per_id"][cid] = stats["per_id"].get(cid, 0) + 1
                if stats["t_first"] is None or t < stats["t_first"]:
                    stats["t_first"] = t
                if stats["t_last"] is None or t > stats["t_last"]:
                    stats["t_last"] = t
                if stats["n"] % 5000 == 0:
                    f.flush()             # 长录制中途断电也不丢已收的帧
        except KeyboardInterrupt:
            # 真机 10 分钟压测靠 Ctrl-C 收尾，已落盘的帧必须保住
            print("\n[record] 收到 Ctrl-C，已落盘部分保留", file=sys.stderr)

    dur = max(time.time() - t_start, 1e-6)
    t_first, t_last = stats["t_first"], stats["t_last"]
    span = (t_last - t_first) if (t_first is not None and t_last is not None) else 0.0
    print(f"[record] {stats['n']} 帧 / 墙钟 {dur:.1f}s → {out}")
    print(f"[record] 未知 ID {stats['unknown']}，短帧 {stats['short']}，乱序 {stats['ooo']}")
    if args.synthetic or args.replay:
        print(f"[record] 帧时间跨度 {span:.1f}s（合成/回放，不按墙钟考核）")
    else:
        print(f"[record] 帧时间跨度 {span:.1f}s（首帧→末帧），与墙钟相差 {dur - span:.1f}s")

    hs = [stats["per_id"].get(c, 0) for c in HIGH_SPD_IDS]
    # 判据要按「帧时间跨度」算，不能按墙钟。
    # 15:26 那次就吃在这里：90.2 s 墙钟里只有 52.0 s 有帧，而 0x251 恰好 10393 帧
    # = 52.0 s × 200 Hz —— 每一路其实**一帧没丢**，但按墙钟除会印成「只收到 115 Hz、
    # 丢 42%」。前者要查断流，后者要查丢帧，处置完全不同，混在一起就会查错方向。
    if span > 0 and hs:
        rate = [n / span for n in hs]
        print(f"[record] 0x251~0x256 每路 {min(rate):.1f}~{max(rate):.1f} Hz"
              f"（按帧跨度 {span:.1f}s 算，标称 {1 / NOMINAL_DT:g} Hz）")
    if args.synthetic:
        exp = int(args.seconds / NOMINAL_DT)          # 合成帧：帧数由时长算
    elif args.replay:
        exp = int(round(stats["n"] / 10))             # 回放：每周期 1 状态+6 高速+3 角度
    else:
        exp = max(int(dur / NOMINAL_DT) - 1, 0)       # 真机：看实际墙钟
    print(f"[record] 0x251~0x256 帧数 {hs}（按墙钟期望 {exp} 上下）")
    if stats["n"] == 0:
        print("[record] 一帧都没收到——先解决总线，别急着解析", file=sys.stderr)
        return 3
    if not (args.synthetic or args.replay) and span > 0 and span < 0.9 * dur:
        print(f"[record][WARN] 断流：末帧落在墙钟第 {span:.1f}s，之后 {dur - span:.1f}s 一帧都没有。"
              f"这不是丢帧率问题，是总线/适配器在录制中途停了。\n"
              f"        先看 ip -s link show can0（RX dropped/overrun），"
              f"再确认没有别的进程同时在动这个接口。", file=sys.stderr)
    missing = [hex(c) for c in HIGH_SPD_IDS if stats["per_id"].get(c, 0) == 0]
    if missing:
        print(f"[record] 以下高速反馈 ID 一帧未收到：{missing}", file=sys.stderr)
    return 0


# ---------------------------------------------------------------- 第 2~6 步：解析 + 对齐

def _load_qcmd(path: Path | None):
    if path is None:
        return None
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip().rstrip(",")
        if not line:
            continue
        parts = line.split(",")
        rows.append((float(parts[0]), [float(x) for x in parts[1:7]]))
    rows.sort(key=lambda r: r[0])
    return rows


def _qcmd_at(rows, t):
    """取 t 时刻最近一次下发的 q_cmd（不插值：插值会伪造出没下发的指令）。"""
    lo, hi = 0, len(rows) - 1
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        if rows[mid][0] <= t:
            best = rows[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    return (best[1], t - best[0]) if best else (None, None)


def _series_name(key) -> str:
    """序列名。注意 0x2A5 装 J1J2、0x2A6 装 J3J4、0x2A7 装 J5J6。"""
    if isinstance(key, tuple):
        j = key[1]
        return f"0x2A{5 + (j - 1) // 2}J{j}"
    return f"0x{key:03X}"


def cmd_audit(args) -> int:
    """体检一份原始层：帧到底覆盖了哪段时间、哪一段空了、0x2A8 有没有内容。

    存在的理由（R30）：15:26 那次 record 报「90.2 s」，实际只有 52.0 s 有帧。
    只看得帧数会以为是「丢 42%」，看得时间跨度才知道是「后半段断流」——
    这两件事的处置完全不同。判断依据必须是时间轴，不是总帧数。
    """
    raw = Path(args.raw)
    bucket = max(args.bucket, 1.0)
    t_first = t_last = None
    buckets: dict[int, dict] = {}
    prev_t = None
    holes: list[tuple[float, float]] = []
    g_n = 0
    g_nonzero = 0
    unknown: dict[int, int] = {}
    planned = None
    source = None

    with raw.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("kind") == "header":
                planned = rec.get("planned_seconds")
                source = rec.get("source")
                continue
            t, cid = rec["t"], rec["id"]
            if t_first is None or t < t_first:
                t_first = t
            if t_last is None or t > t_last:
                t_last = t
            if prev_t is not None and (t - prev_t) * 1000.0 > 100.0:
                holes.append(((prev_t - (t_first or prev_t)), (t - prev_t) * 1000.0))
            prev_t = t if prev_t is None else max(prev_t, t)
            b = int((t - (t_first if t_first is not None else t)) // bucket)
            d = buckets.setdefault(b, {"n": 0, "hs": 0, "ang": 0, "unk": 0})
            d["n"] += 1
            if cid in HIGH_SPD_IDS:
                d["hs"] += 1
            elif cid in JOINT_ANGLE_IDS:
                d["ang"] += 1
            elif cid not in (set(JOINT_ANGLE_IDS) | set(HIGH_SPD_IDS) | {STATUS_ID, GRIPPER_ID}
                             | set(range(0x2A2, 0x2A5)) | set(LOW_SPD_IDS)):
                d["unk"] += 1
                unknown[cid] = unknown.get(cid, 0) + 1
            if cid == GRIPPER_ID:
                g_n += 1
                if any(bytes.fromhex(rec["d"])):
                    g_nonzero += 1

    if t_first is None:
        print("[audit] 原始层里没有帧", file=sys.stderr)
        return 3

    span = t_last - t_first
    print(f"[audit] {raw.name}  来源 {source}")
    print(f"[audit] 帧时间跨度 {span:.2f} s（首帧 {t_first:.3f} → 末帧 {t_last:.3f}）"
          + (f" ／ 录制计划 {planned:.1f} s（覆盖 {100.0 * span / planned:.1f}%）"
             if planned else " ／ 计划时长未知"))
    if planned and span < 0.9 * planned:
        print(f"[audit] 🔴 断流：有帧的部分只占计划的 {100.0 * span / planned:.1f}%")
    print(f"[audit] 每 {bucket:g} s 分桶（n=总帧 / hs=高速六路 / ang=关节角 / unk=未知 ID）")
    print("        桶  时间窗(s)        n      hs     ang     unk")
    for b in sorted(buckets):
        d = buckets[b]
        lo = b * bucket
        hi = min((b + 1) * bucket, span)
        mark = "  ←空" if d["hs"] == 0 else ""
        print(f"    {b:4d}  {lo:6.1f}~{hi:6.1f}  {d['n']:8d} {d['hs']:7d} {d['ang']:7d} {d['unk']:7d}{mark}")
    if holes:
        print(f"[audit] 相邻帧间隔 >100 ms 的空洞 {len(holes)} 处（与 L0 判据同阈值），最大的三处：")
        for off, ms in sorted(holes, key=lambda x: -x[1])[:3]:
            print(f"        在 {off:.2f} s 处空了 {ms / 1000.0:.2f} s")
    else:
        print("[audit] 相邻帧间隔没有 >100 ms 的空洞（帧流是连续的，只是可能提前结束）")

    print(f"[audit] 0x2A8（夹爪角度）：{g_n} 帧，其中载荷非全 0 的 {g_nonzero} 帧"
          + ("  → 全 0：与「本机没装夹爪」一致" if g_n and g_nonzero == 0 else ""))
    if unknown:
        top = sorted(unknown.items(), key=lambda x: -x[1])[:6]
        print("[audit] 未知 ID 前几名：" + "  ".join(f"0x{c:03X}×{n}" for c, n in top))
    return 0


def cmd_parse(args) -> int:
    raw = Path(args.raw)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    qcmd = _load_qcmd(Path(args.qcmd) if args.qcmd else None)

    # 1) 读原始层，按 ID 建时间序列；解码在此进行（原始层不动）
    series: dict = {}
    header = None
    n_raw = 0
    raw_flags = {F_UNKNOWN_ID: 0, F_SHORT_DLC: 0, F_OUT_OF_ORDER: 0, F_ERROR: 0}
    per_raw_id: dict[int, int] = {}
    t_min, t_max = None, None

    with raw.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("kind") == "header":
                header = rec
                continue
            n_raw += 1
            t, cid, data = rec["t"], rec["id"], bytes.fromhex(rec["d"])
            fl = rec.get("f", 0)
            for bit in raw_flags:
                if fl & bit:
                    raw_flags[bit] += 1
            per_raw_id[cid] = per_raw_id.get(cid, 0) + 1
            t_min = t if t_min is None else min(t_min, t)
            t_max = t if t_max is None else max(t_max, t)

            if (fl & F_SHORT_DLC) or len(data) < 8:
                continue                    # 短帧不解码，只计数（已在 raw_flags）
            if cid in HIGH_SPD_IDS:
                series.setdefault(cid, []).append(
                    (t, decode_high_spd(HIGH_SPD_IDS.index(cid) + 1, data, args.pos251_unit)))
            elif cid in JOINT_ANGLE_IDS:
                first = JOINT_ANGLE_IDS.index(cid) * 2 + 1
                for j, v in decode_joint_pair(first, data).items():
                    series.setdefault(("q", j), []).append((t, v))
            elif cid == STATUS_ID:
                series.setdefault(cid, []).append((t, decode_status(data)))
            elif cid in LOW_SPD_IDS:
                series.setdefault(cid, []).append((t, {}))   # 已知但本阶段不解析

    if t_min is None:
        print("[parse] 原始层里没有任何帧", file=sys.stderr)
        return 3

    for k in series:
        series[k].sort(key=lambda r: r[0])

    # 2) 每个 ID 的到达间隔统计（L0 的关键交付之一）
    per_id_stats = {}
    for cid, rows in series.items():
        ts = [r[0] for r in rows]
        dts = [(b - a) * 1000 for a, b in zip(ts, ts[1:])]
        name = _series_name(cid)
        if not dts:
            per_id_stats[name] = {"n": len(ts), "median_ms": None, "p95_ms": None,
                                  "p99_ms": None, "max_ms": None,
                                  "gt40ms": 0, "gt100ms": 0}
            continue
        s = sorted(dts)
        per_id_stats[name] = {
            "n": len(ts),
            "median_ms": round(statistics.median(s), 2),
            "p95_ms": round(s[min(len(s) - 1, int(0.95 * (len(s) - 1)))], 2),
            "p99_ms": round(s[min(len(s) - 1, int(0.99 * (len(s) - 1)))], 2),
            "max_ms": round(s[-1], 2),
            "gt40ms": sum(1 for d in s if d > 40),
            "gt100ms": sum(1 for d in s if d > 100),
        }

    # 3) 按控制周期对齐（第 5 步）
    hz = args.hz
    dt = 1.0 / hz
    stale = args.stale_ms
    ticks = []
    t = t_min
    while t <= t_max + 1e-9:
        ticks.append(t)
        t += dt

    def latest(key, t_now):
        rows = series.get(key) or []
        lo, hi, best = 0, len(rows) - 1, None
        while lo <= hi:
            mid = (lo + hi) // 2
            if rows[mid][0] <= t_now:
                best = rows[mid]
                lo = mid + 1
            else:
                hi = mid - 1
        return best

    n_flag_core = 0      # 有实质问题的样本
    n_flag_all = 0       # 含 no_q_cmd 等提示性标记
    n_with_q = 0
    n_rej = 0
    flag_hist: dict[str, int] = {}
    samples_path = outdir / "samples.jsonl"

    with samples_path.open("w", encoding="utf-8") as fo:
        for t_now in ticks:
            flags: list[str] = []
            q = [None] * 6
            q_rej = {}
            dq, cur = [None] * 6, [None] * 6
            ages = {}

            for j in range(1, 7):
                # 关节角（0x2A5~0x2A7）—— 这是 q 的唯一来源
                hit = latest(("q", j), t_now)
                if hit is None:
                    flags.append(f"missing:q{j}")
                else:
                    age = (t_now - hit[0]) * 1000
                    ages[f"q{j}"] = round(age, 1)
                    if age > stale:
                        flags.append(f"stale:q{j}")
                    elif not (JOINT_MIN[j - 1] - JOINT_LIMIT_TOL
                              <= hit[1] <= JOINT_MAX[j - 1] + JOINT_LIMIT_TOL):
                        # 越界不写进 q：宁可留 null，也不让饱和值混进训练字段
                        flags.append(f"oob:q{j}")
                        q_rej[f"q{j}"] = round(hit[1], 6)
                    else:
                        q[j - 1] = hit[1]

                # 高速反馈（0x251~0x256）—— 提供 dq / cur
                hit = latest(HIGH_SPD_IDS[j - 1], t_now)
                if hit is None:
                    # 1.0 版在这里什么都不打：整路 ID 一帧都没收到时静默通过。
                    # 这是最危险的漏洞——总线只通了一半也会显示「六轴齐备」。
                    flags.append(f"missing:hs{j}")
                else:
                    age = (t_now - hit[0]) * 1000
                    ages[f"hs{j}"] = round(age, 1)
                    if age > stale:
                        flags.append(f"stale:hs{j}")
                    else:
                        dq[j - 1] = hit[1]["dq"]
                        cur[j - 1] = hit[1]["cur"]
                        if abs(hit[1]["cur"]) >= 32.0:
                            # 32 A 远超本机任何合理电流（24 V/2 A 末端接口、本体 4.66 kg），
                            # 超过即说明该路反馈不可信，别把它当负载信号用
                            flags.append(f"saturated:cur{j}")

            if all(v is not None for v in q):
                n_with_q += 1
            n_rej += len(q_rej)

            # q_cmd：第 4 步，绝不拿 q 顶替
            if qcmd is None:
                q_cmd, qc_age = None, None
                flags.append("no_q_cmd")
            else:
                q_cmd, qc_age = _qcmd_at(qcmd, t_now)
                if q_cmd is None:
                    flags.append("cmd_interrupted")
                elif qc_age is not None and qc_age * 1000 > stale:
                    # 落在指令窗之外（录制先起、指令后停）不算「指令中断」：那一段
                    # 本来就没有指令。把它当实质标记，会拿基线段把整份报告染红 ——
                    # 15:26 那次 649/2600 里绝大多数就是这种，看着像出大事，其实没有。
                    if qcmd and (t_now < qcmd[0][0] or t_now > qcmd[-1][0]):
                        flags.append("qcmd_out_of_window")
                    else:
                        flags.append("stale:q_cmd")

            st = latest(STATUS_ID, t_now)
            if st is None:
                arm_status = None
                flags.append("missing:status")
            else:
                arm_status = st[1].get("arm_status")
                if arm_status:
                    flags.append(f"protection:0x{arm_status:02X}")

            span = None
            hs_ages = [ages.get(f"hs{j}") for j in range(1, 7)]
            if all(a is not None for a in hs_ages):
                span = round(max(hs_ages) - min(hs_ages), 1)

            core = [x for x in flags if x not in BENIGN_FLAGS]
            if core:
                n_flag_core += 1
            if flags:
                n_flag_all += 1
            for x in flags:
                flag_hist[x] = flag_hist.get(x, 0) + 1

            rec = {
                "t": round(t_now - t_min, 4),
                "q": [None if v is None else round(v, 6) for v in q],
                "dq": dq, "cur": cur,
                "q_cmd": None if q_cmd is None else [round(v, 6) for v in q_cmd],
                "age_ms": ages, "span_ms": span,
                "arm_status": arm_status,
                "flags": sorted(set(flags)),
            }
            if q_rej:
                rec["q_rejected"] = q_rej
            fo.write(json.dumps(rec, ensure_ascii=False, separators=(",", ":")) + "\n")

    # 4) 0x251 位置单位交叉核验（R12：rad 还是 0.001 rad）
    #    前提是这一路位置字段真的有非零内容。若全为 0，那是「未标定」，不是「单位错」——
    #    两者混淆会把工具自己的问题报成硬件问题，所以分开判。
    ratio = None
    pos_zero = False
    nz_frac = None
    if ("q", 1) in series and 0x251 in series:
        vals = []
        for t_q, v_q in series[("q", 1)][:200]:
            hit = latest(0x251, t_q)
            if hit is None or abs(v_q) <= 0.05:
                continue
            vals.append((hit[1]["pos"], v_q))
        if vals:
            nz = [abs(p / v) for p, v in vals if p != 0]
            nz_frac = round(len(nz) / len(vals), 3)
            pos_zero = not nz
            if len(nz) >= 10:                      # 太少就不下结论
                ratio = round(statistics.median(nz), 3)
    unit_suspect = ratio is not None and (ratio > 100 or ratio < 0.01)

    # 5) 机读判据 + 人读报告
    total = len(ticks)
    hs_stats = [s for n, s in per_id_stats.items() if n.startswith("0x25")]
    # 节拍判据不能写死「中位数 20 ms」。指南说 50 Hz，这台臂实测 200 Hz（5.00 ms），
    # 两者都是正常总线；写死一个值会把另一种正常情况判成失败，反过来也会把
    # 「一半通道 5 ms、一半 20 ms」这种真异常放过。所以：先按中位数判定落在哪个
    # 已知合法档位，再要求**六路都在同一档**，并把测到的档位写进报告。
    CYCLE_BANDS = (("200Hz", 4.0, 6.0), ("50Hz", 15.0, 25.0))
    _meds = [s["median_ms"] for s in hs_stats if s.get("median_ms")]
    _med_med = statistics.median(_meds) if _meds else None
    detected_rate, _band = None, None
    if _med_med is not None:
        for _name, _lo, _hi in CYCLE_BANDS:
            if _lo <= _med_med <= _hi:
                detected_rate, _band = _name, (_lo, _hi)
                break
    ok_cycle = (bool(hs_stats) and _band is not None
                and all(_band[0] <= (s.get("median_ms") or 0) <= _band[1] for s in hs_stats))
    ok_gap = bool(hs_stats) and all((s.get("gt100ms") or 0) == 0 for s in hs_stats)
    ok_rate = n_with_q / max(total, 1) >= 0.99

    # 覆盖判据：原始层的时间跨度是否等于录制计划时长。
    # 没有这一条，「断流」会被误判成「全过」——因为断流之后根本没有帧，
    # 也就不存在「>100 ms 的间隔」被计数。这是最隐蔽的假通过：
    # 数据在 52 s 处停住，报告却写「未发现 >100 ms 间隔：是」。
    span_s = t_max - t_min
    planned = None
    try:
        _p = (header or {}).get("planned_seconds")
        planned = float(_p) if _p is not None else None
    except (TypeError, ValueError):
        planned = None
    ok_cover = (planned is None) or (span_s >= 0.9 * planned)

    metrics = {
        "parser_version": PARSER_VERSION,
        "raw": str(raw),
        "samples": str(samples_path),
        "hz": hz,
        "stale_ms": stale,
        "n_raw_frames": n_raw,
        "raw_flags": {f"0x{k:02X}": v for k, v in raw_flags.items()},
        "n_samples": total,
        "n_q_complete": n_with_q,
        "pc_q_complete": round(100.0 * n_with_q / max(total, 1), 3),
        "n_flagged_core": n_flag_core,
        "pc_flagged_core": round(100.0 * n_flag_core / max(total, 1), 3),
        "n_flagged_all": n_flag_all,
        "n_q_rejected": n_rej,
        "flag_hist": dict(sorted(flag_hist.items())),
        "pos251_unit": args.pos251_unit,
        "ratio_251_over_2a5": ratio,
        "pos251_unit_suspect": unit_suspect,
        "pos251_nonzero_frac": nz_frac,
        "pos251_all_zero": pos_zero,
        "per_id": per_id_stats,
        "detected_hs_rate": detected_rate,
        "hs_median_of_medians_ms": _med_med,
        "record_span_s": round(span_s, 3),
        "planned_span_s": planned,
        "verdict": {"cycle_ok": ok_cycle, "no_gap_gt100ms": ok_gap,
                    "q_rate_ge_99pc": ok_rate, "data_covers_window": ok_cover},
    }
    (outdir / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    rep = []
    rep.append("# L0 同步报告\n")
    rep.append(f"- 解析器：`{PARSER_VERSION}`")
    rep.append(f"- 原始层：`{raw}`（{n_raw} 帧，来源 {header.get('source') if header else '未知'}）")
    rep.append(f"- 采样层：`{samples_path}`（{total} 个样本 @ {hz} Hz）")
    rep.append(f"- 时间窗：{span_s:.3f} s"
               + (f"（录制计划 {planned:.1f} s，有帧覆盖 {100.0 * span_s / planned:.1f}%）"
                  if planned else "（原始层未记录计划时长，无法核验覆盖）"))
    if not ok_cover:
        rep.append(f"- 🔴 **断流**：原始层只覆盖 {span_s:.1f} s，而录制跑了 {planned:.1f} s，"
                   f"末 {planned - span_s:.1f} s 一帧都没有。下面的到达间隔统计**只覆盖有帧那段**，"
                   f"因此「未发现 >100 ms 间隔」不能当作通过 —— 断流本身就是一个更严重的问题")
    # 三位小数：2 位会把 32999/33000 = 99.997% 印成 100.00%，
    # 看起来比实际好 —— 报告里的数字宁丑勿美。
    rep.append(f"- 六轴齐备样本：{n_with_q}/{total} = {100.0 * n_with_q / max(total, 1):.3f}%")
    rep.append(f"- 带实质标记样本：{n_flag_core}/{total} = "
               f"{100.0 * n_flag_core / max(total, 1):.3f}%"
               f"（另有 {n_flag_all - n_flag_core} 个仅带提示标记 no_q_cmd）")
    rep.append(f"- 被拒绝的越界值：{n_rej} 个（只进 `q_rejected`，不进 `q`）")
    rep.append(f"- q_cmd 来源：{'未提供（全部标 no_q_cmd）' if qcmd is None else args.qcmd}")
    if pos_zero:
        rep.append(f"- 0x251 位置字段全为 0（非零占比 {nz_frac}）：该路位置未标定或未启用，"
                   f"单位无法交叉核验，`q` 仍只取 0x2A5~0x2A7")
    else:
        rep.append(f"- 0x251 位置单位：`--pos251-unit {args.pos251_unit}`"
                   + (f"，与 0x2A5 实测比值 = {ratio}（1 ≈ rad；1000 ≈ 0.001 rad）" if ratio
                      else "，样本太少无法交叉核验"))
    if unit_suspect:
        rep.append(f"- ⚠️ 比值 {ratio} 与声明单位不符：改用 "
                   f"`--pos251-unit {'mrad' if ratio > 100 else 'rad'}` 重解析")
    if header and header.get("v") and header["v"] != RAW_VERSION:
        rep.append(f"- ⚠️ 原始层版本 {header['v']} 与当前解析器 {RAW_VERSION} 不一致")

    rep.append("\n## 各序列到达间隔\n")
    rep.append("| 序列 | n | 中位数 ms | p95 | p99 | 最大 | >40ms | >100ms |")
    rep.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for name, s in sorted(per_id_stats.items()):
        rep.append(f"| {name} | {s['n']} | {s.get('median_ms')} | {s.get('p95_ms')} "
                   f"| {s.get('p99_ms')} | {s.get('max_ms')} | {s.get('gt40ms')} | {s.get('gt100ms')} |")

    # 单次长停顿的旁证。即使没触及 100 ms 判据，也把「最大间隔 vs 中位数」说出来：
    # 六路最大值彼此接近 → 同一时刻所有通道一起拉长 → 主机/适配器侧调度抖动，
    # 不是单路丢帧。这也正好解释「带实质标记的样本」——那一刻六个通道都没有新帧。
    _hs = [(n, s.get("max_ms") or 0, s.get("median_ms") or 0)
           for n, s in per_id_stats.items() if n.startswith("0x25")]
    if _hs:
        _mx = max(_hs, key=lambda x: x[1])
        if _mx[2] and _mx[1] >= 4 * _mx[2]:
            _all = [x[1] for x in _hs]
            _viol = sum((s.get("gt100ms") or 0) for n, s in per_id_stats.items()
                        if n.startswith("0x25"))
            _tail = ("未触及 100 ms 判据，但 L1 的时序对齐要知道有这种量级的抖动。"
                     if _viol == 0 else
                     f"**已触及 100 ms 判据（六路合计 {_viol} 次）**，这条不能只当抖动。")
            rep.append(f"\n> **单次停顿**：{_mx[0]} 最大间隔 {_mx[1]} ms（中位数 {_mx[2]} ms，"
                       f"约 {_mx[1] / _mx[2]:.1f}×）。六路最大值落在 "
                       f"{min(_all)}~{max(_all)} ms 的窄带里 → 同一次事件把六路一起拉长，"
                       f"更像主机/适配器侧调度抖动，不是某一路在丢帧。{_tail}")

    if flag_hist:
        rep.append("\n## 标记分布\n")
        rep.append("| 标记 | 样本数 |")
        rep.append("| --- | --- |")
        for k, v in sorted(flag_hist.items(), key=lambda kv: -kv[1]):
            rep.append(f"| `{k}` | {v} |")

    rep.append("\n## 判据与结论\n")
    rep.append(f"- 数据覆盖整个录制窗口：{'是' if ok_cover else '**否（断流，下面三条都不作数）**'}")
    rep.append(f"- 六路高速反馈同档且落在合法档位：{'是' if ok_cycle else '否'}"
               f"（判定档位 = {detected_rate or '未落入任何已知档位'}"
               f"，六路周期中位数 {_med_med if _med_med is not None else 'n/a'} ms；"
               f"合法档位 200Hz=4~6ms / 50Hz=15~25ms）")
    rep.append(f"- 未发现 >100 ms 间隔：{'是' if ok_gap else '否'}")
    rep.append(f"- 有效样本率 ≥99%：{'是' if ok_rate else '否'}")
    rep.append("- 口径提醒：「零丢帧」在无帧序号时不可证明，只能写「未发现超过判据的间隔异常」。")
    rep.append("- 口径提醒：`q` 只来自 0x2A5~0x2A7；0x251 的 `pos` 仅用于交叉核验，不参与 `q`。")
    (outdir / "sync_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")

    print(f"[parse] 样本 {total} 个（帧跨度 {span_s:.1f}s"
          + (f" / 计划 {planned:.1f}s" if planned else "")
          + f"），六轴齐备 {n_with_q}，带实质标记 {n_flag_core}")
    if not ok_cover:
        print(f"[parse][WARN] 断流：原始层只覆盖 {span_s:.1f}s，录制计划 {planned:.1f}s。"
              f"后面的间隔统计只覆盖有帧那段，「无 >100 ms 空档」不成立。", file=sys.stderr)
    print(f"[parse] 报告 → {outdir / 'sync_report.md'}，判据 → {outdir / 'metrics.json'}")
    if ratio:
        print(f"[parse] 0x251 位置 / 0x2A5 角度 比值 = {ratio}（1→rad，1000→0.001 rad）")
    if pos_zero:
        print("[parse] 0x251 位置字段全为 0：该路未标定，单位未做交叉核验（这不影响 q）")
    if unit_suspect:
        # 单位选错时 samples.jsonl 里的 pos 会整列错 1000 倍。虽然 q 不取 pos，
        # 但下游只要有人碰 pos 就会中招，所以这里不当警告、直接判本次解析失败。
        _want = "mrad" if ratio > 100 else "rad"
        print(f"[parse] ✗ 比值 {ratio} 与 --pos251-unit {args.pos251_unit} 不符：本次解析判失败",
              file=sys.stderr)
        print(f"[parse]   修正后重跑（会覆盖本次产物）：", file=sys.stderr)
        print(f"[parse]   python stage1_l0.py parse {raw} --hz {hz:g} --pos251-unit {_want} "
              f"--out {outdir}", file=sys.stderr)
        return 2
    return 0


# ---------------------------------------------------------------- 跟踪判读（L0-b）


def _rmse_lag(q, qc, k):
    """q[i] 对 qc[i+k] 的 RMSE。k 为负 ⇒ 比的是「更早的指令」，即 q 滞后。"""
    n, acc = 0, 0.0
    for i in range(len(q)):
        j = i + k
        if j < 0 or j >= len(qc):
            continue
        a, b = q[i], qc[j]
        if a is None or b is None:
            continue
        acc += (a - b) ** 2
        n += 1
    if n < 10:
        return None
    return math.sqrt(acc / n)


def cmd_track(args) -> int:
    """L0-b 的判读器：`q_cmd` 到底有没有变成 `q`。

    这一步存在的理由很单纯：L1 的输入里有一路是 q_cmd，如果指令根本没生效，
    策略会学到 `q_obs → q_obs` 的恒等映射 —— loss 漂亮、机器人不听指令。
    `parse` 能把两者对齐并落进同一行，但「对齐了」不等于「跟上了」，
    所以必须再有一道独立的量测：响应比、最佳滞后、激励是否足够。
    """
    src = Path(args.samples)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    rows = [json.loads(s) for s in src.read_text(encoding="utf-8").splitlines() if s.strip()]
    if len(rows) < 20:
        print(f"[track] 样本太少（{len(rows)} 个），无法判读", file=sys.stderr)
        return 2
    if not any(r.get("q_cmd") for r in rows):
        print("[track] samples.jsonl 里 q_cmd 全为 null —— parse 时必须带 --qcmd <下发的 CSV>，"
              "否则没有可比对象", file=sys.stderr)
        return 2

    t = [r["t"] for r in rows]
    steps = [b - a for a, b in zip(t, t[1:])]
    dt = statistics.median(steps) if steps else 0.02
    max_lag = max(int(round(args.max_lag_ms / 1000.0 / dt)), 1)

    names = [f"joint_{i}" for i in range(1, 7)]
    per = {}

    def _p2p_deg(vs):
        return (max(vs) - min(vs)) * 180.0 / math.pi

    for j in range(6):
        q = [(r["q"][j] if r.get("q") else None) for r in rows]
        qc = [(r["q_cmd"][j] if r.get("q_cmd") else None) for r in rows]
        qv = [v for v in q if v is not None]
        qcv = [v for v in qc if v is not None]
        if len(qv) < 10 or len(qcv) < 10:
            per[names[j]] = {"state": "样本不足", "n": min(len(qv), len(qcv))}
            continue

        q_std = statistics.pstdev(qv)
        qc_std = statistics.pstdev(qcv)
        p2p_cmd = _p2p_deg(qcv)
        p2p_q = _p2p_deg(qv)

        rmse0 = _rmse_lag(q, qc, 0)
        best_k, best = 0, rmse0
        for k in range(-max_lag, max_lag + 1):
            r = _rmse_lag(q, qc, k)
            if r is not None and (best is None or r < best):
                best_k, best = k, r
        delay_ms = -best_k * dt * 1000.0
        ratio = (q_std / qc_std) if qc_std > 1e-12 else None

        # 指令几乎不动时，任何比值都没有意义 —— 先判"激励不足"，不判"没跟随"。
        if p2p_cmd < args.min_excite_deg:
            state = "未激励"
        elif ratio is None:
            state = "无法判定"
        elif ratio < 0.2:
            state = "未跟随（R19 嫌疑）"
        elif ratio > 2.0:
            state = "幅度异常（明显大于指令）"
        else:
            state = "跟随"

        # dq 量纲旁证：实测 |dq| vs 由 q_cmd 数值微分得到的 |dq|。
        # 峰值不可靠：0x251 的转速字段是逐帧的瞬时量，单点毛刺就能把峰值抬高几倍
        # （15:26 那次 J6 得到 5.49，而响应比只有 0.87，两个数字互相矛盾）。
        # 所以**以 RMS 为准**，峰值只留作参考并明确标注它对毛刺敏感。
        dqv = [v for v in ([(r["dq"][j] if r.get("dq") else None) for r in rows]) if v is not None]
        dqc = [((qc[i] - qc[i - 1]) / dt) for i in range(1, len(qc))
               if qc[i] is not None and qc[i - 1] is not None]
        peak_dq = max(abs(v) for v in dqv) if dqv else None
        peak_dqc = max(abs(v) for v in dqc) if dqc else None
        rms_dq = (math.sqrt(sum(v * v for v in dqv) / len(dqv))
                  if dqv else None)
        rms_dqc = (math.sqrt(sum(v * v for v in dqc) / len(dqc))
                   if dqc else None)

        per[names[j]] = {
            "state": state,
            "n_pairs": len(qv),
            "p2p_cmd_deg": round(p2p_cmd, 4),
            "p2p_q_deg": round(p2p_q, 4),
            "resp_ratio": None if ratio is None else round(ratio, 4),
            "delay_ms": round(delay_ms, 1),
            "rmse0": None if rmse0 is None else round(rmse0, 6),
            "rmse_best": None if best is None else round(best, 6),
            "peak_dq": None if peak_dq is None else round(peak_dq, 6),
            "peak_dqc": None if peak_dqc is None else round(peak_dqc, 6),
            "dq_ratio": (None if (peak_dq is None or not peak_dqc) else round(peak_dq / peak_dqc, 4)),
            "rms_dq": None if rms_dq is None else round(rms_dq, 6),
            "rms_dqc": None if rms_dqc is None else round(rms_dqc, 6),
            "dq_ratio_rms": (None if (rms_dq is None or not rms_dqc)
                             else round(rms_dq / rms_dqc, 4)),
        }

    followed = [k for k, v in per.items() if v.get("state") == "跟随"]
    flat = [k for k, v in per.items() if v.get("state") == "未跟随（R19 嫌疑）"]
    unexcited = [k for k, v in per.items() if v.get("state") == "未激励"]

    if flat:
        r19 = (f"**指令未生效**：{', '.join(flat)} 有指令但几乎不跟随。"
               f"先查 ctrl_mode 是否 CAN_CTRL(0x1)、使能、以及是否有物理阻挡；"
               f"在查清之前，这批数据不能进 L1")
    elif not followed:
        r19 = ("没有关节同时满足「有激励」且「跟随」—— 这份数据不能用于判 R19，"
               "要么幅值太小，要么时长太短")
    else:
        r19 = (f"**指令生效**：{', '.join(followed)} 跟随正常"
               + (f"；{', '.join(unexcited)} 未激励（指令几乎不动，本轮不判）" if unexcited else ""))

    out = {
        "samples": str(src),
        "dt": dt,
        "max_lag_ms": args.max_lag_ms,
        "min_excite_deg": args.min_excite_deg,
        "per_joint": per,
        "verdict": {"followed": followed, "not_following": flat, "unexcited": unexcited,
                    "r19": r19},
    }
    (outdir / "track.json").write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n",
                                       encoding="utf-8")

    rep = ["# L0-b 跟踪判读\n"]
    rep.append(f"- 样本：`{src}`（{len(rows)} 行 @ {1.0 / dt:.1f} Hz）")
    rep.append(f"- 滞后搜索范围：±{args.max_lag_ms:g} ms（{max_lag} 个采样点）")
    rep.append(f"- 激励门限：`q_cmd` 峰峰值 ≥ {args.min_excite_deg:g}° 才判「跟随」\n")
    rep.append("| 关节 | q_cmd 峰峰° | q 峰峰° | 响应比 | 最佳滞后 ms | RMSE@0 | RMSE@best | dq/∂q_cmd(RMS) | 峰值比(参考) | 结论 |")
    rep.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for n in names:
        v = per.get(n, {})
        if v.get("state") == "样本不足":
            rep.append(f"| {n} | — | — | — | — | — | — | — | — | 样本不足 |")
            continue
        rep.append(f"| {n} | {v['p2p_cmd_deg']} | {v['p2p_q_deg']} | {v['resp_ratio']} "
                   f"| {v['delay_ms']} | {v['rmse0']} | {v['rmse_best']} | {v.get('dq_ratio_rms')} "
                   f"| {v['dq_ratio']} | {v['state']} |")
    rep.append(f"\n## 判据与结论\n")
    rep.append(f"- R19（指令是否真的驱动了臂）：{r19}")
    rep.append("- 响应比 = std(q)/std(q_cmd)：≈1 说明幅度跟得上；<0.2 基本等于没动。")
    rep.append("- 最佳滞后 = 让 RMSE 最小的 q_cmd 时间平移。**这是实测延迟，不是猜的**，"
               "L1 的时序对齐要用它。")
    rep.append("- **dq/∂q_cmd(RMS)**：实测 `0x251` 转速 RMS ÷ 由 q_cmd 数值微分得到的 RMS。"
               "≈1 说明 0.001 rad/s 这个量纲对；差一个量级说明量纲选错（R29 相关）。")
    rep.append("- 峰值比(参考)：同一条链但用峰值。**不可作判据** —— `0x251` 的转速是逐帧瞬时量，"
               "单点毛刺能把峰值抬高数倍（15:26 那次 J6 峰值比 5.49，而响应比只有 0.87，两者矛盾）。")
    (outdir / "track_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")

    for n in names:
        v = per.get(n, {})
        if v.get("state") == "样本不足":
            print(f"[track] {n}: 样本不足")
            continue
        print(f"[track] {n}: {v['state']}  指令峰峰 {v['p2p_cmd_deg']}° / "
              f"实测峰峰 {v['p2p_q_deg']}°  响应比 {v['resp_ratio']}  "
              f"滞后 {v['delay_ms']} ms  dq比(RMS) {v.get('dq_ratio_rms')}")
    print(f"[track] 报告 → {outdir / 'track_report.md'}，判据 → {outdir / 'track.json'}")
    print(f"[track] R19：{r19}")
    return 0


# ---------------------------------------------------------------- 自测

def selftest() -> int:
    import tempfile

    fails = []
    tmp = Path(tempfile.mkdtemp(prefix="l0selftest_"))

    def run_record(mode, seconds, name):
        out = tmp / f"{name}.jsonl"
        args = argparse.Namespace(out=str(out), synthetic=mode, replay=None,
                                  iface=None, backend="socketcan", seconds=seconds)
        rc = cmd_record(args)
        return out, rc

    def parse(src, name, pos_unit="rad", qcmd=None, stale_ms=60):
        outdir = tmp / name
        args = argparse.Namespace(raw=str(src), out=str(outdir), hz=50.0,
                                  pos251_unit=pos_unit, qcmd=qcmd, stale_ms=stale_ms)
        rc = cmd_parse(args)
        return outdir, rc

    def load_jsonl(p):
        """读记录文件，跳过 header 行（header 没有 t/id/d）。"""
        out = []
        for line in p.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            rec = json.loads(line)
            if rec.get("kind"):
                continue
            out.append(rec)
        return out

    def load_metrics(outdir):
        return json.loads((outdir / "metrics.json").read_text(encoding="utf-8"))

    # A 正常流：核心标记必须为空（no_q_cmd 不算脏）
    src, rc = run_record("normal", 2.0, "A")
    outdir, rc2 = parse(src, "A_out")
    samples = load_jsonl(outdir / "samples.jsonl")
    if rc or rc2:
        fails.append(f"A: exit {rc}/{rc2}")
    if not samples:
        fails.append("A: 没有样本")
    else:
        dirty = [s for s in samples
                 if [f for f in s["flags"] if f not in BENIGN_FLAGS]]
        if len(dirty) / len(samples) > 0.10:
            fails.append(f"A: 干净样本占比过低 {len(samples) - len(dirty)}/{len(samples)}"
                         f"（首个脏样本 {dirty[0]['flags'] if dirty else '-'}）")
        if any(s["q_cmd"] is not None for s in samples):
            fails.append("A: 没给 qcmd 却出现了 q_cmd（不允许用 q 顶替）")
        if not all("no_q_cmd" in s["flags"] for s in samples):
            fails.append("A: 缺 q_cmd 时没有打 no_q_cmd 标记")
        if not all(all(v is not None for v in s["q"]) for s in samples):
            fails.append("A: 正常流里有样本缺轴")

    # B 缺一个关节（0x255 + 承载它的 0x2A7）
    src, _ = run_record("missing_joint", 2.0, "B")
    outdir, _ = parse(src, "B_out")
    samples = load_jsonl(outdir / "samples.jsonl")
    if not samples:
        fails.append("B: 没有样本")
    else:
        if not all(any(f in (f"missing:q5", "stale:q5") for f in s["flags"]) for s in samples):
            fails.append("B: 缺 0x2A7 时 q5 没有被标记")
        if any(s["q"][4] is not None for s in samples):
            fails.append("B: 缺帧时 q5 被填了值（必须保持 null）")
        if not all(any(f == "missing:hs5" for f in s["flags"]) for s in samples):
            fails.append("B: 缺 0x255 时高速通道没有被标记（静默漏洞）")

    # C 乱序时间戳：原始层必须标记，且不排序
    src, _ = run_record("out_of_order", 2.0, "C")
    hdr = load_jsonl(src)
    if not any(r["f"] & F_OUT_OF_ORDER for r in hdr):
        fails.append("C: 时间戳倒退没有被原始层标记")
    if [r["t"] for r in hdr] == sorted(r["t"] for r in hdr):
        fails.append("C: 原始层被隐式排序了（必须保持到达顺序）")

    # D 饱和：值被拒收进 q_rejected，q 保持 null，并打 oob
    src, _ = run_record("saturated", 1.0, "D")
    outdir, _ = parse(src, "D_out")
    samples = load_jsonl(outdir / "samples.jsonl")
    if not samples:
        fails.append("D: 没有样本")
    else:
        if any(s["q"][2] is not None and abs(s["q"][2]) > 1.0 for s in samples):
            fails.append("D: 饱和值被当成真实角度放过去了")
        if not any(any(f == "oob:q3" for f in s["flags"]) for s in samples):
            fails.append("D: 饱和/超限没有打 oob 标记")
        if not any("q_rejected" in s and "q3" in s["q_rejected"] for s in samples):
            fails.append("D: 越界值没有留在 q_rejected 里（无法事后诊断）")

    # E 短帧
    src, _ = run_record("short_dlc", 1.0, "E")
    recs = load_jsonl(src)
    if not any(r["f"] & F_SHORT_DLC for r in recs):
        fails.append("E: 短帧没有被原始层标记")

    # F 单位判据双向验证：同源数据在两套单位下比值应当是 1 和 ≈1000
    src, _ = run_record("pos_mrad", 2.0, "F")
    out_m, _ = parse(src, "F_mrad", pos_unit="mrad")
    m_m = load_metrics(out_m)
    if not m_m["ratio_251_over_2a5"] or abs(m_m["ratio_251_over_2a5"] - 1.0) > 0.05:
        fails.append(f"F: 声明 mrad 时比值应≈1，实得 {m_m['ratio_251_over_2a5']}")
    if m_m["pos251_unit_suspect"]:
        fails.append("F: 声明 mrad 时不应报单位可疑")

    out_r, _ = parse(src, "F_rad", pos_unit="rad")
    m_r = load_metrics(out_r)
    if not m_r["ratio_251_over_2a5"] or abs(m_r["ratio_251_over_2a5"] - 1000) > 20:
        fails.append(f"F: 0.001 rad 数据按 rad 解析，比值应≈1000，实得 {m_r['ratio_251_over_2a5']}")
    if not m_r["pos251_unit_suspect"]:
        fails.append("F: 单位明显不符却没有报可疑")

    # G 大间隔：1 s 处断 150 ms
    src, _ = run_record("gap", 2.0, "G")
    outdir, _ = parse(src, "G_out")
    rep = (outdir / "sync_report.md").read_text(encoding="utf-8")
    if "未发现 >100 ms 间隔：否" not in rep:
        fails.append("G: 150 ms 空档没有被判据抓到")

    # H 给了 q_cmd：非 null、年龄小、不被误判 stale
    hdr = load_jsonl(src)
    t0 = next(r["t"] for r in hdr if r["id"] == STATUS_ID)
    qc = tmp / "cmd.csv"
    qc.write_text("\n".join(f"{t0 + i * 0.02:.6f},0.1,1.2,-1.3,0.2,0.3,-0.4"
                            for i in range(120)), encoding="utf-8")
    outdir, _ = parse(src, "H_out", qcmd=qc)
    samples = load_jsonl(outdir / "samples.jsonl")
    if not samples:
        fails.append("H: 没有样本")
    elif not any(s["q_cmd"] for s in samples):
        fails.append("H: 给了 q_cmd 却全是 null")
    elif any("stale:q_cmd" in s["flags"] for s in samples):
        fails.append("H: q_cmd 与原始层同一时间基准却被判 stale")

    print()
    if fails:
        print(f"[selftest] FAIL ×{len(fails)}")
        for f_ in fails:
            print("  -", f_)
        print(f"[selftest] 产物保留在 {tmp}")
        return 1
    print(f"[selftest] PASS 8/8：正常流 / 缺关节 / 乱序 / 饱和 / 短帧 / 单位偏差 / 大间隔 / q_cmd")
    print(f"[selftest] 产物：{tmp}")
    return 0


# ---------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description="L0 信号链：原始层 + 解析层 + 自测")
    ap.add_argument("--version", action="version",
                    version=f"stage1_l0 解析器 {PARSER_VERSION} / 原始层 {RAW_VERSION}")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("record", help="原始层落盘（只追加，不解析）")
    r.add_argument("--iface", default="can0")
    r.add_argument("--backend", default="socketcan")
    r.add_argument("--seconds", type=float, default=600.0)
    r.add_argument("--synthetic", choices=["normal", "missing_joint", "out_of_order",
                                           "saturated", "short_dlc", "pos_mrad", "gap"],
                   help="用合成帧代替硬件（自测/联调用）")
    r.add_argument("--replay", help="回放已落盘的原始层文件")
    r.add_argument("--out", required=True)

    p = sub.add_parser("parse", help="解析 + 单位统一 + 时间对齐")
    p.add_argument("raw")
    p.add_argument("--out", required=True)
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--pos251-unit", choices=["rad", "mrad"], default="rad",
                   help="0x251 位置单位：rad（指南）或 mrad（=0.001 rad，FIRST_ARM 实测口径）")
    p.add_argument("--qcmd", help="实际下发的参考位置 CSV（t,j1..j6，epoch 秒），缺省则全部标 no_q_cmd")
    p.add_argument("--stale-ms", type=float, default=60.0)

    tr = sub.add_parser("track", help="q 与 q_cmd 的跟踪判读（L0-b，判 R19）")
    tr.add_argument("samples", help="parse 产出的 episodes/*/samples.jsonl（须带 q_cmd）")
    tr.add_argument("--out", required=True, help="报告输出目录")
    tr.add_argument("--max-lag-ms", type=float, default=400.0,
                    help="滞后搜索范围，默认 ±400 ms")
    tr.add_argument("--min-excite-deg", type=float, default=0.5,
                    help="q_cmd 峰峰值低于此值判「未激励」，不判「没跟随」，默认 0.5°")

    sub.add_parser("selftest", help="合成帧自测（不需要硬件）")

    au = sub.add_parser("audit", help="体检原始层：帧覆盖了哪段时间、哪里空了、0x2A8 有没有内容")
    au.add_argument("raw")
    au.add_argument("--bucket", type=float, default=10.0, help="分桶宽度 s，默认 10")

    args = ap.parse_args()
    if args.cmd == "record":
        return cmd_record(args)
    if args.cmd == "parse":
        return cmd_parse(args)
    if args.cmd == "track":
        return cmd_track(args)
    if args.cmd == "audit":
        return cmd_audit(args)
    return selftest()


if __name__ == "__main__":
    raise SystemExit(main())
