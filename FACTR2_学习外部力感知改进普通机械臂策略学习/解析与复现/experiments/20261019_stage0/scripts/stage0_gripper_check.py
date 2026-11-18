#!/usr/bin/env python3
"""stage0_gripper_check.py -- 夹爪（第 7 电机）识别与致动核验

在【机器人机 / VM 上】运行（Ubuntu + python3.12 + python-can + socketcan can0）。
Windows 侧不能跑（没有 can0），只能跑 `--selftest`。

为什么需要这个脚本
------------------
装夹爪前实测：`0x2A8` 每 5 ms 就来一帧、200 Hz **照常上报，只是载荷全 0**。
所以「总线上有 0x2A8」**不能**当成「夹爪被识别」—— 无夹爪时也有帧。
本脚本把判据拆成四级，只有 C2/C3 才是识别证据：

    C1  0x2A8 帧率 ≈200 Hz        → 通道在。**不能区分有无夹爪**
    C2  status_code bit6 = 1       → 夹爪驱动器已使能   ← 识别证据
    C3  angle 随指令变化           → 机械真的在动       ← 决定性证据
    C4  effort 夹持时非 0          → 有力反馈           ← 旁证

三条硬规矩（与本项目其它工具一致）
----------------------------------
1. **不发没有回读确认的动作**：致动前先读、致动中逐帧判发送成败。
2. **`piper_sdk` 的 `GripperCtrl()` 和 `JointCtrl()` 一样返回 `None`**，发送失败只在
   SDK 自己 logger 里留一行。所以本脚本自建 socket 发 0x159，`send()` 抛异常才算失败，
   **失败的拍不推进目标轨迹**（否则下一拍会跳两拍，行程被悄悄拉长）。
3. **不做自动回零**。`set_zero=0xAE` 会把当前位置定义成 0 点、破坏行程标定，
   脚本不提供该动作，也不要手工发。

用法
----
    python3 stage0_gripper_check.py --version
    python3 stage0_gripper_check.py selftest                 # 离线，任何机器都能跑
    python3 stage0_gripper_check.py read --sec 3             # 只读，不动电机
    python3 stage0_gripper_check.py read --sdk               # 用 SDK 读，验证解析链
    python3 stage0_gripper_check.py enable                   # 只发夹爪使能帧（0x159 code=0x01）
    python3 stage0_gripper_check.py actuate --span 10000     # 小幅开合 ±10 mm

产物：`env/gripper_check.json`（读）、`runs/gripper_actuate.csv`（致动）。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import statistics as st
import sys
import time
from pathlib import Path

VERSION = "1.1-20261019"

# ---------------------------------------------------------------- 协议常量
#
# 依据 piper_sdk 0.6.1 源码（不是手册转述）：
#   piper_sdk/piper_msgs/msg_v2/can_id.py:25      ARM_GRIPPER_FEEDBACK = 0x2A8
#   piper_sdk/piper_msgs/msg_v2/can_id.py:42      ARM_GRIPPER_CTRL     = 0x159
#   .../feedback/arm_feedback_gripper.py:20-35    字段布局与单位
#   .../transmit/arm_gripper_ctrl.py:29-40        控制帧布局与单位

GRIPPER_FEEDBACK = 0x2A8
GRIPPER_CTRL = 0x159
MOTOR_ENABLE = 0x471

STROKE_MIN_MM = 0.0        # 夹爪开口行程 0~70 mm（官方参数表 line 78）
STROKE_MAX_MM = 70.0
UNIT_MM = 0.001            # 0x2A8 / 0x159 行程单位 0.001 mm

# status_code 8 位。**前 6 位的 1 是"坏"，后 2 位的 1 是"好"** —— 方向反了会把
# 「驱动器没使能」读成「一切正常」，所以这里显式带上 polarity。
STATUS_BITS = [
    (0, "电源电压过低", "bad"),
    (1, "电机过温",     "bad"),
    (2, "驱动器过流",   "bad"),
    (3, "驱动器过温",   "bad"),
    (4, "传感器异常",   "bad"),
    (5, "驱动器错误",   "bad"),
    (6, "驱动器已使能", "good"),
    (7, "已回零",       "good"),
]


# ---------------------------------------------------------------- 解码

def clamp_stroke(v: int) -> int:
    """行程硬限幅。piper_sdk 的 start_sdk_gripper_limit 默认是 False（不夹紧），
    所以**限幅必须由调用方做**，不能指望 SDK。"""
    lo = int(round(STROKE_MIN_MM / UNIT_MM))
    hi = int(round(STROKE_MAX_MM / UNIT_MM))
    return max(lo, min(int(v), hi))


def decode_2a8(data: bytes) -> dict | None:
    """0x2A8 解析。

    Byte 0-3  int32 BE  行程，单位 0.001 mm
    Byte 4-5  int16 BE  力矩，单位 0.001 N·m
    Byte 6    uint8     状态码
    Byte 7    保留
    """
    if len(data) < 7:
        return None
    angle_raw = int.from_bytes(data[0:4], "big", signed=True)
    effort_raw = int.from_bytes(data[4:6], "big", signed=True)
    status = data[6]
    return {
        "angle_raw": angle_raw,
        "angle_mm": round(angle_raw * UNIT_MM, 3),
        "effort_raw": effort_raw,
        "effort_nm": round(effort_raw * 0.001, 3),
        "status_code": status,
        "bits": decode_status(status),
    }


def decode_status(code: int) -> dict:
    """把 status_code 拆成可读字典。bad 位 True 表示有故障。"""
    out = {"hex": f"0x{code:02X}", "bad": [], "good": []}
    for bit, name, polarity in STATUS_BITS:
        on = bool(code & (1 << bit))
        entry = f"bit{bit} {name}={'1' if on else '0'}"
        if polarity == "bad":
            if on:
                out["bad"].append(entry)
        else:
            out["good"].append(entry) if on else None
    out["enabled"] = bool(code & (1 << 6))
    out["homed"] = bool(code & (1 << 7))
    return out


def encode_gripper_ctrl(stroke_raw: int, effort: int = 1000,
                        code: int = 0x01, set_zero: int = 0x00) -> bytes:
    """0x159 编码。effort 0~5000 对应 0~5 N·m，默认 1000 = 1 N·m（保守）。"""
    stroke_raw = clamp_stroke(stroke_raw)
    effort = max(0, min(int(effort), 5000))
    return (stroke_raw.to_bytes(4, "big", signed=True)
            + effort.to_bytes(2, "big", signed=False)
            + bytes([code & 0xFF, set_zero & 0xFF]))


# ---------------------------------------------------------------- 总线

def open_bus(iface: str, rcvbuf_mb: int = 4):
    import can
    bus = can.Bus(interface="socketcan", channel=iface)
    try:
        import socket as _s
        sk = getattr(bus, "socket", None)
        if sk is not None:
            want = rcvbuf_mb * 1024 * 1024
            sk.setsockopt(_s.SOL_SOCKET, _s.SO_RCVBUF, want)
            got = sk.getsockopt(_s.SOL_SOCKET, _s.SO_RCVBUF)
            print(f"[bus] SO_RCVBUF 请求 {want // 1024} KB → 实际 {got // 1024} KB")
    except Exception as exc:
        print(f"[bus][WARN] 设置 SO_RCVBUF 失败：{exc!r}", file=sys.stderr)
    return bus


def send_frame(bus, can_id: int, data: bytes, timeout: float = 0.005) -> tuple[bool, str]:
    import can
    try:
        bus.send(can.Message(arbitration_id=can_id, data=data,
                             is_extended_id=False), timeout=timeout)
        return True, ""
    except Exception as exc:
        return False, repr(exc)


def collect_gripper(bus, seconds: float, on_sample=None, counter=None) -> list[dict]:
    """只收 0x2A8。返回每帧 (t, 解码结果)。

    注意：统计频率一律用**帧时间跨度**，不用墙钟。15:26 那次就是按墙钟除，
    得出「丢 42%」，而实际一帧没丢（R30）。

    counter：传入一个 Counter 就会把**所有 ID** 都记进去。这是为了区分
    「整条总线没数据」和「只有夹爪这一路没数据」—— 15:56 那次 0x2A8 零帧，
    只看 0x2A8 根本分不清是夹爪没识别还是 can0 掉了。
    """
    import can
    rows: list[dict] = []
    t_end = time.time() + seconds
    n_err = 0
    while time.time() < t_end:
        try:
            msg = bus.recv(timeout=0.1)
        except can.CanError as exc:
            n_err += 1
            if n_err <= 3:
                print(f"[bus][WARN] 总线错误：{exc}", file=sys.stderr)
            continue
        if msg is None:
            continue
        if counter is not None:
            counter[msg.arbitration_id] += 1
        if msg.arbitration_id != GRIPPER_FEEDBACK:
            continue
        dec = decode_2a8(bytes(msg.data))
        if dec is None:
            continue
        dec["t"] = msg.timestamp
        rows.append(dec)
        if on_sample:
            on_sample(dec)
    return rows


def iface_state(iface: str) -> dict:
    """读 /sys/class/net 判 UP / 波特率，不依赖外部命令。

    「无帧」的第一嫌疑永远是接口没起来，而不是设备没识别 —— 15:05 那次
    `CAN port can0 is not UP` 就是这个。所以必须先看接口，再看设备。
    """
    base = Path("/sys/class/net") / iface
    out = {"exists": base.exists(), "operstate": None, "bitrate": None}
    if not out["exists"]:
        return out
    try:
        out["operstate"] = (base / "operstate").read_text().strip()
    except Exception:
        pass
    for cand in (base / "bitrate", base / "can_bittiming" / "bitrate"):
        try:
            out["bitrate"] = int(cand.read_text().strip())
            break
        except Exception:
            continue
    return out


def bus_context(counter, iface: str) -> dict:
    """把「同期总线上还有什么」整理成人能判读的形式。"""
    total = sum(counter.values()) if counter else 0
    ids = sorted(counter) if counter else []
    hs = [0x251 + i for i in range(6)]
    status = list(range(0x2A1, 0x2A9))
    return {
        "iface": iface,
        "total_frames": total,
        "n_ids": len(ids),
        "ids": [f"0x{x:03X}" for x in ids],
        "hs_hit": sum(1 for x in hs if counter.get(x, 0) > 0) if counter else 0,
        "status_hit": sum(1 for x in status if counter.get(x, 0) > 0) if counter else 0,
        "gripper_frames": counter.get(GRIPPER_FEEDBACK, 0) if counter else 0,
        "state": iface_state(iface),
    }


def describe_context(ctx: dict) -> list[str]:
    """把旁证翻成「下一步查什么」，而不是只报数字。"""
    lines = []
    st = ctx["state"]
    lines.append(f"接口 {ctx['iface']}：存在={st['exists']}、operstate={st['operstate']}、"
                 f"bitrate={st['bitrate']}")
    lines.append(f"同期总线：共 {ctx['total_frames']} 帧 / {ctx['n_ids']} 个 ID；"
                 f"关节高速 0x251~0x256 命中 {ctx['hs_hit']}/6，"
                 f"状态 0x2A1~0x2A8 命中 {ctx['status_hit']}/8")

    if ctx["total_frames"] == 0:
        if not st["exists"]:
            lines.append("→ **整条总线一帧都没有，且接口不存在**：can0 没起来。"
                         "先 `sudo ip link set can0 up type can bitrate 1000000`")
        elif st["operstate"] != "up":
            lines.append("→ **整条总线一帧都没有，接口是 down**：先拉起 can0。"
                         "这是 15:05 那次同型问题（`CAN port can0 is not UP`）")
        else:
            lines.append("→ **整条总线一帧都没有，但接口是 up**：问题不在夹爪。"
                         "查臂是否上电（R17：未上电时 SDK 不报错、只给全 0）、"
                         "USB-CAN 是否掉线（`lsusb | grep 1d50`）")
    elif ctx["hs_hit"] == 0:
        lines.append("→ 总线有帧，但**六个关节高速反馈一个都没有**：臂没在正常上报，"
                     "先解决臂，夹爪结论不作数")
    elif ctx["gripper_frames"] == 0:
        lines.append("→ 关节正常 6/6，**只有 0x2A8 这一路没有帧**：这才是「夹爪没被识别」的证据形态。"
                     "查末端 XT30 是否插到底、夹爪是否需要单独 24 V、末端排线有没有被夹住")
    else:
        lines.append("→ 0x2A8 有帧，进入 C1~C4 判据")
    return lines


def summarize(rows: list[dict], label: str) -> dict:
    if not rows:
        return {"label": label, "n": 0, "verdict": "无帧"}
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    span = max(t1 - t0, 1e-9)
    dts = [rows[i + 1]["t"] - rows[i]["t"] for i in range(len(rows) - 1)]
    angles = [r["angle_raw"] for r in rows]
    efforts = [r["effort_raw"] for r in rows]
    statuses = sorted({r["status_code"] for r in rows})
    return {
        "label": label,
        "n": len(rows),
        "span_s": round(span, 3),
        "hz_by_span": round((len(rows) - 1) / span, 2) if len(rows) > 1 else None,
        "dt_median_ms": round(st.median(dts) * 1000, 2) if dts else None,
        "dt_max_ms": round(max(dts) * 1000, 2) if dts else None,
        "angle_mm": {"min": round(min(angles) * UNIT_MM, 3),
                     "max": round(max(angles) * UNIT_MM, 3),
                     "n_unique": len(set(angles)),
                     "first": round(angles[0] * UNIT_MM, 3),
                     "last": round(angles[-1] * UNIT_MM, 3)},
        "effort_nm": {"min": round(min(efforts) * 0.001, 3),
                      "max": round(max(efforts) * 0.001, 3)},
        "status_codes": [f"0x{s:02X}" for s in statuses],
        "status_hex": [f"0x{s:02X}" for s in statuses],
    }


# ---------------------------------------------------------------- 判据

def judge_read(s: dict, ctx: dict | None = None) -> tuple[str, list[str]]:
    """只读档的判据。明确区分「通道在」与「夹爪在」。"""
    lines: list[str] = []
    if s["n"] == 0:
        if ctx and ctx["total_frames"] == 0:
            lines.append("C1 无帧，但**整条总线也没有任何帧** → 这一条不是夹爪的结论。"
                         "先按上面「整条总线一帧都没有」那行排查（接口 / 上电 / USB-CAN）")
            return "总线无数据（不是夹爪结论）", lines
        if ctx and ctx["hs_hit"] > 0:
            lines.append("C1 0x2A8 零帧，而六个关节高速反馈正常 → "
                         "**这是「夹爪没被识别」的证据形态**，按末端接线排查")
            return "0x2A8 零帧（夹爪未识别）", lines
        lines.append("C1 无帧：单看 0x2A8 无法定位，需总线旁证（本次未采集）")
        return "无帧", lines

    hz = s["hz_by_span"] or 0
    if hz >= 150:
        lines.append(f"C1 帧率 {hz} Hz（按帧跨度算）→ 通道在。"
                     "**这一条不能区分有无夹爪**：无夹爪时同样 200 Hz，只是载荷全 0")
    elif hz >= 20:
        lines.append(f"C1 帧率偏低：{hz} Hz（其它状态帧 200 Hz）。可能是夹爪固件版本不同，"
                     "也可能臂侧轮询被降频")
    else:
        lines.append(f"C1 帧率异常低：{hz} Hz，先别下结论，重跑一次看是否复现")

    a = s["angle_mm"]
    eff = s["effort_nm"]
    nonzero_payload = (a["min"] != 0.0 or a["max"] != 0.0
                       or eff["min"] != 0.0 or eff["max"] != 0.0
                       or s["status_hex"] != ["0x00"])

    if not nonzero_payload:
        lines.append("C2/C4 载荷全 0（行程 0、力矩 0、status 0x00）→ "
                     "与「无夹爪时的占位帧」无法区分。装上了却仍全 0，"
                     "按下面的排查顺序走：① 末端 XT30 接头是否插到底；"
                     "② 夹爪是否需要单独上电（24 V / 2 A）；③ 换个固件版本再读")
        return "载荷全 0（未识别）", lines

    st_code = s["status_hex"]
    lines.append(f"C2 载荷非全 0：行程 {a['min']}~{a['max']} mm、"
                 f"力矩 {eff['min']}~{eff['max']} N·m、status {st_code}")
    enabled = any(int(x, 16) & 0x40 for x in st_code)
    if enabled:
        lines.append("   status bit6 = 1 → **夹爪驱动器已使能**，这是识别证据")
    else:
        lines.append("   status bit6 = 0 → 夹爪还没使能（未使能时仍可能读到编码器位置）。"
                     "跑 `enable` 再读一次")
    lines.append("C3 未致动，跟动无法判定 —— 要跑 `actuate`")
    return "已识别（待致动确认）" if enabled else "载荷非零（未使能）", lines


def judge_actuate(samples: list[dict], plan: list[tuple[float, int]],
                  n_ok: int, n_fail: int) -> tuple[str, list[str]]:
    lines = [f"发送 {n_ok} 成功 / {n_fail} 失败"]
    if n_fail > 0:
        lines.append("有发送失败：失败的拍没有推进目标轨迹（否则行程会被拉长）。"
                     "先查 `ip -s link show can0` 的 dropped/overrun（R30）")

    if not samples:
        lines.append("C3 采样为空：致动期间没收到 0x2A8，无法判定")
        return "无法判定", lines

    angles = [x["angle_raw"] for x in samples]
    span_measured = (max(angles) - min(angles)) * UNIT_MM
    tx_vals = [p[1] for p in plan]
    if not tx_vals:
        lines.append("C3 没有成功发出的指令（全部发送失败），无法比对")
        return "无法判定", lines
    span_tx = (max(tx_vals) - min(tx_vals)) * UNIT_MM
    lines.append(f"C3 指令行程跨度 {span_tx:.2f} mm，反馈行程跨度 {span_measured:.2f} mm")
    if span_measured < 0.5:
        lines.append("   反馈几乎没动 → 夹爪**没跟随**。检查：bit6 是否使能、"
                     "夹爪是否被卡住、effort 是否被限到 0")
        return "未跟随", lines
    ratio = span_measured / span_tx if span_tx > 0 else 0
    lines.append(f"   响应比 {ratio:.3f}（反馈跨度/指令跨度，1.0 为完全跟随）")
    if ratio < 0.3:
        lines.append("   响应比 < 0.3：有动但跟不上，可能是堵转保护或指令太快")
        return "部分跟随", lines
    lines.append("C3 跟随 → **夹爪被识别且可驱动**")
    eff = [x["effort_raw"] for x in samples]
    if max(eff) - min(eff) > 50:
        lines.append(f"C4 力矩有变化 {min(eff) * 0.001:.3f}~{max(eff) * 0.001:.3f} N·m → 力反馈通")
    return "跟随", lines


# ---------------------------------------------------------------- 子命令

def cmd_read(args) -> int:
    out = Path(args.out)
    if args.sdk:
        return _read_sdk(args, out)

    from collections import Counter
    counter: Counter = Counter()
    bus = open_bus(args.iface)
    try:
        print(f"[read] 只读 {args.sec:.0f} s，只发零帧；同时统计所有 ID 作为旁证")
        rows = collect_gripper(bus, args.sec, counter=counter)
    finally:
        bus.shutdown()

    ctx = bus_context(counter, args.iface)
    s = summarize(rows, "read")
    verdict, lines = judge_read(s, ctx)
    print()
    for ln in describe_context(ctx):
        print("[read] " + ln)
    print(f"[read] 0x2A8 帧数 {s['n']}，帧跨度 {s.get('span_s')} s，"
          f"dt 中位 {s.get('dt_median_ms')} ms")
    for ln in lines:
        print("[read] " + ln)
    print(f"[read] 结论：{verdict}")

    if rows:
        print(f"[read] status 位分解（最新一帧 0x{rows[-1]['status_code']:02X}）：")
        for bit, name, pol in STATUS_BITS:
            on = bool(rows[-1]["status_code"] & (1 << bit))
            flag = ("故障" if pol == "bad" else "正常") if on else ""
            print(f"        bit{bit} {name:12s} = {int(on)}  {flag}")

    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {"version": VERSION, "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
               "iface": args.iface, "summary": s, "bus_context": ctx,
               "verdict": verdict, "criteria": lines}
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[read] 已写 {out}")
    return 0 if verdict.startswith(("已识别", "载荷非零")) else 2


def _read_sdk(args, out: Path) -> int:
    """用 SDK 读，验证「解析链」而不只是「总线有帧」。

    单独一个子命令而不是和裸 socket 并行：两个客户端同时开 can0 会抢接收，
    15:26 已经出现过一次并发下的发送失败（R30）。
    """
    from piper_sdk import C_PiperInterface_V2
    piper = C_PiperInterface_V2(args.iface)
    piper.ConnectPort()
    time.sleep(0.3)
    stamps, vals = [], []
    t_end = time.time() + args.sec
    while time.time() < t_end:
        m = piper.GetArmGripperMsgs()
        stamps.append(getattr(m, "time_stamp", None))
        gs = getattr(m, "gripper_state", None)
        if gs is not None:
            vals.append((gs.grippers_angle, gs.grippers_effort, gs.status_code))
        time.sleep(0.005)

    uniq_ts = len({round(s, 6) for s in stamps if s})
    print(f"[read-sdk] {len(stamps)} 次轮询，时间戳唯一 {uniq_ts} 个")
    if uniq_ts <= 1:
        print("[read-sdk] 时间戳不推进 → SDK 没收到过新帧（不是「值没变」，是「没收到」）。"
              "注意 SDK 会丢弃 |行程|>150000 的异常帧，全被丢也会是这个现象")
        print("[read-sdk] 只读档结论：SDK 侧未识别")
        return 2
    last = vals[-1] if vals else None
    print(f"[read-sdk] grippers_angle={last[0]}（{last[0] * 0.001:.3f} mm）、"
          f"effort={last[1]}（{last[1] * 0.001:.3f} N·m）、status=0x{last[2]:02X}")
    print(f"[read-sdk] bit6 使能={bool(last[2] & 0x40)}、bit7 回零={bool(last[2] & 0x80)}")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"version": VERSION, "mode": "sdk",
                               "polls": len(stamps), "unique_ts": uniq_ts,
                               "last": {"angle": last[0], "effort": last[1],
                                        "status": last[2]}},
                              ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[read-sdk] 已写 {out}")
    return 0


def cmd_enable(args) -> int:
    """只发夹爪使能帧。不碰 0x471，避免 EnableArm 的语义歧义。

    歧义记录：手册 line 283 写 `EnableArm(7)  # 7 = 六关节 + 夹爪`，而
    arm_motor_enable_disable.py line 23 写「7 = 夹爪电机，0xFF = 全部（含夹爪）」。
    两处矛盾，所以默认走 0x159 的 code=0x01（夹爪专用使能），无歧义。
    """
    bus = open_bus(args.iface)
    try:
        before = collect_gripper(bus, 1.0)
        cur = before[-1]["angle_raw"] if before else 0
        frame = encode_gripper_ctrl(cur, effort=args.effort, code=0x01)
        print(f"[enable] 当前读数 0x{cur:04X}（{cur * UNIT_MM:.2f} mm），"
              f"发 0x159 {frame.hex(' ')}（code=0x01 使能）")
        if args.via_motor_enable:
            ok, err = send_frame(bus, MOTOR_ENABLE, bytes([7, 0x02, 0, 0, 0, 0, 0, 0]))
            print(f"[enable] 0x471 motor=7 enable=0x02 → {'ok' if ok else 'FAIL ' + err}")
        n_ok = n_fail = 0
        for _ in range(10):                     # 20 Hz × 0.5 s，重复发保证收到
            ok, err = send_frame(bus, GRIPPER_CTRL, frame)
            n_ok += ok
            n_fail += (not ok)
            time.sleep(0.05)
        print(f"[enable] 0x159 发送 {n_ok} 成功 / {n_fail} 失败")
        time.sleep(0.3)
        after = collect_gripper(bus, 1.5)
    finally:
        bus.shutdown()

    if not after:
        print("[enable] 使能后仍收不到 0x2A8")
        return 2
    code = after[-1]["status_code"]
    print(f"[enable] 使能后 status=0x{code:02X}  bit6={bool(code & 0x40)} "
          f"bit7={bool(code & 0x80)}")
    if code & 0x40:
        print("[enable] 结论：夹爪驱动器已使能 → **夹爪被识别**")
        return 0
    print("[enable] 结论：bit6 仍为 0。夹爪没使能成功，可能没接好或不受这条指令控制；"
          "不要反复狂发，先查线")
    return 2


def cmd_actuate(args) -> int:
    out_csv = Path(args.csv)
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    bus = open_bus(args.iface)
    samples: list[dict] = []
    plan: list[tuple[float, int]] = []
    n_ok = n_fail = 0
    import can
    try:
        base = collect_gripper(bus, 1.0)
        mid = base[-1]["angle_raw"] if base else 0
        mid = clamp_stroke(mid)
        span = int(args.span)
        lo = clamp_stroke(mid - span)
        hi = clamp_stroke(mid + span)
        print(f"[actuate] 起始行程 {mid * UNIT_MM:.2f} mm，"
              f"将在 {lo * UNIT_MM:.2f} ~ {hi * UNIT_MM:.2f} mm 之间开合 ±{span * UNIT_MM:.1f} mm")

        # 先使能，再动。顺序反了会出现"发了目标但驱动器没使能"的静默失败。
        en = encode_gripper_ctrl(mid, effort=args.effort, code=0x01)
        for _ in range(10):
            ok, err = send_frame(bus, GRIPPER_CTRL, en)
            n_ok += ok
            n_fail += (not ok)
            time.sleep(0.05)
        print(f"[actuate] 使能帧 {n_ok} 成功 / {n_fail} 失败")

        # 轨迹：mid → lo → mid → hi → mid，每段 hold 1.2 s，斜坡 1.0 s
        segs = [(mid, lo), (lo, mid), (mid, hi), (hi, mid)]
        t0 = time.time()
        dur_seg = 1.0 + 1.2
        total = dur_seg * len(segs)
        next_tick = t0
        idx = 0
        while True:
            t_rel = time.time() - t0
            if t_rel >= total:
                break
            seg = min(int(t_rel // dur_seg), len(segs) - 1)
            local = t_rel - seg * dur_seg
            a, b = segs[seg]
            if local < 1.0:
                u = local / 1.0
                u = u * u * (3 - 2 * u)         # smoothstep，避免阶跃
                tgt = int(round(a + (b - a) * u))
            else:
                tgt = b
            ok, err = send_frame(bus, GRIPPER_CTRL,
                                 encode_gripper_ctrl(tgt, effort=args.effort, code=0x01))
            if ok:
                n_ok += 1
                plan.append((t_rel, tgt))
            else:
                n_fail += 1                  # 失败不推进 plan，轨迹不留假点
                if n_fail <= 3:
                    print(f"[actuate][WARN] 发送失败：{err}", file=sys.stderr)

            # 非阻塞收，保证收发在同一个进程里（R30：双进程会抢）
            while True:
                try:
                    msg = bus.recv(timeout=0)
                except can.CanError:
                    break
                if msg is None:
                    break
                if msg.arbitration_id == GRIPPER_FEEDBACK:
                    dec = decode_2a8(bytes(msg.data))
                    if dec:
                        dec["t"] = msg.timestamp
                        samples.append(dec)

            next_tick += 0.01                # 100 Hz
            sleep = next_tick - time.time()
            if sleep > 0:
                time.sleep(sleep)
            idx += 1
    finally:
        bus.shutdown()

    verdict, lines = judge_actuate(samples, plan, n_ok, n_fail)
    for ln in lines:
        print("[actuate] " + ln)
    print(f"[actuate] 结论：{verdict}")

    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "angle_raw", "angle_mm", "effort_raw", "effort_nm", "status_code"])
        for x in samples:
            w.writerow([f"{x['t']:.6f}", x["angle_raw"], x["angle_mm"],
                        x["effort_raw"], x["effort_nm"], x["status_code"]])
    print(f"[actuate] 已写 {out_csv}（{len(samples)} 行）")
    return 0 if verdict == "跟随" else 2


# ---------------------------------------------------------------- 自测

def cmd_selftest(_args) -> int:
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        print(f"  [{'PASS' if cond else 'FAIL'}] {name} {extra}")
        ok = ok and cond

    print("[selftest] 0x2A8 解码")
    # 行程 25.123 mm、力矩 -1.234 N·m、status 0x40（只有 bit6）
    frame = (25123).to_bytes(4, "big", signed=True) + (-1234).to_bytes(2, "big", signed=True) + bytes([0x40, 0])
    d = decode_2a8(frame)
    check("行程 25123 → 25.123 mm", d["angle_mm"] == 25.123, f"实得 {d['angle_mm']}")
    check("力矩 -1234 → -1.234 N·m", d["effort_nm"] == -1.234, f"实得 {d['effort_nm']}")
    check("bit6 使能 = True", d["bits"]["enabled"] is True)
    check("bit7 回零 = False", d["bits"]["homed"] is False)
    check("无故障位", d["bits"]["bad"] == [], f"实得 {d['bits']['bad']}")

    print("[selftest] 故障位方向（前 6 位 1=坏）")
    d2 = decode_2a8(bytes(4) + bytes(2) + bytes([0xC1, 0]))   # bit0 低压 + bit6 使能 + bit7 回零
    check("0xC1 检出低压故障", any("电源电压过低" in x for x in d2["bits"]["bad"]))
    check("0xC1 使能 = True", d2["bits"]["enabled"] is True)
    check("0xC1 回零 = True", d2["bits"]["homed"] is True)

    print("[selftest] 0x159 编码与限幅")
    f = encode_gripper_ctrl(30000, effort=1000, code=0x01)
    check("8 字节", len(f) == 8, f"实得 {len(f)}")
    check("行程字段 30000", int.from_bytes(f[0:4], "big", signed=True) == 30000)
    check("effort 字段 1000", int.from_bytes(f[4:6], "big") == 1000)
    check("code=0x01", f[6] == 0x01)
    check("set_zero=0x00（本脚本永不回零）", f[7] == 0x00)
    check("超上限被夹到 70000", int.from_bytes(encode_gripper_ctrl(999999)[0:4], "big", signed=True) == 70000)
    check("低于 0 被夹到 0", int.from_bytes(encode_gripper_ctrl(-500)[0:4], "big", signed=True) == 0)

    print("[selftest] 判据分支")
    s0 = {"label": "x", "n": 0}
    check("无帧且无旁证 → 无帧", judge_read(s0)[0] == "无帧")
    ctx_dead = {"total_frames": 0, "hs_hit": 0, "gripper_frames": 0}
    check("整条总线无帧 → 不是夹爪结论",
          judge_read(s0, ctx_dead)[0] == "总线无数据（不是夹爪结论）")
    ctx_only_gripper_missing = {"total_frames": 3000, "hs_hit": 6, "gripper_frames": 0}
    check("关节正常但 0x2A8 零帧 → 夹爪未识别",
          judge_read(s0, ctx_only_gripper_missing)[0] == "0x2A8 零帧（夹爪未识别）")
    s_all_zero = {"n": 500, "hz_by_span": 200.0, "angle_mm": {"min": 0.0, "max": 0.0},
                  "effort_nm": {"min": 0.0, "max": 0.0}, "status_hex": ["0x00"]}
    check("全 0 → 未识别", judge_read(s_all_zero)[0] == "载荷全 0（未识别）")
    samples = [{"angle_raw": 0, "effort_raw": 0}] + [{"angle_raw": v, "effort_raw": 0}
                                                     for v in range(0, 20001, 500)]
    plan = [(0, 0), (1, 20000)]
    check("反馈 20 mm / 指令 20 mm → 跟随", judge_actuate(samples, plan, 100, 0)[0] == "跟随")
    still = [{"angle_raw": 0, "effort_raw": 0}] * 50
    check("反馈不动 → 未跟随", judge_actuate(still, plan, 100, 0)[0] == "未跟随")

    print(f"\n[selftest] {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# ---------------------------------------------------------------- CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="夹爪识别与致动核验（0x2A8 / 0x159）")
    ap.add_argument("--version", action="version",
                    version=f"stage0_gripper_check {VERSION}")
    sub = ap.add_subparsers(dest="cmd")

    def common(p):
        p.add_argument("--iface", default="can0")
        p.add_argument("--sec", type=float, default=3.0)

    p = sub.add_parser("read", help="只读 0x2A8，不发任何帧")
    common(p)
    p.add_argument("--sdk", action="store_true", help="改用 piper_sdk 读（验证解析链）")
    p.add_argument("--out", default="env/gripper_check.json")
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("enable", help="只发夹爪使能帧（0x159 code=0x01）")
    common(p)
    p.add_argument("--effort", type=int, default=1000, help="0.001 N·m，默认 1 N·m")
    p.add_argument("--via-motor-enable", action="store_true",
                   help="额外发 0x471 motor=7（EnableArm 语义有歧义，默认不发）")
    p.set_defaults(func=cmd_enable)

    p = sub.add_parser("actuate", help="小幅开合，判定是否跟随")
    common(p)
    p.add_argument("--span", type=int, default=10000, help="相对当前行程 ±N（0.001 mm）")
    p.add_argument("--effort", type=int, default=1000)
    p.add_argument("--csv", default="runs/gripper_actuate.csv")
    p.set_defaults(func=cmd_actuate)

    p = sub.add_parser("selftest", help="离线自测")
    p.set_defaults(func=cmd_selftest)

    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 1
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
