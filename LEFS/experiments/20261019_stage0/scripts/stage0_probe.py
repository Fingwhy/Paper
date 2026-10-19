#!/usr/bin/env python3
"""stage0_probe.py -- PiPER x FACTR 2 阶段 0 探针（环境与版本冻结）

在【机器人机】上运行（Ubuntu + ROS2 Jazzy + python3.12 + piper_sdk==0.6.1）。
本脚本只做「只读探测 + 使能确认」，不下发任何位置/速度/力矩指令，不会让臂运动。

用法
----
    python3 stage0_probe.py --iface can0                 # 默认：全量只读探测，不发任何帧
    python3 stage0_probe.py --phase env                  # 只采环境（不需要 CAN）
    python3 stage0_probe.py --phase bus --seconds 5      # 只监听总线（不发帧）
    python3 stage0_probe.py --phase sdk                  # 只读状态/使能位/首帧守卫
    python3 stage0_probe.py --phase api                  # 打印 SDK 可用接口
    python3 stage0_probe.py --phase selftest             # 离线自测首帧守卫（任何机器可跑）

    # 允许发"非运动"配置/查询帧：从站配置(0x470)、限位回读(0x472->0x473)、固件搜索
    python3 stage0_probe.py --iface can0 --allow-write --query-limits

与 FIRST_ARM 纪律的对齐
-----------------------
该机 PiPER 上电即自使能，`~/ARM/FIRST_ARM` 红线 #1 是"不发送使能/失能命令"。
因此本脚本默认 **不发 0x471**，只用 GetArmEnableStatus() 只读确认已使能；
`--force-enable` 仅作排障保留，且必须显式确认工作空间安全。

产物：env_probe.json（--out 可改），字段一律带 state，取不到就是 null + 未核验。

为什么不用官方 piper_node 的时间戳做 L0 证据
--------------------------------------------
piper_node 用 create_timer(1/publish_hz) 以 ROS now() 打戳，数据取自 200 Hz
读取线程的缓存，同一 CAN 帧会被重复发布 2~4 次且无法从消息头判断新鲜度。
L0 的「字段年龄 / 跨关节时间跨度」必须由原始层（本脚本的 --phase bus）给出。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------- 常量

# 改了脚本就改这里：用来确认 VM 上跑的到底是不是新版（`--version` 可查）
PROBE_VERSION = "1.12-20261019"

HIGH_SPD_IDS = list(range(0x251, 0x257))   # 关节高速反馈，20 ms ≈ 50 Hz
LOW_SPD_IDS = list(range(0x261, 0x267))    # 关节低速反馈，100 ms ≈ 10 Hz
STATUS_IDS = list(range(0x2A1, 0x2A9))     # 臂主动反馈（心跳/位姿/关节角/夹爪）

# 与 piper_ros humble URDF 一致（比最新手册保守），禁止硬编码到产品代码，
# 这里只作为「合理区间」的守卫边界。
JOINT_MIN = [-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944]
JOINT_MAX = [2.618, 3.14, 0.0, 1.745, 1.22, 2.0944]

NOMINAL_DT = 0.020


# ---------------------------------------------------------------- 工具

def sh(cmd: list[str], timeout: float = 8.0) -> str | None:
    """执行外部命令，失败返回 None，不抛异常。"""
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if out.returncode != 0:
            return None
        return (out.stdout or "") + (out.stderr or "")
    except (OSError, subprocess.SubprocessError):
        return None


def sha256_file(path: Path) -> str | None:
    try:
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def field(value, state: str, **extra):
    """统一字段形状：{value, state, ...}；取不到值时 state=未核验。"""
    d = {"value": value, "state": state}
    d.update(extra)
    return d


# ---------------------------------------------------------------- 首帧守卫

class FirstFrameGuard:
    """使能后作废前若干帧的守卫。

    判定一帧「可用」需同时满足：
      1) 六个关节角有限且落在 URDF 限位内；
      2) 与上一帧的接收间隔落在 [dt_min, dt_max)；
      3) 连续 stable_frames 帧都满足 1)+2) 才放行。

    注意事项：本守卫只能判断「读到的值是否可用」，无法证明 CAN 帧是新的
    （SDK 内部会缓存）。真正的新鲜度由原始层时间戳决定，见 --phase bus。
    """

    def __init__(self, stable_frames: int = 10, dt_min: float = 0.002,
                 dt_max: float = 0.060, q_min=None, q_max=None, strict_limits: bool = False):
        self.stable_frames = stable_frames
        self.dt_min = dt_min
        self.dt_max = dt_max
        # 两级限位：
        #   hard = 物理不可能（|q| > 2π），一定是坏值，必须拒收；
        #   soft = URDF 标称限位，**默认不拒收只计数**。
        # 2026-10-19 13:10 改：原先把 URDF 限位当硬拒绝，而门 8（真实限位回读）还没通过，
        # 这就等于拿一个未核验的假设去否决真机数据。13:05 那次「守卫=不通过」很可能
        # 就是 J3 落在 URDF 表外（本表 J3 ∈ [-2.967, 0]，真机若为正就全被拒）。
        # 用一个未验证的限位去判机器的真假，方向是反的：应该先用真限位，再判守卫。
        self.q_min = q_min if q_min is not None else JOINT_MIN
        self.q_max = q_max if q_max is not None else JOINT_MAX
        self.strict_limits = strict_limits
        self.hard = 2 * math.pi + 0.1
        self.urdf_violations: dict[str, int] = {}
        self.prev_t: float | None = None
        self.streak = 0
        self.open = False
        self.discarded = 0
        self.reasons: dict[str, int] = {}

    def _reject(self, reason: str) -> tuple[bool, str]:
        self.discarded += 1
        key = reason.split(":")[0]
        self.reasons[key] = self.reasons.get(key, 0) + 1
        return False, reason

    def offer(self, t: float, q) -> tuple[bool, str]:
        if len(q) != 6 or not all(math.isfinite(float(v)) for v in q):
            self.streak = 0
            self.prev_t = t
            return self._reject("bad_value:non_finite")
        for i, v in enumerate(q):
            if abs(float(v)) > self.hard:
                self.streak = 0
                self.prev_t = t
                return self._reject(f"bad_value:joint{i + 1}_impossible")
        # URDF 标称限位：只记不计（除非显式 strict_limits，即已拿到真限位）
        for i, v in enumerate(q):
            if not (self.q_min[i] <= float(v) <= self.q_max[i]):
                k = f"joint{i + 1}"
                self.urdf_violations[k] = self.urdf_violations.get(k, 0) + 1
                if self.strict_limits:
                    self.streak = 0
                    self.prev_t = t
                    return self._reject(f"bad_value:{k}_out_of_range")

        if self.prev_t is None:
            self.prev_t = t
            return self._reject("baseline")

        dt = t - self.prev_t
        self.prev_t = t
        if dt < self.dt_min:
            self.streak = 0
            return self._reject(f"dt_too_small:{dt:.4f}")
        if dt >= self.dt_max:
            self.streak = 0
            return self._reject(f"dt_too_large:{dt:.4f}")

        self.streak += 1
        if self.streak >= self.stable_frames:
            self.open = True
        if self.open:
            return True, "ok"
        # 未放行的帧一律算丢弃：它可能被误当成首帧目标
        self.discarded += 1
        self.reasons["accumulate"] = self.reasons.get("accumulate", 0) + 1
        return False, f"accumulate:{self.streak}"


def selftest() -> int:
    """离线自测：不接硬件也能证明守卫逻辑正确。"""
    good = [0.0] * 6
    cases = []

    def run(name, frames, stable=10):
        g = FirstFrameGuard(stable_frames=stable)
        opened_at = None
        for i, (t, q) in enumerate(frames):
            ok, _ = g.offer(t, q)
            if ok and opened_at is None:
                opened_at = i
        cases.append((name, g, opened_at))

    # A: 从第一帧起就正常，20 ms 间隔
    run("A_clean_20ms", [(i * 0.02, good) for i in range(20)])
    # B: 前 5 帧是重复帧（dt=0，SDK 缓存典型表现），之后正常
    b = [(i * 0.001, good) for i in range(5)] + [(0.05 + i * 0.02, good) for i in range(20)]
    run("B_duplicate_head", b)
    # C: 中间出现超限值
    c = [(i * 0.02, good) for i in range(8)] + [(0.16, [9.9] + good[1:])] + \
        [(0.18 + i * 0.02, good) for i in range(15)]
    run("C_out_of_range", c)
    # D: 中间出现 200 ms 大间隔（丢帧），必须重新累积
    d = [(i * 0.02, good) for i in range(8)] + [(0.36 + i * 0.02, good) for i in range(15)]
    run("D_gap_200ms", d)
    # E: 读数落在 URDF 表外但物理合理（J3=+0.5，本表 J3 ∈ [-2.967, 0]）
    #    真限位还没拿到时**不能**拒收，否则整段数据全废
    e = [(i * 0.02, [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]) for i in range(20)]
    run("E_urdf_soft", e)
    # F: 同一序列 + 已拿到真限位（strict）→ 必须拒收且永不 open
    g = FirstFrameGuard(stable_frames=10, strict_limits=True)
    for t, q in e:
        g.offer(t, q)
    cases.append(("F_urdf_strict", g, None))

    failures = []
    for name, g, opened_at in cases:
        print(f"[selftest] {name:18s} open={g.open} at_frame={opened_at} "
              f"discarded={g.discarded} reasons={g.reasons}"
              + (f" urdf={g.urdf_violations}" if g.urdf_violations else ""))
        if name != "F_urdf_strict" and not g.open:
            failures.append(f"{name}: never opened")
        if g.discarded <= 0:
            failures.append(f"{name}: discarded nothing (unexpected)")

    # 语义断言：干净 20 ms 序列应在第 10 个样本（0 基）打开，之前 10 帧全部丢弃
    a = cases[0][1]
    if cases[0][2] != 10 or a.discarded != 10:
        failures.append(f"A_clean_20ms: expected open@10/discard 10, got {cases[0][2]}/{a.discarded}")

    # E/F：URDF 限位只在「已核验」时才可当硬判据
    e_g, f_g = cases[4][1], cases[5][1]
    if not e_g.open:
        failures.append("E_urdf_soft: 落在 URDF 表外就被拒收——等于拿未核验的限位否决真机数据")
    if e_g.urdf_violations.get("joint3", 0) != 20:
        failures.append(f"E_urdf_soft: URDF 越界计数应为 20，实得 {e_g.urdf_violations}")
    if f_g.open:
        failures.append("F_urdf_strict: strict_limits=True 时越界帧竟然放行了")

    # G: 静态名扫描 —— py_compile 查不出「用了不存在的变量」，只有真跑到那一行才炸。
    #    15:10 那次 --to-can-ctrl 就是这样：指令已经发出、JSON 已经写盘，
    #    却在打印摘要时 NameError，把「转移已发生」的证据打印丢了（数据在盘上但人看不到）。
    #    这类 bug 每次都要用户重跑一遍真机才暴露，代价太高，所以固化成一项自测。
    import ast
    import builtins
    _tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    _gset = set(dir(builtins)) | {"__file__", "__name__", "__doc__"}
    for _n in _tree.body:
        if isinstance(_n, (ast.FunctionDef, ast.ClassDef)):
            _gset.add(_n.name)
        elif isinstance(_n, ast.Assign):
            for _t in _n.targets:
                for _m in ast.walk(_t):
                    if isinstance(_m, ast.Name):
                        _gset.add(_m.id)
        elif isinstance(_n, (ast.Import, ast.ImportFrom)):
            for _a in _n.names:
                _gset.add((_a.asname or _a.name).split(".")[0])
        elif isinstance(_n, (ast.If, ast.Try)):
            for _m in ast.walk(_n):
                if isinstance(_m, ast.Name) and isinstance(_m.ctx, ast.Store):
                    _gset.add(_m.id)
    _sus = {}
    for _f in [x for x in _tree.body if isinstance(x, ast.FunctionDef)]:
        _loc = set(_gset)
        for _m in ast.walk(_f):
            if isinstance(_m, ast.Name) and isinstance(_m.ctx, ast.Store):
                _loc.add(_m.id)
            if isinstance(_m, ast.arg):
                _loc.add(_m.arg)
            # 函数内定义的嵌套函数/类也是本作用域的名字（selftest 里的 run/_Mo/_Agg 就是）
            if isinstance(_m, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                _loc.add(_m.name)
            if isinstance(_m, (ast.Import, ast.ImportFrom)):
                for _a in _m.names:
                    _loc.add((_a.asname or _a.name).split(".")[0])
            if isinstance(_m, ast.ExceptHandler) and _m.name:
                _loc.add(_m.name)
        _used = {_m.id for _m in ast.walk(_f)
                 if isinstance(_m, ast.Name) and isinstance(_m.ctx, ast.Load)}
        _bad = sorted(_used - _loc)
        if _bad:
            _sus[_f.name] = _bad
    print(f"[selftest] {'G_name_scan':18s} {'clean' if not _sus else _sus}")
    if _sus:
        failures.append(f"G_name_scan: 未定义名 {_sus}")

    # H: 0x473 解析 —— 聚合对象不可下标、未应答关节给的是 motor_num=0 的全零对象。
    #    这两个坑让门 8 连挂两次（13:17 TypeError、13:05 静默跳过），所以固化成自测。
    class _Mo:
        def __init__(self, n=0, mn=0, mx=0, sp=0):
            self.motor_num, self.min_angle_limit = n, mn
            self.max_angle_limit, self.max_joint_spd = mx, sp

    class _Agg:
        def __init__(self, motors):
            self.motor = motors

    _h_fail = []
    if _limits_from_obj(_Agg([_Mo() for _ in range(7)])) != {}:
        _h_fail.append("全零未应答应返回 {}")
    _real = [(1, -1500, 1500), (2, 0, 1800), (3, -1700, 0),
             (4, -1000, 1000), (5, -700, 700), (6, -1800, 1800)]
    _ms = [_Mo()]
    for _i, _mn, _mx in _real:
        _ms.append(_Mo(_i, _mn, _mx, 300))
    _r = _limits_from_obj(_Agg(_ms))
    if len(_r) != 6:
        _h_fail.append(f"六关节应返回 6，实得 {len(_r)}")
    else:
        if (_r["joint_1"]["min_deg"], _r["joint_1"]["max_deg"]) != (-150.0, 150.0):
            _h_fail.append(f"J1 限位错：{_r['joint_1']}")
        if abs(_r["joint_1"]["max_spd_rad_s"] - 0.3) > 1e-9:
            _h_fail.append(f"J1 vmax 应为 0.3，实得 {_r['joint_1']['max_spd_rad_s']}")
    _part = [_Mo(), _Mo(1, -1500, 1500, 300), _Mo(), _Mo(3, -1700, 0, 300),
             _Mo(), _Mo(), _Mo()]
    if set(_limits_from_obj(_Agg(_part))) != {"joint_1", "joint_3"}:
        _h_fail.append("部分应答应只返回 J1/J3，不能给缺席关节补零")
    print(f"[selftest] {'H_limit_parse':18s} "
          + ("ok（全零→{}、六轴正解、部分应答不补零）" if not _h_fail else _h_fail))
    if _h_fail:
        failures.append(f"H_limit_parse: {_h_fail}")

    if failures:
        print("[selftest] FAIL:", "; ".join(failures))
        return 1
    print("[selftest] PASS: 首帧守卫逻辑可用")
    return 0


# ---------------------------------------------------------------- 环境

def probe_env() -> dict:
    d: dict = {}
    d["os"] = field(f"{platform.system()} {platform.release()}", "已核验")
    osrel = Path("/etc/os-release")
    if osrel.exists():
        kv = {}
        for line in osrel.read_text(encoding="utf-8", errors="replace").splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                kv[k] = v.strip().strip('"')
        d["distro"] = field(f"{kv.get('NAME', '?')} {kv.get('VERSION_ID', '?')}", "已核验")
        d["kernel"] = field(kv.get("PRETTY_NAME", ""), "已核验")
    else:
        d["distro"] = field(None, "未核验", how="仅 Linux 有 /etc/os-release")
    d["machine"] = field(platform.machine(), "已核验")
    d["python"] = field(platform.python_version(), "已核验")
    d["python_executable"] = field(sys.executable, "已核验")
    d["ros_distro"] = field(os.environ.get("ROS_DISTRO"), "已核验" if os.environ.get("ROS_DISTRO") else "未核验",
                            how="source /opt/ros/jazzy/setup.bash 后重跑")

    try:
        from importlib.metadata import version
        pkgs = {}
        for name in ("piper-sdk", "python-can", "numpy", "torch", "h5py", "pyyaml",
                     "scipy", "dynamixel-sdk", "cffi", "setuptools"):
            try:
                pkgs[name] = version(name)
            except Exception:
                pkgs[name] = None
        d["packages"] = field(pkgs, "已核验")
        d["packages_unverified"] = field([k for k, v in pkgs.items() if v is None], "未核验",
                                         how="缺包不一定是错，取决于本阶段是否用到（torch 到阶段 4 才必须）")
    except Exception as exc:  # pragma: no cover
        d["packages"] = field(None, "未核验", error=repr(exc))

    d["uv"] = field(sh(["uv", "--version"], 5.0), "已核验" if sh(["uv", "--version"], 5.0) else "未核验")
    return d


def probe_ros_pkgs() -> dict:
    out = sh(["ros2", "pkg", "list"], 15.0)
    if out is None:
        return field(None, "未核验", how="未 source ROS 或未安装 ros2 CLI")
    want = ("factr2_next", "piper_control", "teacher_arm", "system_bringup")
    present = [w for w in want if w in out]
    return field(present, "已核验", missing=[w for w in want if w not in present])


def probe_repo(path: str | None) -> dict:
    if not path:
        return field(None, "未核验", how="传 --factr2-repo <克隆路径>")
    p = Path(path)
    head = sh(["git", "-C", str(p), "rev-parse", "HEAD"], 5.0)
    dirty = sh(["git", "-C", str(p), "status", "--porcelain"], 8.0)
    if head is None:
        return field(None, "未核验", error="不是 git 仓库")
    return field(head.strip(), "已核验", dirty=bool((dirty or "").strip()))


# ---------------------------------------------------------------- 总线

def probe_bus(iface: str, seconds: float, backend: str) -> dict:
    """只监听、不发送。返回逐 ID 到达统计（L0 的前置证据）。"""
    info: dict = {"iface": iface, "backend": backend, "listen_seconds": seconds}

    details = sh(["ip", "-details", "-statistics", "link", "show", iface], 5.0)
    if details is None:
        info["link"] = field(None, "未核验", how="ip link show 失败，接口可能未建立")
    else:
        import re as _re
        m_state = _re.search(r"state (\w+)", details)
        m_bit = _re.search(r"bitrate (\d+)", details)
        m_restart = _re.search(r"restart-ms (\d+)", details)
        m_sample = _re.search(r"sample-point ([\d.]+)", details)
        counters = {}
        for key in ("bus_error", "error_warning", "error_passive", "bus_off",
                    "arbitration_lost", "restarts"):
            mc = _re.search(rf"{key}\s+(\d+)", details)
            if mc:
                counters[key] = int(mc.group(1))
        info["bus_counters"] = field(counters, "已核验",
                                     note="bus_error/bus_off 非零说明物理层或波特率有问题")
        info["link"] = field({
            "state": m_state.group(1) if m_state else None,
            "bitrate_bps": int(m_bit.group(1)) if m_bit else None,
            "restart_ms": int(m_restart.group(1)) if m_restart else None,
            "sample_point": float(m_sample.group(1)) if m_sample else None,
            "flags_excerpt": details.splitlines()[0] if details.splitlines() else None,
        }, "已核验", expect_bitrate_bps=1000000)

    try:
        import can  # noqa: WPS433
    except Exception as exc:
        info["frames"] = field(None, "未核验", error=f"python-can 不可用: {exc!r}")
        return info

    timestamps: dict[int, list[float]] = {}
    n_total = 0
    n_error = 0
    payload_sample: dict[int, str] = {}
    t_end = time.time() + seconds
    try:
        bus = can.Bus(interface=backend, channel=iface)
    except Exception as exc:
        info["frames"] = field(None, "未核验", error=f"打开总线失败: {exc!r}",
                              how="确认接口名与 backend（socketcan / gs_usb / pcan）")
        return info

    try:
        while time.time() < t_end:
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue
            if getattr(msg, "is_error_frame", False):
                # 总线无 ACK / 位错误会以 error frame 形式刷屏，绝不能当成机器人在说话
                n_error += 1
                continue
            n_total += 1
            payload_sample.setdefault(msg.arbitration_id, bytes(msg.data).hex())
            timestamps.setdefault(msg.arbitration_id, []).append(msg.timestamp)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    def stats(ts: list[float]) -> dict:
        if len(ts) < 2:
            return {"n": len(ts), "median_dt_ms": None}
        dts = sorted((b - a) * 1000.0 for a, b in zip(ts, ts[1:]))
        n = len(dts)
        p95 = dts[min(n - 1, int(round(0.95 * (n - 1))))]
        return {
            "n": len(ts),
            "median_dt_ms": round(dts[n // 2], 2),
            "p95_dt_ms": round(p95, 2),
            "max_dt_ms": round(dts[-1], 2),
            "gaps_gt_100ms": sum(1 for d in dts if d > 100.0),
        }

    group_defs = [
        ("high_speed_0x251_0x256", HIGH_SPD_IDS),
        ("low_speed_0x261_0x266", LOW_SPD_IDS),
        ("status_0x2A1_0x2A8", STATUS_IDS),
    ]
    by_group = {}
    for gname, all_ids in group_defs:
        seen = [i for i in all_ids if i in timestamps]
        by_group[gname] = {
            "ids_seen": [hex(i) for i in seen],
            "missing_ids": [hex(i) for i in all_ids if i not in seen],
            "per_id": {hex(i): stats(timestamps[i]) for i in seen},
        }
    known = set(HIGH_SPD_IDS) | set(LOW_SPD_IDS) | set(STATUS_IDS)
    info["frames"] = field({
        "total_frames": n_total,
        "error_frames": n_error,
        "distinct_ids": len(timestamps),
        "by_group": by_group,
        "other_ids": sorted(hex(i) for i in timestamps if i not in known),
        "other_ids_sample": {hex(i): payload_sample.get(i) for i in timestamps if i not in known},
        "verdict": ("有机器人反馈帧" if any(i in timestamps for i in list(range(0x251, 0x257)) + list(range(0x2A1, 0x2A9)))
                    else "没有任何 0x251-0x256 / 0x2A1 反馈帧：总线在说话，但机器人没在说"),
    }, "已核验" if any(i in timestamps for i in list(range(0x251, 0x257)) + list(range(0x2A1, 0x2A9))) else "不通过")

    info["heartbeat_0x2A1"] = field(hex(0x2A1) in [hex(i) for i in timestamps], "已核验",
                                   how="阶段 0 只要求看到 0x2A1 类心跳反馈；周期指标留给 L0")
    return info


# ---------------------------------------------------------------- SDK

def _iface_exists(iface: str) -> bool:
    """内核里是否真有这个网络接口。比让 SDK 抛异常再猜要准得多。"""
    return Path("/sys/class/net").joinpath(iface).exists()


def _load_sdk(iface: str):
    """打开 SDK 端口。返回 (piper, error, sdk_ver)。

    顺序很重要：先查接口存不存在，再让 SDK 去连。否则 SDK 0.6.1 会抛
    ConnectionError（不是 TypeError），早期版本这里没兜住，直接把栈甩给用户。
    """
    import warnings
    # piper_sdk 的 docstring 里有 \[ 转义，3.12 会刷 6 条 SyntaxWarning，与结论无关
    warnings.filterwarnings("ignore", category=SyntaxWarning)

    try:
        import piper_sdk
        from importlib.metadata import version as _v
        sdk_ver = _v("piper-sdk")
    except Exception as exc:
        return None, f"pip 装 piper-sdk 后重试: {exc!r}", None

    if not _iface_exists(iface):
        return None, (
            f"CAN 接口 {iface} 不存在——USB-CAN 没进虚拟机。"
            "先 lsusb | grep -i 1d50，没有就在 VMware 里把该 USB 设备 Connect (Disconnect from Host)"
        ), sdk_ver

    cls = getattr(piper_sdk, "C_PiperInterface_V2", None) or getattr(piper_sdk, "C_PiperInterface", None)
    if cls is None:
        return None, "piper_sdk 内找不到 C_PiperInterface(_V2)", sdk_ver

    piper = None
    try:
        piper = cls(iface)
    except TypeError:
        try:
            piper = cls(can_name=iface)
        except Exception as exc:
            return None, f"实例化失败（can_name= 形式）: {exc!r}", sdk_ver
    except Exception as exc:
        # 典型：can0 存在但没 up（bitrate 未设定）→ can 层直接拒绝
        return None, (
            f"实例化失败（{iface}）: {exc!r}。"
            f"若接口在但没拉起来：sudo ip link set {iface} up type can bitrate 1000000"
        ), sdk_ver

    try:
        piper.ConnectPort()
    except Exception as exc:
        return None, f"ConnectPort 失败（{iface} 是否 up？）: {exc!r}", sdk_ver
    return piper, None, sdk_ver


def probe_api(piper) -> dict:
    names = [n for n in dir(piper) if not n.startswith("_")]
    pick = [n for n in names if any(k in n for k in
            ("Firmware", "Version", "MasterSlave", "EnableArm", "DisableArm",
             "HighSpd", "LowSpd", "ArmStatus", "ArmJoint", "Gripper", "MitCtrl"))]
    return field(pick, "已核验", note="阶段 0 只读；MitCtrl/Gripper 等写入接口不用")


FEEDBACK_IDS = set(range(0x251, 0x257)) | set(range(0x2A1, 0x2A9))

_STATUS_LABELS = (("ctrl_mode", "Control Mode"), ("arm_status", "Arm Status"),
                  ("mode_feed", "Mode Feed"), ("teach_status", "Teach Status"),
                  ("motion_status", "Motion Status"))


def _fw_ge(version, threshold: str) -> bool:
    """比较固件版本，如 `S-V1.8-8` >= `V1.5-8`。

    为什么要这个：`max_joint_spd` 的量纲在 V1.5-8 换了档（0.001 → 0.01 rad/s），
    读限位时必须按固件分档换算，否则速度上限会差 10 倍——而这个值直接决定
    后面 0x473 回读能不能当安全上限用。
    解析不出来时按「新固件」处理：取 0.01 档宁可达不到也不越界。
    """
    import re as _re

    def _t(v):
        m = _re.search(r"V(\d+)\.(\d+)-(\d+)", str(v or ""))
        return tuple(int(x) for x in m.groups()) if m else None

    a, b = _t(version), _t(threshold)
    if a is None or b is None:
        return True
    return a >= b


def _spd_scale(raw_spd: int) -> tuple:
    """判定 max_joint_spd 的量纲。

    SDK 源码写死了量程，不需要猜：
        piper_sdk 0.6.1 interface/piper_interface.py:3200
        「max_joint_spd: Maximum joint speed, unit 0.001 rad/s. Range [0,3000]」
    → 0.001 档，量程 0..3.0 rad/s。

    「0.01 档」这个说法在我的早期注释里出现过，源码里找不到依据，
    只有 raw > 3000 时才可能成立。所以这里改成按量程硬判，
    不再用「算出来像不像个关节速度」去猜。

    13:24 实测 raw=300：两档都落在合理区间，数学上确实二义。
    但按源码应当取 0.001（0.300 rad/s），并且要把「怎么验证」写成人话，
    而不是把结论标成「数据可疑」——那会让后来读证据的人以为读数有问题。
    """
    if raw_spd > 3000:
        return 100.0, ("raw>3000 已超出 0.001 档量程 [0,3000]，只能是 0.01 档；"
                       "与 SDK 文档冲突，需人工核")
    if raw_spd > 300:
        return 1000.0, "0.001rad/s（SDK 文档；raw>300 已排除 0.01 档的可能）"
    return 1000.0, ("0.001rad/s（SDK 文档 unit=0.001, Range [0,3000]）；"
                    "raw≤300 时 0.01 档也合法，按文档取 0.001 —— "
                    "若实测角速度是该值的 10 倍，说明固件走的是 0.01 档")


def _limits_from_obj(raw) -> dict:
    """把 GetAllMotorAngleLimitMaxSpd() 的返回值拆成六关节。

    真实结构（0.6.1 源码）：
        raw.all_motor_angle_limit_max_spd.motor[1..6] -> max_angle_limit / min_angle_limit / max_joint_spd
    它**不可下标也不是列表**，13:17 那次照「可迭代」写就直接 TypeError。

    关键：未应答的关节是 motor_num=0 的全零对象。必须用 motor_num 认身份，
    否则会把 0 当成实测限位——和 R18（守卫在零值上假阳性）是同一个坑。
    """
    agg = getattr(raw, "all_motor_angle_limit_max_spd", raw)
    motors = getattr(agg, "motor", None)
    if motors is None and isinstance(agg, (list, tuple)):
        motors = agg
    if motors is None:
        raise TypeError(f"无法从 {type(raw).__name__} 取 motor 列表；"
                        f"可用属性={[a for a in dir(raw) if not a.startswith('_')][:15]}")
    res: dict = {}
    for i in range(1, 7):
        try:
            m = motors[i]
        except Exception:
            continue
        if getattr(m, "motor_num", 0) != i:
            continue                      # 还没应答，默认全零，不能当实测值
        mn = getattr(m, "min_angle_limit", None)
        mx = getattr(m, "max_angle_limit", None)
        sp = getattr(m, "max_joint_spd", None)
        if mn is None or mx is None or sp is None:
            continue
        div, how = _spd_scale(sp)
        res[f"joint_{i}"] = {
            "min_deg": round(mn / 10.0, 2),          # 0.1 度，int16 有符号
            "max_deg": round(mx / 10.0, 2),
            "max_spd_rad_s": (round(sp / div, 4) if div else None),
            "spd_raw": sp,
            "spd_scale": how,
        }
    return res


def _guard_hint(reasons: dict) -> str:
    """把 reject_reasons 翻译成一句人话。

    「守卫=不通过」本身不含信息：它可能是读数非法、可能是采样太快拿到缓存帧、
    也可能只是观察窗口太短。不翻译的话，现场只能去翻 json。
    """
    hint = {
        "bad_value": "六轴读数非有限值或落在 URDF 限位外 —— 先确认臂没被手推到限位外（示教模式下很容易发生）",
        "dt_too_small": "两帧间隔 <5 ms，SDK 在返缓存帧 —— 这不代表总线坏，但说明这段时间里读数没有真的刷新",
        "dt_too_large": "两帧间隔 ≥60 ms，读线程被阻塞或总线掉帧 —— 与 12:12 的 5.00 ms 对照看",
    }
    hard = [k for k in reasons if k not in ("accumulate", "baseline")]
    if hard:
        return hint.get(hard[0], f"见 reject_reasons[{hard[0]}]")
    if reasons:
        return "只有 baseline/accumulate：3 s 窗口内没攒够 10 帧，加 --guard-seconds 8 重跑"
    return ""


def _status_digest(st) -> dict:
    """把 0x2A1 状态压成 {name, code}，字段形态随 SDK 版本变化时要兜住。

    0.6.1 里 `GetArmStatus()` 打印出来是「Control Mode: STANDBY(0x0)…」这种多行摘要，
    而 `str(st.arm_status)` 有可能把整段摘要都带出来。所以先按文本标签解析，
    解析不到再退回属性上的 IntEnum。
    """
    import re as _re
    raw = " ".join(str(st).split()) if st is not None else ""
    parsed: dict = {}
    for key, label in _STATUS_LABELS:
        m = _re.search(label + r":\s*([A-Za-z_]+)\s*\(0x([0-9A-Fa-f]+)\)", raw)
        if m:
            parsed[key] = {"name": m.group(1), "code": int(m.group(2), 16)}
    for key, _ in _STATUS_LABELS:
        if key in parsed:
            continue
        attr = getattr(st, key, None)
        if attr is None:
            parsed[key] = None
            continue
        try:
            parsed[key] = {"code": int(attr), "name": getattr(attr, "name", None)}
        except Exception:
            txt = " ".join(str(attr).split())
            parsed[key] = {"text": txt[:40]}
    parsed["raw"] = raw[:160]
    return parsed


def _bus_liveness(iface: str, seconds: float = 0.6) -> dict:
    """信任 SDK 任何读数之前，先确认总线上真有机器人反馈帧。

    为什么必须先查：SDK 的 GetArmStatus() / GetArmJointMsgs() / GetArmEnableStatus()
    在**从未收到帧**时返回的是新建对象的初始默认值（全 0），看起来完全「合法」。
    2026-10-19 那次 `已使能=False`、`守卫=已核验` 就是这么来的——不是臂没使能，
    而是根本没收到一条反馈帧。
    """
    res = {"listened_s": seconds, "error_frames": 0, "frames": 0,
           "feedback_ids": [], "other_ids": [], "has_feedback": False}
    try:
        import can
        bus = can.Bus(interface="socketcan", channel=iface)
    except Exception as exc:
        res["error"] = f"{exc!r}（CAN 是否 up？）"
        return res
    seen: set[int] = set()
    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
            if getattr(msg, "is_error_frame", False):
                res["error_frames"] += 1
                continue
            res["frames"] += 1
            seen.add(msg.arbitration_id)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass
    res["feedback_ids"] = sorted(hex(i) for i in seen if i in FEEDBACK_IDS)
    res["other_ids"] = sorted(hex(i) for i in seen - FEEDBACK_IDS)
    res["has_feedback"] = bool(seen & FEEDBACK_IDS)
    return res


def probe_sdk(piper, iface: str, allow_write: bool, force_enable: bool,
              guard_seconds: float, query_limits: bool = False,
              set_slave: bool = False, to_can_ctrl: bool = False) -> dict:
    """只读为主。任何发帧动作（固件查询、从站配置、限位查询、使能）都要显式开关。

    与 `~/ARM/FIRST_ARM` 既有纪律对齐：该机臂上电即自使能，**不需要也不允许**发
    0x471；所以使能改成 `--force-enable`，默认关闭并打警告。
    """
    out: dict = {}

    # 固件版本：0.6.1 的语义是「读不到返回 -0x4AF」，此时必须先发搜索帧再读一次
    # （见 SDK demo/detect_arm.py:167-173）。搜索帧是 TX，所以仍受 --allow-write 约束。
    fw = None
    fw_err = None
    for fn in ("GetPiperFirmwareVersion", "GetArmFirmwareVersion", "GetFirmwareVersion"):
        if hasattr(piper, fn):
            try:
                fw = getattr(piper, fn)()
                out["firmware_via"] = fn
                break
            except Exception as exc:
                fw_err = repr(exc)
    sentinel = fw is None or fw in (-0x4AF, 0, "")

    if sentinel and allow_write and hasattr(piper, "SearchPiperFirmwareVersion"):
        try:
            piper.SearchPiperFirmwareVersion()
            for _ in range(10):                     # 最多等 0.5 s，别盲等
                time.sleep(0.05)
                fw = piper.GetPiperFirmwareVersion()
                if fw not in (None, -0x4AF, 0, ""):
                    break
            sentinel = fw is None or fw in (-0x4AF, 0, "")
            out["firmware_via"] = "SearchPiperFirmwareVersion → GetPiperFirmwareVersion"
        except Exception as exc:
            fw_err = f"搜索帧失败: {exc!r}"

    out["firmware"] = field(
        None if sentinel else fw,
        "已核验" if not sentinel else "未核验",
        how=("读不到：0.6.1 需要先发 0x?? 搜索帧（SearchPiperFirmwareVersion），"
             "本脚本仅在 --allow-write 时才会发；也可用厂家上位机读同一值"),
        error=fw_err,
    )

    # 臂状态（0x2A1）——0.6.1 的接口名是 GetArmStatus()，不是 GetArmStatusMsgs()
    st_getter = getattr(piper, "GetArmStatus", None) or getattr(piper, "GetArmStatusMsgs", None)
    try:
        st = st_getter()
        out["status_0x2A1"] = field(_status_digest(st), "已核验",
                                    how="ctrl_mode 应为 CAN_CTRL(0x1)；STANDBY(0x0) 表示待机")
    except Exception as exc:
        out["status_0x2A1"] = field(None, "未核验", error=repr(exc))

    # 使能状态：只读。FIRST_ARM 的红线是不发使能/失能，这里只确认"它自己是开的"
    try:
        if hasattr(piper, "GetArmEnableStatus"):
            flags = list(piper.GetArmEnableStatus())
        else:
            ls = piper.GetArmLowSpdInfoMsgs()
            flags = [getattr(getattr(ls, f"motor_{i}").foc_status, "driver_enable_status", False)
                     for i in range(1, 7)]
        out["enable_status_passive"] = field(
            {"all_enabled": bool(flags) and all(flags), "flags": flags}, "已核验",
            note="只读确认，未发送任何使能帧")
    except Exception as exc:
        out["enable_status_passive"] = field(None, "未核验", error=repr(exc))

    # 控制模式：这是整份脚本里最容易搞错的一段。
    #
    # 2026-10-19 13:17 下载 piper_sdk-0.6.1 wheel 逐行核对源码，纠正三个错误：
    #   (a) 0xFC 不是「切程序控」，是随动主从里的「运动输出臂」（follower）。单臂 CAN
    #       直控根本不需要它——发了只是把臂设成随动输出臂，和「能不能收运动指令」无关。
    #   (b) MotionCtrl_1(0x02,0,0) 不是复位。Byte0 是急停：0x00 无效 / 0x01 急停 / 0x02 恢复。
    #       要退出拖动示教得看 Byte2 grag_teach_ctrl：0x00 关闭 / 0x01 开始 / 0x02 结束。
    #   (c) TEACHING_MODE(0x02) 是**状态**不是设置值：MotionCtrl_2 的 ctrl_mode 只有
    #       0x00 待机 / 0x01 CAN / 0x03 以太网 / 0x04 wifi / 0x07 离线，压根没有 0x02。
    #       所以「发一帧切到 CAN_CTRL」这种想法本身就不成立；正确做法是先退出示教。
    #
    # 这就解释了 13:05 与 13:17 的实测：发完 0xFC 再读，仍是 TEACHING_MODE(0x2)。
    # 不是帧没发出去，是发的东西不对。
    cur_mode = (out.get("status_0x2A1") or {}).get("value") or {}
    cur_ctrl = (cur_mode.get("ctrl_mode") or {}).get("name") if isinstance(cur_mode, dict) else None
    after = None

    if set_slave and allow_write and hasattr(piper, "MasterSlaveConfig"):
        # 仅在双臂主从（一个示教输入臂 + 一个运动输出臂）时才用；单臂别开
        try:
            piper.MasterSlaveConfig(0xFC, 0, 0, 0)
            time.sleep(0.3)
            after = _status_digest(st_getter()).get("ctrl_mode")
            out["slave_mode"] = field(
                {"sent": True, "before": cur_ctrl, "after": after, "linkage": "motion_output(0xFC)"},
                "已核验" if after else "未核验",
                note="0xFC 是随动主从的『运动输出臂』，不是程序控开关；单臂 FACTR2 不需要它")
        except Exception as exc:
            out["slave_mode"] = field(None, "未核验", error=repr(exc))
    else:
        out["slave_mode"] = field(
            {"mode_before": cur_ctrl, "sent": False},
            "不适用",
            note="单臂直连不需要主从配置；要拿程序控请用 --to-can-ctrl")

    # 退出示教 → CAN 指令控制。这才是 FACTR2 真正需要的那一步。
    if to_can_ctrl and allow_write:
        try:
            steps = []
            if hasattr(piper, "MotionCtrl_1"):
                piper.MotionCtrl_1(0x00, 0x00, 0x00)      # Byte2=0x00 关闭拖动示教
                steps.append("MotionCtrl_1(0,0,0) 关闭拖动示教")
                time.sleep(0.3)
            if hasattr(piper, "MotionCtrl_2"):
                piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)  # CAN 控制 / MOVE J / 100% / 位置速度
                steps.append("MotionCtrl_2(0x01,0x01,100,0)")
                time.sleep(0.4)
            after = _status_digest(st_getter()).get("ctrl_mode")
            code = (after or {}).get("code")
            out["can_ctrl"] = field(
                {"before": cur_ctrl, "after": after, "steps": steps},
                "已核验" if code == 1 else "不通过",
                note=("已进入 CAN_CTRL(0x1)，可以下发运动指令" if code == 1 else
                      "仍不是 CAN_CTRL(0x1)：若手臂上的示教按钮/手柄被按住，软件指令压不住它，先松手再试"))
        except Exception as exc:
            out["can_ctrl"] = field(None, "未核验", error=repr(exc))
    else:
        # 16:39 这次踩到的坑：用户只打了 --to-can-ctrl，没打 --allow-write。
        # 脚本按设计**一帧都没发**，但汇总里连一行都不打印（打印条件要求 sent/steps 为真），
        # 于是「命令跑完了、6/6 也通过了」看上去一切正常，实际上模式根本没切。
        # 这与 R27/R31 同型：**成功的那部分掩盖了没做的那部分**。
        # 所以这里必须主动喊出来，而且状态不能写成「不适用」——它就是「未发送」。
        out["can_ctrl"] = field(
            {"before": cur_ctrl, "sent": False, "requested": bool(to_can_ctrl)},
            "未发送" if to_can_ctrl else "不适用",
            how="--allow-write --to-can-ctrl",
            note=("带了 --to-can-ctrl 但缺 --allow-write：本次**一帧都没发**，模式没有切换。"
                  "正确命令是 `--allow-write --to-can-ctrl`"
                  if to_can_ctrl else None))
        if to_can_ctrl:
            print("[WARN] --to-can-ctrl 必须与 --allow-write 同时给；"
                  "本次未发送任何帧，ctrl_mode 未改变", file=sys.stderr)

    # 真实限位/最大速度：SearchAllMotorMaxAngleSpd() 发 0x472×6 → 0x473 逐关节应答
    #
    # 2026-10-19 13:17 三修：
    #   (a) 结构：GetAllMotorAngleLimitMaxSpd() 返回的对象**不可下标**。真实结构是
    #       raw.all_motor_angle_limit_max_spd.motor[1..6]，每个 motor 有
    #       motor_num / max_angle_limit / min_angle_limit / max_joint_spd。
    #       13:17 那次就是照着"可迭代"写的，直接 TypeError。
    #   (b) 默认值陷阱：SDK 对还没应答的关节给 motor_num=0 的全零对象。只判"字段存在"
    #       会把 0 当实测值——和 R18（守卫在零值上假阳性）是同一个坑。必须要求 motor_num==i。
    #   (c) 单位：SDK 文档写 0.001 rad/s，我 12:12 按固件改成了 0.01，两者只能对一个。
    #       改成让数据自己说话：两种换算都算，取落在 [0.05, 15] rad/s 的那个；都合理则
    #       按文档取 0.001 并标 ambiguous，都不合理就报错不猜。
    out["hardware_limits"] = field(None, "未核验",
                                   how="--allow-write --query-limits 时发 0x472 并回读 0x473")
    if allow_write and query_limits and hasattr(piper, "SearchAllMotorMaxAngleSpd"):
        try:
            piper.SearchAllMotorMaxAngleSpd()
            t_end = time.time() + 3.0
            raw = None
            per_joint: dict = {}
            while time.time() < t_end:
                raw = piper.GetAllMotorAngleLimitMaxSpd()
                per_joint = _limits_from_obj(raw)
                if len(per_joint) == 6:
                    break
                time.sleep(0.1)
            age_ms = None
            ts = getattr(raw, "time_stamp", None)
            if isinstance(ts, (int, float)) and ts > 0:
                age_ms = round((time.time() - ts) * 1000.0, 1)
            out["hardware_limits"] = field(
                per_joint if per_joint else None,
                "已核验" if len(per_joint) == 6 else ("已回读/部分" if per_joint else "未核验"),
                note=(f"0.1°/关节限位；速度按数据判档（见 spd_scale）；应答后 {age_ms} ms"
                      if age_ms is not None else "0.1°/关节限位；应答时间戳不可用"),
                missing_joints=[i for i in range(1, 7) if f"joint_{i}" not in per_joint],
                age_ms=age_ms,
                raw_repr=str(raw)[:1200],
            )
        except Exception as exc:
            out["hardware_limits"] = field(None, "未核验", error=repr(exc),
                                           raw_repr=str(locals().get("raw"))[:800])
    elif allow_write and query_limits:
        out["hardware_limits"] = field(
            None, "未核验",
            error="piper_sdk 0.6.1 里找不到 SearchAllMotorMaxAngleSpd，接口名需按 SDK 版本再核")

    # 首帧守卫：纯读，任何时候都能跑（本机臂已自使能，不需要额外动作）
    guard = FirstFrameGuard(stable_frames=10)
    t_end = time.time() + guard_seconds
    samples = 0
    opened_at = None
    while time.time() < t_end:
        try:
            js = piper.GetArmJointMsgs().joint_state
            q = [getattr(js, f"joint_{i}") / 1000.0 * math.pi / 180.0 for i in range(1, 7)]
        except Exception:
            time.sleep(0.005)
            continue
        ok, _ = guard.offer(time.time(), q)
        samples += 1
        if ok and opened_at is None:
            opened_at = samples
        if guard.open:
            break
        time.sleep(0.005)

    out["first_frame_guard"] = field({
        "opened": guard.open,
        "opened_after_samples": opened_at,
        "discarded": guard.discarded,
        "reject_reasons": guard.reasons,
        "urdf_violations": guard.urdf_violations,
        "samples_examined": samples,
        "limits_used": ("URDF 标称（软判据，只计数不拒收）"
                        if not guard.strict_limits else "已核验限位（硬判据）"),
        "caveat": "SDK 层只能判「值可用」，不能证明帧新鲜；新鲜度见 --phase bus",
    }, "已核验" if guard.open else "不通过")

    # 使能：默认禁止（与 FIRST_ARM 红线 #1 一致）
    if not force_enable:
        out["enable_command"] = field(None, "不适用",
                                      how="本机臂上电即自使能，0x471 不需要也不应发；确需时 --force-enable")
    else:
        try:
            piper.EnableArm(7)
            time.sleep(0.5)
            flags = list(piper.GetArmEnableStatus()) if hasattr(piper, "GetArmEnableStatus") else []
            out["enable_command"] = field({"sent": True, "flags": flags}, "已核验",
                                          warn="违反 FIRST_ARM 红线 #1，仅限排障且已现场确认")
        except Exception as exc:
            out["enable_command"] = field(None, "未核验", error=repr(exc))

    # 最后一道闸：没有反馈帧时，上面所有「读数」都是 SDK 对象的初始默认值，
    # 必须整体降级为不通过，否则会产出看起来漂亮但完全错误的证据。
    live = _bus_liveness(iface)
    for key, item in list(out.items()):
        if isinstance(item, dict) and item.get("state") == "已核验" and not live.get("has_feedback"):
            item["state"] = "不通过"
            item["invalid_reason"] = ("总线上没有任何 0x251-0x256 / 0x2A1 反馈帧，"
                                      "SDK 返回的是新建对象默认值（全 0），不是实测值")
    out["bus_liveness"] = field(
        live,
        "已核验" if live.get("has_feedback") else "不通过",
        note=("收到反馈帧，读数可信" if live.get("has_feedback") else
              "0.6 s 内没有任何反馈帧；先解决 arm 上电/接线/波特率，再看这里"))
    return out


# ---------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description="PiPER x FACTR2 阶段 0 探针（只读）")
    ap.add_argument("--iface", default="can0", help="SocketCAN 接口名，默认 can0")
    ap.add_argument("--backend", default="socketcan", choices=["socketcan", "gs_usb", "pcan", "kvaser"],
                    help="python-can 后端；USB-CAN 按型号选，默认 socketcan")
    ap.add_argument("--phase", default="all",
                    choices=["all", "env", "bus", "sdk", "api", "selftest"])
    ap.add_argument("--seconds", type=float, default=5.0, help="总线监听时长，默认 5 s")
    ap.add_argument("--guard-seconds", type=float, default=3.0, help="首帧守卫观察窗口")
    ap.add_argument("--query-limits", action="store_true",
                    help="发一帧 0x472 查询并回读 0x473，记录本机真实限位/最大速度")
    ap.add_argument("--set-slave", action="store_true",
                    help="[仅双臂主从用] 发 0xFC 设为随动『运动输出臂』。"
                         "它不是程序控开关，单臂复现不需要，误用会把臂设成随动输出态")
    ap.add_argument("--to-can-ctrl", action="store_true",
                    help="[有副作用，必须同时给 --allow-write] 退出拖动示教并进入 CAN 指令控制："
                         "MotionCtrl_1(0,0,0) → MotionCtrl_2(0x01,0x01,100,0) → 回读 ctrl_mode。"
                         "下发运动指令前必须先过这一关；只给本参数而不给 --allow-write 时一帧都不会发")
    ap.add_argument("--factr2-repo", default=None, help="factr2_next 克隆路径，用于核对 commit")
    ap.add_argument("--allow-write", action="store_true",
                    help="允许发配置/查询帧（从站配置、0x472 限位查询、固件搜索）；不发运动指令")
    ap.add_argument("--force-enable", action="store_true",
                    help="[违反 FIRST_ARM 红线] 发 0x471 使能；本机臂上电即自使能，正常不需要")
    ap.add_argument("--i-understand-motion-risk", action="store_true",
                    help="--force-enable 的前置确认；工作空间内无人无障碍")
    ap.add_argument("--version", action="version",
                    version=f"stage0_probe {PROBE_VERSION}")
    ap.add_argument("--out", default=None,
                    help="输出 JSON 路径；默认 <本阶段目录>/env/<phase>_probe.json")
    args = ap.parse_args()

    # 路径默认值一律相对脚本自身定位，保证在任意 cwd 下运行都不丢产物
    case_dir = Path(__file__).resolve().parent.parent
    if args.factr2_repo is None:
        cand = case_dir.parent / "_vendor" / "factr2_next"
        args.factr2_repo = str(cand) if cand.exists() else None
    if args.out is None:
        args.out = str(case_dir / "env" / f"{args.phase}_probe.json")

    if args.phase == "selftest":
        return selftest()

    if args.force_enable and not args.i_understand_motion_risk:
        print("[ABORT] --force-enable 必须同时给 --i-understand-motion-risk；先清空工作空间。", file=sys.stderr)
        return 2
    if args.force_enable:
        print("[WARN] --force-enable 会发 0x471，与 FIRST_ARM 红线 #1（不发送使能/失能）冲突；"
              "本机臂上电即自使能，此开关正常应保持关闭。", file=sys.stderr)

    report: dict = {
        "probe": f"stage0_probe/{PROBE_VERSION}",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": platform.node(),
        "argv": sys.argv,
    }

    if args.phase in ("all", "env"):
        report["env"] = probe_env()
        report["ros_packages"] = probe_ros_pkgs()
        report["official_impl"] = probe_repo(args.factr2_repo)

    if args.phase in ("all", "bus"):
        report["bus"] = probe_bus(args.iface, args.seconds, args.backend)

    if args.phase in ("all", "sdk", "api"):
        piper, err, sdk_ver = _load_sdk(args.iface)
        if err:
            report["sdk"] = field(None, "未核验", error=err)
        else:
            report["sdk"] = {"version": field(sdk_ver, "已核验")}
            if args.phase in ("all", "api"):
                report["sdk"]["api"] = probe_api(piper)
            if args.phase in ("all", "sdk"):
                report["sdk"].update(probe_sdk(piper, args.iface, args.allow_write, args.force_enable,
                                             args.guard_seconds, args.query_limits,
                                             args.set_slave, args.to_can_ctrl))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # 覆盖前先归档：同一个默认名在同一天会被跑很多次（bus/sdk/query-limits/set-slave 全写
    # sdk_probe.json），后一次直接盖掉前一次。2026-10-19 13:05 就真发生过一次——限位刚
    # 回读到，紧接着 --set-slave 又把同一个文件覆盖，那份 0x473 数据至今没留下。
    # 这类"证据被自己的下一步冲掉"只能靠工具兜住，不能靠人记得改名。
    if out.exists():
        arch_dir = out.parent / "archive"
        arch_dir.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%S", time.localtime(out.stat().st_mtime))
        arch = arch_dir / f"{out.stem}_{stamp}.json"
        n = 0
        while arch.exists():          # 同一秒内连跑两次也不能覆盖旧归档
            n += 1
            arch = arch_dir / f"{out.stem}_{stamp}-{n}.json"
        shutil.copy2(out, arch)
        print(f"[ARCHIVE] 旧产物另存 {arch}")
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[OK] 写入 {out.resolve()}")

    # 限位回读成功后另存一份给 L0 指令层：stage1_cmd.py --limits 吃的就是这个格式。
    # 不手工转抄——手抄一次就多一个出错点，而且没人会去核对六行数。
    hl = ((report.get("sdk") or {}).get("hardware_limits") or {})
    if hl.get("state") == "已核验" and isinstance(hl.get("value"), dict):
        jl = out.parent / "joint_limits.json"
        jl.write_text(json.dumps(hl["value"], ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[OK] 限位另存 {jl}（stage1_cmd.py --limits 直接用）")

    # 一行摘要，方便贴进实验记录
    bus = report.get("bus", {}).get("frames", {})
    if isinstance(bus, dict) and bus.get("value"):
        hs = bus["value"]["by_group"]["high_speed_0x251_0x256"]
        print(f"[SUMMARY] 高速反馈 ID 命中 {len(hs['ids_seen'])}/6，缺失 {hs['missing_ids']}")
    elif isinstance(bus, dict) and bus.get("error"):
        print(f"[SUMMARY] 总线未打通：{bus['error']}")

    # 只有真跑过 sdk/api 相位才会有 report["sdk"]，env/bus 相位不该报「SDK 未加载」
    sdk = report.get("sdk")
    if isinstance(sdk, dict) and "version" not in sdk:
        print(f"[SUMMARY] SDK 未加载：{sdk.get('error')}")
    if isinstance(sdk, dict):
        fw = (sdk.get("firmware") or {}).get("value")
        en = ((sdk.get("enable_status_passive") or {}).get("value") or {})
        mode = (((sdk.get("status_0x2A1") or {}).get("value") or {}).get("ctrl_mode") or {})
        mode_s = (f"{mode.get('name')}(0x{mode.get('code'):x})"
                  if isinstance(mode, dict) and mode.get("code") is not None else "未读到")
        print(f"[SUMMARY] 固件={fw if fw is not None else '未核验'} "
              f"已使能={en.get('all_enabled')} 模式={mode_s}")
        g = sdk.get("first_frame_guard") or {}
        gv = g.get("value") or {}
        print(f"[SUMMARY] 守卫={g.get('state')} 拒收={json.dumps(gv.get('reject_reasons') or {}, ensure_ascii=False)}")
        if gv.get("urdf_violations"):
            print(f"[NOTE] 读数越出 URDF 标称限位 {json.dumps(gv['urdf_violations'], ensure_ascii=False)}"
                  " —— 不是坏值，是标称表不适用（门 8 未完成）；已按软判据只计数不拒收")
        if g.get("state") != "已核验":
            hint = _guard_hint(gv.get("reject_reasons") or {})
            if hint:
                print(f"[GUARD] {hint}")
        lim = sdk.get("hardware_limits") or {}
        lv = lim.get("value") or {}
        lim_state, n_j = lim.get("state"), (len(lv) if isinstance(lv, dict) else 0)
        # 「没查」和「查了没应答」必须长得不一样 —— R27 的坑正是这两者外观相同。
        # 13:34 那次没带 --query-limits，却照样打了「0x473 3 s 内无应答」，
        # 把「没查」误报成硬件故障，方向直接带偏。
        if lim_state == "未核验" and not lim.get("error") and not args.query_limits:
            lim_state = "未查（本次没带 --query-limits）"
        print(f"[SUMMARY] 限位={lim_state} 关节数={n_j}"
              + (f" err={lim.get('error')}" if lim.get("error") else ""))
        # 只有本次真的发了 0x472，零应答才是故障信号。给三条现场能直接验的排查路径，
        # 并把 raw 头一小段打出来——全零和真有数据一眼能分。
        if lim_state == "未核验":

            print("[LIMIT] 0x473 在 3 s 内没有任何关节应答，按顺序排查：")
            print("        1) 固件可能只在 CAN_CTRL(0x1) 回 0x473：加 --to-can-ctrl 再查（有副作用，人守急停）")
            print("        2) 关节未使能：本机 0x471 已使能=True，此项基本排除")
            print("        3) 示教按钮/手柄被按住会压住软件指令：松手再试")
            rr = lim.get("raw_repr")
            if rr:
                print(f"        raw 前 200 字：{rr[:200]}")
        if isinstance(lv, dict) and lv:
            for k in sorted(lv):
                j = lv[k] or {}
                mn, mx, vm = j.get("min_deg"), j.get("max_deg"), j.get("max_spd_rad_s")
                rng = (f"[{mn:+.2f}, {mx:+.2f}] deg" if mn is not None and mx is not None
                       else f"解析不全 {str(j)[:70]}")
                print(f"    {k}: {rng}  " + (f"vmax={vm:.3f} rad/s" if vm is not None else "vmax=?"))
            anyj = next(iter(lv.values())) or {}
            if anyj.get("spd_scale"):
                print(f"[NOTE] 速度量纲判定：{anyj['spd_scale']}（raw={anyj.get('spd_raw')}）")
        cc = sdk.get("can_ctrl") or {}
        ccv = cc.get("value") or {}
        if ccv.get("sent", ccv.get("steps")):
            after_c = ccv.get("after") or {}
            after_n = after_c.get("name") if isinstance(after_c, dict) else after_c
            print(f"[SUMMARY] 切程序控=已发 完成前={ccv.get('before')} 完成后={after_n}"
                  "  判据: 应为 CAN_CTRL(0x1)")
            # before == after 时「判据满足」这句话会被读成「指令生效了」，其实什么都没证明。
            # 13:34 那次就是：发指令前已是 CAN_CTRL，只能说明当前状态对。
            _b = str(ccv.get("before") or "").split("(")[0].strip()
            _a = str(after_n or "").split("(")[0].strip()
            if _b and _a and _b == _a:
                print(f"[NOTE] 发指令前就已是 {_a} —— 本次只证明「当前状态正确」，"
                      "没有观察到 TEACHING→CAN 的转移，指令有效性未验证。"
                      "臂下次断电重启会复位到示教态，那时跑同一条命令即可补上这个转移")
            if cc.get("state") != "已核验" and cc.get("note"):
                print(f"[WARN] {cc['note']}")
        elif cc.get("state") == "未发送":
            # 没发帧也必须留一行，否则「跑完了但什么都没做」和「跑完了且做对了」长得一样
            print(f"[SUMMARY] 切程序控=未发送 当前模式={ccv.get('before')}  "
                  f"判据: 未切换，禁止下发运动指令")
            print(f"[WARN] {cc.get('note')}")
        sm = sdk.get("slave_mode") or {}
        smv = sm.get("value") or {}
        if smv.get("sent"):
            after = smv.get("after") or {}
            print(f"[SUMMARY] 0xFC 主从=已发 完成前={smv.get('before')} 完成后="
                  f"{after.get('name') if isinstance(after, dict) else after}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
