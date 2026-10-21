#!/usr/bin/env python3
"""weigh_arm.py —— 夹爪开合 + 用关节电流称物体重量（单文件工具）

放在 VM：`~/stage0/20261020_stage4/weigh_arm.py`

它能做什么
----------
  status        看夹爪开度/力矩状态
  open          张开夹爪（默认全开 70 mm，可用 --mm 改）
  close         闭合夹爪（默认 1.2 N·m 力限，可用 --effort 改）
  grasp         张开 → 等你 N 秒放物体 → 轻力渐进闭合（一次到位）
  measure       跑两个姿态（朝前 + 掌心朝上）各 30 s，算出物体质量与质心
  full          等价于 grasp + measure，一条命令走完

为什么两个姿态
--------------
载荷力矩 `τ_pay = m·(p0(q) + A3(q)·Φ)`。把未知量换成 **(m, Ψ=mΦ)** 后是线性的：

    −Δτ = m·p0(q) + A3(q)·Ψ

但「只朝前」的一堆姿态里 p0/A3 张成的方向几乎相同，单方位**秩亏**（第 4 个奇异值=0），
m 与 Φ 分不开；加一个「掌心朝上」（工具轴竖直、重力方向变成 (0,0,−1)）才满秩。
实测 500 g 砝码：两段解出 0.468 kg（−6.5%），单段 0.604 kg（+20.8%，不可用）。

为什么不需要神经网络
--------------------
同姿态空载差分 `Δτ = K·(I_带载 − I_空载)` 里，`τ̂_free(q)` 被完全消掉，
所以只用电流 + 几何（URDF）就够，不依赖阶段 4 的 L1 模型，VM 上无需 torch。

空载基线从 `episodes/M0_A` 与 `episodes/M0_U1` 现算（这两个段与物体无关，可永久复用）。

修改夹爪参数
------------
文件末尾 `CFG` 里集中放了「全开行程 / 夹持力限 / 等你放物体的秒数」，
改那三个数就行，不必翻代码。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STAGE3 = HERE.parent / "20261019_stage3"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(STAGE3))

import s3_design as D      # noqa: E402  提供 load()/deg()/tcp_and_axis
import stage2_design as S  # noqa: E402  提供 END_LINK

# ---------------------------------------------------------------------------
# 可以直接改的参数
# ---------------------------------------------------------------------------
CFG = dict(
    open_mm=70.0,        # 张开行程（mm）。机械上限约 68.5 mm，给 70 就是全开
    open_effort=400,     # 张开时的力限（0.001 N·m）。太大没必要，400 足够
    close_effort=1200,   # 夹持力限（0.001 N·m）= 1.2 N·m。要更紧改成 2000
    hold_seconds=5.0,    # grasp 时张开后等你放物体的秒数
    squeeze_mm=0.6,      # 夹到接触后再压进多少 mm（形成稳定夹持力）
    hold_s=12,           # measure 每个姿态静止保持秒数（越长噪声越低）
    rec_s=30,            # measure 每个姿态录制秒数（必须 > 到达时间 + hold_s）
)

K = np.array([4.7185, 4.7185, 4.7185, 0.7446, 0.7446, 0.7446])   # 阶段 2 实测
ONLY = [1, 2, 3, 4]          # J2~J5
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
FS = 50.0                    # episodes 记录频率
G = 9.81

# 两个互补的姿态：朝前（工具轴水平）+ 掌心朝上（工具轴竖直）
POSE_FWD = (0, 30, -50, 0, 25, 50.6)
POSE_UP = (0, 30, -50, 0, -65, 50.6)
DT = 0.04


# ---------------------------------------------------------------------------
# 夹爪
# ---------------------------------------------------------------------------
def _kill_stale():
    """杀掉会跟本工具抢夹爪/关节的旧进程。

    `grip.py hold` 每 0.1 s 把夹爪往固定开度拉，`hold_pose.py` 每 10 ms 重发关节角；
    只要它们还在跑，`open/close/grasp` 就会和它们对着干 →
    现场表现是「夹爪一直抖动」（2026-10-20 19:4x 踩到，用户以为臂在晃）。
    """
    import signal
    killed = []
    for pat in ("grip.py hold", "hold_pose.py"):
        out = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True).stdout
        for pid in out.split():
            try:
                os.kill(int(pid), signal.SIGKILL)
                killed.append(f"{pat}({pid})")
            except Exception:
                pass
    if killed:
        print(f"[清理] 已停止抢控制权的进程：{', '.join(killed)}")
        time.sleep(1.0)


def _piper(iface="can0"):
    from piper_sdk import C_PiperInterface
    p = C_PiperInterface(iface)
    if p.ConnectPort() is False:      # 注意：0.6.1 的 ConnectPort 返回 None，不能用 not
        raise SystemExit("连不上 can0：检查总线/供电")
    time.sleep(1.0)
    return p


def _grip(p):
    st = p.GetArmGripperMsgs().gripper_state
    return int(st.grippers_angle), int(st.grippers_effort)


def _send(p, ang, effort, n=2):
    for _ in range(n):
        p.GripperCtrl(int(ang), int(effort), 0x01, 0)
        time.sleep(DT / n)


def _ramp(p, target, effort, step=1500, lag=3000, stop_on_contact=False):
    """渐进改变开度：每级 1.5 mm、40 ms，绝不一步到位（猛夹会撞坏东西）。"""
    cur, _ = _grip(p)
    while True:
        if stop_on_contact:
            if cur <= target:
                break
            cur = max(target, cur - step)
        else:
            if cur == target:
                break
            cur = min(target, cur + step) if target > cur else max(target, cur - step)
        _send(p, cur, effort)
        meas, _ = _grip(p)
        if stop_on_contact and meas > cur + lag:
            return meas, True          # 被物体挡住
        if not stop_on_contact and abs(meas - cur) > lag:
            cur = meas                 # 跟不上就按实测走，不硬顶
    return _grip(p)[0], False


def cmd_status(a):
    p = _piper(a.iface)
    ang, eff = _grip(p)
    js = p.GetArmJointMsgs().joint_state
    q = [getattr(js, f"joint_{i}") / 1000.0 for i in range(1, 7)]
    print(f"夹爪开度 {ang}（{ang/1000:.1f} mm）  effort {eff}")
    print(f"关节角 {[round(v, 1) for v in q]}  "
          f"ctrl_mode {p.GetArmStatus().arm_status.ctrl_mode}")


def cmd_open(a):
    _kill_stale()
    p = _piper(a.iface)
    tgt = int(round(a.mm * 1000))
    cur, _ = _grip(p)
    print(f"张开：{cur/1000:.1f} mm → {tgt/1000:.1f} mm（力限 {a.effort}）")
    _ramp(p, tgt, a.effort)
    ang, eff = _grip(p)
    print(f"完成：{ang/1000:.1f} mm（effort {eff}）"
          f"{'  ← 已到机械上限' if ang < tgt - 2000 else ''}")


def cmd_close(a):
    _kill_stale()
    p = _piper(a.iface)
    cur, _ = _grip(p)
    print(f"闭合：{cur/1000:.1f} mm → 0（力限 {a.effort}，碰到即停）")
    hit, hitflag = _ramp(p, 0, a.effort, stop_on_contact=True)
    hold = max(0, hit - int(CFG["squeeze_mm"] * 1000))
    for _ in range(12):
        _send(p, hold, a.effort)
    ang, eff = _grip(p)
    print(f"完成：开度 {ang/1000:.1f} mm"
          f"{'（夹到物体）' if hitflag else '（空夹，到底了）'}  effort {eff}")


def cmd_grasp(a):
    _kill_stale()
    p = _piper(a.iface)
    tgt = int(round(CFG["open_mm"] * 1000))
    cur, _ = _grip(p)
    print(f"1) 张开：{cur/1000:.1f} → {CFG['open_mm']:.1f} mm")
    _ramp(p, tgt, CFG["open_effort"])
    ang, _ = _grip(p)
    print(f"   已张开 {ang/1000:.1f} mm")
    n = int(a.hold_s)
    print(f"2) >>> 请在 {n} 秒内置入物体 <<<")
    for s in range(n, 0, -1):
        print(f"   {s} …", flush=True)
        time.sleep(1)
    print(f"3) 轻力闭合（力限 {a.effort}）")
    hit, hitflag = _ramp(p, 0, a.effort, stop_on_contact=True)
    hold = max(0, hit - int(CFG["squeeze_mm"] * 1000))
    for _ in range(12):
        _send(p, hold, a.effort)
    ang, eff = _grip(p)
    print(f"4) 夹持开度 {ang/1000:.1f} mm，effort {eff}"
          f"{'' if hitflag else '  ← 没夹到东西，请重试'}")


# ---------------------------------------------------------------------------
# 数据与求解
# ---------------------------------------------------------------------------
def steady_current(ep_dir):
    """取该段末 10 s 的稳态电流均值（跳过段首接近段）。"""
    rows = [json.loads(l) for l in open(Path(ep_dir) / "samples.jsonl", encoding="utf-8")]
    first = next((i for i, r in enumerate(rows) if r.get("q_cmd") is not None), 0)
    rows = [r for r in rows[first:] if r.get("cur") and r.get("q") and None not in r["q"]]
    if not rows:
        raise SystemExit(f"{ep_dir} 没有有效稳态帧")
    rows = rows[-int(10.0 * FS):]
    cur = np.array([r["cur"] for r in rows], float)
    q = np.array([r["q"] for r in rows], float).mean(axis=0)
    return cur.mean(axis=0), cur.std(axis=0), q


def basis(arm, q_deg):
    """返回 (p0, A3)：τ_pay(1 kg, Φ) = p0 + A3·Φ。

    **p0 不能省**——`tau_point_force` 的 local 是相对法兰的偏移，local=0 时力矩≠0。
    """
    q = D.G.deg(*q_deg)
    F = [0, 0, -G]
    p0 = np.asarray(arm.tau_point_force(list(q), S.END_LINK, [0.0, 0.0, 0.0], F))[ONLY]
    cols = []
    for k in range(3):
        e = np.eye(3)[k]
        cols.append(np.asarray(arm.tau_point_force(list(q), S.END_LINK, list(e), F))[ONLY])
    A3 = np.stack(cols, axis=1) - p0[:, None]
    return p0, A3


def solve_mass(tag):
    """用 {tag}_A 与 {tag}_U1 相对空载基线，解出 (m, Ψ) 并给出 Φ。"""
    urdf, arm = D.load()
    rows, y, dlist = [], [], []
    for pose, seg, base, qd in (("A", f"{tag}_A", "M0_A", POSE_FWD),
                                ("U1", f"{tag}_U1", "M0_U1", POSE_UP)):
        i_new, s_new, q_new = steady_current(HERE / "episodes" / seg)
        i_base, _, q_base = steady_current(HERE / "episodes" / base)
        dtau = K * (i_new - i_base)
        dq = np.degrees(q_new - q_base)
        if np.max(np.abs(dq)) > 1.5:
            print(f"  ！{seg} 与基线姿态差 {np.max(np.abs(dq)):.1f}°（应 <1.5°），"
                  f"该段可能被别的进程干扰")
        print(f"  {seg}：Δτ(J2~J5) = " + " ".join(f"{v:+.3f}" for v in dtau[ONLY]) + " N·m"
              + f"   （单帧 std 最大 {np.max(K[ONLY]*s_new[ONLY]):.3f} N·m）")
        dlist.append(dict(seg=seg, pose=pose,
                          dtau=[float(v) for v in dtau[ONLY]],
                          dq_max_deg=float(np.max(np.abs(dq))),
                          std_max_nm=float(np.max(K[ONLY] * s_new[ONLY]))))
        p0, A3 = basis(arm, qd)
        rows.append(np.hstack([p0[:, None], A3]))
        y.append(-dtau[ONLY])
    A = np.vstack(rows)
    y = np.concatenate(y)
    s = np.linalg.svd(A, compute_uv=False)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    rms = float(np.sqrt(np.mean((A @ x - y) ** 2)))
    return x, s, rms, dlist


def cmd_measure(a):
    tag = a.tag or time.strftime("N%m%d_%H%M%S")
    if getattr(a, "reuse", False):   # `full` 没有这个参数，必须用 getattr
        print(f"=== 复用已有数据 {tag}_A / {tag}_U1（不动机器人）===")
        x, s, rms, dl = solve_mass(tag)
        _report(tag, x, s, rms, dl)
        return
    print(f"=== 称重 {tag}：跑「朝前」+「掌心朝上」两段 ===")
    base_env = dict(os.environ, ENABLE="1", RATE="6", MAX_CURRENT_A="3.0",
                    HOLD_S=str(CFG["hold_s"]), REC_S=str(CFG["rec_s"]))
    for grp, pose in (("V", "A"), ("U", "U1")):
        env = dict(base_env, TAG=tag, GROUP=grp, ONLY_POSES=pose)
        print(f"\n--- {grp} 族 / 姿态 {pose} ---")
        rc = subprocess.call(["bash", str(HERE / "run_s5_load.sh")],
                             cwd=str(HERE), env=env)
        if rc != 0:
            raise SystemExit(f"{grp}/{pose} 采集失败（退出码 {rc}），看 raw/s5_*.log")

    print("\n=== 解算 ===")
    x, s, rms, dl = solve_mass(tag)
    _report(tag, x, s, rms, dl)


def _report(tag, x, s, rms, dl):
    m, psi = float(x[0]), x[1:]
    print(f"设计矩阵奇异值 {np.round(s, 3).tolist()}  条件数 {s[0]/s[-1]:.1f}")
    if s[-1] < 1e-3 * s[0]:
        print("！秩亏：两个姿态方向太接近，解不可信")
    print(f"残差 RMS = {rms:.4f} N·m")
    print()
    print(f"  ==> 质量  m = {m:+.3f} kg")
    print(f"      质心  Φ = {np.round(psi / m * 1000, 1).tolist()} mm"
          f"（|Φ| = {np.linalg.norm(psi / m) * 1000:.1f} mm）")
    # ---- 落盘：每次称重追加一行，便于事后对比/画图 ----
    rec = dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), tag=tag,
               mass_kg=m, phi_mm=[float(v) for v in psi / m * 1000],
               phi_norm_mm=float(np.linalg.norm(psi / m) * 1000),
               cond=float(s[0] / s[-1]) if s[-1] > 0 else None,
               rms_nm=rms, segments=dl)
    out = HERE / "runs" / "weigh_results.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    print(f"\n结果已追加到 {out}（每行一条，tag={tag}）")
    print(f"查看：tail -n 5 {out}")


def cmd_full(a):
    cmd_grasp(a)
    time.sleep(1.0)
    cmd_measure(a)


def main():
    ap = argparse.ArgumentParser(description="夹爪开合 + 电流称重")
    ap.add_argument("--iface", default="can0")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status", help="看夹爪/关节状态")

    o = sub.add_parser("open", help="张开")
    o.add_argument("--mm", type=float, default=CFG["open_mm"])
    o.add_argument("--effort", type=int, default=CFG["open_effort"])

    c = sub.add_parser("close", help="闭合（碰到即停）")
    c.add_argument("--effort", type=int, default=CFG["close_effort"])

    g = sub.add_parser("grasp", help="张开→等N秒→轻力闭合")
    g.add_argument("--hold-s", type=float, default=CFG["hold_seconds"])
    g.add_argument("--effort", type=int, default=CFG["close_effort"])

    m = sub.add_parser("measure", help="两段采集并解出质量")
    m.add_argument("--tag", default=None)
    m.add_argument("--reuse", action="store_true", help="不采集，直接用已有的 {tag}_A / {tag}_U1 解算")

    f = sub.add_parser("full", help="grasp + measure")
    f.add_argument("--hold-s", type=float, default=CFG["hold_seconds"])
    f.add_argument("--effort", type=int, default=CFG["close_effort"])
    f.add_argument("--tag", default=None)

    a = ap.parse_args()
    {"status": cmd_status, "open": cmd_open, "close": cmd_close,
     "grasp": cmd_grasp, "measure": cmd_measure, "full": cmd_full}[a.cmd](a)


if __name__ == "__main__":
    main()
