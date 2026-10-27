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
    open_effort=800,     # 张开力限（0.001 N·m）。从 0 mm 极限位起步阻力大，400 会顶住不动
    close_effort=2000,   # 夹持力限（0.001 N·m）= 2.0 N·m（这是上限）。要温柔改成 600~1200
    hold_seconds=5.0,    # grasp 时张开后等你放物体的秒数
    squeeze_mm=1.0,      # 碰到物体后再往里压多少 mm（把「轻碰」变成「夹紧」）
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


def _ramp(p, target, effort, step=1500, lag=3000, stop_on_contact=False,
          timeout_s=25.0, stall_rounds=30):
    """渐进改变开度：每级 1.5 mm、40 ms，绝不一步到位（猛夹会撞坏东西）。

    2026-10-21 修两个现场问题：
    1. **死循环**：旧写法「跟不上就 cur = meas」，下一轮又 cur += step；
       当夹爪完全不动时，指令在 0→1500→3000→4500→0 之间转圈，
       永远到不了目标 —— 表现为「张开到一半卡住不动」（用户实际踩到）。
       现在：基准一律取实测值，并加「连续 stall_rounds 轮无变化即退出」。
    2. **零位起动阻力大**：从 0 mm 起步时容易顶住，加总超时避免无限等待。
    """
    t0 = time.time()
    cum, _ = _grip(p)
    last = cum
    stall = 0
    while True:
        if time.time() - t0 > timeout_s:
            print(f"    ！超时 {timeout_s:.0f}s，停在 {last/1000:.1f} mm")
            return last, False
        if stop_on_contact:
            if cum <= target:
                break
            cmd = max(target, cum - step)
        else:
            if abs(cum - target) <= step:
                cmd = target                      # 最后一步直接给目标
            else:
                cmd = cum + step if target > cum else cum - step
        _send(p, cmd, effort)
        meas, _ = _grip(p)
        if stop_on_contact and meas > cum + lag:
            return meas, True                     # 被物体挡住
        if abs(meas - last) < 50:                 # <0.05 mm 认为没动
            stall += 1
            if stall >= stall_rounds:
                print(f"    ！连续 {stall_rounds} 轮无进展，停在 {meas/1000:.1f} mm 退出")
                return meas, False
        else:
            stall = 0
        last = meas
        cum = meas                                # 指令跟实测走
        if not stop_on_contact and abs(cum - target) <= 50:
            break
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


def close_by_force(p, target_effort=None, step_mm=1.0, hold_frames=2,
                   stall_steps=5, stop_ratio=0.85, verbose=False):
    """边走边读夹持力，力到就停 —— 不按预定行程硬走。

    为什么换掉旧做法「开度差 > 3 mm 判接触」
    ------------------------------------
    * 物体薄/软：一步 1.5 mm 直接压过去，开度差来不及显出来 → 程序以为没夹到，继续顶；
    * 物体硬：一步顶到位，把东西推走 → 现场表现就是「合不上 / 夹不住」。

    三个必须注意的实测细节（2026-10-21 现场踩到，全是夹爪开环特性）
    ------------------------------------------------------------
    1. **夹爪不响应单帧指令**。只发 2 帧（40 ms）就换下一个目标，实测开度**一动不动**
       （指令 39.2 → 实测 39.5 恒定）。⇒ 每步必须**持续重发同一目标**若干帧，
       给夹爪时间真的走过去。这也是 `grip.py hold` 每 0.1 s 重发的原因。
    2. **单步延迟 ≠ 被物体挡住**。所以「没进展」要**连续 stall_steps 步**才算接触。
    3. **effort 会随开合方向变号**（张开读到负值）。⇒ 判力用 `abs(eff)`。

    返回 (开度, 力, 是否夹到东西)。
    """
    tgt = int(target_effort if target_effort is not None else CFG["close_effort"])
    cur, eff = _grip(p)
    if verbose:
        print(f"  起始开度 {cur/1000:.1f} mm，力限 {tgt}")
    stall = 0
    while cur > 0:
        nxt = max(0, cur - int(step_mm * 1000))
        # 持续重发同一目标：夹爪需要连续指令才会真正移动
        for _ in range(hold_frames):
            _send(p, nxt, tgt, n=2)
            meas, eff = _grip(p)
            if abs(eff) >= tgt * stop_ratio:      # ① 力到了 ⇒ 已经接触
                if verbose:
                    print(f"    指令 {nxt/1000:5.1f}  实测 {meas/1000:5.1f} mm  "
                          f"力 {eff:5d}  ← 接触，准备补压")
                # 「碰到」不等于「夹稳」：只轻轻接触时摩擦力不够，重物会滑落
                # （2026-10-21 用户放偏重物体，夹爪停在 48.3 mm、力随后掉回 −39，
                #   就是碰到了但没夹住）。这里再往里压 squeeze_mm，
                # 力由 tgt 限住所以不会夹坏；压完才认为夹稳。
                extra = int(CFG.get("squeeze_mm", 1.0) * 1000)
                tgt_ang = max(0, meas - extra)
                for _ in range(8):
                    _send(p, tgt_ang, tgt, n=2)
                meas2, eff2 = _grip(p)
                if verbose:
                    print(f"    补压 {extra/1000:.1f} mm → 实测 {meas2/1000:.1f} mm  "
                          f"力 {eff2:5d}")
                elif True:
                    print()
                return meas2, eff2, True
        if verbose:
            print(f"    指令 {nxt/1000:5.1f}  实测 {meas/1000:5.1f} mm  "
                  f"力 {eff:5d}  停滞 {stall}")
        else:
            # 单行刷新：用户能看到夹爪在动、力在涨，而不是「毫无反应」
            print(f"\r  闭合中… {meas/1000:5.1f} mm   力 {eff:6d}", end="", flush=True)
        # ② 连续多步没有进展 ⇒ 被挡住
        if meas <= cur - 100:                 # 真的往里走了（0.1 mm）
            stall = 0
            cur = meas
        else:
            stall += 1
            if stall >= stall_steps:
                if not verbose:
                    print()
                return meas, eff, True
            cur = meas                         # 卡住就别再往下压指令
    if not verbose:
        print()
    return cur, eff, False


def start_hold(angle, effort):
    """夹住之后必须有人**持续**发夹爪指令，否则会松掉。

    这里有两个坑，2026-10-21 用户报「夹以后没有继续施加力」时全中：
      1. **`GripperCtrl` 是「设定目标」而不是「一直施力」**。若保持目标 == 实测开度，
         夹爪认为已经到位、不再出力，实测 effort 立刻从 1.0 N·m 掉回 −0.03 N·m，
         重物就滑了。⇒ 保持目标必须**压进 squeeze_mm**（小于实测开度）。
      2. **进程退出后没人发帧**，夹爪回到松弛状态。⇒ 起一个后台进程每 0.1 s 重发。
         这个进程在菜单后续操作里会一直活着；要松开就张开夹爪（会自动把它清掉）。

    返回后台进程 pid。
    """
    _kill_stale()
    cmd = [sys.executable, str(HERE / "grip.py"), "hold",
           "--angle", str(int(angle)), "--effort", str(int(effort)), "--secs", "0"]
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(0.4)          # 等它把 CAN 连上并开始发帧
    return p.pid


def _hold_angle(measured_ang):
    """保持目标 = 实测 - squeeze，保证夹爪持续出力。"""
    return max(0, int(measured_ang) - int(CFG["squeeze_mm"] * 1000))


def cmd_close(a):
    _kill_stale()
    p = _piper(a.iface)
    cur, _ = _grip(p)
    print(f"闭合：{cur/1000:.1f} mm 起，边合边读夹持力，"
          f"到力限 {a.effort} 的 85% 就停")
    ang, eff, hit = close_by_force(p, a.effort, verbose=getattr(a, "verbose", False))
    if hit and ang >= 1000:
        hold_ang = _hold_angle(ang)
        pid = start_hold(hold_ang, a.effort)
        print(f"完成：已夹住，实测开度 {ang/1000:.1f} mm"
              f"（≈物体宽度 {ang/1000:.1f} mm）")
        print(f"  保持目标 {hold_ang/1000:.1f} mm，力限 {a.effort/1000:.1f} N·m，"
              f"后台保持 pid {pid}")
        print("  夹持力会一直保持。要松开：菜单按 2 张开；"
              "或手动 pkill -f 'grip.py hold'")
    else:
        print(f"完成：夹爪合到底（{ang/1000:.1f} mm），里面没有东西")
        print("  如果确实放了物体：它可能没在两指之间，或比夹爪行程还宽")


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
    print(f"3) 边合边读夹持力（力限 {a.effort}）")
    ang, eff, hit = close_by_force(p, a.effort, verbose=getattr(a, "verbose", False))
    if hit and ang >= 1000:
        hold_ang = _hold_angle(ang)
        pid = start_hold(hold_ang, a.effort)
        print(f"4) 已夹住：实测开度 {ang/1000:.1f} mm，保持力 {a.effort/1000:.1f} N·m，"
              f"后台保持 pid {pid}")
    else:
        print(f"4) 空夹（合到底没阻力）：开度 {ang/1000:.1f} mm  ← 没夹到东西，请重试")


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

    c = sub.add_parser("close", help="闭合（边合边读力，力到即停）")
    c.add_argument("--effort", type=int, default=CFG["close_effort"])
    c.add_argument("--verbose", action="store_true", help="逐步打印指令/实测/力")

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
