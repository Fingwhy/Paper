#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""weigh_app.py —— 机械臂称重的菜单式应用（给日常使用，不用记参数）

放在 VM：`~/stage0/20261020_stage4/weigh_app.py`

设计原则
--------
1. **不重复实现硬件交互**：夹爪/采集/解算全部调用 `weigh_arm.py` 里已经实测过的函数。
   这个文件只负责「菜单 + 参数收集 + 结果显示」，出问题的地方一定是老代码，好查。
2. **两种称法，用途不同**：
   * 双姿态（朝前 + 掌心朝上）：不用任何预标定，能同时解出质量与质心，误差 3~6.5%；
   * 单姿态（朝前）：借用已标定的质心 Φ，只需一半时间，**300 g 以上约 2%**。
     留出验证实测：300 g +1.7%、500 g +1.8%、100 g +25%（100 g 不要用单姿态）。
3. **启动时自检环境**：CAN 是否 up、bitrate 是否 1000000、是否 CAN_CTRL 模式、
   有没有抢控制权的残留进程。这些是现场最常卡住的地方。

安全（写死在这里，不做成选项）
------------------------------
* 夹爪闭合永远是「每 1.5 mm 一步、碰到即停、再压 0.6 mm」，不会一步夹死；
* 默认夹持力 1.2 N·m，要改必须在菜单里显式输入；
* **J6 单侧行程 [0°, +343°]，0° 是硬端点**：本应用不会让 J6 往负方向走；
* 退出不会失能机械臂，只是不再发指令（约 10 min 后固件自动回落示教模式）。
  **断电前一定要用手托住**——实测断电六轴会自由落体。

用法（VM 上）：
    python3 weigh_app.py            # 进菜单
    python3 weigh_app.py --selftest # 不连臂，用已有数据验算解算是否正常
"""

import sys
import time
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import weigh_arm as W  # noqa: E402  夹爪 / 采集 / 解算全部复用

# 阶段 5 全数据标定的质心（mm，相对法兰）。单姿态解算要用它。
PHI_MM = np.array([1.5, 1.6, 93.2])
HISTORY = HERE / "runs" / "weigh_results.jsonl"


# ---------------------------------------------------------------------------
# 单姿态解算
# ---------------------------------------------------------------------------
def solve_single(tag, pose="A"):
    """只用「朝前」一段，借已标定的 Φ 反解质量。

    符号约定（与 `weigh_arm.solve_mass` 一致，别自己推）：
        Δτ = K·(I_带载 − I_空载) = −m·(p0 + A3·Φ)
    ⇒   y = −Δτ = m·(p0 + A3·Φ)，4 个方程（J2~J5）最小二乘解 1 个未知数 m。
    **负号不能省**：直接用 +Δτ 解会得到 −0.30 kg（幅度对、符号反，2026-10-21 踩过）。
    """
    urdf, arm = W.D.load()
    qd = W.POSE_FWD if pose == "A" else W.POSE_UP
    seg, base = f"{tag}_{pose}", f"M0_{pose}"
    i_new, s_new, q_new = W.steady_current(HERE / "episodes" / seg)
    i_base, _, q_base = W.steady_current(HERE / "episodes" / base)
    dtau = W.K * (i_new - i_base)
    dq = np.degrees(q_new - q_base)
    p0, A3 = W.basis(arm, qd)
    v = p0 + A3 @ (PHI_MM / 1000.0)          # 每 1 kg 在该姿态产生的力矩（带方向）
    y = -dtau[W.ONLY]
    m = float(y @ v / (v @ v))
    rms = float(np.sqrt(np.mean((y - m * v) ** 2)))
    noise = float(np.max(W.K[W.ONLY] * s_new[W.ONLY]))
    return dict(tag=tag, pose=pose, mass_kg=m, rms_nm=rms,
                dq_max_deg=float(np.max(np.abs(dq))), std_max_nm=noise,
                dtau=[float(x) for x in dtau[W.ONLY]])


def _save(rec):
    HISTORY.parent.mkdir(parents=True, exist_ok=True)
    with open(HISTORY, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


# ---------------------------------------------------------------------------
# 环境自检
# ---------------------------------------------------------------------------
def _fix_hint():
    return ["", "修复 CAN（sudo 要密码）：",
            "  sudo ip link set can0 down",
            "  sudo ip link set can0 type can bitrate 1000000",
            "  sudo ip link set can0 up",
            "切到程序控制模式：",
            "  cd ~/stage0/20261019_stage0/scripts",
            "  python3 stage0_probe.py --iface can0 --phase sdk --allow-write --to-can-ctrl"]


def check_env(verbose=True):
    """返回 (ok, 说明列表)。只读检查，不改系统状态。"""
    import subprocess
    msgs, ok = [], True

    def sh(cmd):
        # errors="replace"：Windows 上 `ip`/`pgrep` 的输出是 GBK，按 UTF-8 解码会抛
        # UnicodeDecodeError（2026-10-21 在 Windows 侧踩到）。VM 上是 UTF-8，不受影响。
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              errors="replace").stdout.strip()

    def dump():
        if verbose:
            for m in msgs:
                print("  " + m)

    # 1) CAN 接口
    out = sh("ip -d link show can0 2>/dev/null")
    if not out:
        msgs.append("✗ 找不到 can0：USB-CAN 没插好，或 gs_usb 驱动没加载")
        msgs.append("  试：sudo modprobe gs_usb")
        dump()
        return False, msgs
    if "state UP" not in out:
        # 不往下走：can0 是 DOWN 时 SDK 会打一串难懂的 ERROR，不如直接给处理办法
        msgs.append("✗ can0 是 DOWN（机械臂断电，或接口没拉起）")
        msgs += _fix_hint()
        dump()
        return False, msgs
    if "bitrate 1000000" not in out:
        msgs.append("✗ can0 的 bitrate 不是 1000000：数据会全乱")
        msgs += _fix_hint()
        dump()
        return False, msgs
    msgs.append("✓ can0 已 up，bitrate 1000000")

    # 2) 抢控制权的进程
    busy = sh("pgrep -af 'grip.py hold|hold_pose.py' | grep -v pgrep")
    if busy:
        msgs.append(f"⚠ 有进程在抢控制权（测重会抖动），选 9 可清理：\n    {busy}")
    else:
        msgs.append("✓ 没有抢控制权的残留进程")

    # 3) ctrl_mode
    try:
        from piper_sdk import C_PiperInterface
        p = C_PiperInterface("can0")
        if p.ConnectPort() is False:
            raise RuntimeError("ConnectPort 失败")
        time.sleep(0.8)
        mode = p.GetArmStatus().arm_status.ctrl_mode
        en = p.GetArmEnableStatus()
        en_ok = all(en) if isinstance(en, (list, tuple)) else bool(en)
        msgs.append(f"{'✓' if mode == 1 else '✗'} ctrl_mode = {mode}"
                    f"（1=CAN_CTRL 可用，2=TEACHING 需切换）")
        msgs.append(f"{'✓' if en_ok else '✗'} 六轴使能 = {en_ok}")
        if mode != 1 or not en_ok:
            ok = False
    except Exception as e:
        msgs.append(f"✗ 连不上机械臂：{e}")
        ok = False

    if not ok:
        msgs += _fix_hint()

    dump()
    return ok, msgs


# ---------------------------------------------------------------------------
# 各菜单动作
# ---------------------------------------------------------------------------
def do_status():
    W.cmd_status(type("A", (), {"iface": "can0"}))


def do_open():
    s = input("  开度 mm（回车 = 全开 70）。先看物体多宽，开得比它宽 5~10 mm 才放得进去：").strip()
    mm = float(s) if s else W.CFG["open_mm"]
    if not (0 <= mm <= 70):
        print("  开度要在 0~70 mm 之间")
        return
    W.cmd_open(type("A", (), {"iface": "can0", "mm": mm,
                              "effort": W.CFG["open_effort"]}))
    print("  保持这个开度，把物体塞进两指之间（不是放在指尖外面）")


def do_close():
    s = input(f"  力度 100~2000（回车 = {W.CFG['close_effort']}，数值越大夹得越紧）：").strip()
    eff = int(s) if s else W.CFG["close_effort"]
    if not (100 <= eff <= 2000):
        print("  力度要在 100~2000 之间")
        return
    print("  边合边读夹持力，力到就停（不按固定行程硬走）")
    W.cmd_close(type("A", (), {"iface": "can0", "effort": eff}))


def _ask_tag():
    s = input("  给这个物体起个名字（回车 = 用时间戳）：").strip()
    return s or time.strftime("N%m%d_%H%M%S")


def do_measure_full():
    """双姿态：不用预标定，同时给质量和质心。"""
    print("\n  双姿态称重：朝前 + 掌心朝上，约 1.5~2 分钟")
    print("  适合：物体像砝码那样垂挂（质心离夹爪远）")
    print("  注意：质心靠近夹爪的物体可能高估 30%（实测 400 g 测成 507/530）")
    print("        这种情况请改用第 5 项（单姿态）")
    tag = _ask_tag()
    W.cmd_measure(type("A", (), {"iface": "can0", "tag": tag, "reuse": False}))


def do_measure_fast():
    """单姿态：借已标定的 Φ，省一半时间。"""
    print("\n  单姿态称重：只跑「朝前」一个姿态，约 40 秒")
    print("  适用：300 g 以上（实测 400 g 物体测出 402~430 g，偏 +0.5~7.5%）")
    print("  不适用：100 g 这种很轻的（误差会到 25%）")
    tag = _ask_tag()
    print(f"\n  === 采集 {tag}_A ===")
    import os
    import subprocess
    env = dict(os.environ, ENABLE="1", RATE="6", MAX_CURRENT_A="3.0",
               HOLD_S=str(W.CFG["hold_s"]), REC_S=str(W.CFG["rec_s"]),
               TAG=tag, GROUP="V", ONLY_POSES="A")
    rc = subprocess.call(["bash", str(HERE / "run_s5_load.sh")], cwd=str(HERE), env=env)
    if rc != 0:
        print(f"  采集失败（退出码 {rc}），看 raw/s5_*.log")
        return
    print("\n  === 解算 ===")
    r = solve_single(tag, "A")
    print(f"  姿态差 {r['dq_max_deg']:.2f}°（应 <1.5°）")
    print(f"  残差 RMS {r['rms_nm']:.4f} N·m   单帧噪声 {r['std_max_nm']:.3f} N·m")
    if r["dq_max_deg"] > 1.5:
        print("  ！姿态差偏大：可能被别的进程干扰，或没走到位，结果不可信")
    print()
    print(f"  ==> 质量  {r['mass_kg']:.3f} kg  （{r['mass_kg']*1000:.0f} g）")
    _save(dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), tag=tag, mode="single_A",
               mass_kg=r["mass_kg"], rms_nm=r["rms_nm"],
               dq_max_deg=r["dq_max_deg"], phi_mm=PHI_MM.tolist()))
    print(f"  已记录到 {HISTORY.name}")


def do_full_auto():
    """张开 → 等你放 → 夹紧 → 双姿态称重，一条龙。"""
    print("\n  一键称重：张开夹爪 → 你放东西 → 自动夹紧 → 自动称重")
    tag = _ask_tag()
    n = int(W.CFG["hold_seconds"])
    print(f"\n  接下来夹爪会张开，然后有 {n} 秒让你把东西放进去。")
    input("  准备好了按回车开始…")
    W.cmd_grasp(type("A", (), {"iface": "can0", "hold_s": n,
                               "effort": W.CFG["close_effort"]}))
    input("\n  夹稳了吗？按回车开始称重（要移开手）…")
    W.cmd_measure(type("A", (), {"iface": "can0", "tag": tag, "reuse": False}))


def do_history():
    if not HISTORY.exists():
        print("  还没有记录")
        return
    rows = [json.loads(l) for l in open(HISTORY, encoding="utf-8")]
    rows = rows[-12:]
    print(f"\n  最近 {len(rows)} 次称重（{HISTORY.name}）")
    print("  " + "-" * 66)
    print(f"  {'时间':<17} {'名称':<14} {'质量':>10}  {'方式':<9} 残差")
    print("  " + "-" * 66)
    for r in rows:
        mode = r.get("mode", "dual" if "phi_norm_mm" in r else "?")
        print(f"  {r['time'][:16]:<17} {r['tag'][:13]:<14} "
              f"{r['mass_kg']*1000:>8.0f} g  {mode:<9} {r.get('rms_nm', 0):.3f}")
    # 同一物体多次称重的一致性：**按完整名字分组**，不能按前缀。
    # 踩过：按 "_" 前缀分组会把 N1020_195135 / _195640 / _195925 当成同一个物体，
    # 显示「极差 240 g」，其实是三次不同的测量（不同时间/不同水位）。
    from collections import defaultdict
    grp = defaultdict(list)
    for r in rows:
        grp[r["tag"]].append(r["mass_kg"])
    rep = {k: v for k, v in grp.items() if len(v) >= 2}
    if rep:
        print("\n  同名物体的重复性：")
        for k, v in rep.items():
            print(f"    {k}: {[round(x*1000) for x in v]} g，"
                  f"极差 {1000*(max(v)-min(v)):.0f} g")


def do_check():
    print("\n  环境自检：")
    check_env(verbose=True)


def do_kill():
    print("  注意：这会连「夹爪保持」进程一起清掉 —— 正夹着东西的话会掉下来！")
    W._kill_stale()
    print("  已清理（如果没有输出，说明本来就没有）")


# ---------------------------------------------------------------------------
# 菜单
# ---------------------------------------------------------------------------
def banner():
    print("\n" + "=" * 62)
    print("  机械臂称重  ·  电流称重工具")
    print("=" * 62)
    print("  1  状态        看夹爪开度 / 关节角")
    print("  2  张开        张开夹爪（可指定开度）")
    print("  3  夹紧        轻力闭合（可指定力度）")
    print("  4  称重·双姿态  物体像砝码那样垂挂时才用；普通物体可能高估 30%")
    print("  5  称重·单姿态  【推荐】默认用这个，300 g 以上约 2~7%")
    print("  6  一键称重    张开放物 → 夹紧 → 双姿态称重")
    print("  7  历史        看最近 12 次记录")
    print("  8  环境自检")
    print("  9  清理争抢控制权的进程")
    print("  0  退出")
    print("=" * 62)


def main():
    if "--selftest" in sys.argv:
        print("自检：用已有数据验算单姿态解算（不连机械臂）")
        for tag, truth in (("M1", 0.10), ("M2", 0.30), ("M3", 0.50)):
            try:
                r = solve_single(tag, "A")
                print(f"  {tag}_A  真值 {truth:.2f} kg → 解出 {r['mass_kg']:+.3f} kg  "
                      f"（{100*(r['mass_kg']/truth-1):+.1f}%）  残差 {r['rms_nm']:.3f}")
            except Exception as e:
                print(f"  {tag}_A  失败：{e}")
        return

    print("提示：Ctrl+C 可随时中断；中断不会失能机械臂。断电前请先用手托住。")
    ok, _ = check_env(verbose=False)
    if not ok:
        print("\n  环境检查未通过，先看第 8 项。")
    else:
        print("  环境正常。")

    actions = {"1": do_status, "2": do_open, "3": do_close,
               "4": do_measure_full, "5": do_measure_fast,
               "6": do_full_auto, "7": do_history,
               "8": do_check, "9": do_kill}
    while True:
        banner()
        try:
            c = input("  选择：").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  已退出。")
            return
        if c in ("0", "q", ""):
            print("  已退出。断电前记得先托住机械臂。")
            return
        fn = actions.get(c)
        if not fn:
            print("  没有这个选项")
            continue
        try:
            fn()
        except KeyboardInterrupt:
            print("\n  已中断当前操作。机械臂保持在当前位置（没有失能）。")
        except SystemExit as e:
            print(f"\n  中止：{e}")
        except Exception as e:
            print(f"\n  出错：{type(e).__name__}: {e}")
        # 这一行也必须保护：用户常在「等我按回车」时按 Ctrl+C 想退出，
        # 没有 try 就会打印一大段 Traceback（2026-10-21 用户实际踩到）。
        try:
            input("\n  按回车回到菜单…")
        except (EOFError, KeyboardInterrupt):
            print("\n  已退出。断电前记得先托住机械臂。")
            return


if __name__ == "__main__":
    main()
