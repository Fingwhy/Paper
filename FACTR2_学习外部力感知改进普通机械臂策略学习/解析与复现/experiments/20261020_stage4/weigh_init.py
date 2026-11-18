#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""weigh_init.py —— 称重前的一键初始化

放在 VM：`~/stage0/20261020_stage4/weigh_init.py`

它按顺序做五件事，任何一步失败就停下并告诉你下一步该干嘛：
  1. 找 CAN 设备（没有就尝试加载 gs_usb 驱动）
  2. 激活 CAN（未起或 bitrate 不对时，走「down → bitrate 1000000 → up」三步）
  3. 验机械臂信号（读 1.2 s 的 CAN 帧；0 帧说明臂没上电/线没插好）
  4. 检查控制模式（示教模式会自动切到 CAN_CTRL，并报告六轴使能）
  5. 启动称重菜单

为什么单独写成程序而不是几行 shell
--------------------------------
* 三步激活要 sudo。sudo 需要可交互终端，shell 脚本一旦被管道/重定向跑就静默失败，
  Python 里用 subprocess 直接继承终端，sudo 自己弹密码，失败也能被判出来。
* 「有没有信号」这件事只能靠真的去收帧，`ip link` 显示 UP 不等于臂在说话
  （现场踩过：接口 UP、bitrate 也对，但 USB-CAN 芯片卡死，一帧都没有）。
* 每一步的失败原因不同，得给不同的处理办法，不能一句「初始化失败」。

用法（VM 上）：
    python3 weigh_init.py            # 初始化成功后直接进称重菜单
    python3 weigh_init.py --no-app   # 只初始化，不启动称重
    python3 weigh_init.py --force    # 忽略「已激活」，强制重走三步
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
STAGE0_SCRIPTS = Path.home() / "stage0" / "20261019_stage0" / "scripts"
Iface = "can0"
BITRATE = "1000000"


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def sh(cmd):
    """跑一条只读命令，返回 stdout（容错：Windows/GBK 都不炸）。"""
    return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                          errors="replace").stdout


def sudo(args, note=""):
    """用 sudo 执行；sudo 会自己弹密码提示（必须在可交互终端里跑）。

    返回 True/False。故意不捕获输出——密码提示要显示给用户。
    """
    try:
        r = subprocess.run(["sudo"] + args)
        return r.returncode == 0
    except FileNotFoundError:
        print(f"  ✗ 找不到 sudo")
        return False


def step(n, title):
    print(f"\n[{n}/5] {title}")


def fail(reason, hints):
    print(f"  ✗ {reason}")
    if hints:
        print("  接下来可以试：")
        for h in hints:
            print(f"    · {h}")
    return False


# ---------------------------------------------------------------------------
# 各步
# ---------------------------------------------------------------------------
def link_info():
    return sh(f"ip -d link show {Iface} 2>/dev/null")


def count_frames(seconds=1.2):
    """真的去收帧。返回 (帧数 or None, 出现过的 CAN ID 集合)。"""
    try:
        import can
    except ImportError:
        return None, set()
    try:
        bus = can.interface.Bus(channel=Iface, interface="socketcan")
    except Exception:
        return None, set()
    n, ids = 0, set()
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            msg = bus.recv(timeout=0.1)
            if msg:
                n += 1
                ids.add(msg.arbitration_id)
    finally:
        bus.shutdown()
    return n, ids


def read_mode():
    """读 ctrl_mode 与使能；失败返回 (None, None)。"""
    try:
        from piper_sdk import C_PiperInterface
        p = C_PiperInterface(Iface)
        if p.ConnectPort() is False:      # 0.6.1 返回 None，不能用 not
            return None, None
        time.sleep(0.8)
        mode = p.GetArmStatus().arm_status.ctrl_mode
        en = p.GetArmEnableStatus()
        en_ok = all(en) if isinstance(en, (list, tuple)) else bool(en)
        return int(mode), en_ok
    except Exception:
        return None, None


def to_can_ctrl():
    """切到 CAN_CTRL。复用阶段 0 已验证过的探测脚本，不自己发原始帧。"""
    if not (STAGE0_SCRIPTS / "stage0_probe.py").exists():
        print(f"  ✗ 找不到 {STAGE0_SCRIPTS}/stage0_probe.py")
        return False
    r = subprocess.run(
        ["python3", "stage0_probe.py", "--iface", Iface, "--phase", "sdk",
         "--allow-write", "--to-can-ctrl"],
        cwd=str(STAGE0_SCRIPTS), capture_output=True, text=True, errors="replace")
    out = (r.stdout or "") + (r.stderr or "")
    for line in out.splitlines():
        if "完成后=" in line or "切程序控" in line:
            print("  " + line.strip())
    return r.returncode == 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="称重前的一键初始化")
    ap.add_argument("--no-app", action="store_true", help="只初始化，不启动称重菜单")
    ap.add_argument("--force", action="store_true", help="强制重走三步激活")
    args = ap.parse_args()

    print("=" * 62)
    print("  称重初始化")
    print("=" * 62)

    # ---- 1) 找设备 ----
    step(1, "查找 CAN 设备")
    info = link_info()
    if not info:
        print(f"  没有 {Iface}，尝试加载 gs_usb 驱动…")
        sudo(["modprobe", "gs_usb"])
        time.sleep(1.0)
        info = link_info()
    if not info:
        fail(f"仍然找不到 {Iface}", [
            "USB-CAN 是否插好？（换个 USB 口试试）",
            "sudo dmesg | tail -20  看内核有没有认到设备",
            "若是芯片卡死：sudo modprobe -r gs_usb && sudo modprobe gs_usb（会重建接口）",
            "机械臂和 USB-CAN 是两路：USB-CAN 插电脑，CAN 线才连机械臂",
        ])
        return 1
    print(f"  ✓ 找到 {Iface}")

    # ---- 2) 激活 ----
    step(2, "激活 CAN")
    up = "state UP" in info
    br = f"bitrate {BITRATE}" in info
    if up and br and not args.force:
        print(f"  ✓ 已激活（UP，bitrate {BITRATE}）")
    else:
        print(f"  当前状态：{'UP' if up else 'DOWN'}，"
              f"bitrate {'正确' if br else '未设置'}")
        print("  执行三步激活（可能要求输入 sudo 密码）…")
        # 必须分三步，不能一体化：先 down 才能改 bitrate
        for desc, cmd in (("设为 down", ["ip", "link", "set", Iface, "down"]),
                          (f"设 bitrate {BITRATE}", ["ip", "link", "set", Iface,
                                                     "type", "can", "bitrate", BITRATE]),
                          ("拉起 up", ["ip", "link", "set", Iface, "up"])):
            if not sudo(cmd):
                fail(f"「{desc}」失败", [
                    "sudo 需要密码，请在有终端的窗口里运行本程序",
                    "或手动执行：",
                    f"  sudo ip link set {Iface} down",
                    f"  sudo ip link set {Iface} type can bitrate {BITRATE}",
                    f"  sudo ip link set {Iface} up",
                ])
                return 1
        time.sleep(1.0)
        info = link_info()
        if "state UP" not in info:
            fail("执行完三步但接口仍然不是 UP", [
                "USB-CAN 可能没插稳，拔下重插后再跑一次"])
            return 1
        if f"bitrate {BITRATE}" not in info:
            fail("bitrate 没有生效", ["拔插 USB-CAN 后重跑本程序"])
            return 1
        print(f"  ✓ 已激活（UP，bitrate {BITRATE}）")
        # restart-ms 在 gs_usb 上不支持，会报 "Device doesn't support restart from Bus Off"，
        # 这个报错可以忽略 —— 只要 bitrate 对了就能通信。这里故意不设它。

    # ---- 3) 验信号 ----
    step(3, "检查机械臂信号（实收 CAN 帧）")
    n, ids = count_frames(1.2)
    if n is None:
        fail("读不了 CAN（python-can 不可用或接口被占用）", [
            "检查：python3 -c 'import can; print(can.__version__)'",
            "是否有别的程序正在占用 can0（菜单第 9 项可清理）",
        ])
        return 1
    if n == 0:
        fail("1.2 秒内一帧都没收到 —— 接口通了，但机械臂没在说话", [
            "机械臂电源打开了吗？（这个程序只负责 CAN，不管供电）",
            "急停按钮是否被按下？",
            "CAN 线是否一端插 USB-CAN、另一端插机械臂底座？",
            "机械臂上的 CAN 口有两路，换另一路试试",
        ])
        return 1
    arm_ids = [i for i in ids if 0x251 <= i <= 0x257 or 0x2A1 <= i <= 0x2A8]
    print(f"  ✓ 收到 {n} 帧 / 1.2 s，{len(ids)} 个不同 ID")
    if not arm_ids:
        print("  ！没有看到机械臂的反馈 ID（0x251~0x257 / 0x2A1~0x2A8）")
        print("    可能这是别的 CAN 设备，或者波特率不匹配 —— 继续下去大概率读不到臂")
    else:
        print(f"  ✓ 其中机械臂反馈 ID {len(arm_ids)} 个（0x251~0x257 / 0x2A1~0x2A8）")

    # ---- 4) 控制模式 ----
    step(4, "检查控制模式")
    mode, en_ok = read_mode()
    if mode is None:
        fail("连不上机械臂（SDK 初始化失败）", [
            "跑一次：python3 weigh_app.py --selftest  看基础链路是否正常",
            "或手动：cd ~/stage0/20261019_stage0/scripts && "
            "python3 stage0_probe.py --iface can0 --phase sdk",
        ])
        return 1
    names = {1: "CAN_CTRL（可用）", 2: "TEACHING（示教，需切换）",
             3: "未使能/其他"}
    print(f"  当前模式：{mode} = {names.get(mode, '未知')}")
    if mode != 1:
        print("  → 正在切到 CAN_CTRL…")
        if not to_can_ctrl():
            fail("切换失败", [
                "手动执行：",
                f"  cd {STAGE0_SCRIPTS}",
                f"  python3 stage0_probe.py --iface {Iface} --phase sdk "
                f"--allow-write --to-can-ctrl",
            ])
            return 1
        mode, en_ok = read_mode()
        if mode != 1:
            fail(f"切换后仍然是 {mode}", ["拔插 USB-CAN 后重跑本程序"])
            return 1
        print("  ✓ 已切到 CAN_CTRL")
    else:
        print("  ✓ 已经是 CAN_CTRL")

    if en_ok is True:
        print("  ✓ 六轴已使能")
    elif en_ok is False:
        print("  ！六轴未全部使能 —— 称重采集会失败")
        print("    手动使能：cd ~/stage0/20261019_stage0/scripts && "
              "python3 stage0_set_zero.py hold --iface can0 --from-joint 1 --to-joint 6")
    else:
        print("  ？使能状态读不出来（不影响继续尝试）")

    # ---- 5) 启动称重 ----
    step(5, "启动称重")
    if args.no_app:
        print("  （--no-app，已跳过）")
        print("\n初始化完成。")
        return 0
    print("  环境就绪，进入称重菜单…\n")
    time.sleep(0.4)
    return subprocess.call([sys.executable, str(HERE / "weigh_app.py")])


if __name__ == "__main__":
    sys.exit(main())
