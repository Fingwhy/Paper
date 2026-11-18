#!/usr/bin/env bash
# poweron_collect.sh —— 给臂上电之后，一条命令采齐阶段 0 剩下的硬件项
#
# 为什么要它：阶段 0 有 6 个硬件项要取证，分散在 bus / sdk 两个相位里，
# 而且顺序不能乱（总线没通就发查询帧没有意义，还会误判成「使能策略不同」）。
# 这个脚本把顺序和判定写死，避免再出现「跑了但证据无效」。
#
# 用法：
#   bash env/poweron_collect.sh              # 默认 can0
#   bash env/poweron_collect.sh can1
#
# 它**自己不做 sudo**：接口没起来时会打印该敲的那一行，由你决定。
# 它**不发** 0x471 使能帧、不发任何运动帧。

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CASE_DIR="$(cd "$HERE/.." && pwd)"
IFACE="${1:-can0}"
PY=python

say() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '\n[FAIL] %s\n' "$*" >&2; exit 1; }

say "阶段 0 硬件项采集（接口 $IFACE）"
cd "$CASE_DIR" || die "进不去 $CASE_DIR"

# --- 0) 解释器：必须在 venv 里，否则没有 piper_sdk
$PY -c "import piper_sdk, can" 2>/dev/null || die "当前 python 没有 piper_sdk/python-can。
      先 source $CASE_DIR/.venv-py3.12/bin/activate"

# --- 1) 接口存在且 UP
if [[ ! -d "/sys/class/net/$IFACE" ]]; then
  echo "接口 $IFACE 不存在。按顺序查："
  echo "  lsusb | grep -i 1d50        # 期望 1d50:606f"
  echo "  lsmod | grep gs_usb"
  echo "  sudo dmesg | tail -30"
  exit 2
fi

STATE=$(ip -details link show "$IFACE" | head -1)
BITRATE=$(ip -details link show "$IFACE" | grep -o 'bitrate [0-9]*' | awk '{print $2}' | head -1)
OPERSTATE=$(cat "/sys/class/net/$IFACE/operstate" 2>/dev/null || echo unknown)
if [[ "$OPERSTATE" != "up" ]]; then
  die "接口存在但没 up（operstate=$OPERSTATE）。执行：
      sudo ip link set $IFACE up type can bitrate 1000000
      然后再跑本脚本"
fi
echo "接口状态: ${BITRATE:-未知} bps"

# --- 1.5) 先保住上电前的失败证据
# 上电前那两份 bus_probe/sdk_probe 是 R17（总线无 ACK）与 R18（守卫零值假阳性）
# 的唯一原始样本，而下面两步会用同名文件覆盖它们。这里先快照，且不覆盖已有快照。
ARCH="$CASE_DIR/env/prepower_snapshot"
if [[ -f "$CASE_DIR/env/bus_probe.json" || -f "$CASE_DIR/env/sdk_probe.json" ]]; then
  mkdir -p "$ARCH"
  for f in bus_probe.json sdk_probe.json env_probe.json requirements-lock.txt robot_env_lock.json; do
    src="$CASE_DIR/env/$f"; dst="$ARCH/$f"
    [[ -f "$src" ]] || continue
    [[ -f "$dst" ]] || cp -p "$src" "$dst"
  done
  echo "上电前旧产物已快照到 env/prepower_snapshot/（已有快照不覆盖）"
fi

# --- 2) 总线：必须先看到机器人反馈帧，否则后面全是默认值
say "① 监听 10 s，确认机器人是否在说话"
$PY scripts/stage0_probe.py --phase bus --seconds 10 || die "bus 相位异常退出"

$PY - <<'PY' || exit 3
import json, pathlib, sys
d = json.loads(pathlib.Path("env/bus_probe.json").read_text(encoding="utf-8"))
fr = (d.get("bus", {}).get("frames") or {}).get("value") or {}
fb = (fr.get("by_group", {}).get("high_speed_0x251_0x256", {}).get("ids_seen") or [])
st = (fr.get("by_group", {}).get("status_0x2A1_0x2A8", {}).get("ids_seen") or [])
print(f"   反馈帧: 0x251~0x256 命中 {len(fb)}/6；0x2A1 类命中 {len(st)}")
print(f"   error_frames={fr.get('error_frames')}  其它 ID={fr.get('other_ids')}")
if not fb and not st:
    print("\n[STOP] 总线上仍然没有机器人反馈帧，后面的 sdk 相位只会产出默认值，已中止。")
    print("       先查：臂是否上电 / 急停是否松开 / CAN 线两端 / 臂侧波特率 / 120Ω 终端电阻")
    sys.exit(3)
PY

# --- 3) SDK：固件、从站、限位、使能位、守卫
say "② 查询固件/从站/限位，并读状态与守卫"
$PY scripts/stage0_probe.py --phase sdk --allow-write --query-limits \
    --factr2-repo "$HOME/factr2_next" || die "sdk 相位异常退出"

# --- 4) 摘要
say "③ 摘要"
$PY scripts/stage0_dump.py || true

# --- 5) 模式守卫
#
# 为什么单列一步：ctrl_mode 决定「CAN 运动指令会不会被执行」，而这件事在
# 12:12 首采时被埋在一行字里，很容易看漏。若它是 TEACHING_MODE，L1 的 q_cmd
# 会静默失效——策略学成 q_obs→q_obs 的恒等映射，loss 很好看、机器人不动。
# 宁可每次采集都吼一句，也不要等到训练完才发现。
$PY - "$CASE_DIR/env/sdk_probe.json" <<'PY' || true
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
    cm = ((d.get("status_0x2A1") or {}).get("value") or {}).get("ctrl_mode") or {}
    name, code = cm.get("name"), cm.get("code")
except Exception as e:
    print(f"\n[MODE] 读不到 sdk_probe.json：{e!r}")
    raise SystemExit

print()
if code == 1:
    print("[MODE] CAN_CTRL(0x1) —— 可以下发运动指令")
elif code == 2:
    print("[MODE][WARN] TEACHING_MODE(0x2) —— 示教模式下 CAN 运动指令很可能被忽略。")
    print("        FACTR2 需要程序控。先摆好姿态、人守急停，再跑：")
    print("        python scripts/stage0_probe.py --phase sdk --allow-write --set-slave")
elif code is None:
    print("[MODE][WARN] ctrl_mode 读不到（None）—— 可能停在模式切换的过渡态。")
    print("        先断电重启回干净上电状态，别直接发运动指令。")
else:
    print(f"[MODE] ctrl_mode={name}(0x{code:x}) —— 非 CAN_CTRL，运动指令行为未验证")
PY

say "完成。产物在 $CASE_DIR/env/"
cat <<EOF

下一步：
  - 把上面整段摘要贴回复盘方
  - 或把 env/*.json 拉回 Windows：
      scp "zzz@192.168.242.128:$CASE_DIR/env/*" "<Windows 侧 env 目录>"
EOF
