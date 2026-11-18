#!/usr/bin/env bash
# run_s20.sh —— S2-0 单姿态静态采集
#   几何闸门 → 切 CAN 控制模式 → 使能 → 起录制 → pose 到位并保持 → 停录制 → parse → audit
#
# 为什么要有这个脚本，而不是手敲几条命令
# --------------------------------------
# 前面踩过的每个坑都是**顺序**问题：
#   * 录制晚起 3 秒    ⇒ 斜坡段没有帧，事后分不清「没到位」和「没录到」
#   * 忘了切 CAN 模式  ⇒ 断电重启后 ctrl_mode=STANDBY，pose 预检直接 Abort
#   * 忘了先使能       ⇒ pose 预检 Abort，白等
# 顺序写死比每次靠记忆敲可靠。
#
# 用法（VM，~/stage0/20261019_stage2 下）：
#   NAME=s20_p1 TARGET=0,45,-60,0,45,0   HOLD=60 bash run_s20.sh
#   NAME=s20_p4 TARGET=0,100,-75,0,60,0  HOLD=60 bash run_s20.sh
#   ENABLE=0          # 已经使能过就跳过使能段
#   RATE=5            # 速率 °/s，默认 6
#   CONFIRM=1         # 交互确认（人手在急停上）
#
# 跑之前：工作空间清空、手放急停。
#
# 2026-10-20 10:1x 事故后的改动（R42）
# ------------------------------------
# 原 P4 = (0,100,-20,0,60,0) 让夹爪弯进底盘立柱：URDF 正解算出来 TCP z=67 mm、
# 净空 −28 mm，本来就能提前发现。现在：
#   1. 【步骤 3/9】几何闸门 stage2_pose_guard.py，**使能之前**先算目标与整条路径净空，
#      不合格直接退出，一帧都不发。
#   2. P4 换成 (0,100,-75,0,60,0)：净空 +244 mm、TCP z=203 mm、J2 力矩仍反号（−2.60）。
#   3. pose 侧三道运行时守卫：卡死（窗口内指令走了实测没走）、电流（>3 A）、
#      末端下降（>150 mm）。任一触发**原地抱住**并退出，不再往目标顶。
#   4. 速率从 8 降到 6 °/s。

set -uo pipefail

IFACE=${IFACE:-can0}
NAME=${NAME:-s20_p1}
TARGET=${TARGET:-0,45,-60,0,45,0}
HOLD=${HOLD:-60}
RATE=${RATE:-6}
ENABLE=${ENABLE:-1}
CONFIRM=${CONFIRM:-0}
SKIP_GUARD=${SKIP_GUARD:-0}

STALL_WIN_S=${STALL_WIN_S:-1.0}
STALL_EPS_DEG=${STALL_EPS_DEG:-0.08}
MAX_CURRENT_A=${MAX_CURRENT_A:-3.0}
MAX_DROP_MM=${MAX_DROP_MM:-auto}
LEAD_MAX_DEG=${LEAD_MAX_DEG:-2.0}

STAGE0=$HOME/stage0/20261019_stage0
STAGE1=$HOME/stage0/20261019_stage1_l0
STAGE2=$HOME/stage0/20261019_stage2
LIMITS=$STAGE0/env/joint_limits.json

REC_SECS=$(( HOLD + 60 ))

say()  { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
die()  { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

say "0/9 环境"
python3 - <<'PY' || die "python3 缺 piper_sdk / python-can"
import sys, piper_sdk, can
print("python3   :", sys.executable)
print("piper_sdk :", piper_sdk.__file__)
print("python-can:", can.__version__)
PY

[ -f "$LIMITS" ] || die "缺 $LIMITS（没有包络就不许发六轴指令）"
[ -f "$STAGE1/stage1_cmd.py" ] || die "缺 $STAGE1/stage1_cmd.py"
[ -f "$STAGE1/stage1_l0.py" ]  || die "缺 $STAGE1/stage1_l0.py"
[ -f "$STAGE2/stage2_pose_guard.py" ] || die "缺 stage2_pose_guard.py —— 没有几何闸门不许发六轴指令"
mkdir -p "$STAGE2"/{raw,cmd,episodes}

say "1/9 计划：$NAME  目标=$TARGET  速率=${RATE}°/s  保持=${HOLD}s  录制=${REC_SECS}s"
if [ "$CONFIRM" = "1" ]; then
  printf "臂周围清空、手能碰到急停？输入 yes 继续："
  read -r ans
  [ "$ans" = "yes" ] || die "操作者未确认"
fi

say "2/9 can0 状态"
ip -br link show "$IFACE"
ip -br link show "$IFACE" | grep -qw UP || die "$IFACE 不是 UP（三步拉起：down → bitrate 1000000 → up）"

say "3/9 几何闸门（stage2_pose_guard，不动机器人）"
GUARD_LOG=$STAGE2/raw/${NAME}_guard.txt
MAX_DROP_EFF=$MAX_DROP_MM
if [ "$SKIP_GUARD" = "1" ]; then
  echo "[WARN] SKIP_GUARD=1 —— 跳过闸门。上一次跳过的结果是夹爪撞进底盘。"
  [ "$MAX_DROP_EFF" = "auto" ] && MAX_DROP_EFF=150
else
  python3 "$STAGE2/stage2_pose_guard.py" --iface "$IFACE" --start-from-can \
      --target-deg "$TARGET" --min-clear-mm 100 --min-tcp-z-mm 150 --min-path-mm 20 \
      2>&1 | tee "$GUARD_LOG"
  grep -q '\[guard\] 通过' "$GUARD_LOG" \
      || die "几何闸门不放行：目标或路径离基座太近。**一帧都没发**。换姿态，不要绕过。"
  # 高度守卫的下限不能写死 150 mm：P1→P4′ 这一步**计划上就要下降 214 mm**，
  # 固定 150 mm 会把合法动作误杀成「掉下去了」（2026-10-20 11:2x 实际发生过）。
  # 判据改成「比计划多掉 60 mm」：计划下降量由闸门算出的两端 TCP z 直接取。
  if [ "$MAX_DROP_EFF" = "auto" ]; then
    MAX_DROP_EFF=$(python3 - "$GUARD_LOG" <<'PY'
import re, sys
txt = open(sys.argv[1], encoding="utf-8").read()


def z_of(tag):
    m = re.search(r"^ *" + tag + r".*?TCP=\(\s*([+-][\d.]+),\s*([+-][\d.]+),\s*([+-][\d.]+)\)",
                  txt, re.M)
    return float(m.group(3)) if m else None


a, b = z_of("起点"), z_of("目标 1")
if a is None or b is None:
    print(150.0)
else:
    # 余量从 +60 提到 +150（2026-10-20 12:5x 实测教训）：
    #   闸门算的是 **URDF link6 原点 Z**，守卫监视的是 **固件「末端位姿」Z**，
    #   两者不是同一个点，偏差随姿态在 −15 ~ +0 mm 之间变（K8 起点差 14.4，
    #   中途某姿态差 0.4）。K8→K5 计划只降 76 mm，实机降了 150.4 mm，
    #   旧余量 60 让阈值停在 150，被 0.4 mm 判死 —— 守卫没错，余量算错了。
    # 这个守卫的职责只是「防失控掉落」，几何安全由 stage2_pose_guard +
    # s21_design 的带载胶囊闸门保证，所以宽松一点不损失安全。
    print(round(max(150.0, abs(b - a) + 150.0), 1))
PY
)
    echo "[run] 高度守卫自动阈值：--max-drop-mm $MAX_DROP_EFF（计划下降量 + 60 mm 余量）"
  fi
fi

say "4/9 切 CAN 控制模式（断电重启后会回到 STANDBY）"
( cd "$STAGE0/scripts" && python3 stage0_probe.py --iface "$IFACE" --phase sdk \
    --allow-write --to-can-ctrl ) || die "切模式失败"
( cd "$STAGE0/scripts" && python3 stage0_set_zero.py check --iface "$IFACE" | tail -7 ) \
  | tee /tmp/_s20_check.txt

say "5/9 使能六轴（stage0_set_zero.py hold，不碰夹爪）"
if [ "$ENABLE" = "1" ]; then
  ( cd "$STAGE0/scripts" && python3 stage0_set_zero.py hold --iface "$IFACE" \
      --from-joint 1 --to-joint 6 ) || die "使能失败"
else
  echo "跳过（ENABLE=0）"
fi

say "6/9 后台上录制（$REC_SECS s）"
rm -f "$STAGE2/raw/$NAME.jsonl"
( cd "$STAGE1" && exec python3 stage1_l0.py record --iface "$IFACE" \
    --seconds "$REC_SECS" --out "$STAGE2/raw/$NAME.jsonl" ) &
RECPID=$!
trap 'kill $RECPID 2>/dev/null' INT TERM
sleep 3
kill -0 $RECPID 2>/dev/null || die "录制进程没起来"

say "7/9 pose 到目标（$NAME）"
( cd "$STAGE1" && python3 stage1_cmd.py pose --iface "$IFACE" \
    --limits "$LIMITS" --target-deg "$TARGET" \
    --rate-deg-s "$RATE" --hold-s "$HOLD" \
    --stall-win-s "$STALL_WIN_S" --stall-eps-deg "$STALL_EPS_DEG" \
    --max-current-a "$MAX_CURRENT_A" --max-drop-mm "$MAX_DROP_EFF" \
    --lead-max-deg "$LEAD_MAX_DEG" \
    --out "$STAGE2/cmd/$NAME" --yes )
POSE_RC=$?

say "8/9 等录制收尾"
wait $RECPID 2>/dev/null
trap - INT TERM

if [ "$POSE_RC" -ne 0 ]; then
  case "$POSE_RC" in
    2) die "pose 退出码 2：目标越出安全包络，零帧下发" ;;
    3) die "pose 退出码 3：未到位（被挡/卡涩）" ;;
    4) die "pose 退出码 4：运行时守卫触发（卡死/电流/高度），已原地抱住。看上面 ABORT 行" ;;
    *) die "pose 退出码 $POSE_RC。录制已保留" ;;
  esac
fi

say "9/9 parse + audit"
# --pos251-unit mrad 必须给：0x251 的位置字段是 0.001 rad，默认按 rad 解析会
# 在比值自检（实测 999.3）处判失败并返回非零 —— 2026-10-20 12:2x 的 K15 就这么
# 被中断了整批采集（数据其实是好的）。
( cd "$STAGE1" && python3 stage1_l0.py parse "$STAGE2/raw/$NAME.jsonl" \
    --hz 50 --pos251-unit mrad --qcmd "$STAGE2/cmd/$NAME.csv" --out "$STAGE2/episodes/$NAME" ) \
  || die "parse 失败"
( cd "$STAGE1" && python3 stage1_l0.py audit "$STAGE2/raw/$NAME.jsonl" ) || true

printf '\n\033[1m完成：%s\033[0m\n' "$NAME"
echo "  raw    : $STAGE2/raw/$NAME.jsonl"
echo "  cmd    : $STAGE2/cmd/$NAME.csv"
echo "  episode: $STAGE2/episodes/$NAME"
echo "  下一步（本机 Windows）: python stage2_design.py check --samples episodes/$NAME/samples.jsonl"
