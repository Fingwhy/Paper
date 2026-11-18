#!/usr/bin/env bash
# run_s3.sh —— 阶段 3（自由运动数据集）单段采集
#   闸门 → 切 CAN_CTRL → 使能 → 起录制 → pose 到中心 → 轨迹 → 停录制 → parse
#
# 为什么每段都要走完整流程，而不是一次录 10 分钟
# ----------------------------------------------
# 指南 §6.4 要求「先按 episode / 采集批次划分训练、验证、测试，再构窗；禁止跨段构窗」。
# 一次长录再硬切，边界处必然出现「前一格的 q 属于上一条轨迹、下一格已经到下一条」，
# 那就是跨界污染。每段一个 raw + 一个 episode，切分时天然干净。
#
# 为什么段间不用重新停下来问人
# ----------------------------
# 段与段之间的移动距离都很小（同中心姿态的段之间是零位移），且每段前面都过闸门。
# 但**每段都重新切一次 CAN_CTRL**：R49 实测 `CAN_CTRL` 空闲会回落 `TEACHING`，
# 40 段跑一两个小时，中间任何一次回落都会让后续 pose 直接 Abort。
#
# 用法（VM，~/stage0/20261019_stage3 下）：
#   NAME=S1_1_lo bash run_s3.sh
#   NAME=E1_J2high RATE=5 bash run_s3.sh
#   ENABLE=0          # 已经使能过就跳过（省时间，但每段仍会重切模式）
#   SKIP_ROUTE=1      # 只查段内几何，不读当前姿态（调试用）
#
# 跑之前：工作空间清空、手放急停。

set -uo pipefail

IFACE=${IFACE:-can0}
NAME=${NAME:-}
RATE=${RATE:-6}
ENABLE=${ENABLE:-1}
CONFIRM=${CONFIRM:-0}
SKIP_ROUTE=${SKIP_ROUTE:-0}

STALL_WIN_S=${STALL_WIN_S:-1.0}
STALL_EPS_DEG=${STALL_EPS_DEG:-0.08}
MAX_CURRENT_A=${MAX_CURRENT_A:-4.0}
# 下降守卫阈值不再写死：逐段按「计划下降 + DROP_MARGIN」现算（见 s3_drop.py)。
# 2026-10-20 14:4x：E2_J3deep→E3_wrist 计划就降 400 mm，写死 400.0 差 0.3 mm 误杀，
# 连带 V1/V2/V3/T1/T2/T3 六段没跑。
DROP_MARGIN=${DROP_MARGIN:-150}
DROP_FALLBACK=${DROP_FALLBACK:-600}
LEAD_MAX_DEG=${LEAD_MAX_DEG:-2.0}

STAGE0=$HOME/stage0/20261019_stage0
STAGE1=$HOME/stage0/20261019_stage1_l0
S3=$HOME/stage0/20261019_stage3
LIMITS=$STAGE0/env/joint_limits.json

say() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
die() { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

[ -n "$NAME" ] || die "没给 NAME。用法：NAME=S1_1_lo bash run_s3.sh"
cd "$S3" || die "进不了 $S3"

say "0/8 环境"
python3 - <<'PY' || die "python3 缺 piper_sdk / python-can"
import sys, piper_sdk, can
print("python3   :", sys.executable)
print("piper_sdk :", piper_sdk.__file__)
print("python-can:", can.__version__)
PY
[ -f "$LIMITS" ] || die "缺 $LIMITS（没有包络就不许发六轴指令）"
[ -f "$STAGE1/stage1_cmd.py" ] || die "缺 $STAGE1/stage1_cmd.py"
[ -f "$STAGE1/stage1_l0.py" ]  || die "缺 $STAGE1/stage1_l0.py"
[ -f "$S3/s3_design.py" ]      || die "缺 s3_design.py"
mkdir -p "$S3"/{raw,cmd,episodes}

say "1/8 取段参数（唯一真相源 = s3_design.py 的 TRACKS）"
REQ=$(python3 s3_design.py --emit "$NAME") || die "未知段 $NAME（python3 s3_design.py --list 看全部）"
eval "$REQ"
echo "  $NAME  kind=$KIND  split=$SPLIT  时长=${DUR}s  录制=${REC}s"
echo "  center=$CENTER"
[ "$KIND" = "sine" ] && echo "  amp=[$AMP]  f=${FREQ}Hz"
echo "  说明：$NOTE"

say "2/8 can0 状态"
ip -br link show "$IFACE" | grep -qw UP || die "$IFACE 不是 UP（三步拉起：down → bitrate 1000000 → up）"
ip -br link show "$IFACE"

say "3/8 发帧前闸门（段内包络 + 移动路径；不动机器人）"
GATE_LOG="raw/${NAME}_gate.txt"
if [ "$SKIP_ROUTE" = "1" ]; then
  python3 s3_design.py --gate "$NAME" --skip-route 2>&1 | tee "$GATE_LOG"
else
  python3 s3_design.py --gate "$NAME" --iface "$IFACE" 2>&1 | tee "$GATE_LOG"
fi
RC_GATE=${PIPESTATUS[0]}
if [ "$RC_GATE" -ne 0 ]; then
  case "$RC_GATE" in
    1) die "段内几何不合格 —— 这段本身不能走。**一帧都没发**。改 TRACKS，不要绕过。" ;;
    2) die "移动路径不合格 —— 这段能走但不能从当前位置直达，要先加中转段。**一帧都没发**。" ;;
    3) die "读不到当前姿态（can0 未 UP / 臂未使能）。" ;;
    *) die "闸门异常退出码 $RC_GATE" ;;
  esac
fi

if [ "$CONFIRM" = "1" ]; then
  printf "臂周围清空、手能碰到急停？输入 yes 继续："
  read -r ans
  [ "$ans" = "yes" ] || die "操作者未确认"
fi

say "4/8 切 CAN 控制模式（每段都切：R49 空闲会回落 TEACHING）"
( cd "$STAGE0/scripts" && python3 stage0_probe.py --iface "$IFACE" --phase sdk \
    --allow-write --to-can-ctrl ) || die "切模式失败"

say "5/8 使能六轴"
if [ "$ENABLE" = "1" ]; then
  ( cd "$STAGE0/scripts" && python3 stage0_set_zero.py hold --iface "$IFACE" \
      --from-joint 1 --to-joint 6 ) || die "使能失败"
else
  echo "跳过（ENABLE=0）"
fi

say "6/8 后台上录制（${REC}s）"
rm -f "$S3/raw/$NAME.jsonl"
( cd "$STAGE1" && exec python3 stage1_l0.py record --iface "$IFACE" \
    --seconds "$REC" --out "$S3/raw/$NAME.jsonl" ) &
RECPID=$!
trap 'kill $RECPID 2>/dev/null' INT TERM
sleep 3
kill -0 $RECPID 2>/dev/null || die "录制进程没起来"

say "6b/8 算本段允许下降量（计划下降 + ${DROP_MARGIN} mm）"
DROP_MM=$( cd "$S3" && python3 s3_drop.py --iface "$IFACE" --target-deg "$CENTER" \
    --margin "$DROP_MARGIN" --fallback-mm "$DROP_FALLBACK" ) || die "算下降阈值失败"
DROP_MM=$(echo "$DROP_MM" | tail -1)
case "$DROP_MM" in
  ''|*[!0-9]*) die "下降阈值不是整数：'$DROP_MM'" ;;
esac
echo "  --max-drop-mm=$DROP_MM"

say "7/8 pose 到中心姿态（$CENTER @ ${RATE}°/s）"
( cd "$STAGE1" && python3 stage1_cmd.py pose --iface "$IFACE" \
    --limits "$LIMITS" --target-deg "$CENTER" \
    --rate-deg-s "$RATE" --hold-s 3 \
    --stall-win-s "$STALL_WIN_S" --stall-eps-deg "$STALL_EPS_DEG" \
    --max-current-a "$MAX_CURRENT_A" --max-drop-mm "$DROP_MM" \
    --lead-max-deg "$LEAD_MAX_DEG" \
    --out "$S3/cmd/${NAME}_pre" --yes )
PRE_RC=$?
if [ "$PRE_RC" -ne 0 ]; then
  kill $RECPID 2>/dev/null
  case "$PRE_RC" in
    2) die "pre-pose 退出码 2：目标越出安全包络，零帧下发" ;;
    3) die "pre-pose 退出码 3：未到位（被挡/卡涩）" ;;
    4) die "pre-pose 退出码 4：运行时守卫触发（卡死/电流/高度），已原地抱住" ;;
    *) die "pre-pose 退出码 $PRE_RC" ;;
  esac
fi

if [ "$KIND" = "sine" ]; then
  say "7b/8 正弦轨迹  amp=[$AMP]  f=${FREQ}Hz  ${DUR}s"
  ( cd "$STAGE1" && python3 stage1_cmd.py sine --iface "$IFACE" \
      --limits "$LIMITS" --amp-deg $AMP --freq-hz "$FREQ" \
      --duration "$DUR" --phase-spread 0 \
      --max-step-deg 1.0 --limit-tol-deg 3.0 \
      --out "$S3/cmd/$NAME" --yes )
  RUN_RC=$?
else
  say "7b/8 静态保持 ${DUR}s（pose 段）"
  ( cd "$STAGE1" && python3 stage1_cmd.py pose --iface "$IFACE" \
      --limits "$LIMITS" --target-deg "$CENTER" \
      --rate-deg-s "$RATE" --hold-s "$DUR" \
      --stall-win-s "$STALL_WIN_S" --stall-eps-deg "$STALL_EPS_DEG" \
      --max-current-a "$MAX_CURRENT_A" --max-drop-mm "$DROP_MM" \
      --lead-max-deg "$LEAD_MAX_DEG" \
      --out "$S3/cmd/$NAME" --yes )
  RUN_RC=$?
fi

say "8/8 等录制收尾"
wait $RECPID 2>/dev/null
trap - INT TERM

if [ "$RUN_RC" -ne 0 ]; then
  case "$RUN_RC" in
    2) die "轨迹退出码 2：目标越出安全包络" ;;
    3) die "轨迹退出码 3：未到位（被挡/卡涩）" ;;
    4) die "轨迹退出码 4：运行时守卫触发，已原地抱住" ;;
    *) die "轨迹退出码 $RUN_RC。录制已保留：raw/$NAME.jsonl" ;;
  esac
fi

# parse：--pos251-unit mrad 必须给（0x251 位置字段是 0.001 rad）。
# 2026-10-20 12:2x 的 K15 就是因为漏了它被判失败、中断了整批。
( cd "$STAGE1" && python3 stage1_l0.py parse "$S3/raw/$NAME.jsonl" \
    --hz 50 --pos251-unit mrad --qcmd "$S3/cmd/$NAME.csv" --out "$S3/episodes/$NAME" ) \
  || die "parse 失败（数据在 raw/$NAME.jsonl，可手工重跑）"

printf '\n\033[1m完成：%s\033[0m\n' "$NAME"
echo "  raw    : $S3/raw/$NAME.jsonl"
echo "  cmd    : $S3/cmd/$NAME.csv"
echo "  episode: $S3/episodes/$NAME"
