#!/usr/bin/env bash
# run_s21.sh —— S2-1 多姿态静态采集（一次跑完 10 个姿态）
#
# 为什么要有批处理，而不是手工敲 10 次
# ------------------------------------
# S2-1 要「同一水瓶、多个姿态」的最小二乘。10 个姿态手工跑会出两类错：
#   * 顺序错：某段路径没检查就想走（带载后水瓶比法兰低 20 cm，S2-0 的净空判据管不到）
#   * 漏点：少一个姿态，某轴的 τ 跨度就不够，整个 K 拟合降级成「未标定」
# 所以顺序、速率、几何闸门全部写死在这里。
#
# 三层安全，按顺序执行
# --------------------
#   0) `s21_design.py --seq --start-from-can`：**带载**几何闸门（含水瓶 22 cm 胶囊），
#      起点用固件实测姿态，整条 10 段路线逐段验证。不合格 ⇒ 一帧不发。
#   1) 每段仍然走 `run_s20.sh`：它内部的 stage2_pose_guard 再验一次臂本体，
#      然后 pose 自带四道运行时守卫（卡死/电流/高度/超时），触发就原地抱住。
#   2) 任何一段非零退出 ⇒ 立即停，不再继续后面的点。
#
# 用法（VM，~/stage0/20261019_stage2 下）
# --------------------------------------
#   HOLD=40 RATE=3 MAX_CURRENT_A=4.0 bash run_s21.sh
#   ONLY=K3,K10 bash run_s21.sh        # 只跑指定几段（断点续跑用）
#
# 跑之前：水瓶已绑好、工作空间上方与前方清空、手放急停。

set -uo pipefail

STAGE2=$HOME/stage0/20261019_stage2
HOLD=${HOLD:-40}
RATE=${RATE:-3}
IFACE=${IFACE:-can0}
MAX_CURRENT_A=${MAX_CURRENT_A:-4.0}
ONLY=${ONLY:-}
# 空载/带载两轮的分野：episode 名前缀。
#   带载（水瓶绑在夹爪上）⇒ EP_PREFIX=s21_   （默认，历史数据就是这个名字）
#   空载（水瓶已拆）      ⇒ EP_PREFIX=s21e_
# 两轮必须**同姿态表、同顺序、同速率**，否则差分的「同姿态相减」不成立。
EP_PREFIX=${EP_PREFIX:-s21_}
# 载荷几何包络：带载轮按 22 cm 水瓶判，空载轮按「夹爪 11 cm 轴线」判。
# **必须跟着臂上实际挂没挂水瓶走**，否则空载轮会被不存在的瓶身判死
# （2026-10-20 12:45 实测：断电落点法兰 183 mm、夹爪掠过立柱顶角 18.7 mm，
#  带载口径给 −16 mm，空载口径给 +18.7 mm）。
NO_BOTTLE=${NO_BOTTLE:-0}
PACK_FLAG=""
[ "$NO_BOTTLE" = "1" ] && PACK_FLAG="--no-bottle"
PACK_LABEL="水瓶 22 cm 胶囊"
[ "$NO_BOTTLE" = "1" ] && PACK_LABEL="夹爪 11 cm 轴线（空载）"

declare -A TARGET=(
  [LIFT]="0,0,-20,0,25,0"
  [K3]="0,30,-110,0,-40,0"
  [K10]="-45,30,-80,0,50,0"
  [K15]="0,45,-60,0,45,30"
  [K14]="0,45,-60,0,45,90"
  [K9]="0,60,-50,90,40,90"
  [K8]="0,60,-50,60,40,0"
  [K5]="0,75,-30,0,-40,0"
  [K2]="0,60,-50,0,40,0"
  [P1]="0,45,-60,0,45,0"
  [K12]="0,120,-90,90,40,0"
  [K9R]="0,60,-50,90,40,0"
)
ORDER=(LIFT K3 K10 K15 K14 K9 K8 K2 K9R K12)

say() { printf '\n\033[1m### %s ###\033[0m\n' "$*"; }
die() { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

cd "$STAGE2" || die "进不了 $STAGE2"

# 要跑哪些点必须先定下来 —— 闸门必须**只校验这次真要走的段**。
# 2026-10-20 12:5x 踩过：闸门硬编码跑完整 ROUTE，而臂停在半路（K5 中止的位置），
# 「当前→LIFT」那一段根本不在本次计划里，于是闸门把整批判死、一帧没发。
LIST=("${ORDER[@]}")
if [ -n "$ONLY" ]; then
  IFS=',' read -r -a LIST <<< "$ONLY"
fi
SEQ=$(IFS=,; echo "${LIST[*]}")

say "0/3 几何闸门（不动机器人；${PACK_LABEL} + 本次要走的 ${#LIST[@]} 段）"
python3 s21_design.py --seq "$SEQ" --start-from-can --iface "$IFACE" --rate "$RATE" $PACK_FLAG \
  | tee "raw/s21_route_gate.txt"
grep -q '全段放行' "raw/s21_route_gate.txt" \
  || die "带载闸门不放行。**一帧都没发**。看 raw/s21_route_gate.txt 里标 ✗ 的那一段。"

say "1/3 can0"
ip -br link show "$IFACE" | grep -qw UP || die "$IFACE 不是 UP"

say "2/3 开始采集：${LIST[*]}   （每点 保持 ${HOLD}s、速率 ${RATE}°/s、电流上限 ${MAX_CURRENT_A}A、前缀 ${EP_PREFIX}）"
i=0
for n in "${LIST[@]}"; do
  i=$((i+1))
  tgt=${TARGET[$n]:-}
  [ -n "$tgt" ] || die "未知姿态 $n"
  say "2/3 [$i/${#LIST[@]}] $n → $tgt"
  NAME="${EP_PREFIX}$n" TARGET="$tgt" HOLD="$HOLD" RATE="$RATE" ENABLE=0 CONFIRM=0 \
    MAX_CURRENT_A="$MAX_CURRENT_A" bash run_s20.sh \
    || die "$n 采集失败（看上面 ABORT 行）。已停在当前位置，检查后用 ONLY=... 断点续跑。"
done

say "3/3 完成"
echo "episode 目录："
ls -d "$STAGE2"/episodes/"${EP_PREFIX}"* 2>/dev/null
echo
echo "回本机后："
echo "  scp -r zzz@192.168.242.128:'~/stage0/20261019_stage2/episodes/${EP_PREFIX}*' ./episodes/"
echo "  python s21_pair_fit.py    # 需要 s21e_*（空载）与 s21_*（带载）齐了才能跑"
