#!/usr/bin/env bash
# run_s3_all.sh —— 阶段 3 批处理：按 s3_design.py 的 ORDER 逐段采集
#
# 设计要点
# --------
# 1. **顺序唯一真相源**是 `s3_design.py --seq`，脚本里不再维护第二份段表
#    （S2-1 那次就因为 run_s21.sh 自己写了一版 TARGET，改姿态后两边不同步）。
# 2. 起批前跑一次**全矩阵判定 + 段间路径**，任一不放行就一段都不跑。
# 3. 每段调 `run_s3.sh`，它自己会再查一次单段闸门（段内 + 到中心的路径）。
# 4. 失败即停，`ONLY=...` 断点续跑。
#
# 用法（VM，~/stage0/20261019_stage3 下）：
#   bash run_s3_all.sh                              # 全部 40 段
#   ONLY=S1_1_lo,S1_1_mid bash run_s3_all.sh        # 只跑指定段
#   SPLIT_ONLY=train bash run_s3_all.sh             # 只跑 split=train
#   ENABLE=0 bash run_s3_all.sh                     # 跳过每段的使能（省时间）

set -uo pipefail

IFACE=${IFACE:-can0}
ONLY=${ONLY:-}
SPLIT_ONLY=${SPLIT_ONLY:-}
RATE=${RATE:-6}
ENABLE=${ENABLE:-1}
MAX_CURRENT_A=${MAX_CURRENT_A:-4.0}
DROP_MARGIN=${DROP_MARGIN:-150}
DROP_FALLBACK=${DROP_FALLBACK:-600}

S3=$HOME/stage0/20261019_stage3
cd "$S3" || { echo "[ABORT] 进不了 $S3" >&2; exit 1; }
mkdir -p raw cmd episodes

say() { printf '\n\033[1m### %s ###\033[0m\n' "$*"; }
die() { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

# 任何中断路径都要「继续发帧」，否则 10 min 后回落示教态、臂垂下来（R49）。
park_hold() {
  pgrep -f "hold_pose" >/dev/null && return 0
  setsid nohup python3 "$HOME/stage0/20261019_stage2/hold_pose.py" "$IFACE" 1800 \
      > "$S3/raw/hold_tail.log" 2>&1 < /dev/null &
  sleep 2
  pgrep -f "hold_pose" >/dev/null \
    && echo "[park] hold_pose 已起，姿态被钉住（pid $(pgrep -f hold_pose | head -1)）" >&2 \
    || echo "[park][WARN] hold_pose 没起来，臂约 10 min 后会回落示教态" >&2
}

say "0/4 can0"
ip -br link show "$IFACE" | grep -qw UP || die "$IFACE 不是 UP（三步拉起）"

say "1/4 全矩阵判定（段内几何 + 激励 + 限位）"
python3 s3_design.py --check 2>&1 | tee raw/s3_matrix_check.txt
grep -q '段全部放行' raw/s3_matrix_check.txt \
  || die "矩阵判定不放行。一段都不跑。看 raw/s3_matrix_check.txt 里标 ✗ 的段。"

say "2/4 段间路径判定（rate 口径，从当前实测姿态起）"
python3 s3_design.py --route --iface "$IFACE" 2>&1 | tee raw/s3_route_check.txt
grep -q '全段放行' raw/s3_route_check.txt \
  || die "段间路径不放行。要加中转段或调顺序。看 raw/s3_route_check.txt。"

say "3/4 组装执行序列"
LIST_STR=$(python3 s3_design.py --seq)
if [ -n "$ONLY" ]; then
  LIST_STR="$ONLY"
fi
IFS=',' read -r -a ALL <<< "$LIST_STR"

LIST=()
for n in "${ALL[@]}"; do
  n=$(echo "$n" | tr -d ' ')
  [ -n "$n" ] || continue
  if [ -n "$SPLIT_ONLY" ]; then
    sp=$(python3 s3_design.py --emit "$n" | sed -n 's/^SPLIT=//p')
    [ "$sp" = "$SPLIT_ONLY" ] || continue
  fi
  LIST+=("$n")
done
[ "${#LIST[@]}" -gt 0 ] || die "执行序列为空（检查 ONLY / SPLIT_ONLY）"
echo "  共 ${#LIST[@]} 段：${LIST[*]}"

say "4/4 开始采集（每段独立 episode，失败即停）"
i=0
for n in "${LIST[@]}"; do
  i=$((i+1))
  say "[$i/${#LIST[@]}] $n"
  NAME="$n" RATE="$RATE" ENABLE="$ENABLE" MAX_CURRENT_A="$MAX_CURRENT_A" \
    DROP_MARGIN="$DROP_MARGIN" DROP_FALLBACK="$DROP_FALLBACK" \
    IFACE="$IFACE" bash run_s3.sh \
    || { park_hold; die "$n 采集失败（看上面 ABORT 行）。已停在当前位置、已起 hold_pose 钉住，检查后用 ONLY=... 断点续跑。"; }
done

say "全部完成"

# 收尾必须「继续发帧」
# --------------------
# CAN_CTRL 下空闲约 10 min 会自己回落 TEACHING_MODE，臂随即在重力下垂下来（R49）。
# 2026-10-20 14:43 就是这样：批处理一结束没人发帧，臂从 (0,90,-69,60,50,90) 垂到
# (6.5,10.2,-34.2,-17.1,12.9,81.9)。所以批处理末尾常驻一个 hold_pose 把姿态钉住，
# 时长给 HOLD_TAIL_S 秒（默认 1800 = 30 min），足够人过来处理或接着跑下一批。
HOLD_TAIL_S=${HOLD_TAIL_S:-1800}
if [ "$HOLD_TAIL_S" -gt 0 ]; then
  echo
  echo "=== 收尾：起 hold_pose 钉住姿态 ${HOLD_TAIL_S}s（防 R49 空闲回落）==="
  setsid nohup python3 "$HOME/stage0/20261019_stage2/hold_pose.py" "$IFACE" "$HOLD_TAIL_S" \
      > "$S3/raw/hold_tail.log" 2>&1 < /dev/null &
  sleep 2
  pgrep -f "hold_pose" >/dev/null \
    && echo "  hold_pose 已在跑（pid $(pgrep -f hold_pose | head -1)），别断电、别拔 CAN" \
    || echo "  [WARN] hold_pose 没起来，臂约 10 min 后会回落示教态，尽快过来处理"
fi

echo "episode 目录："
ls -d "$S3"/episodes/*/ 2>/dev/null | wc -l
echo
echo "回本机后："
echo "  scp -r zzz@192.168.242.128:'~/stage0/20261019_stage3/episodes/*' ./episodes/"
echo "  python s3_split.py"
