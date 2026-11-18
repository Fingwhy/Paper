#!/usr/bin/env bash
# run_s4_topup.sh —— 阶段 4 补采：q2∈[60,105] 稀疏区
#
# 与 run_s3.sh 的区别只有一个：段参数来自 s4_topup_design.py，不是 s3_design.py。
# 其余流程（闸门 → 切 CAN_CTRL → 使能 → 录制 → pose → sine → parse）完全一致，
# 因为那套流程已经被 42 段验证过（含中断恢复、电流守卫、卡死守卫、下降阈值）。
#
# 每段都重新切 CAN_CTRL：R49 实测 CAN_CTRL 空闲约 10 分钟会回落 TEACHING。
#
# 用法（VM，~/stage0/20261020_stage4 下）：
#   NAME=P2_J2mid bash run_s4_topup.sh
#   ALL=1 bash run_s4_topup.sh          # 跑完全部 7 段
#   ENABLE=0  RATE=6  MAX_CURRENT_A=3.0

set -uo pipefail

IFACE=${IFACE:-can0}
NAME=${NAME:-}
ALL=${ALL:-0}
RATE=${RATE:-6}
ENABLE=${ENABLE:-1}
MAX_CURRENT_A=${MAX_CURRENT_A:-3.0}
STALL_WIN_S=${STALL_WIN_S:-1.0}
STALL_EPS_DEG=${STALL_EPS_DEG:-0.08}
LEAD_MAX_DEG=${LEAD_MAX_DEG:-2.0}
DROP_MARGIN=${DROP_MARGIN:-150}
DROP_FALLBACK=${DROP_FALLBACK:-600}
REC_EXTRA=${REC_EXTRA:-12}

STAGE0=$HOME/stage0/20261019_stage0
STAGE1=$HOME/stage0/20261019_stage1_l0
STAGE3=$HOME/stage0/20261019_stage3
S4=$HOME/stage0/20261020_stage4
LIMITS=$STAGE0/env/joint_limits.json

say() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
die() { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

cd "$S4" || die "进不了 $S4"
mkdir -p "$S4"/{raw,cmd,episodes}

run_one() {
  local NAME="$1"
  say "段 $NAME"

  local REQ NM CENTER AMP FREQ DUR REC
  REQ=$(python3 s4_topup_design.py --emit "$NAME") || die "未知段 $NAME"
  IFS='|' read -r NM CENTER AMP FREQ DUR <<< "$REQ"
  REC=$(( DUR + REC_EXTRA ))
  echo "  center=[$CENTER]  amp=[$AMP]  f=${FREQ}Hz  ${DUR}s（录制 ${REC}s）"

  say "1/8 can0 状态"
  ip -br link show "$IFACE" | grep -qw UP || die "$IFACE 不是 UP"
  # 发帧前的硬判据：总线上必须有第二个节点在应答。0 帧 = 臂没上电或 CAN 没插，
  # 这时候切模式必失败，早点停比跑到一半再停干净。
  local NF
  NF=$(timeout 4 candump -n 100000 "$IFACE" 2>/dev/null | wc -l)
  echo "  4 s 内收到 $NF 帧"
  [ "$NF" -gt 100 ] || die "总线静默（$NF 帧）—— 臂侧没供电或 CAN 未接通，先查电源与航插"

  say "2/8 段内几何闸门（不动机器人）"
  python3 s4_topup_design.py --gate > "raw/${NAME}_gate.txt" 2>&1 \
    || die "闸门脚本失败，见 raw/${NAME}_gate.txt"
  grep -q "^| $NAME |.*PASS" "raw/${NAME}_gate.txt" \
    || { tail -12 "raw/${NAME}_gate.txt"; die "$NAME 几何不合格，一帧都没发"; }
  echo "  闸门 PASS"

  say "3/8 切 CAN 控制模式"
  ( cd "$STAGE0/scripts" && python3 stage0_probe.py --iface "$IFACE" --phase sdk \
      --allow-write --to-can-ctrl ) || die "切模式失败"

  say "4/8 使能六轴"
  if [ "$ENABLE" = "1" ]; then
    ( cd "$STAGE0/scripts" && python3 stage0_set_zero.py hold --iface "$IFACE" \
        --from-joint 1 --to-joint 6 ) || die "使能失败"
  else
    echo "跳过（ENABLE=0）"
  fi

  say "5/8 后台上录制（${REC}s）"
  rm -f "$S4/raw/$NAME.jsonl"
  ( cd "$STAGE1" && exec python3 stage1_l0.py record --iface "$IFACE" \
      --seconds "$REC" --out "$S4/raw/$NAME.jsonl" ) &
  local RECPID=$!
  trap 'kill '"$RECPID"' 2>/dev/null' INT TERM
  sleep 3
  kill -0 "$RECPID" 2>/dev/null || die "录制进程没起来"

  say "6/8 算本段允许下降量（计划下降 + ${DROP_MARGIN} mm）"
  local DROP_MM
  DROP_MM=$( cd "$STAGE3" && python3 s3_drop.py --iface "$IFACE" --target-deg "$CENTER" \
      --margin "$DROP_MARGIN" --fallback-mm "$DROP_FALLBACK" ) || die "算下降阈值失败"
  DROP_MM=$(echo "$DROP_MM" | tail -1)
  case "$DROP_MM" in ''|*[!0-9]*) die "下降阈值不是整数：'$DROP_MM'" ;; esac
  echo "  --max-drop-mm=$DROP_MM"

  say "7/8 pose 到中心姿态（@ ${RATE}°/s）"
  ( cd "$STAGE1" && python3 stage1_cmd.py pose --iface "$IFACE" \
      --limits "$LIMITS" --target-deg $CENTER \
      --rate-deg-s "$RATE" --hold-s 3 \
      --stall-win-s "$STALL_WIN_S" --stall-eps-deg "$STALL_EPS_DEG" \
      --max-current-a "$MAX_CURRENT_A" --max-drop-mm "$DROP_MM" \
      --lead-max-deg "$LEAD_MAX_DEG" \
      --out "$S4/cmd/${NAME}_pre" --yes )
  local PRE_RC=$?
  if [ "$PRE_RC" -ne 0 ]; then
    kill "$RECPID" 2>/dev/null
    die "pre-pose 退出码 $PRE_RC（2=越包络 3=未到位 4=守卫触发）"
  fi

  say "7b/8 正弦轨迹  amp=[$AMP]  f=${FREQ}Hz  ${DUR}s"
  ( cd "$STAGE1" && python3 stage1_cmd.py sine --iface "$IFACE" \
      --limits "$LIMITS" --amp-deg $AMP --freq-hz "$FREQ" \
      --duration "$DUR" --phase-spread 0 \
      --max-step-deg 1.0 --limit-tol-deg 3.0 \
      --max-current-a "$MAX_CURRENT_A" --cur-grace-s 0.3 \
      --out "$S4/cmd/$NAME" --yes )
  local RUN_RC=$?

  say "8/8 等录制收尾"
  wait "$RECPID" 2>/dev/null
  trap - INT TERM

  if [ "$RUN_RC" -ne 0 ]; then
    die "轨迹退出码 $RUN_RC（2=越包络 3=未到位 4=守卫触发）。录制已保留 raw/$NAME.jsonl"
  fi

  ( cd "$STAGE1" && python3 stage1_l0.py parse "$S4/raw/$NAME.jsonl" \
      --hz 50 --pos251-unit mrad --qcmd "$S4/cmd/$NAME.csv" --out "$S4/episodes/$NAME" ) \
    || die "parse 失败（数据在 raw/$NAME.jsonl）"

  printf '\n\033[1m完成：%s\033[0m  episode=%s\n' "$NAME" "$S4/episodes/$NAME"
}

if [ "$ALL" = "1" ]; then
  LIST=$(python3 -c "import json;print(' '.join(t['name'] for t in json.load(open('topup_tracks.json'))))")
  echo "批量：$LIST"
  for n in $LIST; do
    NAME="$n" run_one "$n" || die "$n 失败，已停。修好后改名重跑单段。"
  done
  echo "全部完成"
else
  [ -n "$NAME" ] || die "没给 NAME。用法：NAME=P2_J2mid bash run_s4_topup.sh  或  ALL=1 bash run_s4_topup.sh"
  run_one "$NAME"
fi
