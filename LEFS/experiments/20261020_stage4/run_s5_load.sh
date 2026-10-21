#!/usr/bin/env bash
# run_s5_load.sh —— 阶段 5（L2）称重采集：一档质量 × 6 个姿态
#
# 为什么按「质量档」分批，而不是一次跑 24 段
# ------------------------------------------
# 换重物必须由人来做，而且每次都要称重记录。脚本按档跑完就退出，
# 你在两次之间换重物、重新称重、告诉我实际质量，再跑下一档。
# 这样「哪个质量对应哪批数据」不靠记忆，靠文件名（M0_A / M1_A …）。
#
# 用法（VM，~/stage0/20261020_stage4 下）：
#   TAG=M0 bash run_s5_load.sh                    # 空载 6 段（V 组：竖直夹持）
#   TAG=M1 bash run_s5_load.sh                    # 装好 100 g 后
#   TAG=M2 bash run_s5_load.sh                    # 300 g
#   TAG=M3 bash run_s5_load.sh                    # 500 g
#   GROUP=U TAG=M1 bash run_s5_load.sh            # 同一质量、工具轴近竖直（朝上放东西）
#   DRY=1 TAG=M0 bash run_s5_load.sh              # 只打印不执行
#
# 三组夹持方位：
#   V = 工具轴水平（ax_z≈0，两指左右夹住），6 姿态 A~F  ← 默认
#   U = 工具轴近竖直（ax_z 0.87~0.97，东西放在掌面上），4 姿态 U1~U4
#   H = 工具轴水平但 J3 更深、净空更大，4 姿态 H1~H4（备用，默认不跑）
#
# 每组都要有**同姿态的空载基线**（M0_A + M1_A 这样成对），
# 因为 L1 模型在新位形上的系统偏置是姿态相关的，只能用同姿态差分消掉。
#
# 安全：沿用阶段 3/4 的全部守卫（电流 3 A / 卡死 / 高度），不新写逻辑。
# 另加「夹持完整性」检查：每段跑完回读夹爪开度，与段首比超过 3 mm 就 ABORT
# —— 重物滑动会让整段数据作废，必须当场停。

set -uo pipefail

IFACE=${IFACE:-can0}
TAG=${TAG:-}
GROUP=${GROUP:-U}
GRIP_TOL=${GRIP_TOL:-3000}   # 夹爪开度允许漂移（0.001 mm）= 3 mm
RATE=${RATE:-6}
HOLD_S=${HOLD_S:-12}
REC_S=${REC_S:-40}
ENABLE=${ENABLE:-1}
MAX_CURRENT_A=${MAX_CURRENT_A:-3.0}
STALL_WIN_S=${STALL_WIN_S:-1.0}
STALL_EPS_DEG=${STALL_EPS_DEG:-0.08}
LEAD_MAX_DEG=${LEAD_MAX_DEG:-2.0}
DROP_MARGIN=${DROP_MARGIN:-150}
DROP_FALLBACK=${DROP_FALLBACK:-600}
DRY=${DRY:-0}

STAGE0=$HOME/stage0/20261019_stage0
STAGE1=$HOME/stage0/20261019_stage1_l0
STAGE3=$HOME/stage0/20261019_stage3
S5=$HOME/stage0/20261020_stage4
LIMITS=$STAGE0/env/joint_limits.json

say() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
die() { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

cd "$S5" || die "进不了 $S5"
mkdir -p raw cmd episodes

[ -n "$TAG" ] || die "没给 TAG。用法：TAG=M0 bash run_s5_load.sh"
# 允许任意标签：M0~M3 是标定用的质量档，新物体可用 OBJ1 / N1020_1 之类
case "$TAG" in
  *[!A-Za-z0-9_-]*) die "TAG 只能含字母、数字、下划线、连字符（收到 '$TAG'）" ;;
esac
case "$GROUP" in
  U) POSES="U1 U2 U3" ;;        # 掌心朝上（放取物体用），3 段
  V) POSES="A B C" ;;           # 掌心竖直、工具轴水平，3 段
  *) die "GROUP 只能是 U / V" ;;
esac
# ONLY_POSES="A" 之类可以只跑指定姿态（称一个新物体只需要「朝前」+「掌心朝上」各一段）
POSES=${ONLY_POSES:-$POSES}
NP=$(echo $POSES | wc -w)

# 夹爪开度（0.001 mm）。读不到就返回空串。
grip_angle() {
  python3 - <<'PY' 2>/dev/null || true
import time
from piper_sdk import C_PiperInterface as C
p = C("can0"); p.ConnectPort(); time.sleep(0.8)
print(p.GetArmGripperMsgs().gripper_state.grippers_angle)
PY
}

say "0/6 环境与总线（GROUP=$GROUP，$NP 段）"

# 关键：必须杀掉 hold_pose.py。它每 10 ms 把「实测姿态」当目标重发，
# 与本脚本的 pose 伺服指令互相打架 → 臂在两个姿态之间来回跳，
# 采到的 q 与 q_cmd 完全对不上（2026-10-20 18:2x 就这么废了 M0 的 3 段）。
for p in $(pgrep -f "hold_pose[.]py" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 1
pgrep -f "hold_pose[.]py" >/dev/null && die "hold_pose 杀不掉，先手动处理（它会与 pose 抢控制权）"
echo "  hold_pose 已确认停止"
python3 -c "import piper_sdk, can; print('piper_sdk ok, python-can', can.__version__)" \
  || die "缺 piper_sdk / python-can"
ip -br link show "$IFACE" | grep -qw UP || die "$IFACE 不是 UP"
NF=$(timeout 4 candump -n 100000 "$IFACE" 2>/dev/null | wc -l)
echo "  4 s 内 $NF 帧"
[ "$NF" -gt 100 ] || die "总线静默（$NF 帧）—— 臂侧没供电或 CAN 未接通"

say "1/6 几何闸门（不动机器人，GROUP=$GROUP）"
python3 s5_load_design.py --gate --group "$GROUP" > raw/${TAG}_${GROUP}_gate.txt 2>&1 || die "闸门脚本失败"
grep -q "全部放行" raw/${TAG}_${GROUP}_gate.txt || { tail -16 raw/${TAG}_${GROUP}_gate.txt; die "姿态集未全放行"; }
echo "  $NP/$NP 放行"

say "2/6 切 CAN_CTRL"
( cd "$STAGE0/scripts" && python3 stage0_probe.py --iface "$IFACE" --phase sdk \
    --allow-write --to-can-ctrl ) || die "切模式失败"

say "3/6 使能六轴"
if [ "$ENABLE" = "1" ]; then
  ( cd "$STAGE0/scripts" && python3 stage0_set_zero.py hold --iface "$IFACE" \
      --from-joint 1 --to-joint 6 ) || die "使能失败"
else
  echo "跳过（ENABLE=0）"
fi

for P in $POSES; do
  NAME="${TAG}_${P}"
  say "段 $NAME"
  CENTER=$(python3 s5_load_design.py --emit "$P" --group "$GROUP") || die "取不到姿态 $P"
  echo "  center=[$CENTER]  保持 ${HOLD_S}s  录制 ${REC_S}s"

  G0=$(grip_angle)
  echo "  段首夹爪开度 = ${G0:-读不到}"

  if [ "$DRY" = "1" ]; then
    echo "  [DRY] 跳过执行"
    continue
  fi

  say "4/6 后台上录制（${REC_S}s）"
  rm -f "$S5/raw/$NAME.jsonl"
  ( cd "$STAGE1" && exec python3 stage1_l0.py record --iface "$IFACE" \
      --seconds "$REC_S" --out "$S5/raw/$NAME.jsonl" ) &
  RECPID=$!
  trap 'kill '"$RECPID"' 2>/dev/null' INT TERM
  sleep 3
  kill -0 "$RECPID" 2>/dev/null || die "录制进程没起来"

  DROP_MM=$( cd "$STAGE3" && python3 s3_drop.py --iface "$IFACE" --target-deg "$CENTER" \
      --margin "$DROP_MARGIN" --fallback-mm "$DROP_FALLBACK" ) || die "算下降阈值失败"
  DROP_MM=$(echo "$DROP_MM" | tail -1)
  case "$DROP_MM" in ''|*[!0-9]*) kill "$RECPID" 2>/dev/null; die "下降阈值非法：'$DROP_MM'" ;; esac

  say "5/6 pose 到位并保持 ${HOLD_S}s（@ ${RATE}°/s）"
  ( cd "$STAGE1" && python3 stage1_cmd.py pose --iface "$IFACE" \
      --limits "$LIMITS" --target-deg $CENTER \
      --rate-deg-s "$RATE" --hold-s "$HOLD_S" \
      --stall-win-s "$STALL_WIN_S" --stall-eps-deg "$STALL_EPS_DEG" \
      --max-current-a "$MAX_CURRENT_A" --max-drop-mm "$DROP_MM" \
      --lead-max-deg "$LEAD_MAX_DEG" \
      --out "$S5/cmd/$NAME" --yes )
  RC=$?
  if [ "$RC" -ne 0 ]; then
    kill "$RECPID" 2>/dev/null
    die "$NAME pose 失败，退出码 $RC（2=越包络 3=未到位 4=守卫触发）"
  fi

  say "6/6 等录制收尾并 parse"
  wait "$RECPID" 2>/dev/null
  trap - INT TERM
  ( cd "$STAGE1" && python3 stage1_l0.py parse "$S5/raw/$NAME.jsonl" \
      --hz 50 --pos251-unit mrad --qcmd "$S5/cmd/$NAME.csv" --out "$S5/episodes/$NAME" ) \
    || die "$NAME parse 失败（raw/$NAME.jsonl 保留）"

  # ---- 姿态一致性：实测稳态必须真的停在目标上（防「两个进程抢控制权」这类污染）----
  POSE_CHK=$(python3 - "$S5/episodes/$NAME/samples.jsonl" $CENTER <<'PY'
import json, sys
p = sys.argv[1]; tgt = [float(x) for x in sys.argv[2:8]]
rows = [json.loads(l) for l in open(p, encoding="utf-8")]
first = next((i for i, r in enumerate(rows) if r.get("q_cmd") is not None), 0)
rows = rows[first:][-500:]                     # 末 10 s
qs = [[v * 57.29578 for v in r["q"]] for r in rows if r.get("q") and None not in r["q"]]
if not qs:
    print("NAN"); raise SystemExit
m = [sum(c) / len(qs) for c in zip(*qs)]
err = max(abs(a - b) for a, b in zip(m, tgt))
print(f"{err:.3f} " + " ".join(f"{v:.1f}" for v in m))
PY
)
  echo "  稳态实测 = ${POSE_CHK#* }   与目标最大偏差 = ${POSE_CHK%% *}°"
  case "$POSE_CHK" in NAN*) die "$NAME 稳态无有效帧" ;; esac
  POSE_ERR=${POSE_CHK%% *}
  awk -v e="$POSE_ERR" 'BEGIN{exit !(e>1.5)}' && die "$NAME 稳态与目标差 ${POSE_ERR}° > 1.5°，本段作废（常见原因：有其它进程在抢控制权）"

  # ---- 夹持完整性：重物滑了整段作废，必须当场停 ----
  G1=$(grip_angle)
  if [ -n "$G0" ] && [ -n "$G1" ]; then
    DG=$(( G1 - G0 )); DG=${DG#-}
    echo "  段尾夹爪开度 = $G1（变化 $DG）"
    [ "$DG" -le "$GRIP_TOL" ] || die "$NAME 夹爪开度变化 $DG > $GRIP_TOL（重物可能滑动），整段作废，检查夹持"
  fi
  echo "  完成 → episodes/$NAME"
done

say "本档 $TAG（$GROUP 组，$NP 段）完成"
echo "  段名：$(for P in $POSES; do printf '%s_%s ' "$TAG" "$P"; done)"
