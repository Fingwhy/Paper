#!/usr/bin/env bash
# L0-b 六轴档：一条命令跑完 record → sine → 前后计数 → parse → track → audit
#
# 为什么要有这个脚本：这一档前面踩过的每个坑都是「顺序」问题——
#   * 少 --allow-write  ⇒ 模式没切，后面全白跑（16:39 那次）
#   * 缺 piper_sdk     ⇒ 只跑到一半就崩
#   * record 与 cmd 抢着起 ⇒ 前 3 秒的指令没人录
# 把顺序写死在脚本里，比每次靠记忆敲四条命令可靠。
#
# 用法（VM 上，~/stage0/20261019_stage1_l0）：
#   bash run_l0b_six.sh              # 默认 60 s、±2°
#   DUR=40 TCP_DEG=1 bash run_l0b_six.sh
#
# 跑之前：工作空间清空、手放急停。这是第一次六轴同时动。

set -uo pipefail

IFACE=${IFACE:-can0}
DUR=${DUR:-60}
TCP_DEG=${TCP_DEG:-2}
FREQ=${FREQ:-0.1}
REC_SECS=$(( DUR + 15 ))

STAGE0=$HOME/stage0/20261019_stage0
STAGE1=$HOME/stage0/20261019_stage1_l0
LIMITS=$STAGE0/env/joint_limits.json

say()  { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
die()  { printf '\n[ABORT] %s\n' "$*" >&2; exit 1; }

RECPID=""
cleanup() { [ -n "$RECPID" ] && kill "$RECPID" 2>/dev/null; }
trap cleanup INT TERM

# ---------------------------------------------------------------- 0 环境
say "0/7 环境：piper_sdk / python-can 必须在同一个 python3 里"
python3 - <<'PY' || die "python3 缺 piper_sdk 或 python-can。装法：pip3 install --user --break-system-packages 'piper-sdk==0.6.1'"
import sys
import piper_sdk, can
print("python3  :", sys.executable)
print("piper_sdk:", piper_sdk.__file__)
print("python-can:", can.__version__)
PY

[ -f "$LIMITS" ] || die "缺 $LIMITS —— 没有固件∩URDF 包络就不许发六轴指令"
echo "limits   : $LIMITS"

# ---------------------------------------------------------------- 1 版本
say "1/7 版本核对（对不上就说明 VM 上是旧脚本）"
python3 "$STAGE0/scripts/stage0_probe.py" --version
python3 "$STAGE1/stage1_l0.py"            --version
python3 "$STAGE1/stage1_cmd.py"           --version 2>/dev/null || echo "stage1_cmd 无 --version"

if ! python3 "$STAGE0/scripts/stage0_probe.py" --version 2>/dev/null | grep -q '1\.12'; then
    die "probe 不是 1.12 —— 1.11 在缺 --allow-write 时会静默不发帧，会把你骗过去"
fi

# ---------------------------------------------------------------- 2 前置计数
say "2/7 前置计数（R30 要用前后差值，不记初值这趟白跑）"
ip -s link show "$IFACE" | tee /tmp/l0b_six_pre.txt
echo "已存 /tmp/l0b_six_pre.txt"

# ---------------------------------------------------------------- 3 切模式
say "3/7 切 CAN_CTRL（唯一验收标准：完成后=CAN_CTRL）"
OUT=$( cd "$STAGE0" && python3 scripts/stage0_probe.py --iface "$IFACE" \
        --allow-write --to-can-ctrl --guard-seconds 8 2>&1 )
echo "$OUT" | grep -E '^\[(SUMMARY|WARN|GUARD|LIMIT)\]' || true
echo "$OUT" | tee /tmp/l0b_six_mode.txt >/dev/null

echo "$OUT" | grep -q '完成后=CAN_CTRL' || die \
"没有观察到 完成后=CAN_CTRL。
  若显示『切程序控=未发送』→ 参数没带全；
  若显示『仍不是 CAN_CTRL(0x1)』→ 多半是臂上的示教按钮/手柄被按住，松手再跑；
  总线静默期间**绝对不要**继续发运动指令。"

# ---------------------------------------------------------------- 4 录制 + 下发
say "4/7 录制（后台，${REC_SECS}s）+ 六轴正弦（${DUR}s，±${TCP_DEG}° @ ${FREQ}Hz）"
mkdir -p "$STAGE1/runs" "$STAGE1/cmd" "$STAGE1/episodes"
cd "$STAGE1" || die "进不去 $STAGE1"

python3 stage1_l0.py record --iface "$IFACE" --seconds "$REC_SECS" --out runs/l0b_six.jsonl &
RECPID=$!
sleep 3                                    # 让录制先跑起来，否则前 3 s 的指令没人录（这就是 15:26 那个坑）

python3 stage1_cmd.py sine --iface "$IFACE" --out cmd/l0b_six \
    --duration "$DUR" --hz 100 \
    --amp-deg "$TCP_DEG" "$TCP_DEG" "$TCP_DEG" "$TCP_DEG" "$TCP_DEG" "$TCP_DEG" \
    --freq-hz "$FREQ" --max-step-deg 1.0 --limits "$LIMITS"
CMD_RC=$?
[ $CMD_RC -eq 0 ] || echo "[WARN] stage1_cmd 退出码 $CMD_RC（非 0 时先看它自己打的 ABORT 原因）"

wait "$RECPID"; RECPID=""
echo "录制结束，原始层 runs/l0b_six.jsonl"

# ---------------------------------------------------------------- 5 后置计数
say "5/7 后置计数（判断 R30 在接收端还是发送端）"
ip -s link show "$IFACE" | tee /tmp/l0b_six_post.txt
echo
echo "--- 前置 ---"; grep -A2 '^RX:\|^TX:' /tmp/l0b_six_pre.txt  || true
echo "--- 后置 ---"; grep -A2 '^RX:\|^TX:' /tmp/l0b_six_post.txt || true
echo "看 dropped / overrun / errors 的增量：涨了才是丢，没涨就是没丢（R30 的分母错已经吃过一次）"

# ---------------------------------------------------------------- 6 解析
say "6/7 解析 + 跟踪"
# --pos251-unit mrad 必须显式给：解析器默认写的是 rad（故意的，当绊线用），
# 16:45 那次就是没给，比值算出来 997.743 被判定失败。R12 已定案 0x251 = 0.001 rad。
python3 stage1_l0.py parse runs/l0b_six.jsonl --out episodes/l0b_six \
        --hz 50 --pos251-unit mrad --qcmd cmd/l0b_six.csv
PARSE_RC=$?
[ $PARSE_RC -eq 0 ] || echo "[WARN] parse 退出码 $PARSE_RC（2 = 单位判据不符，看上面那行比值）"

python3 stage1_l0.py track episodes/l0b_six/samples.jsonl --out episodes/l0b_six

# ---------------------------------------------------------------- 7 体检
say "7/7 体检：帧覆盖了哪段时间、哪里空了"
# audit 是解析器 1.7 起才有的子命令；VM 上若是 1.5 会报 invalid choice，跳过即可
if python3 stage1_l0.py --help 2>/dev/null | grep -q '{record,parse,track,audit'; then
    python3 stage1_l0.py audit runs/l0b_six.jsonl --bucket 10
else
    echo "[SKIP] 本机解析器没有 audit 子命令（1.7 起才有）：先传 stage1_l0.py 再跑这条"
fi

say "完成"
cat <<EOF
产物：
  $STAGE1/runs/l0b_six.jsonl
  $STAGE1/cmd/l0b_six.csv
  $STAGE1/episodes/l0b_six/（samples.jsonl + 报告）
请把上面 3/7 的模式行、5/7 计数差值、6/7 的六轴响应比/滞后、7/7 的覆盖率贴回来。
EOF
