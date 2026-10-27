#!/bin/bash
# 全量棘轮：每轮 张开 -> 回 J6=170 -> 夹紧 -> twist_log 拧到 0
# 用法: ./ratchet_log.sh [轮数] [文件名前缀]
#   ./ratchet_log.sh 3 "3.全量棘轮"
set -u

N=${1:-3}
BASE=${2:-"3.全量棘轮"}
STAGE6="$HOME/stage0/20261021_stage6"
STAGE4="$HOME/stage0/20261020_stage4"
QLOCK="149.97,126.05,-102.20,0.00,70.27"   # 2026-10-23 用户指定基准位姿

for i in $(seq 1 "$N"); do
  echo "========== 第 $i / $N 轮 =========="

  echo "[$i] 张开"
  ( cd "$STAGE4" && python3 -u grip.py open --angle 70000 --effort 2000 2>&1 | tail -1 )

  echo "[$i] 回位 J6=170"
  ( cd "$STAGE6" && python3 -u s6_k6_calib.py goto \
      --target-deg "$QLOCK,170" --rate 15 --yes 2>&1 | tail -1 )

  echo "[$i] 夹紧"
  ( cd "$STAGE4" && python3 -u grip.py close --effort 5000 --step 500 2>&1 | tail -2 )

  # 夹空保护：clamp 后开度 < 20mm 说明两指之间没有瓶盖，本轮不跑
  GA=$( cd "$STAGE4" && python3 -u grip.py status 2>&1 | grep -oE '[0-9]+（' | head -1 | grep -oE '[0-9]+' )
  if [ -z "$GA" ] || [ "$GA" -lt 20000 ]; then
    echo "[$i] ⚠ 夹持开度 ${GA:-?} (<20000) 疑似夹空，跳过本轮，请检查瓶子位置"
    break
  fi
  echo "[$i] 夹持开度 ${GA} ok"

  echo "[$i] 全量拧 170°"
  ( cd "$STAGE6" && python3 -u twist_log.py --target 0 --rate 15 \
      --name "${BASE}${i}" --grip-effort 5000 --grip-squeeze 1000 2>&1 | tail -5 )
done

echo "========== 完成，文件列表 =========="
ls -l "$STAGE6/data/${BASE}"*.csv 2>/dev/null | awk '{print $5, $9}'
