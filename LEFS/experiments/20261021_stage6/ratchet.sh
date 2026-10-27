#!/bin/bash
# ratchet.sh —— 自动棘轮拧盖：N 轮 [张开 → J6 回 170 → 满力夹紧 → 拧到 0]
# 每轮单独存 twist_cyc<i>.csv，最后打印每轮的 I6 均值便于判断是否已脱盖。
cd ~/stage0/20261021_stage6 || exit 1
N=${1:-3}
QLOCK="150.06,117.34,-87.31,2.39,65.79"
for i in $(seq 1 "$N"); do
  echo "======== 轮 $i / $N ========"
  ( cd ../20261020_stage4 && python3 -u grip.py open --angle 70000 --effort 1500 2>&1 | tail -1 )
  python3 -u s6_k6_calib.py goto --target-deg "$QLOCK",170 --rate 15 --yes 2>&1 | tail -1
  ( cd ../20261020_stage4 && python3 -u grip.py close --effort 5000 --step 500 2>&1 | tail -1 )
  python3 -u s6_k6_calib.py twist --target 0 --rate 15 --tag "rc$i" \
      --grip-effort 5000 --grip-squeeze 1000 2>&1 | tail -3
done
echo "======== 汇总 ========"
python3 - << 'PY'
import csv, glob, statistics as st
for fn in sorted(glob.glob("data/twist_rc*.csv")):
    rows=list(csv.reader(open(fn,encoding="utf-8")))[1:]
    i6=[abs(float(r[3])) for r in rows]
    n=len(i6)
    print("%-26s 帧=%4d  I6前1/5=%.4f  I6后1/5=%.4f" % (
        fn.split("/")[-1], n, sum(i6[:n//5])/(n//5), sum(i6[-n//5:])/(n//5)))
PY
