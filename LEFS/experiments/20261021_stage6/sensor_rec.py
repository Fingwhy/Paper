#!/usr/bin/env python
"""力矩传感器记录器（50 Hz，带绝对时间戳），用于和机械臂电流做同步对比。

运行机器与目录（Windows 本机）
------------------------------
    C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6/

为什么单独写一个，而不是用 torque485.py 的 log
----------------------------------------------
torque485.py 的 `log` 记的是「相对开始的时间」，而这一轮实验要和**另一台机器**
（VM 上的机械臂）的记录对齐，所以必须落**绝对 epoch 时间**，才能和臂侧按时间戳配对。

输出（jsonl）
-------------
    {"type":"header", "port":..., "baud":..., "addr":..., "span":..., "full_display":...,
     "t0_epoch":..., "t0_iso":...}
    {"t": <epoch 秒>, "raw": <有符号原始值>, "tau": <N·m>}

用法
----
    python sensor_rec.py --port COM3 --seconds 60 --out runs/sync1_sensor.jsonl
    python sensor_rec.py --port COM3 --seconds 0            # 0 = 一直记到 Ctrl+C
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import torque485 as T  # noqa: E402  复用读寄存器的实现


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--parity", default="N")
    ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--span", type=float, default=5.0, help="传感器量程 N·m")
    ap.add_argument("--full-display", type=float, default=5.0,
                    help="满量程对应的显示值（默认 5.0 ⇒ 1 count = 0.001 N·m）")
    ap.add_argument("--seconds", type=float, default=60.0, help="记录时长，0=不限")
    ap.add_argument("--interval", type=float, default=0.0,
                    help="读取间隔秒；0=能多快就多快（实测约 50 Hz）")
    ap.add_argument("--out", default=str(HERE / "runs" / "sensor_rec.jsonl"))
    ap.add_argument("--nm-per-count", type=float, default=None,
                    help="标定系数 N·m/count（由 sensor_calib1.py 给出）；给了就忽略 span/full-display")
    ap.add_argument("--zero-nm", type=float, default=0.0, help="零点补偿 N·m")
    a = ap.parse_args()

    c = T.make_client(a.port, a.baud, a.parity, timeout=0.3)
    if not c.connect():
        print("串口打不开：", a.port)
        return 1

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    f = open(out, "w", encoding="utf-8")
    t0 = time.time()
    f.write(json.dumps(dict(type="header", port=a.port, baud=a.baud, addr=a.addr,
                            span=a.span, full_display=a.full_display,
                            t0_epoch=round(t0, 6),
                            t0_iso=time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0))),
                       ensure_ascii=False) + "\n")
    f.flush()

    print(f"记录到 {out}")
    print(f"起点 epoch = {t0:.3f}（臂侧对齐时要用这个数）")
    print(f"时长 {a.seconds:g} s，量程 ±{a.span} N·m\n")

    n = err = 0
    buf, sec, last_sec = [], 0, -1
    try:
        while a.seconds <= 0 or time.time() - t0 < a.seconds:
            tick = time.time()
            v = T._read_torque(c, a)
            now = time.time()
            if v is None:
                err += 1
            else:
                raw, tau, _dot = v
                f.write(json.dumps(dict(t=round(now, 6), raw=int(raw),
                                        tau=round(tau, 6))) + "\n")
                n += 1
                buf.append(tau)
                sec += 1
            el = int(now - t0)
            if el > last_sec:
                last_sec = el
                if buf:
                    print(f"  {el:4d}s  {sec:3d} 帧  τ {min(buf):+.4f} ~ {max(buf):+.4f} N·m",
                          flush=True)
                buf, sec = [], 0
            rest = a.interval - (time.time() - tick)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        print("\n已中断，文件已保留")
    finally:
        c.close()
        f.close()

    dt = time.time() - t0
    print(f"\n完成：{n} 帧 / {dt:.1f} s = {n/max(dt,1e-9):.1f} Hz，读取失败 {err} 次")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
