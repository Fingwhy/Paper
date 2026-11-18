#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""torque485.py —— 力矩传感器 Modbus RTU / RS485 接入与扫描工具

为什么先扫描
------------
力矩传感器的寄存器表**因厂商而异**（见文末「三类常见协议」）。铭牌上通常只写
量程、供电、输出类型，不写寄存器地址。未知型号时的顺序只能是：
    先确认波特率 → 再确认从站地址 → 再找数据寄存器 → 最后定标换算系数。

依赖
----
    pip install pymodbus pyserial

用法（Windows 端口形如 COM3，Linux 形如 /dev/ttyUSB0）
--------------------------------------------------------
    # 0) 先纯监听：判断仪表是「主动连续发送」还是「Modbus 从站」
    python3 torque485.py --port COM3 sniff

    # 1) 若是从站（sniff 没数据），扫波特率与从站地址
    python3 torque485.py --port COM3 scan

    # 2) 转储 0x0000~0x001F 的全部保持寄存器，人工找力矩值
    python3 torque485.py --port COM3 --baud 9600 --addr 1 dump

    # 3) 连续读取并换算（YBNJ12 型：0~2000 映射 ±量程）
    python3 torque485.py --port COM3 --baud 9600 --addr 1 read --raw-max 2000 --span 100

    # 4) 连续读取（ATO 型：原始值 × 0.001 N·m/LSB）
    python3 torque485.py --port COM3 --baud 38400 --addr 1 read --scale 0.001

Linux 需要串口权限：sudo usermod -aG dialout $USER （重新登录生效）
"""

import argparse
import sys
import time
from pathlib import Path

BAUDS = [9600, 19200, 38400, 57600, 115200, 4800, 2400]
PARITIES = ["N", "E", "O"]


def make_client(port, baud, parity="N", timeout=0.3):
    """port 传 "D2XX" 或 "D2XX:序列号" 时走 FTDI 的 D2XX 直连，不需要 COM 口。

    2026-10-26 现场：USB-485 插上后 Windows 只枚举出 `USB Serial Converter`
    （VID_0403&PID_6001），VCP 驱动没绑定、**没生成 COM 口**，pyserial 列 0 个串口。
    这时可以用 D2XX 层直接收发字节自己拼 Modbus 帧（见 d2xx_modbus.py）。
    """
    p = str(port or "")
    if p.upper().startswith("D2XX"):
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from d2xx_modbus import D2xxClient
        sn = p.split(":", 1)[1] if ":" in p else None
        return D2xxClient(sn, baud, timeout)
    from pymodbus.client import ModbusSerialClient
    return ModbusSerialClient(port=port, baudrate=baud, parity=parity,
                              stopbits=1, bytesize=8, timeout=timeout)


def read_regs(client, addr, count, slave):
    """兼容 pymodbus 2.x/3.x 的从站参数写法（slave / unit / device_id）。"""
    for kw in ({"slave": slave}, {"device_id": slave}, {"unit": slave}):
        try:
            return client.read_holding_registers(addr, count=count, **kw)
        except TypeError:
            continue
        except Exception:
            return None
    return None


def is_err(res):
    if res is None:
        return True
    fn = getattr(res, "isError", None)
    return bool(fn()) if callable(fn) else False


def cmd_scan(a):
    print(f"扫描 {a.port}：{len(BAUDS)} 种波特率 × {a.parity} 校验 × 地址 1~{a.max_addr}")
    print("（每组合读一次 0x0000，超时 0.3 s；全部没响应就换 A/B 线序或检查供电）\n")
    found = []
    for baud in BAUDS:
        c = make_client(a.port, baud, a.parity)
        if not c.connect():
            print(f"  {baud}: 串口打不开，先确认端口号和驱动")
            return 1
        hit = []
        for addr in range(1, a.max_addr + 1):
            r = read_regs(c, 0, 1, addr)
            if not is_err(r):
                val = r.registers[0]
                hit.append(addr)
                print(f"  ✓ {baud} {a.parity}8 1  地址 {addr:3d}  0x0000 = "
                      f"{val} (0x{val:04X})")
        c.close()
        if hit:
            found.append((baud, hit))
    if not found:
        print("\n一个都没响应。按顺序排查：")
        print("  1. 供电：传感器要 24 V（万用表量电源端）")
        print("  2. 线序：RS485 A/B 接反是**静默失败**，把 A/B 对调再扫")
        print("  3. 校验位：试 --parity E 或 O")
        print("  4. 屏蔽线是否只在一端接地（两端都接会引入地环流）")
        print("  5. 用厂商上位机（多数厂家提供）先验证传感器本身是好的")
        return 2
    print("\n可用组合：")
    for baud, hit in found:
        print(f"  --baud {baud} --parity {a.parity} --addr {hit[0]}")
    return 0


def cmd_dump(a):
    c = make_client(a.port, a.baud, a.parity)
    if not c.connect():
        print("串口打不开")
        return 1
    print(f"读 {a.baud} {a.parity}8 1  地址 {a.addr}  寄存器 0x0000~0x{a.count-1:04X}\n")
    print(" 寄存器   十进制    十六进制")
    for off in range(0, a.count, 8):
        r = read_regs(c, off, min(8, a.count - off), a.addr)
        if is_err(r):
            print(f"  0x{off:04X}  —— 无响应 ——")
            continue
        for i, v in enumerate(r.registers):
            # 同一位可能有符号，负值更符合力矩
            sv = v - 65536 if v > 32767 else v
            print(f"  0x{off+i:04X}  {v:6d}   0x{v:04X}   （int16: {sv}）")
    c.close()
    return 0


def _conv(v, a):
    """把原始寄存器值换算成 N·m。三种常见口径，按参数选一种。"""
    if a.scale is not None:                      # ATO 型：原始值 × 系数
        return v * a.scale
    if a.raw_max is not None and a.span is not None:
        # YBNJ12 型：0~raw_max 线性映射 [-span, +span]
        lo, hi = -a.span, a.span
        return (hi - lo) * v / a.raw_max + lo
    return None


def cmd_read(a):
    c = make_client(a.port, a.baud, a.parity, timeout=a.timeout)
    if not c.connect():
        print("串口打不开")
        return 1
    mode = ("×%.6g" % a.scale) if a.scale is not None else (
        f"0~{a.raw_max}→±{a.span} N·m" if a.raw_max and a.span else "未换算，只显示原始值")
    print(f"读 0x{a.reg:04X}  地址 {a.addr}  换算：{mode}   （Ctrl+C 结束）\n")
    print("    时间     原始值   int16      力矩(N·m)")
    t0 = time.time()
    n = err = 0
    try:
        while a.count_reads <= 0 or n < a.count_reads:
            r = read_regs(c, a.reg, 1, a.addr)
            ts = time.time() - t0
            if is_err(r):
                err += 1
                print(f"  {ts:7.2f}s   —— 无响应（累计 {err} 次）")
            else:
                v = r.registers[0]
                sv = v - 65536 if v > 32767 else v
                tv = _conv(sv if a.signed else v, a)
                print(f"  {ts:7.2f}s  {v:6d}  {sv:6d}   "
                      + ("%.4f" % tv if tv is not None else "—"))
                n += 1
            time.sleep(a.interval)
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        c.close()
    return 0


def cmd_sniff(a):
    """纯监听：不发任何字节，只看仪表有没有主动往外发。

    为什么要这个：五位显示控制仪有两种 485 行为——
      * **主动连续发送**（很多数显表/称重表这样）：开机就一直往外吐数，
        Modbus 扫描会因为「问不到」而全部无响应，但数据其实一直在线上；
      * **Modbus 从站**：不问你就不说话，只能用 scan 主动读。
    仪表是第一种时，sniff 能立刻看到波特率和帧格式；是第二种时，sniff 一无所获，
    转头用 scan。所以先 sniff 再 scan，能少走弯路。
    """
    import serial
    print(f"纯监听 {a.port}：对 {len(BAUDS)} 种波特率各听 1.2 s，不发任何数据\n")
    hit = []
    for baud in BAUDS:
        try:
            s = serial.Serial(a.port, baud, bytesize=8, parity=a.parity,
                              stopbits=1, timeout=0.15)
        except Exception as e:
            print(f"  串口打不开：{e}")
            return 1
        s.reset_input_buffer()
        t0 = time.time()
        got = b""
        while time.time() - t0 < 1.2:
            d = s.read(128)
            if d:
                got += d
        s.close()
        if got:
            hit.append(baud)
            print(f"  ✓ {baud} {a.parity}8 1：{len(got)} 字节 / 1.2 s"
                  f"（约 {len(got)/1.2:.0f} B/s）")
            print(f"      前 32 字节：{got[:32].hex(' ')}")
            print(f"      可打印部分：{got[:60]!r}")
    if not hit:
        print("\n没有任何主动数据 —— 仪表大概是 Modbus 从站（不问你就不说）。")
        print("下一步用：python3 torque485.py --port %s scan" % a.port)
        return 3
    print(f"\n猜一下：最像的波特率是 {hit[0]}"
          f"（若帧长 8 字节且以 CRC 结尾，多半就是 Modbus RTU 应答格式）")
    return 0


def cmd_cypy(a):
    """CYPY-I 系列专用：一次读 0x54(小数点) + 0x55(符号) + 0x56(显示值)。

    这个表最容易踩的坑：**符号不在显示值里**。
    0x56 只给幅值 0~60000，正负单独放在 0x55（0=正，1=负）。
    只读 0x56 按 int16 解释 → 负力矩会被当成一个很大的正数（或者干脆看不出符号）。

    换算：力矩 = ±(显示值 / 10^Sd) × span / full_display
      span        量程上限（本表 5 N·m）
      full_display 仪表标定时「满量程」对应的显示值（看 SH 标定值，或自己设）
    """
    c = make_client(a.port, a.baud, a.parity, timeout=a.timeout)
    if not c.connect():
        print("串口打不开")
        return 1
    print(f"读 0x54~0x56  地址 {a.addr}  "
          f"量程 ±{a.span} N·m  显示满量程 {a.full_display}")
    print("（换算：±显示值/10^Sd × %.4g/%.4g）\n" % (a.span, a.full_display))
    print("    时间   小数点  符号   原始值   显示值      力矩(N·m)")
    t0 = time.time()
    n = err = 0
    try:
        while a.count_reads <= 0 or n < a.count_reads:
            r = read_regs(c, 0x54, 3, a.addr)
            ts = time.time() - t0
            if is_err(r) or len(r.registers) < 3:
                err += 1
                print(f"  {ts:7.2f}s   —— 无响应（累计 {err}）")
            else:
                dot, sign, val = r.registers[0], r.registers[1], r.registers[2]
                disp = val / (10 ** dot) if dot <= 4 else val
                torque = _to_tau(-val if sign else val, dot, a)
                print(f"  {ts:7.2f}s   Sd={dot}   {'负' if sign else '正'}  "
                      f"{val:6d}  {disp:9.2f}  {torque:+10.4f}")
                n += 1
            time.sleep(a.interval)
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        c.close()
    return 0


def cmd_watch(a):
    """变化监视：只打印「数值变了」的时刻，最后给一段统计。

    为什么需要它 —— 判断仪表到底有没有真的在采信号，有两个互斥的可能：
      * **传感器没接到仪表**（断线/接错/激励没出）：读数会长期卡在同一个数上，
        一动不动，看起来像「很稳」，其实是死值；
      * **仪表滤波很重**（FIL 设得大）：读数也会很稳，但施加力矩时会缓慢爬。
    两者光看静止读数分不开，必须施加一个真实力矩，看它动不动。
    所以 watch 的目的就是配一次施力试验，把这两个可能区分开。
    """
    c = make_client(a.port, a.baud, a.parity, timeout=a.timeout)
    if not c.connect():
        print("串口打不开")
        return 1
    print(f"监视 {a.seconds:.0f} s（每 {a.interval:.2f}s 读一次）—— 请在这段时间里施加力矩\n")
    print("    时间   原始值    符号   力矩(N·m)   变化")
    t0 = time.time()
    prev = None
    vals, changes, errs = [], 0, 0
    try:
        while time.time() - t0 < a.seconds:
            r = read_regs(c, 0x54, 3, a.addr)
            ts = time.time() - t0
            if is_err(r) or len(r.registers) < 3:
                errs += 1
                time.sleep(a.interval)
                continue
            dot, sign, val = r.registers[0], r.registers[1], r.registers[2]
            disp = val / (10 ** dot) if dot <= 4 else val
            signed = -val if sign else val
            torque = _to_tau(signed, dot, a)
            vals.append(signed)
            if prev is None or signed != prev:
                mark = "" if prev is None else f"{signed - prev:+d}"
                if prev is not None:
                    changes += 1
                print(f"  {ts:6.2f}s  {signed:7d}   {'负' if sign else '正'}  "
                      f"{torque:+10.4f}   {mark}")
                prev = signed
            time.sleep(a.interval)
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        c.close()
    if not vals:
        print("\n一次都没读到（通信有问题）")
        return 2
    unit = a.span / a.full_display / (10 ** 0)
    print(f"\n共 {len(vals)} 次采样，通信失败 {errs} 次，变化 {changes} 次")
    t_span = (max(vals) - min(vals)) * a.span / a.full_display / (10 ** dot if dot <= 4 else 1)
    print(f"原始值 min={min(vals)}  max={max(vals)}  跨度={max(vals)-min(vals)}"
          f"  → 力矩跨度 {t_span:.4f} N·m")
    if changes == 0:
        print("\n全程一个数都没变 —— 两种可能：① 这段时间确实没施力；"
              "② 传感器信号没进到仪表（查激励、查四线接法）。")
    return 0


def _to_tau(signed: int, dot: int, a) -> float:
    """把「有符号原始值 + 小数点」换算成 N·m（纯函数，不碰串口）。"""
    k = getattr(a, "nm_per_count", None)
    if k is None:
        disp = signed / (10 ** dot) if dot <= 4 else signed
        tau = disp * a.span / a.full_display
    else:
        tau = signed * k
    return tau + (getattr(a, "zero_nm", 0.0) or 0.0)


def _read_torque(c, a):
    """读一次，返回 (原始有符号值, 力矩 N·m, 小数点) ；失败返回 None。

    两种换算口径：
    * 默认按仪表口径 `--span` / `--full-display`（假设 5.000 显示 = 5 N·m）；
    * 若给了 `--nm-per-count`，就直接用它当「每 count 多少 N·m」的标定系数，
      再叠加 `--zero-nm` 零点。这是**不动机表**的标定方式 —— 只乘一个系数，
      不覆盖原厂 SH 标定（SH 要求负载 ≥ 半量程，改坏了也回不去）。
    """
    r = read_regs(c, 0x54, 3, a.addr)
    if is_err(r) or len(r.registers) < 3:
        return None
    dot, sign, val = r.registers[0], r.registers[1], r.registers[2]
    signed = -val if sign else val
    return signed, _to_tau(signed, dot, a), dot


def cmd_log(a):
    """按固定频率把力矩写进 jsonl，供后续与 J6 电流做对账。

    每秒打印一行摘要，方便现场判断有没有采到数据。
    """
    import json
    import os
    c = make_client(a.port, a.baud, a.parity, timeout=a.timeout)
    if not c.connect():
        print("串口打不开")
        return 1
    out = os.path.abspath(a.out)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    print(f"记录到 {out}")
    print(f"时长 {a.seconds:.0f} s，目标 {1/a.interval:.0f} Hz，量程 ±{a.span} N·m\n")
    f = open(out, "w", encoding="utf-8")
    f.write(json.dumps(dict(type="header", port=a.port, baud=a.baud, addr=a.addr,
                            span=a.span, full_display=a.full_display,
                            target_hz=round(1 / a.interval, 2),
                            t0_iso=time.strftime("%Y-%m-%dT%H:%M:%S")),
                        ensure_ascii=False) + "\n")
    t0 = time.time()
    n = err = 0
    sec, cnt_sec, last_sec = [], 0, 0
    try:
        while a.seconds <= 0 or time.time() - t0 < a.seconds:
            tick = time.time()
            v = _read_torque(c, a)
            if v is None:
                err += 1
            else:
                raw, tau, dot = v
                f.write(json.dumps(dict(t=round(time.time() - t0, 4), raw=raw,
                                        tau=round(tau, 6), dot=dot)) + "\n")
                n += 1
                sec.append(tau)
                cnt_sec += 1
            if int(time.time() - t0) > last_sec:
                last_sec = int(time.time() - t0)
                if sec:
                    print(f"  {last_sec:4d}s  {cnt_sec:3d} 帧  "
                          f"τ {min(sec):+.4f} ~ {max(sec):+.4f} N·m")
                sec, cnt_sec = [], 0
            rest = a.interval - (time.time() - tick)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        print("\n已中断，文件已保留")
    finally:
        c.close()
        f.close()
    dt = time.time() - t0
    print(f"\n完成：{n} 帧 / {dt:.1f} s = {n/dt:.1f} Hz，失败 {err} 次")


def cmd_calib(a):
    """标定：拿已知力矩对当前读数做线性拟合。

    不写进仪表，只在本地算一个「raw → N·m」的系数。理由：仪表里的 SL/SH 标定
    需要按键操作、而且要拆装负载，改错了很难查；本地拟合可以随时重算、留痕。
    """
    import json
    c = make_client(a.port, a.baud, a.parity, timeout=a.timeout)
    if not c.connect():
        print("串口打不开")
        return 1
    print("标定：每次加一个已知力矩，回车采样 3 s 取均值。力矩 = 质量(kg) × 9.80665 × 力臂(m)")
    print("直接输 0 表示空载取零点；输 q 结束并拟合\n")
    pts = []
    try:
        while True:
            s = input("  已知力矩 N·m（0=空载，q=结束）：").strip()
            if s.lower() in ("q", "quit", ""):
                break
            try:
                tau_true = float(s)
            except ValueError:
                print("  请输入数字")
                continue
            buf = []
            t_end = time.time() + 3.0
            while time.time() < t_end:
                v = _read_torque(c, a)
                if v:
                    buf.append(v[1])
                time.sleep(a.interval)
            if not buf:
                print("  采样失败")
                continue
            mean = sum(buf) / len(buf)
            sd = (sum((x - mean) ** 2 for x in buf) / max(1, len(buf) - 1)) ** 0.5
            pts.append((tau_true, mean))
            print(f"  → 真值 {tau_true:+.4f}  读数 {mean:+.4f}  (n={len(buf)}, sd={sd:.5f})\n")
    finally:
        c.close()
    if len(pts) < 2:
        print("至少两个点才能拟合")
        return 1
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    k = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    b = (sy - k * sx) / n
    resid = [p[1] - (k * p[0] + b) for p in pts]
    rms = (sum(r * r for r in resid) / n) ** 0.5
    print("拟合结果   真值 = %.6f × 读数 %+.6f" % (k, b))
    print("           ⇒ 读数换算：τ = (raw × span/full_display %+.6f) / %.6f" % (b, k))
    print("残差 RMS = %.5f N·m（占满量程 %.3f%%）" % (rms, 100 * rms / a.span))
    if a.save:
        with open(a.save, "w", encoding="utf-8") as f:
            json.dump(dict(points=pts, slope=k, intercept=b, rms=rms,
                           span=a.span, full_display=a.full_display,
                           t_iso=time.strftime("%Y-%m-%dT%H:%M:%S")),
                      f, ensure_ascii=False, indent=2)
        print("已保存到", a.save)
    return 0


def main():
    ap = argparse.ArgumentParser(description="力矩传感器 Modbus RTU 接入工具")
    ap.add_argument("cmd", choices=("scan", "sniff", "dump", "read", "cypy", "watch", "log", "calib"))
    ap.add_argument("--port", required=True, help="COM3 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--parity", default="N", choices=PARITIES)
    ap.add_argument("--addr", type=int, default=1, help="从站地址（默认 1）")
    ap.add_argument("--max-addr", type=int, default=32, help="scan 时扫到几号地址")
    ap.add_argument("--count", type=int, default=32, help="dump 时读多少个寄存器")
    ap.add_argument("--reg", type=lambda x: int(x, 0), default=0, help="read 的寄存器地址")
    ap.add_argument("--scale", type=float, default=None, help="原始值 × scale = N·m")
    ap.add_argument("--raw-max", type=int, default=None, help="满量程对应的原始值（如 2000）")
    ap.add_argument("--span", type=float, default=5.0,
                    help="传感器量程上限（VYJN1-5Nm 就是 5）")
    ap.add_argument("--signed", action="store_true", default=True, help="按 int16 有符号解释")
    ap.add_argument("--interval", type=float, default=0.2, help="读取间隔秒")
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--count-reads", type=int, default=0, help="读多少次后停（0=一直读）")
    ap.add_argument("--seconds", type=float, default=30.0, help="watch/log 时长（秒），0=不限")
    ap.add_argument("--out", default="torque_log.jsonl", help="log 的输出文件")
    ap.add_argument("--save", default="", help="calib 的结果保存路径")
    ap.add_argument("--nm-per-count", type=float, default=None,
                    help="标定系数：1 count = 多少 N·m。给了就忽略 span/full-display（不动机表）")
    ap.add_argument("--zero-nm", type=float, default=0.0,
                    help="零点补偿 N·m，配合 --nm-per-count 用")
    ap.add_argument("--full-display", type=float, default=5.0,
                    help="cypy：显示满量程（对应 --span 的显示值，默认 5000）")
    a = ap.parse_args()
    # 依赖检查放在最前面，避免用户在几百行 traceback 里找原因
    try:
        import serial  # noqa: F401
        import pymodbus  # noqa: F401
    except ImportError as e:
        print(f"缺少依赖：{e.name}")
        print("安装：pip install pyserial pymodbus")
        return 1
    return {"scan": cmd_scan, "sniff": cmd_sniff, "dump": cmd_dump,
            "read": cmd_read, "cypy": cmd_cypy, "watch": cmd_watch,
            "log": cmd_log, "calib": cmd_calib}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
