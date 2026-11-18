#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拧盖全过程记录：200 Hz 全量实时数据 + 实时力矩估计。

与 `s6_k6_calib.py twist` 的区别
--------------------------------
1. **记录 200 Hz**（每 5 ms 一帧），指令仍 50 Hz 下发（20 ms）——不抢总线。
   `twist` 是 50 Hz 记录，看不到 20 ms 级的 stick-slip。
2. **记全量**：q1~q6、dq1~dq6、q_cmd、i1~i6、夹爪开度/力矩。
3. **加实时估计列**：τ_j = K_j·I_j（6 轴）、τ_ext6 = K6·|i6| − τ_f、I6 滑动平均、实时咬合比。
4. **无条件落盘**：无论瓶盖拧不拧得动、无论守卫是否中断，`finally` 里都写文件。

用法
----
  python3 twist_log.py --target 0 --rate 15 --tag log1 --grip-effort 5000 --grip-squeeze 1000
  python3 twist_log.py --target 0 --rate 15 --tag log1 --dry-run     # 只记录不动

安全守卫（与 twist 一致，触发即停但数据保留）
----------------------------------------------
- 任一轴 |I| > 3.0 A  → 停
- J6 |cmd − meas| > 5° → 停（转不动）
"""
import argparse
import csv
import time
from pathlib import Path

from piper_sdk import C_PiperInterface as Piper

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"

CTRL_CAN = 0x01
MAX_CUR_A = 3.0
LEAD_MAX_DEG = 5.0
DT_REC = 0.005          # 200 Hz 记录
DT_SEND = 0.020         # 50 Hz 下发

# K 表：J2=J3=4.7185、J4=J5=0.7446（stage2 k_table.json）
#       J6=0.3889（stage6 k6_result.json）
#       J1 未标定，暂用 SDK 标称 1.18125（**不可用于定量结论**）
K_JOINT = [1.18125, 4.7185, 4.7185, 0.7446, 0.7446, 0.3889]
K6 = 0.3889
TAU_F6 = 0.86           # J6 摩擦（由 K6 标定截距反推）

JOINT_IDS = (0x155, 0x156, 0x157)   # 每帧装两个关节，int32，0.001 度，大端有符号


class Hold:
    """固定目标关节保持。自己建 socket 是为了逐帧判定发送成败
    （SDK 的 JointCtrl 返回值恒为 None，发失败只有 logger 里一行）。"""

    def __init__(self, iface):
        import can
        self._can = can
        self.bus = can.Bus(interface="socketcan", channel=iface)
        self.n_ok = 0
        self.n_fail = 0

    def send(self, q_deg):
        vals = [int(round(v * 1000.0)) for v in q_deg]
        ok = True
        for i, cid in enumerate(JOINT_IDS):
            data = (vals[2 * i].to_bytes(4, "big", signed=True)
                    + vals[2 * i + 1].to_bytes(4, "big", signed=True))
            try:
                self.bus.send(self._can.Message(arbitration_id=cid, data=data,
                                                is_extended_id=False), timeout=0.005)
                self.n_ok += 1
            except Exception:                            # noqa: BLE001
                ok = False
                self.n_fail += 1
        return ok

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:                                # noqa: BLE001
            pass

COLS = (["t_s"]
        + [f"q{i}" for i in range(1, 7)]
        + [f"dq{i}" for i in range(1, 7)]
        + ["qcmd6"]
        + [f"i{i}" for i in range(1, 7)]
        + ["grip_ang_mm", "grip_eff_nm"]
        + [f"tau{i}_k" for i in range(1, 7)]
        + ["tau6_ext", "i6_ma", "bite_ratio", "slip"])


# ---------------------------------------------------------------- 读取
def connect(iface):
    p = Piper(iface)
    p.ConnectPort()
    time.sleep(1.0)
    return p


def read_q(p):
    j = p.GetArmJointMsgs().joint_state
    return [getattr(j, f"joint_{i}") / 1000.0 for i in range(1, 7)]


def read_dq(p):
    m = p.GetArmHighSpdInfoMsgs()
    out = []
    for i in range(1, 7):
        mo = getattr(m, f"motor_{i}")
        spd = getattr(mo, "motor_speed", 0)     # 0.001 rad/s
        out.append(spd / 1000.0 * 57.29577951308232)   # deg/s
    return out


def read_cur(p):
    m = p.GetArmHighSpdInfoMsgs()
    return [getattr(m, f"motor_{i}").current / 1000.0 for i in range(1, 7)]


def read_grip(p):
    g = p.GetArmGripperMsgs().gripper_state
    return int(g.grippers_angle) / 1000.0, int(g.grippers_effort) / 1000.0


def read_mode(p):
    try:
        return p.GetArmStatus().arm_status.ctrl_mode
    except Exception:                                        # noqa: BLE001
        return None


def read_enable(p):
    try:
        e = p.GetArmEnableStatus()
        return list(e) if isinstance(e, (list, tuple)) else [bool(e)] * 6
    except Exception:                                        # noqa: BLE001
        return [True] * 6


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="拧盖全过程记录（200 Hz 全量 + 实时力矩估计）")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--target", type=float, required=True, help="J6 目标角（deg）")
    ap.add_argument("--rate", type=float, default=15.0, help="J6 转速 deg/s")
    ap.add_argument("--tag", default=None, help="输出文件名 tag")
    ap.add_argument("--name", default=None,
                    help="直接指定文件名（不含 .csv），如 '2.全量新盖打开'。"
                         "推荐：文件名自带序号和结果，以后不用查表")
    ap.add_argument("--no-grip", action="store_true",
                    help="完全不发夹爪指令（用于空载基线：夹爪保持张开、不受力）")
    ap.add_argument("--grip-effort", type=int, default=5000, help="夹爪力矩上限（0.001 N·m）")
    ap.add_argument("--grip-squeeze", type=int, default=1000,
                    help="相对当前开度再压进多少（0.001 mm）")
    ap.add_argument("--reissue", type=int, default=1, help="每几拍重发一次夹爪指令")
    ap.add_argument("--speed-pct", type=int, default=20, help="MotionCtrl_2 速度百分比")
    ap.add_argument("--dry-run", action="store_true", help="只记录，不动 J6")
    ap.add_argument("--settle", type=float, default=1.0, help="夹紧后静置秒数")
    ap.add_argument("--max-cur", type=float, default=MAX_CUR_A, help="任一轴电流上限 A")
    ap.add_argument("--max-cur6", type=float, default=None,
                    help="J6 单独电流上限 A（默认同 --max-cur）。接 5 N·m 传感器时务必给 0.37")
    a = ap.parse_args()

    p = connect(a.iface)
    en = read_enable(p)
    if not all(en):
        raise SystemExit(f"[ABORT] 有轴未使能: {en}")
    mode = read_mode(p)
    if mode is not None and mode != CTRL_CAN:
        raise SystemExit(f"[ABORT] ctrl_mode={mode} 不是 CAN_CTRL；先跑 weigh_init.py --no-app")

    q0 = read_q(p)
    ang0, eff0 = read_grip(p)
    if ang0 < 1.0:
        raise SystemExit(f"[ABORT] 夹爪开度 {ang0:.1f} mm，先夹住瓶盖再记录")
    tgt = float(a.target)
    if not a.dry_run and abs(tgt - q0[5]) < 0.5:
        raise SystemExit("[ABORT] 目标与当前 J6 相同（要只记录请加 --dry-run）")

    print(f"[log] J6 {q0[5]:.2f}° → {tgt:.2f}°  @ {a.rate:g}°/s  "
          f"{'（dry-run 不动）' if a.dry_run else ''}")
    print(f"[log] 夹爪 {ang0:.2f} mm / {eff0:+.3f} N·m，压进 {a.grip_squeeze / 1000:.2f} mm，"
          f"effort 上限 {a.grip_effort}")
    print(f"[log] 记录 {1 / DT_REC:.0f} Hz，下发 {1 / DT_SEND:.0f} Hz")

    recs = []
    aborted = None
    t_start = time.perf_counter()
    seg_end = -1
    hold = Hold(a.iface)
    try:
        p.MotionCtrl_2(0x01, 0x01, a.speed_pct, 0x00)
        # 先补发夹爪（记录前就让夹持力到位）
        if not a.no_grip:
            p.GripperCtrl(max(0, int(ang0 * 1000) - a.grip_squeeze), a.grip_effort, 0x01, 0)
        hold.send(q0)
        time.sleep(a.settle)

        span = 0.0 if a.dry_run else (tgt - q0[5])
        dur = abs(span) / max(a.rate, 0.1)
        n_send = max(2, int(round(dur / DT_SEND)))
        qcmd = list(q0)

        next_send = time.perf_counter()
        k_send = 0
        i6_win = []                      # 0.5 s 窗（100 帧）
        base_win = []                    # 前 2 s 基准
        while True:
            now = time.perf_counter()
            if k_send <= n_send and now >= next_send:
                f = min(1.0, k_send / n_send)
                qcmd = list(q0)
                qcmd[5] = q0[5] + span * f
                hold.send(qcmd)
                if k_send % a.reissue == 0:
                    if not a.no_grip:
                        p.GripperCtrl(max(0, int(ang0 * 1000) - a.grip_squeeze),
                                      a.grip_effort, 0x01, 0)
                k_send += 1
                next_send += DT_SEND
                if k_send > n_send:
                    seg_end = time.perf_counter()

            cur = read_cur(p)
            qm = read_q(p)
            dq = read_dq(p)
            ga, ge = read_grip(p)
            t = time.perf_counter() - t_start

            i6a = abs(cur[5])
            i6_win.append(i6a)
            if len(i6_win) > 100:
                i6_win.pop(0)
            i6_ma = sum(i6_win) / len(i6_win)
            if t <= 2.0:
                base_win.append(i6_ma)
            base = (sum(base_win) / len(base_win)) if base_win else i6_ma
            ratio = (i6_ma / base) if base > 1e-6 else 1.0
            slip = 1 if ratio >= 0.90 else 0

            recs.append([t] + qm + dq + [qcmd[5]] + cur + [ga, ge]
                        + [K_JOINT[j] * cur[j] for j in range(6)]
                        + [K6 * i6a - TAU_F6, i6_ma, ratio, slip])

            worst = max(range(6), key=lambda j: abs(cur[j]))
            _lim = a.max_cur
            if abs(cur[worst]) > _lim:
                raise RuntimeError(f"J{worst + 1} 电流 {cur[worst]:+.2f} A 超 {_lim} A，停发")
            # J6 单独再查一次：其它轴离 3 A 还远时，J6 可能早已超过传感器量程
            _lim6 = a.max_cur6 if a.max_cur6 is not None else _lim
            if abs(cur[5]) > _lim6:
                raise RuntimeError(
                    f"J6 电流 {cur[5]:+.2f} A 超 {_lim6} A（≈{abs(cur[5]) * K6:.1f} N·m），停发")
            if not a.dry_run:
                lead = abs(qcmd[5] - qm[5])
                if lead > LEAD_MAX_DEG:
                    raise RuntimeError(f"J6 lead {lead:.2f}° > {LEAD_MAX_DEG}°，转不动（阻矩超出力）")

            if int(t * 4) % 8 == 0 and abs(t - round(t)) < DT_REC:
                print(f"  t={t:5.2f}s cmd={qcmd[5]:+7.2f} meas={qm[5]:+7.2f} "
                      f"I6={cur[5]:+8.4f} τext={K6 * i6a - TAU_F6:+6.2f} "
                      f"grip={ga:5.2f}mm {ratio:.2f}", flush=True)

            if seg_end > 0 and k_send > n_send and time.perf_counter() > seg_end + 1.0:
                break
            if t > dur + 5.0:
                break
            time.sleep(DT_REC)

    except Exception as exc:                                 # noqa: BLE001
        aborted = str(exc)
        print(f"[STOP] {exc}（数据仍会落盘）", flush=True)
    finally:
        hold.close()
        DATA.mkdir(exist_ok=True)
        if a.name:
            out = DATA / f"{a.name}.csv"
        else:
            tag = a.tag or time.strftime("%Y%m%d_%H%M%S")
            out = DATA / f"twistlog_{tag}.csv"
        with out.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(COLS)
            for r in recs:
                w.writerow([f"{r[0]:.4f}"] + [f"{v:.5f}" for v in r[1:]])
        # 注意：recs[0][0] 从 1.0s 起（前面 1s 是 settle），
        # 真实时长必须用 t_last - t_first，否则帧率会被系统性低估（曾把 174Hz 报成 161Hz）
        t0, t1 = recs[0][0], recs[-1][0]
        dur = max(1e-9, t1 - t0)
        dt = dur / max(1, len(recs) - 1)
        print(f"\n[log] {len(recs)} 帧 / {dur:.2f}s / 实测 {1 / dt:.1f} Hz → {out}")
        print(f"[log] 列数 {len(COLS)}，文件 {out.stat().st_size / 1024:.0f} KB")
        if aborted:
            print(f"[log] ⚠ 中途停止：{aborted}")


if __name__ == "__main__":
    main()
