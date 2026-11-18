#!/usr/bin/env python3
"""stage0_dump.py —— 把阶段 0 的三个探针 JSON 压成一页可读摘要

用途：探针产出的 json 又长又嵌套，直接贴聊天窗口很难读。跑这个，把输出贴给
复盘方即可（约 40 行，含全部关键判定所需字段）。

用法：
    cd ~/stage0/20261019_stage0
    source .venv-py3.12/bin/activate
    python scripts/stage0_dump.py                # 读 env/ 下的默认文件名
    python scripts/stage0_dump.py --dir env      # 指定目录

不修改任何文件，纯读取。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

CASE_DIR = Path(__file__).resolve().parent.parent
DUMP_VERSION = "1.1-20261019"


def load(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except Exception as exc:
        return {"__error__": repr(exc)}


def g(d, *keys, default=None):
    """安全取值：g(d,'a','b') 等价 d['a']['b']，任一层缺失返回 default。"""
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def val(d, *keys, default=None):
    """取 field() 包装里的 value。"""
    return g(d, *keys, "value", default=default)


def state(d, *keys, default="—"):
    return g(d, *keys, "state", default=default)


def fmt_status(v):
    """{'name','code'} → STANDBY(0x0)；兼容旧版存的整段字符串。"""
    if isinstance(v, dict):
        if "name" in v and "code" in v:
            return f"{v['name']}(0x{v['code']:X})"
        return v.get("text") or v
    if isinstance(v, str):
        return v
    return v


def section(title: str) -> None:
    print(f"\n=== {title} " + "=" * max(0, 58 - len(title)))


def main() -> int:
    ap = argparse.ArgumentParser(description="阶段 0 探针结果摘要")
    ap.add_argument("--dir", default=str(CASE_DIR / "env"), help="json 所在目录")
    args = ap.parse_args()
    d = Path(args.dir)

    env = load(d / "env_probe.json")
    bus = load(d / "bus_probe.json")
    sdk = load(d / "sdk_probe.json")
    lock = load(d / "robot_env_lock.json")

    print(f"stage0_dump {DUMP_VERSION}")
    print(f"目录: {d.resolve()}")

    section("环境 env_probe.json")
    if env is None:
        print("  (缺失，先跑 --phase env)")
    else:
        print(f"  probe_id    : {g(env,'probe')}")
        print(f"  host        : {g(env,'host')}")
        print(f"  generated   : {g(env,'generated_at')}")
        e = g(env, "env", "value") or {}
        for k in ("os", "kernel", "python", "ros_distro", "cpu_count", "arch"):
            if k in e:
                print(f"  {k:<12}: {e[k]}")
        oi = g(env, "official_impl") or {}
        print(f"  official_impl: state={oi.get('state')} value={oi.get('value')} err={oi.get('error')}")

    section("总线 bus_probe.json")
    if bus is None:
        print("  (缺失，先跑 --phase bus)")
    else:
        print(f"  probe_id    : {g(bus,'probe')}")
        b = g(bus, "bus") or {}
        link = (b.get("link") or {}).get("value") or {}
        print(f"  link        : state={link.get('state')} bitrate={link.get('bitrate_bps')} "
              f"sample_point={link.get('sample_point')} restart_ms={link.get('restart_ms')}")
        bc = (b.get("bus_counters") or {}).get("value") or {}
        if bc:
            print("  counters    : " + " ".join(f"{k}={v}" for k, v in bc.items()))
        fr = (b.get("frames") or {})
        if fr.get("error"):
            print(f"  frames      : 未采集 —— {fr['error']}")
        else:
            fv = fr.get("value") or {}
            print(f"  frames      : total={fv.get('total_frames')} "
                  f"error_frames={fv.get('error_frames')} distinct_ids={fv.get('distinct_ids')}")
            if fv.get("verdict"):
                print(f"  verdict     : {fv['verdict']}")
            samples = fv.get("other_ids_sample") or {}
            if samples:
                print("  other_payload: " + ", ".join(f"{k}={v}" for k, v in samples.items()))
            for grp, gv in (fv.get("by_group") or {}).items():
                print(f"  [{grp}] seen={len(gv.get('ids_seen') or [])} missing={gv.get('missing_ids')}")
                for idhex, st in (gv.get("per_id") or {}).items():
                    print(f"      {idhex:<8} n={st.get('n'):<4} median={st.get('median_dt_ms')} ms "
                          f"p95={st.get('p95_dt_ms')} max={st.get('max_dt_ms')}")
            if fv.get("other_ids"):
                print(f"  other_ids   : {fv['other_ids']}")

    section("SDK sdk_probe.json")
    if sdk is None:
        print("  (缺失，先跑 --phase sdk)")
    else:
        print(f"  probe_id    : {g(sdk,'probe')}")
        s = g(sdk, "sdk") or {}
        if "version" not in s:
            print(f"  SDK 未加载  : {s.get('error')}")
        else:
            print(f"  version     : {s.get('version', {}).get('value')}")
            print(f"  firmware    : {val(s,'firmware')} (via {s.get('firmware_via')})")
            stt = val(s, "status_0x2A1") or {}
            print(f"  ctrl_mode   : {fmt_status(stt.get('ctrl_mode'))}  "
                  f"arm_status={fmt_status(stt.get('arm_status'))}  "
                  f"motion={fmt_status(stt.get('motion_status'))}  "
                  f"teach={fmt_status(stt.get('teach_status'))}")
            en = val(s, "enable_status_passive") or {}
            print(f"  enable      : all={en.get('all_enabled')} flags={en.get('flags')}")
            print(f"  slave_mode  : {state(s,'slave_mode')} {val(s,'slave_mode')}")
            lim = val(s, "hardware_limits")
            if isinstance(lim, dict):
                for j, lv in lim.items():
                    print(f"  limits {j:<8}: {lv.get('min_deg')} ~ {lv.get('max_deg')} deg, "
                          f"max {lv.get('max_spd_rad_s')} rad/s")
            else:
                print(f"  limits      : {state(s,'hardware_limits')} (未查询)")
            lv = (s.get("bus_liveness") or {})
            lvv = lv.get("value") or {}
            print(f"  bus_liveness: state={lv.get('state')} has_feedback={lvv.get('has_feedback')} "
                  f"feedback_ids={lvv.get('feedback_ids')} other={lvv.get('other_ids')} "
                  f"err={lvv.get('error_frames')}")
            gf = val(s, "first_frame_guard") or {}
            print(f"  guard       : opened={gf.get('opened')} discarded={gf.get('discarded')} "
                  f"reasons={gf.get('reject_reasons')}")

    section("环境冻结 robot_env_lock.json")
    if lock is None:
        print("  (缺失，先跑 freeze_robot_env.sh)")
    else:
        print(f"  os          : {g(lock,'os_pretty') or g(lock,'os')}")
        print(f"  kernel      : {g(lock,'kernel')}")
        print(f"  venv python : {g(lock,'venv_python')}")
        print(f"  track       : {g(lock,'track')}  rclpy_in_venv={g(lock,'rclpy_importable_in_venv')}")
        pk = g(lock, "pip_packages") or {}
        print(f"  packages    : " + ", ".join(f"{k}={v}" for k, v in pk.items() if v))
        print(f"  lock_sha256 : {g(lock,'lock_sha256')}")

    print("\n把以上内容整段贴回即可；无需再发原始 json。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
