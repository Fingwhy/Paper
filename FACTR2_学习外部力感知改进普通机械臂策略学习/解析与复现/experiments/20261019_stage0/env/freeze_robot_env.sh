#!/usr/bin/env bash
# freeze_robot_env.sh —— 在【机器人机 / Ubuntu 虚拟机】上建立并冻结 Python 环境
#
# 背景（FIRST_ARM 归档）：该机是 Ubuntu + ROS 2 Lyrical + Python 3.14.4。
# 官方 factr2_next 要求 Python 3.12 + numpy<2.0，而 numpy 1.x 没有 cp314 轮子
# （cp314 从 numpy 2.3.2 起才有）。所以默认建 3.12 的独立 venv 走"训练/离线轨"，
# 不要把系统 3.14 的 numpy 升/降到和官方打架。
#
# 用法（脚本用 BASH_SOURCE 定位自身，**在任何 cwd 下都能跑**）：
#   bash <本阶段目录>/env/freeze_robot_env.sh --check           # 只体检，不装任何东西
#   bash <本阶段目录>/env/freeze_robot_env.sh --minimal         # 只装 python-can + piper-sdk（约 40 MB，阶段 0 够用）
#   bash <本阶段目录>/env/freeze_robot_env.sh                   # 3.12 venv + 核心依赖（推荐）
#   bash <本阶段目录>/env/freeze_robot_env.sh --with-hardware   # 另装 piper_sdk / python-can
#   bash <本阶段目录>/env/freeze_robot_env.sh --torch-cpu       # torch 走 CPU 源（250 MB，而非 2.3 GB 的 CUDA 版）
#   bash <本阶段目录>/env/freeze_robot_env.sh --python=3.14     # 走 ROS 轨（numpy>=2.3.2，显式偏离官方）
#   ... --no-install                                            # 缺 uv 时不自动装，直接报错退出
#
# 安装器：优先 uv（能自己下载 CPython 3.12，不动系统 python）；
#         没有 uv 且允许安装时自动装；仍不行则退到本机 python3.12 + venv/pip。
#
# 磁盘闸门：可用 < 6 GB 时拒绝完整安装，并给出 --minimal / --torch-cpu / 清盘三条出路。
#
# 产物：env/requirements-lock.txt + env/robot_env_lock.json

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CASE_DIR="$(cd "$HERE/.." && pwd)"
REQ_IN="$HERE/requirements.in"
LOCK="$HERE/requirements-lock.txt"
JSON="$HERE/robot_env_lock.json"

WITH_HW=0
MINIMAL=0
TORCH_CPU=0
PYVER_REQ="3.12"
CHECK_ONLY=0
ALLOW_INSTALL=1
for a in "$@"; do
  case "$a" in
    --with-hardware) WITH_HW=1 ;;
    --minimal)       MINIMAL=1 ;;
    --torch-cpu)     TORCH_CPU=1 ;;
    --python=*)      PYVER_REQ="${a#*=}" ;;
    --python)        shift; PYVER_REQ="${1:-3.12}" ;;
    --check)         CHECK_ONLY=1 ;;
    --no-install)    ALLOW_INSTALL=0 ;;
    *) echo "[WARN] 未知参数：$a" ;;
  esac
done
VENV="$CASE_DIR/.venv-py${PYVER_REQ}"

say() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '[FAIL] %b\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 前置检查
say "前置检查"
echo "本阶段目录: $CASE_DIR"
echo "目标 venv   : $VENV"

command -v python3 >/dev/null || die "缺 python3"
SYS_PY="$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
echo "系统 python: $SYS_PY ／ 目标 venv python: $PYVER_REQ"
[[ -f "$REQ_IN" ]] || die "找不到 $REQ_IN（目录不完整？确认整个 20261019_stage0 都传进来了）"

# --check：只体检，**绝不**联网安装、绝不建 venv
if [[ $CHECK_ONLY -eq 1 ]]; then
  say "体检模式（--check）：不建 venv、不装包"
  HAS_UV=0; command -v uv >/dev/null 2>&1 && HAS_UV=1
  HAS_PY=0; command -v "python$PYVER_REQ" >/dev/null 2>&1 && HAS_PY=1
  printf '%-16s %s\n' "python3" "$(python3 -V 2>&1)"
  printf '%-16s %s\n' "python$PYVER_REQ" "$( [[ $HAS_PY -eq 1 ]] && python$PYVER_REQ -V 2>&1 || echo '未安装')"
  printf '%-16s %s\n' "uv" "$( [[ $HAS_UV -eq 1 ]] && uv --version 2>&1 || echo '未安装')"
  printf '%-16s %s\n' "curl" "$(command -v curl >/dev/null && echo yes || echo no)"
  printf '%-16s %s\n' "git" "$(git --version 2>/dev/null || echo '未安装')"
  printf '%-16s %s\n' "can0" "$(ip link show can0 >/dev/null 2>&1 && echo '存在' || echo '不存在（USB-CAN 未直通？）')"
  printf '%-16s %s\n' "ROS_DISTRO" "${ROS_DISTRO:-未设置}"
  printf '%-16s %s\n' "磁盘可用MB" "$(( $(df -Pk "$CASE_DIR" 2>/dev/null | awk 'NR==2{print $4}') / 1024 ))"
  printf '%-16s %s\n' "pip(系统)" "$(python3 -m pip --version 2>/dev/null || echo '无（Ubuntu 26.04 默认不带，需 python3-pip 或 venv）')"
  echo
  CHK_MB=$(( $(df -Pk "$CASE_DIR" 2>/dev/null | awk 'NR==2{print $4}') / 1024 ))
  if (( CHK_MB < 6144 )); then
    echo "结论：磁盘仅 ${CHK_MB} MB，装不下完整训练栈 → bash $0 --minimal（阶段 0 够用，约 40 MB）"
  elif [[ $HAS_UV -eq 1 ]]; then
    echo "结论：uv 可用、磁盘 ${CHK_MB} MB 够用 → bash $0 --with-hardware --torch-cpu"
  elif [[ $HAS_PY -eq 1 ]]; then
    echo "结论：无 uv，但有 python$PYVER_REQ → 脚本会退到 venv/pip 建环境（需系统里有 pip/venv 包）"
  else
    echo "结论：无 uv 且无 python$PYVER_REQ → 正式跑时会尝试联网装 uv；"
    echo "      无网就执行：sudo apt install python${PYVER_REQ} python${PYVER_REQ}-venv"
  fi
  exit 0
fi

# 安装器选择：uv 能自己下载 CPython，最省事；没有就退到本机 python$PYVER_REQ
INSTALLER=""
if command -v uv >/dev/null 2>&1; then
  INSTALLER=uv
  echo "安装器      : uv $(uv --version 2>/dev/null | awk '{print $2}')"
else
  echo "安装器      : 未找到 uv"
  if [[ $ALLOW_INSTALL -eq 1 ]]; then
    say "尝试安装 uv（官方脚本，装到 ~/.local/bin，不动系统 python）"
    if curl -LsSf https://astral.sh/uv/install.sh | sh >/dev/null 2>&1; then
      export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
      command -v uv >/dev/null 2>&1 && INSTALLER=uv && echo "  → uv $(uv --version | awk '{print $2}') 已就绪"
    else
      echo "  → uv 安装失败（多半是没网或 curl 被拦）"
    fi
  fi
  if [[ -z "$INSTALLER" ]]; then
    if command -v "python$PYVER_REQ" >/dev/null 2>&1; then
      INSTALLER=venv
      echo "  → 退到本机 python$PYVER_REQ + venv/pip（$(python$PYVER_REQ -V 2>&1)）"
    else
      die "既没有 uv 也没有 python$PYVER_REQ。二选一：\n"\
"        1) 装 uv：curl -LsSf https://astral.sh/uv/install.sh | sh\n"\
"        2) 装解释器：sudo apt install python${PYVER_REQ}\n"\
"      或者先用 --python=$SYS_PY 走系统解释器（numpy 口径会偏离官方）"
    fi
  fi
fi

# 各安装器的等价调用
_venv_create() {
  if [[ "$INSTALLER" == uv ]]; then uv venv --python "$PYVER_REQ" --prompt ros "$VENV"
  else "python$PYVER_REQ" -m venv "$VENV"; fi
}
_pip() {
  if [[ "$INSTALLER" == uv ]]; then uv pip install "$@"
  else python -m pip install --disable-pip-version-check "$@"; fi
}
_freeze() {
  if [[ "$INSTALLER" == uv ]]; then uv pip freeze
  else python -m pip freeze; fi
}

if [[ $CHECK_ONLY -eq 1 ]]; then exit 0; fi

[[ -n "${ROS_DISTRO:-}" ]] || printf '[WARN] ROS_DISTRO 未设置（本脚本不依赖 ROS，可忽略）\n'
if [[ "$SYS_PY" == "3.14" && "$PYVER_REQ" == "3.12" ]]; then
  cat <<'EOF'
[NOTE] 系统是 3.14（ROS 2 Lyrical 用的解释器），但你选了 3.12 venv：
       这是有意的。numpy<2.0 没有 cp314 轮子，官方代码按 3.12 写的；
       uv 会自己下载一个 3.12 解释器，不动系统 python。
       代价：这个 venv 里 **import rclpy 会失败**（rclpy 是给 3.14 编译的），
       所以它只用于训练/离线评估，不用于 ROS 节点。下面会实测给你看。
EOF
fi

# ---------------------------------------------------------------- 磁盘闸门
# 这只 VM 只有 1 G 上下可用；torch 的 PyPI 默认包带 CUDA（2.3 GB），装全套一定撑爆。
FREE_MB=$(( $(df -Pk "$CASE_DIR" | awk 'NR==2{print $4}') / 1024 ))
echo "磁盘可用    : ${FREE_MB} MB"
if [[ $MINIMAL -eq 1 ]]; then NEED_MB=256; else NEED_MB=6144; fi
if (( FREE_MB < NEED_MB )); then
  if [[ $MINIMAL -eq 1 ]]; then
    say "磁盘只剩 ${FREE_MB} MB，最小安装实测约 40 MB；继续"
  else
    cat >&2 <<EOF

[FAIL] 磁盘只剩 ${FREE_MB} MB，完整安装（numpy/scipy/h5py/torch）约需 ${NEED_MB} MB。
       这只 VM 装全套不现实。三选一：
         1) 只装阶段 0 需要的最小集（约 40 MB，够跑 bus/sdk 探针）：
              bash $0 --minimal
         2) 训练用 CPU 版 torch（约 250 MB 而非 2.3 GB 的 CUDA 版）：
              bash $0 --with-hardware --torch-cpu
         3) 先清盘：删旧的 .venv* / docker system prune -af / 扩虚拟磁盘
              df -h ~ ; du -sh ~/* 2>/dev/null | sort -h | tail
EOF
    exit 1
  fi
fi

# ---------------------------------------------------------------- 建 venv
if [[ -d "$VENV" ]]; then
  say "复用已存在的 venv：$VENV"
else
  say "创建 venv (python $PYVER_REQ)"
  _venv_create
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"

if [[ $MINIMAL -eq 1 ]]; then
  # 阶段 0 只需要「能读 CAN、能解析协议」。piper-sdk 的唯一依赖是 python-can
  # （wheel METADATA 只有这一条），源码里没有 import numpy，所以不装训练栈。
  say "最小安装（阶段 0 够用：python-can + piper-sdk，约 40 MB）"
  _pip install "setuptools==79.0.1" python-can "piper-sdk==0.6.1"
else
  say "安装基础工具"
  _pip install pip wheel "setuptools==79.0.1"

  say "安装声明依赖（$( [[ $WITH_HW -eq 1 ]] && echo '含硬件 demo' || echo '仅核心' )）"
  REQ_USE="$REQ_IN"
  if [[ $WITH_HW -eq 0 ]]; then
    grep -vE '^(python-can|piper-sdk|dynamixel-sdk|pyserial)' "$REQ_IN" > /tmp/req_core.txt
    REQ_USE=/tmp/req_core.txt
  fi
  if [[ $TORCH_CPU -eq 1 ]]; then
    grep -vE '^torch\b' "$REQ_USE" > /tmp/req_notorch.txt
    _pip install -r /tmp/req_notorch.txt
    say "单独安装 CPU 版 torch"
    _pip install --index-url https://download.pytorch.org/whl/cpu torch
  else
    _pip install -r "$REQ_USE"
  fi
fi

# ---------------------------------------------------------------- 事实核验
say "核验关键版本"
python - <<PY
import sys, importlib.metadata as md
print("python", sys.version.split()[0])
for name in ("piper-sdk", "python-can", "numpy", "torch", "h5py", "setuptools"):
    try:
        print(f"{name:12s}", md.version(name))
    except Exception:
        print(f"{name:12s}", "MISSING")
import numpy
v = tuple(int(x) for x in numpy.__version__.split(".")[:2])
if "$PYVER_REQ" == "3.12":
    assert v < (2, 0), f"3.12 轨要求 numpy<2.0，实际 {numpy.__version__}"
    print("OK  numpy<2.0（与官方口径一致）")
else:
    assert v >= (2, 3), f"{'$PYVER_REQ'} 轨要求 numpy>=2.3（cp314 轮子下限），实际 {numpy.__version__}"
    print(f"WARN numpy {numpy.__version__}（3.14 轨，显式偏离官方的 <2.0）")
PY

say "实测 rclpy 可用性（只报告，不阻断）"
python - <<'PY' || true
try:
    import rclpy  # noqa
    print("rclpy: OK —— 该 venv 可直接跑 factr2_next 的 ROS 节点")
except Exception as exc:
    print(f"rclpy: NO ({type(exc).__name__}) —— 该 venv 只做训练/离线；ROS 节点请用系统 python")
PY

# ---------------------------------------------------------------- 冻结
say "写 lock"
_freeze > "$LOCK"
LOCK_SHA="$(sha256sum "$LOCK" | cut -d' ' -f1)"

say "采集环境指纹"
python - "$JSON" "$LOCK" "$LOCK_SHA" "$WITH_HW" "$PYVER_REQ" <<'PY'
import json, platform, subprocess, sys, os
from importlib.metadata import version

json_path, lock_path, lock_sha, with_hw, py_req = sys.argv[1:6]
with_hw = with_hw == "1"

def sh(argv):
    try:
        return subprocess.run(argv, capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception:
        return None

def osrel():
    out = {}
    try:
        for line in open("/etc/os-release", encoding="utf-8", errors="replace"):
            if "=" in line:
                k, v = line.split("=", 1)
                out[k] = v.strip().strip('"')
    except OSError:
        pass
    return out

def pkg_version(name):
    try:
        return version(name)
    except Exception:
        return None

try:
    import rclpy  # noqa
    rclpy_ok = True
except Exception:
    rclpy_ok = False

osr = osrel()
data = {
    "frozen_at": sh(["date", "-Iseconds"]),
    "host": platform.node(),
    "kernel": platform.release(),
    "os": f"{osr.get('NAME','?')} {osr.get('VERSION_ID','?')}",
    "os_pretty": osr.get("PRETTY_NAME"),
    "venv_python": platform.python_version(),
    "venv_python_requested": py_req,
    # 必须用绝对路径：本脚本常在已激活的 venv 里跑，裸 python3 会解析到 venv，
    # 把「系统 3.14 / venv 3.12」这条双轨的界线抹掉（2026-10-19 实测踩过）。
    "system_python": sh(["/usr/bin/python3", "-c",
                         "import sys;print('.'.join(map(str,sys.version_info[:3])))"]),
    "system_python_path": "/usr/bin/python3",
    "python3_on_path": sh(["bash", "-lc", "command -v python3 || echo none"]),
    "python_executable": sys.executable,
    "ros_distro": os.environ.get("ROS_DISTRO"),
    "rclpy_importable_in_venv": rclpy_ok,
    "apt_ros_packages": sh(["bash", "-lc",
        "dpkg -l | awk '/ros-.*(ros-base|desktop|pinocchio)/ {print $2\" \"$3}'"]),
    "pip_packages": {n: pkg_version(n) for n in
                     ("piper-sdk", "python-can", "numpy", "torch", "h5py", "pyyaml",
                      "scipy", "dynamixel-sdk", "cffi", "setuptools")},
    "with_hardware_demo": with_hw,
    "track": "A_train_offline_py312" if py_req == "3.12" else f"B_ros_py{py_req}",
    "lock_file": os.path.basename(lock_path),
    "lock_sha256": lock_sha,
    "lock_lines": sum(1 for _ in open(lock_path, encoding="utf-8")),
    "note": "torch 与 numpy 的实际版本以本文件与 lock 为准；两轨不得混装",
}
with open(json_path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
print("lock_sha256", lock_sha)
print("numpy", data["pip_packages"].get("numpy"), "torch", data["pip_packages"].get("torch"))
print("rclpy_in_venv", rclpy_ok)
PY

say "完成"
cat <<EOF

产物：
  $LOCK
  $JSON

下一步（复制粘贴即可，已用绝对路径，不依赖当前 cwd）：
  source "$VENV/bin/activate"
  cd "$CASE_DIR"
  python scripts/stage0_probe.py --phase selftest
  python scripts/stage0_probe.py --phase env
  python scripts/stage0_probe.py --phase bus --seconds 10
  python scripts/stage0_probe.py --phase sdk --allow-write --query-limits

  # 不加 --out 时，结果默认落到 $CASE_DIR/env/<phase>_probe.json

把 $JSON 的 lock_sha256 抄进 manifest.json 的 software_stack.robot_side.lock。
EOF
