#!/bin/bash
# weigh.sh —— 一键：初始化 CAN + 检查机械臂 + 进称重菜单
#
# 用法：
#   weigh              # 完整流程（推荐）
#   weigh --no-app     # 只初始化，不进菜单
#   weigh --force      # 强制重走三步激活
#   weight             # 同一个东西（少拼一个 t 也能用）
#
# 安装（一次）：
#   cp weigh.sh ~/.local/bin/weigh && chmod +x ~/.local/bin/weigh
#   cp weigh.sh ~/.local/bin/weight && chmod +x ~/.local/bin/weight
#
# 说明：三步激活 CAN 需要 sudo，必须跑在有终端的窗口里（不能重定向/管道），
#       否则 sudo 读不到密码。这一点由 weigh_init.py 判断并提示。

set -u

SELF="$(readlink -f "$0")"
DIR="$(dirname "$SELF")"

# 这个脚本可能被放在两个地方：项目目录里，或 ~/.local/bin 里被当成命令。
# **不能** `cd $(dirname $0)` 再相对调用 —— 装到 ~/.local/bin 后会去那里找
# weigh_init.py，直接报 "can't open file '/home/zzz/.local/bin/weigh_init.py'"。
# （2026-10-21 现场踩到，改成先探测再执行）
if [ -f "$DIR/weigh_init.py" ]; then
  PROJ="$DIR"
elif [ -f "$HOME/stage0/20261020_stage4/weigh_init.py" ]; then
  PROJ="$HOME/stage0/20261020_stage4"
else
  echo "✗ 找不到 weigh_init.py"
  echo "  项目目录应该在 ~/stage0/20261020_stage4/"
  echo "  如果在别处，改本脚本里的 PROJ 路径"
  exit 1
fi

exec python3 "$PROJ/weigh_init.py" "$@"
