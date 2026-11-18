# 文件与命令落点对照

> 上一轮的命令我混在一段里给了，没标「在哪台机器敲」，是我的问题。这张表之后，每条命令只有唯一答案。
>
> **一句话**：只有两件事（上传、下载）在 Windows 上做，其余全在 VM 里做。

---

## A 两台机器，各有一份目录

| | **Windows**（改代码的地方） | **VM**（跑实验的地方，`zzz@zzz-VMware-Virtual-Platform`） |
| --- | --- | --- |
| 阶段 0 目录 | `C:\Users\asus\WorkBuddy\论文解析复现\LEFS\experiments\20261019_stage0\` | `/home/zzz/stage0/20261019_stage0/` |
| 谁维护 | 我 | 你（只按命令跑，不用改文件） |
| 同步方式 | — | `scp` 单向推送打包文件，见 §B |

VM 上的目录是 11:07 你自己建的：
```
mkdir -p ~/stage0 && tar -xzf ~/stage0_bundle.tar.gz -C ~/stage0
# → /home/zzz/stage0/20261019_stage0/
```

---

## B 每个文件放哪

| 文件 | Windows 侧 | VM 侧 | 说明 |
| --- | --- | --- | --- |
| `STAGE0_ACTIONS.md` | `…\20261019_stage0\` | `~/stage0/20261019_stage0/` | 你在看的这份清单 |
| `STAGE0_REPORT.md` | 同上 | 同上 | 阶段 0 报告 |
| `manifest.json` | 同上 | 同上 | 版本登记表，最后**由我**回填 |
| `source_commit.txt` | 同上 | 同上 | 冻结的 commit |
| `scripts/stage0_probe.py` | `…\scripts\` | `…/scripts/` | 探针，**不改** |
| `scripts/stage0_dump.py` | `…\scripts\` | `…/scripts/` | 把三个 json 压成一页摘要，贴给我用这个 |
| `env/freeze_robot_env.sh` | `…\env\` | `…/env/` | 建环境，**不改** |
| `env/requirements.in` | `…\env\` | `…/env/` | 依赖意图（`--minimal` 时不读它） |
| `env/dev_env_windows.json` | `…\env\` | **不用传** | 开发机快照，仅供参考 |

**只在 VM 上产生的文件**（这些就是我要的产出）：

| 文件 | 绝对路径 | 什么时候出现 |
| --- | --- | --- |
| `env/requirements-lock.txt` | `/home/zzz/stage0/20261019_stage0/env/` | 跑完 `freeze_robot_env.sh` |
| `env/robot_env_lock.json` | 同上 | 同上 |
| `env/env_probe.json` | 同上 | `--phase env` |
| `env/bus_probe.json` | 同上 | `--phase bus` |
| `env/sdk_probe.json` | 同上 | `--phase sdk` |
| venv | `/home/zzz/stage0/20261019_stage0/.venv-py3.12/` | 同上，不用管 |

**只在 VM 上存在的第三方代码**：

| 目录 | 路径 | 怎么来 |
| --- | --- | --- |
| 官方实现 | `/home/zzz/factr2_next/` | `git clone https://github.com/philiphan0109/factr2_next.git ~/factr2_next`（你上一步漏了） |
| 既有 ROS 工程 | `/home/zzz/agx_arm_ws/` | 本来就有，**别动**，也别和探针同时跑 |

---

## C 每条命令在哪敲

### 【Windows】只有 2 条

在 PowerShell 里（不是 VM 的终端）：

```powershell
# C-1 上传（每次我改了文件都要重传）
scp "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\tmp\stage0_bundle.tar.gz" zzz@192.168.242.128:~/

# C-2 下载产物（跑完之后，把 5 个 json/txt 拉回 Windows）
scp "zzz@192.168.242.128:~/stage0/20261019_stage0/env/*" "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\experiments\20261019_stage0\env\"
```

### 【VM】其余全部

每条前面标了「在哪个目录」。

```bash
# C-3  目录：~（家目录）
mkdir -p ~/stage0 && tar -xzf ~/stage0_bundle.tar.gz -C ~/stage0

# C-4  目录：~（家目录）—— 只做一次
git clone https://github.com/philiphan0109/factr2_next.git ~/factr2_next
git -C ~/factr2_next rev-parse HEAD        # 期望 0df02c5b…

# C-5  目录：任意（脚本按自身位置定位）—— 体检，不装东西
bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh --check

# C-6  目录：任意 —— 装最小依赖（约 30 MB）
bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh --minimal

# C-7  目录：任意 —— 让 can0 出现（先做，否则后面两项是空报告）
lsusb | grep -i 1d50
lsmod | grep gs_usb || sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0

# C-8  目录：~/stage0/20261019_stage0（必须先 cd 并激活 venv）
cd ~/stage0/20261019_stage0
source .venv-py3.12/bin/activate

# C-9  目录：~/stage0/20261019_stage0（接着 C-8 的同一个 shell）
python scripts/stage0_probe.py --phase selftest
python scripts/stage0_probe.py --phase env --factr2-repo ~/factr2_next
python scripts/stage0_probe.py --phase bus --seconds 10
python scripts/stage0_probe.py --phase sdk --allow-write --query-limits --factr2-repo ~/factr2_next
```

> C-8 的两行是**有状态的**：`cd` 和 `source` 只对当前这个终端窗口有效。
> 关掉窗口、或新开一个标签页，就要重新执行 C-8 的两行，否则 `python scripts/…` 会报 `No such file`。

---

## D 为什么总是踩「路径不存在」

| 你敲的 | 实际发生 | 正确写法 |
| --- | --- | --- |
| `bash env/freeze_robot_env.sh` | 在 `~` 下找 `./env/freeze_robot_env.sh`，没有 | `bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh` |
| `python scripts/stage0_probe.py` | 只有在 `~/stage0/20261019_stage0` 里才存在这个相对路径 | 先 `cd ~/stage0/20261019_stage0`，或写全 `python ~/stage0/20261019_stage0/scripts/stage0_probe.py` |
| `--phase selftest` | 当成一条命令去找，找不到 | 它是 python 的参数，必须跟在 `…stage0_probe.py` 后面 |

`cd` 到目录里再用相对路径，或者**一律用 `~/stage0/20261019_stage0/...` 绝对路径**，两种都行，别混。

**怎么确认「文件真的传进去了」**（每次我改完都先跑这一条，省得白折腾）：

```bash
cd ~/stage0/20261019_stage0 && source .venv-py3.12/bin/activate
python scripts/stage0_probe.py --version      # 期望：stage0_probe 1.2-20261019
ls scripts/                                   # 期望：stage0_dump.py  stage0_probe.py
```

| 现象 | 含义 | 处置 |
| --- | --- | --- |
| `No such file or directory` | 新包根本没上传/没解包 | 回 Windows 跑 C-1，再跑 C-3 |
| `unrecognized arguments: --version` | 传的是旧包（1.1 之前的脚本没有这个开关） | 同上，重传重解包 |
| 打印 `1.2-20261019` | 是最新的，可以继续 | — |

---

## E 一张流程图

```
Windows                                          VM (192.168.242.128)
────────────────────────────────────             ────────────────────────────────
C:\...\LEFS\tmp\stage0_bundle.tar.gz
        │
        │  C-1  scp  ─────────────────────────►  ~/stage0_bundle.tar.gz
        │                                              │ C-3 tar -xzf
        │                                              ▼
        │                                        ~/stage0/20261019_stage0/
        │                                          ├─ scripts/stage0_probe.py
        │                                          ├─ env/freeze_robot_env.sh
        │                                          └─ env/*.json   ← 产物在这里
        │                                              │
        │  C-2  scp  ◄───────────────────────────────┘
        ▼
C:\...\20261019_stage0\env\*.json  →  我回填 manifest.json
```

---

## F 现在该做什么（按顺序，只 4 步）

1. 【Windows】C-1 上传
2. 【VM】C-3 解包 → C-5 体检 → C-6 装依赖
3. 【VM】C-7 让 can0 出现 → C-4 clone 官方实现 → C-8 激活 → C-9 四条探针
4. 【Windows】C-2 把 `env/*` 拉回来，或直接把 json 内容贴给我
