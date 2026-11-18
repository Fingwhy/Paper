# 阶段 0 现场动作清单（按你现有机器改写）

> 读自 `C:/Users/asus/ARM/FIRST_ARM`（总纲 / README / 项目总卷，证据截止 2026-10-16）
> 结论先说：门**没过**，但**没有一项需要改代码、也不用买东西**。6 个动作、约 20 分钟，全是「读四个数 + 拍两张照片」。
>
> **先看 [`WHERE_TO_PUT.md`](./WHERE_TO_PUT.md)**：哪台机器、哪个目录、哪条命令，一张表说清。本文下面每段代码都标了【Windows】或【VM】。

---

## 0.5 当前状态（2026-10-19 12:12 上电首采后）

> **已通。** `0x251~0x256` 6/6、`0x2A1~0x2A8` 8/8，10 s 收 31396 帧、0 错帧。固件 `S-V1.8-8`、六轴全使能、
> 首帧守卫 `discarded=10`（真帧，不是零值）。
> **要判断"阶段 0 还差什么"，读 [`POWERON_1212_RESULT.md`](POWERON_1212_RESULT.md)**，别读本节。
> 本节以下内容保留为 R17 的原始失败证据（「总线只剩 0x24」那次），不再代表现状。

**还剩三件事**：① 限位重跑（`probe 1.4`，无副作用，1 分钟）；② 从站/示教模式确认（**有副作用**，需人守急停）；
③ 末端总重称重（不用电）。其中 ② 是 L1 之前的硬阻塞——`ctrl_mode` 现在是 `TEACHING_MODE(0x2)`，
这种模式下 CAN 运动指令很可能被忽略，届时 `q_cmd` 失效、策略学成恒等映射而 loss 看着很好。

---

## 0.5.1 上电前的失败记录（R17 原始证据，勿删）

> **2026-10-19 12:03 更新：已解除。** 臂已上电、邻机已拆除、铭牌已取证（S/N `MA100101000019006400985`）。

> **2026-10-19 11:47 更新：臂尚未上电，阶段 0 就此挂起。** 硬件 6 项全部标记为「待上电」，不做「跳过即通过」的处理。
> 恢复只需一条命令：`bash ~/stage0/20261019_stage0/env/poweron_collect.sh`（详见 §0.6）。

软件侧全部通过，硬件侧**一条有效反馈帧都没收到**。

| 观察 | 数值 | 含义 |
| --- | --- | --- |
| `can0` 状态 | UP，1 Mbps，sample_point 0.75 | 接口配好了 |
| 10 s 收到帧数 | **47768**，但 `distinct_ids = 1` | 总线在忙，但只有一种帧 |
| 唯一的 ID | **0x24** | 不是 0x251~0x256，也不是 0x2A1 |
| SDK 发帧 | 固件搜索 / 限位查询 / 从站配置 **全部 `SEND_MESSAGE_FAILED(100017)`** | 适配器发出去没人 ACK |

结论：**CAN 物理层没通**——不是代码问题，是「对端不在线或不回声」。最可能依次是：臂没上电 / 急停按下 / CAN 线只插了一端 / 臂侧波特率不是 1 Mbps。

**这不是「臂没使能」。** 上一轮看到的 `enable=False`、`ctrl_mode=STANDBY(0x0)`、`守卫=已核验`，全部是 SDK 在没收到帧时返回的**初始默认值**，不是实测。1.3 版探针会先做 0.6 s 总线活性检查，没反馈帧就把所有「已核验」整体降级为「不通过」。

现场先做这一条（1 分钟）：

```bash
python scripts/stage0_probe.py --phase bus --seconds 5
```

看 `distinct_ids` 是不是还是 1、`other_ids` 是不是还是 `['0x24']`、`error_frames` 是不是和 `total_frames` 同量级。**只有当 `0x2A1` 和 `0x251~0x256` 出现，后面的 sdk 相位才有意义。**

---

## 0.6 臂上电之后：一条命令恢复

**不要**上电后直接跑 `--phase sdk`。先按顺序确认三件事，否则又会产出「看起来成功其实全是默认值」的报告：

1. 臂已上电（底座指示灯亮）、**急停已松开**
2. CAN 线两端都插好，120 Ω 终端电阻在位
3. `can0` 已 up：`sudo ip link set can0 up type can bitrate 1000000`

然后：

```bash
cd ~/stage0/20261019_stage0 && source .venv-py3.12/bin/activate
bash env/poweron_collect.sh
```

它会：

| 步骤 | 做什么 | 不通过时 |
| --- | --- | --- |
| ① | 确认解释器里有 `piper_sdk`/`can` | 提示先激活 venv 并退出 |
| ② | 确认 `can0` 存在且 `operstate=up` | 打印该敲的 `ip link` 命令并退出 |
| ③ | 监听 10 s，检查 `0x2A1` 与 `0x251~0x256` | **没有任何反馈帧就中止**，不再盲发查询帧 |
| ④ | `--phase sdk --allow-write --query-limits` | 取固件 / 从站 / 限位 / 使能位 / 守卫 |
| ⑤ | `stage0_dump.py` 打印摘要 | 直接把这段贴回来即可 |

它**不会发** `0x471` 使能、不发任何运动帧。三个脚本替代不了的人工项仍然要做：铭牌+序列号拍照、末端总重称重、邻机间隙实测。

---

## 0.7 上电**之前**就能做完的 4 件事（2026-10-19 11:56 复核后新增）

之前把 9 项门里"没做的"统一记成"等上电"，这不准确。下面 4 件事**不需要通电**，堆到上电后做只会拖时间：

| # | 做什么 | 怎么做 | 花多久 |
| --- | --- | --- | --- |
| 1 | 铭牌 / 序列号归档 | 手机拍臂体铭牌（底座侧面或大臂内侧），要能看清型号、序列号、生产日期；同时抄成文字 | 2 分钟 |
| 2 | 末端总重称重 | 把夹爪（含相机、线缆）从法兰拆下，厨房秤或称重挂钩称；记到 0.01 kg | 5 分钟 |
| 3 | 邻机间隙实测 | 卷尺量机械臂运动半径内与最近障碍/邻机的最小距离 | 2 分钟 |
| 4 | 把 VM 产物拉回 Windows | 见下 | 1 分钟 |

第 4 件最容易被漏，但它决定"版本冻结"是否真的可复现：

```powershell
# 【Windows】PowerShell 里敲
scp "zzz@192.168.242.128:~/stage0/20261019_stage0/env/*" "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\experiments\20261019_stage0\env\"
```

拉回来后本地 `env/` 应该有这 5 个文件（现在一个都没有）：

| 文件 | 内容 |
| --- | --- |
| `requirements-lock.txt` | 已装包的精确版本 + 哈希 |
| `robot_env_lock.json` | 解释器/OS/包/lock 的 sha256 汇总 |
| `env_probe.json` | `--phase env` 的环境体检 |
| `bus_probe.json` | `--phase bus` 的总线实测（含 `distinct_ids=1` 那次） |
| `sdk_probe.json` | `--phase sdk` 的 SDK 读数（含 TX 失败记录） |

> 为什么要留 `bus_probe.json` 和 `sdk_probe.json` 这两份「失败的报告」：它们是 R17（总线无 ACK）与 R18（守卫零值假阳性）的**唯一原始证据**。
> 上电后 `poweron_collect.sh` 会先自动把它们快照到 `env/prepower_snapshot/`（已有快照不覆盖），再跑新的一轮——不用你手动改名。

---

## 0.9 首采已完成（12:12）· 现在只差三步

12:12 那次 `poweron_collect.sh` 已经拿到有效结果：`0x251~0x256` 6/6、`0x2A1~0x2A8` 8/8、
31396 帧 / 0 错帧、固件 `S-V1.8-8`、六轴全使能、守卫 `discarded=10`。
**完整判读见 [`POWERON_1212_RESULT.md`](POWERON_1212_RESULT.md)**，下面只列动作。

### ① 重跑限位（无副作用，1 分钟）

VM 上那份 `probe` 是 1.3，1.3 里 `GetCurrentMotorAngleLimitMaxVel` 这个方法名根本不存在，
`hasattr` 为假 → 整段静默跳过，所以上次显示「未查询」。1.4 已改用
`SearchAllMotorMaxAngleSpd` / `GetAllMotorAngleLimitMaxSpd`，并按固件档位换算速度单位。

> **13:05 那次限位数据已经丢了**：1.4 的默认输出名是 `env/sdk_probe.json`，紧接着跑的
> `--set-slave` 又写同一个文件，直接覆盖。1.5 起（1.6 沿用）每次写入前自动把旧文件复制到
> `env/archive/sdk_probe_<时间戳>.json`，并在终端打 `[ARCHIVE]` 一行。

```powershell
# 【Windows】
scp "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\tmp\stage0_bundle.tar.gz" zzz@192.168.242.128:~/
```
```bash
# 【VM】cwd: ~   —— 一行一条，每条以 && 结尾，粘进终端断行也不会出错
mkdir -p ~/stage0 && tar -xzf ~/stage0_bundle.tar.gz -C ~/stage0 &&
cd ~/stage0/20261019_stage0 && source .venv-py3.12/bin/activate &&
python scripts/stage0_probe.py --version &&
python scripts/stage0_probe.py --phase sdk --allow-write --query-limits
```

**1.6 的摘要会多打四行**，贴回来就够判：

```
[SUMMARY] 固件=S-V1.8-8 已使能=True 模式=TEACHING_MODE(0x2)
[SUMMARY] 守卫=不通过 拒收={"baseline": 1, "bad_value": 520}
[GUARD] 六轴读数非有限值或落在 URDF 限位外 —— 先确认臂没被手推到限位外
[SUMMARY] 限位=已核验 关节数=6
    joint_1: [-154.00, +154.00] deg  vmax=5.000 rad/s
```

限位成功时会多打一行 `[OK] 限位另存 .../env/joint_limits.json`——这份文件直接喂给 L0 的
`stage1_cmd.py --limits`，不用手工转抄六行数（手抄一次就多一个出错点，而且没人会去核）。

`守卫=不通过` 单看没有信息，必须看 `拒收=` 那一栏：`bad_value` 是读数超限（示教模式下
手推过最容易出现）、`dt_too_small` 是拿到缓存帧、`dt_too_large` 是读线程被阻塞、
只有 `baseline/accumulate` 则是 3 s 窗口太短（加 `--guard-seconds 8`）。

> **13:05 那次「守卫=已核验 → 不通过」1.6 起不会再误判**：守卫原来拿 URDF 标称限位当硬拒绝，
> 而本表 J3 是 `[-2.967, 0]`（只允许负向），真机 J3 一旦为正就整段拒收，报告写「不通过」，
> 看着像臂坏了，其实是**拿一个没核验的假设否决真机数据**。现在分两级：物理不可能（|q|>2π）
> 才硬拒；URDF 越界只计数，在摘要里打 `[NOTE] 读数越出 URDF 标称限位 {...}`。
> 等门 8 拿到真限位，再开 `strict_limits` 当硬判据。


### ② 进程序控（**有副作用**，人守急停）

**先纠正 13:05 那次为什么没成功。** 我原来写的是「`0xFC` → `MotionCtrl_1(0x02,0,0)` 复位 →
`MotionCtrl_2(0x01,...)`」，三处都有问题，13:17 从 `piper_sdk-0.6.1` wheel 源码逐行核对后确认：

| 我原来的说法 | 源码里的真相 |
| --- | --- |
| `0xFC` = 切程序控 | `0xFC` 是**随动主从**里的「运动输出臂」，只有双臂联动才用；单臂 CAN 直控跟它无关 |
| `MotionCtrl_1(0x02,0,0)` = 复位 | Byte0 是急停：`0x00` 无效 / `0x01` 急停 / `0x02` **恢复**。它根本没在复位 |
| 「发一帧切到 CAN_CTRL」 | `MotionCtrl_2` 的 ctrl_mode 只有 `0x00/0x01/0x03/0x04/0x07`，**没有 0x02** |
| — | 退出拖动示教要看 `MotionCtrl_1` 的 **Byte2**（`grag_teach_ctrl`）：`0x00` 关闭 |

所以 13:17 读回 `模式=TEACHING_MODE(0x2)` 不是帧没发出去，是**发的东西不对**。正确的顺序是先退出示教：

```bash
# 【VM】cwd: ~/stage0/20261019_stage0，已 source .venv-py3.12/bin/activate
# 先把手臂上的示教按钮/手柄松开，末端不承重，手放急停上
python scripts/stage0_probe.py --phase sdk --allow-write --to-can-ctrl
```

它发 `MotionCtrl_1(0x00,0x00,0x00)`（关闭拖动示教）→ `MotionCtrl_2(0x01,0x01,100,0x00)`
（CAN 控制 / MOVE J / 100% / 位置速度）→ 回读 `ctrl_mode`。

**判据**：`[SUMMARY] 切程序控=已发 ... 完成后=CAN_CTRL`。若仍是 `TEACHING_MODE(0x2)`，
说明有物理因素压着（示教按钮/手柄没松、底座开关在示教位），软件指令压不住它——**停下来告诉我，别发运动指令**。

> `--set-slave` 保留但改标注：它只做随动主从，单臂复现**不要用**。
### ③ 称末端（不用电，5 分钟）

夹爪 + 相机 + 线缆一起拆下来称 kg。1.5 kg 是含工具的上限。

---

## 0.8（已完成·保留作记录）臂已上电：当时跑的三段

> 这一节是 12:03 写的、12:12 已执行完。**先读 §0.9**，本节只作过程存档。


前提已满足：臂已上电、邻机已拆除、铭牌已归档（S/N `MA100101000019006400985`）。
剩 3 项门（固件 / 从站模式 / 真实限位）+ 首帧守卫现场证据，一条命令能全采。

### 先认清自己在哪个终端

| 提示符长这样 | 你在哪 | 这一段该不该在这敲 |
| --- | --- | --- |
| `PS C:\Users\asus>` | Windows PowerShell | ✅ 只有 A 段（上传/下载） |
| `zzz@zzz-VMware-Virtual-Platform:~$` | VM 的 Ubuntu 终端 | ✅ B 段（所有实验） |

**最容易错的一步**：在 VM 里敲 `scp ... zzz@192.168.242.128` —— 那是 VM 自己连自己，会卡住或报 host key。A 段必须在 Windows 敲。

---

### A 段【Windows · PowerShell】上传（30 秒）

```powershell
scp "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\tmp\stage0_bundle.tar.gz" zzz@192.168.242.128:~/
```

这一步不能跳：包里的 `poweron_collect.sh` 刚加了「上电前证据自动快照」，旧的没有。

---

### B 段【VM · Ubuntu 终端】四步

```bash
# B-1 解包（会覆盖同名文件；env/ 里的旧产物会被下面 B-4 自动快照，不会丢）
mkdir -p ~/stage0 && tar -xzf ~/stage0_bundle.tar.gz -C ~/stage0

# B-2 进目录 + 激活 venv（这两行只对当前终端窗口有效，新开窗口要重来）
cd ~/stage0/20261019_stage0 && source .venv-py3.12/bin/activate

# B-2.5 确认跑的是新脚本，不是 VM 上那份旧的
python scripts/stage0_probe.py --version     # 期望 stage0_probe 1.3-20261019

# B-3 让 can0 就位（VM 重启过就必须做；没重启可直接看结果）
lsusb | grep -i 1d50
lsmod | grep gs_usb || sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0 | head -3

# B-4 一条命令采齐（不发使能、不发任何运动帧）
bash env/poweron_collect.sh
```

**B-2.5 的期望**：打印 `stage0_probe 1.3-20261019`。
如果打印的是 `1.2-20261019`，说明 A 段没传或 B-1 没解开——**先停下**，回去重做 A 段。你回传的那份 `sdk_probe.json` 就是 1.2 跑的，1.3 修掉了它里面的两个问题（高速通道缺失不打标记、守卫在零值上假阳性）。

**B-3 的期望**：`lsusb` 出现 `1d50:606f`，`ip -details link show can0` 里有 `bitrate 1000000` 和 `state UP`。
`sudo` 会要密码，输密码时屏幕不显示字符，正常。

**B-4 的期望**：跑到 `① 监听 10 s` 那句时，会出现

```
   反馈帧: 0x251~0x256 命中 6/6；0x2A1 类命中 1
```

**只要出现这行，说明上电前那个「总线只剩 0x24」的问题已经解决**，后面会自动往下走完。

如果仍打印 `[STOP] 总线上仍然没有机器人反馈帧`，停下来别重试，按 §3 的定位表查（顺序：臂上电指示灯 → 急停 → CAN 线两端 → 120Ω 终端电阻 → 臂侧波特率）。

---

### C 段【Windows · PowerShell】把结果拉回来（30 秒）

```powershell
scp -r "zzz@192.168.242.128:~/stage0/20261019_stage0/env/." "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\experiments\20261019_stage0\env\"
```

拉完本地 `env/` 里应该多出 **6 个东西**：

| 文件 | 是什么 |
| --- | --- |
| `bus_probe.json` | 上电后的总线实测（6/6 命中那份） |
| `sdk_probe.json` | 固件 / 从站 / 限位 / 使能位 / 守卫 |
| `env_probe.json` | 环境体检 |
| `requirements-lock.txt` | 包版本 + 哈希 |
| `robot_env_lock.json` | lock 汇总 |
| `prepower_snapshot/` | **上电前那两份失败证据的副本**（R17/R18 原始样本） |

---

### 跑完贴什么给我

最省事：直接贴 `bash env/poweron_collect.sh` 终端里 `③ 摘要` 那一整段。
精确一点：贴 `env/sdk_probe.json` 里这几个键 —— `firmware`、`slave_mode`、`limits_0x473`、`enable_all_joints`、`first_frame_guard`。

收到之后我会：把第 1/5/6/8 项判掉、回填 `0x473` 真实限位到 `JOINT_MIN/MAX`、给 L1 的 `--limits` 参数定值。

---

### 这段期间不要做的三件事

| 别做 | 为什么 |
| --- | --- |
| 别在 VM 里同时跑 `~/agx_arm_ws` 的 ROS 节点 | 两套 SDK 抢同一条 CAN，帧会互相踩，采到的时间序列不可用 |
| 别发 `0x471` 使能 | FIRST_ARM 红线：本机臂上电即自使能，再发会重复使能 |
| 别让臂运动 | 这一阶段只要静态读数；一运动就要开始记 `q_cmd`，那是 L0-b 的事 |

---

## 0 你的机器（归档里已有的，不用再查）

| 项 | 值 |
| --- | --- |
| 机械臂 | AgileX PiPER 六轴，型号 `六轴机械臂 PIPER`，S/N `MA100101000019006400985`（2026-10-19 12:03 铭牌取证） |
| 系统 | Ubuntu（VMware 虚拟机）`zzz@192.168.242.128`，主机名 `zzz-VMware-Virtual-Platform` |
| ROS | **ROS 2 Lyrical**（`/opt/ros/lyrical`）+ MoveIt 2 |
| 系统 Python | **3.14.4** |
| 工作区 | `/home/zzz/agx_arm_ws`，驱动 `agx_arm_ros@ceda565`（URDF 子模块 `6a7f2e2`） |
| 既有 SDK | **pyAgxArm 1.0.0**（`d90e234`）——和本复现要用的 `piper_sdk 0.6.1` 是两套 |
| CAN | `can0`，1 Mbit/s，`gs_usb` CandleLight `1d50:606f`（USB 直通进虚拟机） |
| 实测反馈 | `/feedback/joint_states` **369.1 Hz**（聚合）、中继 204.7 Hz、JTC 200 Hz |
| 已知未闭环 | ~~铭牌序列号~~（12:03 已取证）、~~邻机间隙~~（邻机已拆除）、固件版本（M10，§0.8 采）、0.083° 静差、`initial_positions` 全 0 |
| 场地变更 | **邻机已于 2026-10-19 12:03 拆除** → 单臂工作空间；原「旁边还有其他机械臂」的安全前提作废，但可达半径内仍需清空 |

这份配置直接推翻了阶段 0 的三个原设定，已改到脚本里：

1. **不用发使能**：该臂上电即自使能，归档红线 #1 明确「不发送使能/失能命令」→ 探针默认不发 `0x471`，只用 `GetArmEnableStatus()` 只读确认。
2. **ROS 不是 Jazzy 而是 Lyrical**：官方 `factr2_next` 只验证过 Jazzy。
3. **Python 不是 3.12 而是 3.14**：官方依赖 `numpy<2.0` 在 3.14 上**装不了**（实测 numpy 1.26.4 只有 cp312 轮子，cp314 从 2.3.2 才有）。torch 没问题（≥2.9 有 cp314 轮子）。

---

## 1 门为什么没过：9 项逐一交代

| # | 验收项 | 状态 | 需要你做什么 | 耗时 |
| --- | --- | --- | --- | --- |
| 1 | piper_sdk 版本 | ✅ 通过 | 无（已冻 `0.6.1` + wheel sha256） | — |
| 2 | 官方实现版本 | ✅ 通过 | 无（`0df02c5b`） | — |
| 3 | `source_commit.txt` | ✅ 通过 | 无 | — |
| 4 | 软件版本齐全（另一侧） | ✅ 通过 | 无 | — |
| 5 | **固件版本** | ⬜ 未评估 | **§0.8 的 B-4 一条命令** | 1 min |
| 6 | **从站模式确认** | ⬜ 未评估 | **§0.8 的 B-4**（单臂反馈已通，大概率「不适用」） | 2 min |
| 7 | **首帧丢弃已体现** | ❌ 不通过 | **§0.8 的 B-4** 留证（逻辑已实现，离线自测 PASS） | 2 min |
| 8 | **铭牌 / 序列号** | ✅ **通过**（2026-10-19 12:03） | 无 —— S/N `MA100101000019006400985`，见 `evidence/nameplate.json` | — |
| 9 | **真实限位 + 末端负载** | ⬜ 未评估 | 限位=**§0.8 的 B-4**；负载仍需一个秤 | 限位 0 min / 负载 5 min |

---

## 2 动作 A · 建环境（10 分钟，不动臂）

**为什么不用系统 3.14**：官方要 `numpy<2.0`，而 numpy 1.x 没有 cp314 轮子。硬在 3.14 上装只会去编源码，编出来也不是官方验过的口径。所以训练/离线走 3.12 独立 venv，系统 3.14 留着跑 ROS 和 MoveIt，两边用 H5 交换数据。

**① 在【Windows】的 PowerShell 里**（只有这一条命令在 Windows 上敲）：

```powershell
scp "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\tmp\stage0_bundle.tar.gz" zzz@192.168.242.128:~/
```

**② 回到【VM】，下面全部在 VM 里**（每条都用绝对路径，`cd` 在哪都行）：

```bash
# 解包到 ~/stage0/20261019_stage0
mkdir -p ~/stage0 && tar -xzf ~/stage0_bundle.tar.gz -C ~/stage0
ls ~/stage0/20261019_stage0          # 期望看到 env/ scripts/ manifest.json ...

# 拉官方实现并核对 commit（_vendor 不入库，所以要自己 clone）
git clone https://github.com/philiphan0109/factr2_next.git ~/factr2_next
git -C ~/factr2_next rev-parse HEAD        # 期望 0df02c5b26e33911078152b6c587d6e8a160d279

# 先体检（不装任何东西，10 秒），确认 uv / python3.12 / can0 / 磁盘 状态
bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh --check

# 装依赖。按这台 VM 的实际情况二选一：
#   a) 磁盘只有 1.2 G → 只装阶段 0 需要的最小集（python-can + piper-sdk，约 30 MB）
bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh --minimal
#   b) 腾出 6 G 以上、要跑训练时 → 完整安装（torch 走 CPU 源，250 MB 而非 2.3 GB）
# bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh --with-hardware --torch-cpu
```

每个文件在哪、哪条命令在哪台机器敲，见 [`WHERE_TO_PUT.md`](./WHERE_TO_PUT.md)。

**期望输出**（`--minimal`）：`piper-sdk 0.6.1`、`python-can …`、`numpy torch h5py MISSING（最小安装，本次不需要）`、`SKIP numpy 口径断言`、`can 模块 OK`、`piper_sdk OK`。

> 为什么最小集就够：`piper_sdk` 的唯一依赖是 `python-can`（wheel METADATA 里只有这一条），源码里根本没有 `import numpy`。阶段 0 只读 CAN 帧，不需要训练栈。

> 如果 `uv` 拉不到 3.12（离线/内网），退路是 `--python=3.14`，它会装 `numpy>=2.3.2`。这条路与官方 `<2.0` 不符，要在实验记录里标成「显式偏离」，后面 L1 报误差时得提一句。

---

## 3 动作 B · 只读三连（5 分钟，不发任何帧）

**前置：先让 `can0` 出现**。你这次 `--check` 报的是 `can0 不存在`，说明 USB-CAN 没进虚拟机（FIRST_ARM 归档里它时有时无，就是这个原因）。

**【VM】让 can0 出现**（不需要 cd，任何目录都行）：

```bash
lsusb | grep -i 1d50          # 期望看到 OpenMoko/CandleLight 1d50:606f
lsmod | grep gs_usb || sudo modprobe gs_usb
ip link | grep can0
# 出现了再拉起来。**必须分三步**，`up type can bitrate ...` 写在一条里会报
# `RTNETLINK answers: No such device`（R34：netlink 事务中途失败、netdev 半提交，
# 不是设备丢了；16:08 被这条错误文案误导过一轮）
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0    # 确认 state UP + bitrate 1000000 + can state ERROR-ACTIVE
```

`lsusb` 里什么都没有才是真的没进 VM → 在**宿主 Windows** 上：先 `Get-Service VMUSBArbService` 看是否在跑（它会抢设备），再到 VMware 菜单 `VM → Removable Devices` 里把那个 CAN 适配器 **Connect (Disconnect from Host)**，然后回 VM 重新 `lsusb`。这一步没通之前，`--phase bus` 和 `--phase sdk` 都没有意义。

**`sudo ip link set can0 up` 报 `Cannot find device "can0"` 时，按这三步定位**（你 11:24 就是卡在这）：

```bash
lsusb | grep -iE '1d50|gs_usb|candle|CAN'      # ① 内核认不认这个 USB 设备
lsmod | grep gs_usb                            # ② 驱动加载了没
sudo dmesg | tail -30                          # ③ 插拔那一刻内核说了什么
```

| ① `lsusb` | ② `lsmod` | 结论与处置 |
| --- | --- | --- |
| 空 | — | USB 设备根本没进 VM。VMware `VM → Removable Devices` 里连接它；Windows 侧先 `Get-Service VMUSBArbService` 若在跑就停掉再插 |
| 有 `1d50:606f` | 空 | 设备在但驱动没起：`sudo modprobe gs_usb`，然后 `ip link` 应出现 `can0` |
| 有 `1d50:606f` | 有 | 驱动正常却无 `can0`：看 ③ 有没有枚举错误；多半是 USB 线/供电，换口重插 |
| 有但 ID 不是 `1d50` | — | 不是 gs_usb 那款适配器，`python-can` 要用别的 backend（`--backend pcan` 等），先拍照告诉我芯片型号 |

`can0` 一旦出现，后续 `ip link set can0 up type can bitrate 1000000` 才可能成功；在它出现之前，探针会直接报「CAN 接口 can0 不存在——USB-CAN 没进虚拟机」，不会再甩 Python 栈。

**【VM】目录：`~/stage0/20261019_stage0`**（这两行 `cd`+`source` 只对当前终端窗口有效，新开窗口要重敲）：

```bash
cd ~/stage0/20261019_stage0
source .venv-py3.12/bin/activate

python scripts/stage0_probe.py --phase selftest          # 期望 PASS
python scripts/stage0_probe.py --phase env  --factr2-repo ~/factr2_next
python scripts/stage0_probe.py --phase bus  --seconds 10
```

> 不加 `--out` 时结果默认落到 `<本阶段目录>/env/<phase>_probe.json`，在哪个目录下运行都一样。

`bus` 这一步是**只监听**，不发帧，可以放心跑。它一次给出三件东西：

- `can0` 状态与 bitrate（应为 1 Mbps、state UP）
- `0x2A1` 心跳是否在
- **六个 `0x251~0x256` 各自的到达周期中位数**——这一步顺便回答了你归档里「369 Hz 到底是聚合还是逐 ID」的疑问（369 ≈ 300(6×50) + 状态帧，逐 ID 应该是 20 ms）

---

## 4 动作 C · 发查询帧（2 分钟，不运动）

**【VM】目录：`~/stage0/20261019_stage0`**（沿用动作 B 那个已激活 venv 的终端）：

```bash
python scripts/stage0_probe.py --phase sdk --allow-write --query-limits \
  --factr2-repo ~/factr2_next
```

它**只发**这几类帧，不发运动指令：

| 帧 | 用途 | 备注 |
| --- | --- | --- |
| `0x470` MasterSlaveConfig | 确认从站/反馈配置 | 你单臂反馈已经通了，大概率本来就不用改 |
| `0x472` 参数查询 | 回读六轴真实限位 + 最大速度 | 解决 J6「±170 还是 ±120」的悬案 |
| 固件搜索帧 | 读不到版本时才会发 | 若仍失败，用厂家上位机抄一行给我 |

**它不会发**：`0x471` 使能/失能、`0x150` 急停、`0x151/0x155~0x157` 任何运动帧。

跑之前确认三件事：臂已固定、物理急停在手边、**不要同时开着 `agx_arm_ctrl` 驱动**（同一个 `can0` 两套写者，判读会乱）。

---

## 5 动作 D · 一个秤（照片已拍，剩称重）

1. ~~**铭牌照片**~~ **已完成（12:03）**：S/N `MA100101000019006400985`，归档在 `evidence/nameplate.json` + `evidence/nameplate_20261019.jpg`。
2. **末端**：夹爪型号 + 整条末端（夹爪+支架+线缆）**实际称重 kg**。上限 1.5 kg 是含工具的，后面加相机/传感器都从这里扣。
3. ~~量邻机最小间隙~~ **已作废（12:03 邻机已拆除）**。替换为：量**单臂可达半径内的最近硬物**（墙、桌沿、货架），单位 mm —— 它决定 L1 之后每一次动作的幅度上限。

---

## 6 动作 E · 把结果发我

需要回传这些，我直接回填 `manifest.json` 并把门状态改成通过：

**方式一（推荐）：在【Windows】PowerShell 里一条命令拉回来**

```powershell
scp "zzz@192.168.242.128:~/stage0/20261019_stage0/env/*" "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\experiments\20261019_stage0\env\"
```

**方式二（更省事）：在【VM】里跑摘要脚本，把输出贴给我**

```bash
cd ~/stage0/20261019_stage0 && source .venv-py3.12/bin/activate
python scripts/stage0_dump.py
```

它只读不写，输出约 40 行，含 env / 总线逐 ID 周期 / SDK 状态与限位 / lock_sha256，比贴原始 json 清楚得多。

需要回传的东西：

```
env/env_probe.json
env/bus_probe.json
env/sdk_probe.json
env/robot_env_lock.json        ← 里面的 lock_sha256 就是环境冻结凭据
铭牌照片 / 序列号 / 末端重量 / 邻机间隙
```

拿到后我会做三件事：回填 9 项门状态 → 用实测限位替换所有硬编码 → 给出 L0 开工前的最后三个冻结值（谁打戳、`dq` 用哪一路、速度上限百分比）。

---

## 7 这阶段**不要**做的四件事

- ❌ **不要发 `0x471` 使能**。这台臂上电就有力矩，发了既没必要也违反你自己的红线 #1。
- ❌ **不要 `ros2 launch` 任何真机 launch**。`start_single_agx_arm*.launch.py` 会走 `go_to_home`，臂会动。
- ❌ **不要同时跑驱动和探针**。同一 `can0` 两套写者，出问题无法归因。
- ❌ **不要现在标 K、不要买力传感器**。那是阶段 2 的「尺度门」，与阶段 0 无关。

---

## 8 三件已知会挡路的事（提前说，免得现场懵）

| 冲突 | 现状 | 处置 |
| --- | --- | --- |
| J6 限位 | 归档记 ±170°，手册/URDF 记 ±120° | 动作 C 的 `0x473` 回读定案；定案前按 ±120° 保守用 |
| `0x251` 位置单位 | SDK 文档写 `rad`，你逐字节实测记 `0.001 rad` | 差 1000 倍。`q` 走 `0x2A5`（0.001°）不受影响，但自建原始层若用 `0x251` 位置必须先用 `0x2A5` 交叉定标 |
| `can0` 时有时无 | 依赖 USB 直通；Windows 侧 `VMUSBArbService` 会抢设备 | L0 压测前把直通配置固定下来，并记录「压测期间 Windows 未连接同一适配器」 |
| **使能位读出来是 False**（实测 2026-10-19 11:29） | 归档写「上电即自使能」，但 `GetArmEnableStatus()` 六位全 False。可能是：① 臂未上电/急停按下；② `ctrl_mode` 不是 `0x01`（CAN 指令模式），电机不响应；③ 归档里的「自使能」其实是起了 `agx_arm_ctrl` 之后的状态 | **不要用 `0x471` 去试**（红线 #1）。先看 `stage0_dump.py` 里的 `ctrl_mode`：若是 `0x01` 而 enable 仍 False，说明需要显式使能，这条归档结论就要改写；若 `ctrl_mode` 是别的值，先切到 CAN 指令模式再判 |

---

## 9 跑完这些，门会变成什么

- 现在：**未通过**（4 通过 / 5 未评估 / 1 不通过）
- 跑完 A–D：**通过**（9 项全部有值有证据；「邻机间隙」属 L4 安全项，不卡阶段 0）
- 卡点不在技术，在你什么时候能站到那台臂旁边 20 分钟。

---

## 10 报错对照表（已经踩过的照这个改）

| 你看到的 | 真实原因 | 正确做法 |
| --- | --- | --- |
| `bash: env/freeze_robot_env.sh: No such file or directory` | 当前在 `~`，而这个文件在 `~/stage0/20261019_stage0/env/` | `bash ~/stage0/20261019_stage0/env/freeze_robot_env.sh --check`（脚本按自身位置定位，cwd 无关） |
| `--phase: command not found` | `--phase` 是**给 python 的参数**，不是命令；漏了 `python scripts/stage0_probe.py` | `python scripts/stage0_probe.py --phase selftest` |
| `ModuleNotFoundError: piper_sdk` | 没装 `piper-sdk`，或没激活 venv | 先 `bash …/freeze_robot_env.sh --with-hardware`，再 `source ~/stage0/20261019_stage0/.venv-py3.12/bin/activate` |
| `chmod +x` 后仍 `Permission denied` | 用 `bash 脚本` 调用时不看可执行位；直接 `./脚本` 才看 | 统一用 `bash <路径>` 调用，不要 `./` |
| `[FAIL] 缺 uv` 或 uv 装不上 | VM 无网 / curl 被拦 | 脚本会自动退到 `python3.12 -m venv`；若也没有 3.12：`sudo apt install python3.12`，或临时用 `--python=3.14`（要标注「显式偏离」） |
| `[WARN] can0 不存在` | USB-CAN 没直通进虚拟机，或 Windows 侧 `VMUSBArbService` 占着设备 | VMware 里重新连接该 USB 设备；Windows 服务里把 `VMUSBArbService` 停掉再插 |
| `OSError: [Errno 98] Address already in use` / 帧全是旧的 | `agx_arm_ctrl` 或 `ros2 launch` 还开着，两套写者在抢 `can0` | 先停掉所有 ROS 进程再跑探针 |
| `--phase bus` 里 bitrate 显示 null | `ip -details` 输出格式随版本变，解析是尽力而为 | 直接 `ip -details link show can0` 看原文，应为 `bitrate 1000000` |
| `error: No solution found when resolving dependencies … install was not found in the package registry` | 脚本 bug：`_pip` 里已经写了 `uv pip install`，调用处又传了一个 `install`，变成 `uv pip install install pip …` | 已在 2026-10-19 11:10 修掉（`_pip` 改为 `uv pip "$@"`）。拉新包重跑即可 |
| `[FAIL] 磁盘只剩 1 G，完整安装约需 6 G` | 这台 VM 装不下 numpy+scipy+h5py+torch（CUDA 版 2.3 GB） | `--minimal`（阶段 0 够用）或 `--torch-cpu`（250 MB）；训练前先扩盘 |
| `[SUMMARY] SDK 未加载：pip 装 piper-sdk 后重试` | 依赖没装成（上一条的后果），`--phase sdk` 走的是空报告 | 装上 `--minimal` 后重跑；现在这行会显式打出来，不再静默 |
| `Cannot find device "can0"` | USB-CAN 没进 VM，`ip link` 无从下手 | 别急着 `ip link`，先按 §3 的三步定位：`lsusb` → `lsmod` → `dmesg` |
| `[SUMMARY] SDK 未加载：CAN 接口 can0 不存在——USB-CAN 没进虚拟机` | 探针自己在实例化前先查了 `/sys/class/net/can0`，直接给结论 | 同上；这条是**人话错误**，不再抛 `ConnectionError` 栈 |
| `SyntaxWarning: invalid escape sequence '\['` | `piper_sdk` 自己 docstring 里的转义问题，与你的实验无关 | 已在新版探针里用 `warnings.filterwarnings` 压掉；旧版忽略即可 |
| `ValueError: too many values to unpack (expected 2)`（`--phase bus`） | 脚本 bug：统计分组的字典推导写错了两层解包 | 已修（改为显式循环）。换新包重跑 `--phase bus` 即可 |
| `Command 'python' not found` | Ubuntu 26.04 没有 `python` 这个别名，且 venv 未激活 | 先 `source .venv-py3.12/bin/activate`，提示符会变成 `(ros)`；或直接用 `python3` 跑纯标准库的 `--phase selftest/env` |
| `firmware` 仍是未核验 | `GetPiperFirmwareVersion()` 读不到时返回 `-0x4AF`，必须先发一次搜索帧 | 新版探针在 `--allow-write` 下会自动 `SearchPiperFirmwareVersion()` 再读；重跑 `--phase sdk --allow-write` 即可 |
| `SEND_MESSAGE_FAILED (100017)` | 帧发出去了，但总线上没有任何节点 ACK → 适配器认为发送失败 | 不是代码问题。查臂电/急停/CAN 线/波特率；`--phase bus` 里 `distinct_ids` 应 >1 |
| `--phase bus` 里 `distinct_ids=1`、`other_ids=['0x24']` | 总线上只有一个残缺 ID 在刷，几乎全是 error frame | 同上；1.3 版会把 error frame 单独计入 `error_frames`，与 `total_frames` 对比即可确认 |
| `ctrl_mode=STANDBY(0x0)` 且 `enable=False` | **不是没使能，是没数据**：SDK 没收到 0x2A1 时返回初始默认值 | 先让总线通（见 §0.5）；通了之后再判使能策略，**不要发 0x471** |
| `env_probe.json` 里 `official_impl` 是未核验 | `~/factr2_next` 没克隆，`--factr2-repo` 指向不存在的路径 | `git clone https://github.com/philiphan0109/factr2_next.git ~/factr2_next` 后重跑 `--phase env` |
