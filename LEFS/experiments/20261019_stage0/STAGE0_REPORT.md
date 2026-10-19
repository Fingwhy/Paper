# 阶段 0 报告：环境与版本冻结

> 配套：`PiPER_FACTR2_复现指南` §2.3 版本登记表 / §3 阶段 0
> 运行目录：`LEFS/experiments/20261019_stage0/`
> 日期：2026-10-19 ｜ 状态：**挂起（软件侧完成 / 需上电 4 项、人工项 3 项、证据归档 1 项待办）**

---

## 0 复核结论（2026-10-19 11:56，用户问「除了上电是否完成」）

**答案：没有。** 未完成项分三类，只有第一类真的需要上电。

| # | 门项 | 状态 | 真的需要上电？ | 现在能不能做 |
| --- | --- | --- | --- | --- |
| 1 | 固件版本号齐全 | 不通过 | ✅ 需要 | 只能等上电（`0x472` 搜索帧 TX 失败，R17） |
| 2 | piper_sdk 版本号齐全 | 通过 | — | 已完成（0.6.1 + wheel sha256） |
| 3 | 官方实现版本号齐全 | 通过 | — | 已完成（`0df02c5b`） |
| 4 | `source_commit.txt` 存在 | 通过 | — | 已完成 |
| 5 | 从站模式确认 | 不通过 | ✅ 需要 | 只能等上电（`0x470` 无回读） |
| 6 | 首帧丢弃逻辑已体现 | 部分 | ✅ 需要 | 逻辑+离线自测已过；现场证据是零值假阳性（R18），需真反馈重跑 |
| 7 | 机械臂编号 / 铭牌 | 未评估 | ❌ **不需要** | **现在就能做**：手机拍铭牌 + 抄型号序列号 |
| 8 | 真实限位回读 | 不通过 | ✅ 需要 | 只能等上电（`0x473` 未取得回读） |
| 9 | 末端配置与总负载 | 未评估 | ❌ **不需要** | **现在就能做**：末端拆下用秤称（夹爪+相机+线缆） |

> **两处登记错误已更正**：第 7、9 项此前被统一挂上 `blocked_by = 臂未上电`，但拍照和称重都不涉及通电。已改 `manifest.json` 并留 `correction` 字段，避免以后又按"必须等上电"排期。
> 另有一项同样不需要上电：**邻机间隙实测**（卷尺/塞尺即可），此前只列在 `resume.not_done_by_script` 里。

**第三类：证据未归档（不依赖上电，现在就能补）**

VM 侧 5 个产物一个都没回传到 Windows，`env/` 目录里只有脚本、没有结果：

| 文件 | 现状 |
| --- | --- |
| `env/requirements-lock.txt` | **缺**（sha256 `5d174257…` 只存在于对话记录，文件未归档） |
| `env/robot_env_lock.json` | 缺 |
| `env/env_probe.json` | 缺 |
| `env/bus_probe.json` | 缺 |
| `env/sdk_probe.json` | 缺 |

后果：VM 一旦重装或目录被覆盖，环境就不可复现——而"版本冻结"的全部意义就是可复现。补法见 `WHERE_TO_PUT.md` C-2。

**所以"软件侧完成"的准确说法是**：脚本、判据、登记表完成；**证据尚未落袋**。

---

## 1 结论

> ⚠️ 以下 §1 原文写于 11:14（VM 尚未跑探针之前），叙述已被 §0 取代，保留作为过程记录。

软件侧的三件事今天全部钉死了：官方实现 commit、SDK 版本（含 wheel 哈希）、URDF 来源与哈希；并且把「配置口径」从官方默认值上调成我们自己的冻结值（`history 50 → 25`）。

硬件侧的六项（固件、序列号、从站模式、末端配置、真实限位、总线实测）**一项都没做**——本机是 Windows，无 CAN 接口、无串口设备（WSL 的检查命令被本机安全策略拦截，未验证），指南 §2.2 明确写了它不能当实验环境。所以阶段 0 的门现在是 **未通过**，不是「通过但有小问题」。这是如实记录，不是遗漏。

> **2026-10-19 10:5x 补记**：读完 `C:/Users/asus/ARM/FIRST_ARM` 后，目标机不再是「Ubuntu + Jazzy + Python 3.12」，而是 **Ubuntu(VM) + ROS 2 Lyrical + Python 3.14.4**，且该臂**上电即自使能、不许发 `0x471`**。这三条已改写进 `manifest.json` 与探针脚本；现场要做的事单独成册，见 **`STAGE0_ACTIONS.md`**（6 个动作、约 20 分钟）。

另外，复核官方实现时发现 7 处与指南口径冲突的地方，其中 3 处会直接影响 L0 的证据质量，必须在开采集前定掉（见 §6）。

## 2 本阶段做了什么、没做什么

**做了（可在本机复核）**

- 克隆并冻结 `factr2_next` 到 `0df02c5b`，写 `source_commit.txt`（含 tree hash、分支、提交日期）
- 核验 `piper-sdk==0.6.1` 的 wheel sha256（下载实体比对 PyPI 元数据，一致）
- 解包该 wheel，逐条核对阶段 0 要用的接口签名——不是照记忆写代码
- 冻结 URDF：`agilexrobotics/piper_ros@017ffefa` 的 `piper_description.urdf`，sha256 已记录
- 写 `scripts/stage0_probe.py`：只读探测 + 使能确认 + 首帧守卫，带离线自测（已 PASS）
- 写 `env/requirements.in` + `env/freeze_robot_env.sh`：机器人侧一键建环境并产出 lock
- 填 `manifest.json` 的 §2.3 登记表：能核验的写值与证据，不能核验的写「未核验」

**没做（需硬件现场）**

- 读固件版本、记录臂体序列号
- 确认从站模式、总线 1 Mbps、`0x2A1` 心跳
- 六 ID 到达周期实测
- 末端配置与总负载称重
- 真实限位/最大速度回读（`0x472→0x473`）

## 3 版本登记表（指南 §2.3 填表结果）

| 字段 | 值 | 状态 |
| --- | --- | --- |
| 机械臂编号 / 固件 | — | **未核验** |
| piper_sdk | `0.6.1`（wheel sha256 `743557e1…65c`） | 已核验 |
| factr2_next commit | `0df02c5b26e33911078152b6c587d6e8a160d279`（main，2026-09-04） | 已核验 |
| URDF 来源与哈希 | `piper_ros@017ffefa` / `piper_description.urdf` sha256 `22c15f95…002b` | 已核验 |
| 控制模式 / 速度上限 | MIT（`JointMitCtrl`）；速度上限 **未核验** | 部分 |
| 末端配置与总负载 | — | **未核验** |
| K 标定版本 ID | — | **未核验**（阶段 2 产出） |
| 数据版本哈希 / 随机种子 | 预置种子 0，数据哈希待阶段 3 | **未核验** |

三个值得记下来的细节：

- `piper-sdk` 最新是 `0.6.2` / `0.7.0a1`，但官方实现写死 `0.6.1`，**不追新**。升级 SDK 会改变接口名（下面 §6 R8 就是例子）。
- `factr2_next` 仓库只有 3 个提交，另有一个 `glorbot2-integration` 分支（`cc7643ff`），**不使用**。
- `piper_ros` 只有 `noetic / foxy / humble` 分支，**没有 Jazzy**。我们只借它的 URDF，不引入它的 ROS1 驱动。

## 4 环境冻结

### 4.1 两侧分工

| 侧 | 用途 | 环境 |
| --- | --- | --- |
| 开发机（本机） | 文档、代码、数据后处理 | Windows 11 26200 / Python 3.13.14 / Git 2.55.0，**无 CAN 能力** |
| 机器人机 | 采集、训练、实验 | Ubuntu 24.04 + ROS2 Jazzy + Python 3.12 + `uv` |

本机的环境快照已落盘：`env/dev_env_windows.json`（由探针生成，含「未核验」项，没装 numpy/torch 也照实写 null）。

### 4.2 机器人侧

`env/requirements.in` 只写意图，真正的冻结物是现场生成的 `env/requirements-lock.txt`：

```bash
cd ~/20261019_stage0
bash env/freeze_robot_env.sh --with-hardware      # 建 .venv-py3.12 → 装包 → uv pip freeze → robot_env_lock.json
```

脚本会顺带采集内核、发行版、`ROS_DISTRO`、apt 层 `ros-*` 与 `pinocchio` 版本，并断言 `numpy<2.0`（3.12 轨）或 `numpy>=2.3.2`（3.14 轨）。**`torch` 官方故意不 pin**，所以它的版本只能由这份 lock 定死——lock 的 sha256 要抄回 `manifest.json`。

> 目标机实测的系统 Python 是 **3.14.4**（ROS 2 Lyrical），而 numpy 1.x 没有 cp314 轮子（1.26.4 仅到 cp312，cp314 自 2.3.2 起才有）。所以默认建 3.12 venv 走「训练/离线轨」，系统 3.14 留给 ROS——代价是该 venv 里 `import rclpy` 会失败，脚本会实测并打印这一行（`rclpy: NO` 是预期）。细节见 `STAGE0_ACTIONS.md` §2。

## 5 配置冻结：一处显式偏离

| 参数 | 官方默认 | 本复现冻结 | 理由 |
| --- | --- | --- | --- |
| sample rate | `record.target_hz: 50` | 50 Hz | 电流原生 50 Hz |
| **history** | **`train.history: 50`** | **25** | 50 点在 50 Hz 下是 1.0 s；论文 H=50@100 Hz 是 0.5 s，取 25 才同长 |
| precontact | — | 1.0 s | FIRST 接触前窗口 |
| state_mode | stateless | stateless | 与论文一致 |
| 归一化 | 训练集拟合 | 同 | 部署复用同一组参数 |

这是唯一一处主动改官方参数的地方，登记在 `manifest.json:config_freeze.history_deviation`。改它的代价是「和官方 README 的命令行默认值不同」，所以每条实验记录都要写明实际用的是 25。

## 6 复核官方实现发现的 7 个问题

判据是「会不会污染 L0/L1 的证据」，不是「代码是否优雅」。

| ID | 问题 | 影响 | 处置 |
| --- | --- | --- | --- |
| R1 | 观测时间戳是 ROS `now()`，不是 CAN 帧到达时间；同帧被重复发布 2~4 次 | L0 的「字段年龄 / 跨关节跨度」拿不到 | 原始层自己打戳，不采信话题时间 |
| R2 | `joint_vel_obs` 是位置差分（200 Hz 差分 100~200 Hz 的位置），不是电机测速；`motor_speed` 解析了但没发布 | NEXT 的 `dq` 输入被量化 | L0 同时存两路速度并对比 |
| R3 | **没有首帧丢弃逻辑**：`enable_arm()` 轮询到使能就返回，随后立即 `go_to_home_config()` | 验收门这一项直接不成立 | 已实现 `FirstFrameGuard`，见 §8 |
| R4 | 关节限位硬编码，且等于**旧版 URDF 值**，小于最新手册 | 不会越限，但会提前撞软件边界 | 改成 `0x472→0x473` 回读，见 §7 |
| R5 | MIT `T_ref` 与 N·m 不是 1:1（注释：18.0 ≈ 8.0 N·m，即 1 ≈ 0.444 N·m） | 拿 T_ref 当真值会错 2.25 倍 | 只在阶段 2 当控制量，绝不当真值 |
| R6 | `effort` 不是单系数 K·I：**J1~J3 用 1.18125、J4~J6 用 0.95844**（单位 0.001 N·m） | 六关节 K 并不同值；这是厂家标称而非本机校准 | 阶段 2 用砝码验证，见下 |
| R7 | URDF 的 `velocity=5 rad/s`（≈286°/s）大于手册 180~225°/s；`effort=100` 也是占位 | 拿 URDF 当限速依据会超速 | 速度上限只认手册 + `0x473` 回读 |

**R6 值得单独说**：`piper_sdk` 的 `cal_effort()` 里，力矩 = 原始电流整数 × 系数，J1~J3 与 J4~J6 两组系数不同。折算下来 `K_sdk ≈ [1.18×3, 0.96×3] N·m/A`。这解释了为什么「六关节 K 相同」的直觉是错的，也提醒阶段 2 不能只标一个关节就推广——这组系数不含个体差异、减速比分解、效率与符号，只能当起点。

**R8（顺带记下）**：`piper_sdk 0.6.1` 里的接口名是 `GetArmStatus()`，**没有** `GetArmStatusMsgs()`；使能是 `EnableArm(7)` 走 `0x471`，不是 `0x15A`。指南里的示例代码和实测 SDK 有细微出入，探针脚本已按 0.6.1 的真实签名写，换版本必须重新核对。

## 7 限位：三方对照

| 关节 | 官方手册（最新） | URDF（humble） | `factr2_next` 硬编码 | 差 |
| --- | --- | --- | --- | --- |
| J1 | ±154°（±2.688） | ±2.618（150°） | 同 URDF | −4° |
| J2 | 0~195°（3.403） | 0~3.140（180°） | 同 URDF | −15° |
| J3 | −175°~0（−3.054） | −2.967（−170°） | 同 URDF | −5° |
| J4 | ±102°（±1.780） | ±1.745（100°） | 同 URDF | −2° |
| J5 | ±75°（±1.309） | ±1.220（70°） | 同 URDF | −5° |
| J6 | ±120°（±2.094） | ±2.0944 | 同 URDF | 0 |

结论：URDF 与代码是**旧版限位**，保守但可用；真实可达范围以 `0x473` 回读为准。另外 `rest_pos` 的第三关节取 `0.0`，正好压在 J3 上界上——回零动作贴边，阶段 1 采集前建议改成 `-0.05` 之类留余量。

## 8 首帧守卫

官方实现缺这段，所以自建。规则是「连续 10 帧同时满足：六关节角有限、落在 URDF 限位内、与上一帧间隔 ∈ [5 ms, 60 ms)」才放行；在此之前所有帧一律丢弃（它们可能被误当成保持目标）。

`FirstFrameGuard` 已随探针交付，并有离线自测（本机已跑）：

```
[selftest] A_clean_20ms     open=True at_frame=10 discarded=10
[selftest] B_duplicate_head open=True at_frame=14 discarded=14   # 前 5 帧 dt=0 全部作废
[selftest] C_out_of_range   open=True at_frame=18 discarded=18   # 越限帧重置累积
[selftest] D_gap_200ms      open=True at_frame=18 discarded=18   # 丢帧后重新累积
[selftest] PASS: 首帧守卫逻辑可用
```

诚实说明：这个守卫在 SDK 层只能判「读到的值可不可用」，**证明不了帧是新的**（SDK 内部有缓存）。真正的新鲜度必须靠 `--phase bus` 的原始时间戳。所以门里那一项的正确表述是「守卫已实现并通过离线自测，现场需复验」。

## 9 现场执行顺序

> 按现有机器（Ubuntu VM + Lyrical + Python 3.14 + gs_usb）改写后的完整版在 **`STAGE0_ACTIONS.md`**；这里只留骨架。

```bash
# 0) 建 3.12 venv（系统 3.14 装不了 numpy<2.0）
cd ~/20261019_stage0
bash env/freeze_robot_env.sh --with-hardware
source .venv-py3.12/bin/activate

# 1) 守卫自测（不接硬件）
python scripts/stage0_probe.py --phase selftest

# 2) 环境快照
python scripts/stage0_probe.py --phase env \
  --factr2-repo ~/factr2_next --out env/env_probe.json

# 3) 总线：只监听，不发帧（can0 已由直通+ip link 配好，1 Mbps）
python scripts/stage0_probe.py --phase bus --seconds 10 --out env/bus_probe.json

# 4) 只发配置/查询帧：从站确认 + 真实限位回读 + 首帧守卫留证
#    不发 0x471 使能（该臂上电自使能），不发任何运动帧
python scripts/stage0_probe.py --phase sdk --allow-write --query-limits \
  --factr2-repo ~/factr2_next --out env/sdk_probe.json

# 5) 把 env/*.json 里已核验的值抄回 manifest.json，改 gate_status
```
```

第 4 步不做任何位置/速度/力矩下发，只发使能、从站配置和一次 `0x472` 查询。注意官方 `piper_node` 一启动就会 `go_to_home_config()`，所以**不要**用 `ros2 launch` 做这一步。

## 10 门状态

按 `manifest.json:gate_status`（9 项，2026-10-19 补记后）：

| # | 验收项 | 状态 | 卡在哪 |
| --- | --- | --- | --- |
| 1 | piper_sdk 版本齐全 | 通过 | 0.6.1 + wheel sha256 |
| 2 | 官方实现版本齐全 | 通过 | `0df02c5b` |
| 3 | `source_commit.txt` 存在 | 通过 | 本目录 |
| 4 | 软件版本齐全（另一侧） | 通过 | — |
| 5 | 固件版本 | 未评估 | 该机 M10 从未做过，需现场读一次 |
| 6 | 从站模式确认 | 未评估 | 单臂反馈已 369 Hz，大概率「不适用」，仍需一次回读 |
| 7 | 首帧丢弃逻辑已体现 | **不通过** | 官方无此逻辑；探针已实现并离线自测，待现场留证 |
| 8 | 机械臂编号 / 铭牌 | 未评估 | 拍照 + 抄写，人工归档 |
| 9 | 真实限位 + 末端负载 | 未评估 | `0x473` 回读 + 称重 |

口径提醒：指引 §3 的门默认「官方代码里就有这些」，实测发现其中一项不成立。门的判据不变，但要按「自建 + 现场留证」来打，不能拿官方仓库当证据。

**怎么让它通过**：`STAGE0_ACTIONS.md` 的 A–D 四个动作，约 20 分钟，不需要改代码、不需要买东西。

## 11 下一步

开 L0 之前必须先定掉三件事，否则采集出来的数据要返工：

1. **原始层谁打戳**：官方驱动的时间戳不可用（R1），L0 采集节点必须直接读 CAN 帧并记录到达时间。这是阶段 1 的第一块代码。
2. **`dq` 用哪一路**：一个是电机测速（50 Hz、真实带宽），一个是位置差分（被量化）。选一个当 NEXT 输入，另一个留作对照，写进 manifest。
3. **速度上限**：等 `0x473` 回读的 `max_joint_spd` 出来，按 30% 起，逐级放开，别用 URDF 的 5 rad/s。

## 附：本目录文件

```
experiments/20261019_stage0/
├── manifest.json          # §2.3 登记表 + 配置冻结 + 7 个风险 + 门状态（机器可读）
├── source_commit.txt      # 官方实现 commit（首行纯哈希，可直接 git rev-parse 比对）
├── STAGE0_REPORT.md       # 本文件
├── STAGE0_ACTIONS.md      # 现场动作清单（按 FIRST_ARM 现有机器改写）
├── WHERE_TO_PUT.md        # 文件/命令落点对照：Windows 侧 vs VM 侧
├── env/
│   ├── requirements.in            # 依赖意图（官方 pin 照抄）
│   ├── freeze_robot_env.sh        # 机器人侧一键建环境 + 产 lock
│   └── dev_env_windows.json       # 本机环境快照（探针生成）
└── scripts/
    └── stage0_probe.py            # 只读探针：env / bus / sdk / api / selftest
```

第三方克隆（`experiments/_vendor/`，不入库）：`factr2_next@0df02c5b`、`piper_ros` 的 URDF、`piper_sdk-0.6.1` wheel 解包。
