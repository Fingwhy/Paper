# 阶段 1（L0）：信号链与同步 —— 计划与操作规程

> **当前状态**：软件侧完成。`stage1_l0.py selftest` → `PASS 8/8`，不需要硬件。
> **真机部分**：等阶段 0 解除挂起（臂上电 + `bash env/poweron_collect.sh` 通过）后再跑。
> **一句话**：L0 只证明一件事 ——「我落盘的时间序列，可以当作物理量使用」。

---

## 0 为什么不能跳过 L0

| 如果 L0 没做 | 到了 L1（训练）会怎样 | 到了 L2（接触）会怎样 |
| --- | --- | --- |
| 不知道 `0x251` 位置是 rad 还是 0.001 rad | 观测尺度差 1000 倍，loss 直接 NaN 或学到常数 | 力估计全错，但**看起来像**「模型不行」 |
| 不知道帧间隔的真实分布 | 把 200 ms 的空档当 20 ms，速度由差分算出来放大 10 倍 | 接触瞬间的微分项爆掉 |
| 缺帧被静默填零 | 训练集里混入「关节瞬移到 0」的样本，策略学会抽搐 | 误判为碰撞，反复触发保护 |
| 用 `q` 顶替 `q_cmd` | 学成 `q_obs → q_obs` 的恒等映射，验证集 loss 很低但完全不响应指令 | 力控环路退化成位置环 |

这四条都是「事后无法补救」的：原始层写坏一次，那一天的数据全废。所以 L0 的产出不是模型，是**能被信任的数据管道**。

---

## 1 软件侧已完成（`stage1_l0.py`，v1.2）

> **12:12 上电首采后校订了三处口径**（详见 `../20261019_stage0/POWERON_1212_RESULT.md`）。
> 这三条不是文档笔误，是照文档抄、被真机推翻的：
>
> | 项 | v1.1（照文档） | 真机实测 | 影响 |
> | --- | --- | --- | --- |
> | 源帧周期 | 20 ms / 50 Hz | **5.00 ms / 200 Hz**（p95 5.2，max 13.5） | 丢帧判据偏松 4 倍，真丢一半也看不出来 |
> | 状态类 ID | `0x2A1` 一个 | **`0x2A1~0x2A8` 八个**，其中 `0x2A8` 是夹爪 | 会把夹爪当第七对关节 |
> | 低速通道 | 1.5 kg 机型多为 0 | 6/6 全在发，**25 ms / 40 Hz** | 别把电流监测当「没有」 |
>
> 另有一条实测事实：总线上还有 3.2% 的未知 ID（`0x1c0~0x1c3` 载荷全 0、`0x212=04010f02`）。
> 原始层照记并打 `F_UNKNOWN_ID`，解析层不计入关节——**不静默丢弃，也不混进分母**。

纯标准库 + `python-can`，一行 numpy 都不用 —— 因此能在阶段 0 的 `--minimal` 环境（无 numpy）里直接跑。

| # | 用例 | 注入什么 | 期望行为 | 实测 |
| --- | --- | --- | --- | --- |
| A | 正常流 | 6 高速 + 3 角度 + 1 状态 @200 Hz（真机实测口径） | 核心标记为 0；`q_cmd` 全 null 且带 `no_q_cmd` | ✅ 100/100 干净，100% 齐备 |
| B | 缺关节 | 不发 `0x255`，同时不发承载它的 `0x2A7` | `q5`/`q6` 保持 null，打 `missing:q5` + `missing:hs5` | ✅ 三者全中 |
| C | 乱序 | 第 30 周期时间戳倒退 40 ms | 原始层打 `F_OUT_OF_ORDER`，**且不重排** | ✅ 原始层保持到达顺序 |
| D | 饱和 | `0x2A6` 里 J3 的 int32 置 `0x7FFFFFFF` | 值被拒收进 `q_rejected`，`q3`=null，打 `oob:q3` | ✅ 37480.66 rad 被拦下 |
| E | 短帧 | `0x255` 只发 5 字节 | 原始层打 `F_SHORT_DLC`，解析层跳过 | ✅ 50 帧被标记 |
| F | 单位偏差 | 按 0.001 rad 编码，分别按 `mrad`/`rad` 解析 | 比值 1.00 / 1000.6，后者判 `unit_suspect` | ✅ 双向都对 |
| G | 大间隔 | 1.0 s 处断 150 ms | 报告判「未发现 >100 ms 间隔：否」 | ✅ 6 个样本受影响 |
| H | 有 q_cmd | 同一时间基准的指令 CSV | `q_cmd` 非 null，且不误判 `stale:q_cmd` | ✅ |

**自测发现并已修掉的 3 个真实缺陷**（不是测试用例的问题，是工具的问题）：

1. **高速通道整路缺失时没有任何标记** —— 1.0 版 `if hit is not None:` 没有 else 分支。总线只通了一半（只有 `0x2A5~7`、没有 `0x251~6`）时，报告仍显示「六轴齐备 100%」。已补 `missing:hs{j}`。
2. **越界值被写进了 `q`** —— 饱和时只是打个 `oob` 标记，值照样进 `q` 字段，下游忽略 flags 就会拿到 37480 rad。已改为：越界值只进 `q_rejected`，`q` 留 null。
3. **`0x251` 位置字段全 0 被误报成「单位选错」** —— 真机上这更可能是「该路位置没标定」。两者处置完全不同（前者改参数、后者换数据源），已拆成 `pos251_unit_suspect` 与 `pos251_all_zero` 两个独立判据。

---

## 2 数据流

```
CAN 帧 ──► raw/*.jsonl ──────────────► parse ──► samples.jsonl
           只存事实（t/id/d/f）         │        sync_report.md
           一帧一行，不解释任何字节      │        metrics.json
           绝不重排、绝不改值            │
                                        └─► q_cmd CSV（另行采集，见 §6.3）
```

原始层一行长这样（`d` 是 8 字节的 hex，`f` 是异常位）：

```
{"t":1000000.02,"id":593,"d":"000a0096000006","f":0}
```

`id` 用十进制存（`593` = `0x251`），避免解析方在 hex/dec 上再踩一次坑。

---

## 3 产物与字段口径

| 字段 | 来源 | 单位 | 缺失时 |
| --- | --- | --- | --- |
| `q[0..5]` | `0x2A5`/`0x2A6`/`0x2A7`，int32 × 0.001 度 | rad | `null` + `missing:q{j}` / `stale:q{j}` / `oob:q{j}` |
| `dq[0..5]` | `0x251~0x256` 字节 0-1，int16 × 0.001 | rad/s | `null` + `missing:hs{j}` |
| `cur[0..5]` | `0x251~0x256` 字节 2-3，uint16 × 0.001 | A（符号未定，见 §6.2） | `null` |
| `q_cmd[0..5]` | 外部 CSV，**不插值** | rad | `null` + `no_q_cmd` / `cmd_interrupted` / `stale:q_cmd` |
| `age_ms` | 每路字段的年龄 | ms | — |
| `span_ms` | 六路高速帧年龄的**极差**（同一时刻六个关节的到达分散度） | ms | 任一路缺 → `null` |
| `q_rejected` | 被拒收的越界值 | rad | 字段不出现 |
| `arm_status` | `0x2A1` 字节 1 | 原始码 | `null` + `missing:status`，非 0 时打 `protection:0xNN` |

**`span_ms` 是这一阶段最容易被忽略、但最关键的量**：它不是帧间隔，而是「我拿来做同一个控制周期快照的六个关节值，最老和最新差了多少」。源周期是 5 ms（12:12 实测），所以正常值应该 < 10 ms；一旦接近 20 ms，说明六轴其实是错开好几个周期采的，`q` 不是同一时刻的姿态。

### 标记分类

| 类别 | 标记 | 含义 |
| --- | --- | --- |
| 提示 | `no_q_cmd` | 没给指令 CSV。**不算脏**，判干净样本时必须排除，否则 L0-a 会 100% 被判脏 |
| 缺 | `missing:q{j}` / `missing:hs{j}` / `missing:status` | 整路从未收到 |
| 旧 | `stale:q{j}` / `stale:hs{j}` / `stale:q_cmd` | 收到过，但超过 `--stale-ms`（默认 60 ms = 3 个周期） |
| 越界 | `oob:q{j}` | 超出 URDF 区间，值被拒收 |
| 饱和 | `saturated:cur{j}` | `cur_raw = 0xFFFF` |
| 保护 | `protection:0xNN` | `arm_status ≠ 0` |
| 指令 | `cmd_interrupted` | 指令流在样本时刻之前没有记录 |

---

## 4 真机压测剧本（臂上电后）

分两档，**L0-a 现在就能验收，L0-b 需要一个还没写的下发脚本**。

### L0-a 纯被动（不需要下发、不需要能动）

| 段 | 时长 | 动作 | 验什么 |
| --- | --- | --- | --- |
| 1 | 0–60 s | 上电后**不动**，保持 STANDBY | 冷启阶段是否有 ID 迟到、`0x2A1` 是否稳定 |
| 2 | 60–660 s | 继续静止 | 10 分钟稳态：丢帧、周期抖动、时间戳跳变 |

```bash
# 目录：~/stage0/20261019_stage1_l0
cd ~/stage0/20261019_stage1_l0 && source ~/stage0/20261019_stage0/.venv-py3.12/bin/activate
python stage1_l0.py record --iface can0 --seconds 660 --out raw/l0a_static.jsonl
python stage1_l0.py parse raw/l0a_static.jsonl --hz 50 --out episodes/l0a_static
```

> **磁盘**：10 分钟原始层约 16 MB（30 万帧 × ~55 B），`samples.jsonl` 约 6 MB。VM 剩 1009 MB，够录 40 次，但别重复录。
> **时间戳**：`msg.timestamp` 是 epoch（`CLOCK_REALTIME`）。录制前别让它被 NTP 步进 —— 真跳了会以 `F_OUT_OF_ORDER` 的形式暴露出来，这正是不重排原始层的用处。

### L0-b 主动低速（**前置条件：先写 `stage1_cmd.py`**）

匀速正弦，单关节、小幅、慢速，每轴 30 s：

```
J1 ±0.15 rad @ 0.1 Hz    J4 ±0.15 rad @ 0.1 Hz
J2 ±0.15 rad @ 0.1 Hz    J5 ±0.10 rad @ 0.1 Hz
J3 ±0.15 rad @ 0.1 Hz    J6 ±0.20 rad @ 0.1 Hz
```

下发的同时把 `(t, q_cmd[6])` 写成 CSV，**同一时间基准**（epoch 秒），再：

```bash
python stage1_l0.py parse raw/l0b_sine.jsonl --hz 50 --qcmd cmd/l0b_sine.csv --out episodes/l0b_sine
```

**为什么 L0-b 不能省**：L0-a 的 `q_cmd` 全是 null，验不了「指令与观测对不对得上」。而 FACTR 2 的动作条件就是 `q_cmd`，一旦它和 `q` 差一个常量偏置或一个周期延迟，L1 会把延迟学进策略里。

---

## 5 验收判据（L0-a，全部在 `metrics.json` 里）

| 判据 | 阈值 | 依据 |
| --- | --- | --- |
| 六个 `0x25x` 周期中位数 | 5.0 ± 1 ms（200 Hz，12:12 实测 median 5.00 / p95 5.2） | `verdict.cycle_ok` |
| 10 分钟内无 >100 ms 间隔 | 计数 = 0 | `verdict.no_gap_gt100ms` |
| 六轴齐备样本率 | ≥ 99% | `verdict.q_rate_ge_99pc` |
| `span_ms` p95 | ≤ 10 ms（**人工看**） | 报告表格，超了说明六轴不同步 |
| `arm_status` | 全程 = 0 | 无 `protection:*` 标记 |
| 原始层帧数 | 660 s ×（11 路 × 200 Hz + 6 路 × 40 Hz）≈ **1.6 M 帧 / 约 75 MB** | 与 `n_raw_frames` 对账 |
| `pos251_all_zero` | 若为 true，记录在案 | 决定 `0x251` 位置能否用作 `q` 的第二来源 |

L0-b 追加：

| 判据 | 阈值 |
| --- | --- |
| `q_cmd → q` 互相关峰值滞后 | ≤ 2 个控制周期（40 ms） |
| 匀速段跟踪误差 RMS | ≤ 0.01 rad（**首轮实测后再定**，现在是估值） |
| `cmd_interrupted` | 0 次 |

---

## 6 三个必须先定的口径

### 6.1 `0x251` 的位置单位（R12，未定）

指南写 rad，FIRST_ARM 逐字节实测记 0.001 rad。工具已经能**自己判**：`ratio_251_over_2a5` ≈ 1 就是 rad，≈ 1000 就是 0.001 rad，判完写进 `metrics.json`。

**注意**：如果 `pos251_all_zero = true`，说明这一路位置字段根本没在输出有效值 —— 那不是单位问题，是「这路不可用」，`q` 只能继续取 `0x2A5~0x2A7`。第一次真机跑完请把这两个字段贴给我。

> 12:12 首采已用合成帧验证：`sine` 轨迹跑完后这两个判据都能稳定给出结论。等 L0-a 真机数据回来即可定稿。

### 6.2 `cur` 的符号（未定）

`0x251~0x256` 字节 2-3 的电流是 uint16，符号位在哪一段没定。工具同时保留 `cur_raw` 与 `cur`，**不猜**。定标需要一段「已知负重、单向运动」的数据，属于 L2 的活，但原始层从 L0 就开始存，不会丢。

### 6.3 `q_cmd` 从哪来（**已解决**：方案 A，`stage1_cmd.py`）

`record` 只收帧、不下发，所以没有指令层时 `q_cmd` 全 null。已按方案 A 落地 `stage1_cmd.py`：

```
python stage1_cmd.py hold|sine|step --iface can0 --out cmd/xxx [--dry-run]
```

三条硬规矩写在文件头，这里只记**为什么**：

1. **记录"发出去的值"，不是"算出来的值"**。夹紧、限幅之后真正写进 CAN 的那一帧才进 CSV。这条规矩让 `--max-step-deg` 的夹紧次数变成可见数字——正是它把 1.0 版 sine 的**首拍 4.33° 突跳**（相位偏移导致 t=0 不在零位）暴露出来。没有它，那一下会直接发给臂。
2. **掉拍不补点**。少一行就是少一次真实下发；补点等于伪造机器从未收到的指令。
3. **模式不对就不发**。`ctrl_mode != CAN_CTRL(0x1)` 直接中止（`--force-mode` 才能越过）。这一条直接对着 R19：示教模式下跑完只会得到一份"看着很成功"的废数据。

产物：`<out>.csv`（`t_wall,j1..j6`，rad，无表头，`parse --qcmd` 直接吃）+ `<out>.meta.json`（轨迹参数、calls/clamp/skip 计数、模式、是否被中断）。

已用合成链路验证：dry-run 生成 CSV → `parse --qcmd` 消费 → **250/250 样本有 `q_cmd`、零 `stale`**。

**还没验的**：真机下"`q_cmd` 变化而 `q` 不动"这种情况（R19 的判据）。这必须在 `--set-slave` 之后、用 `hold` 跑一次才能确认。

---

## 7 命令（每条只有一个正确答案）

> **粘贴规矩**：`【Windows】` 段只能在 PowerShell 里敲，`【VM】` 段只能在 Ubuntu 终端里敲。
> 两段连着复制会出现 `scp: usage:` 或 `python: can't open file` —— 13:07 就发生过一次。
> 下面 VM 段每行都以 `&&` 结尾，即使粘贴时断了行也不会变成一条错命令。

### 【Windows】

```powershell
# W-1 上传（我每次改完都要重传）
scp "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\tmp\stage1_bundle.tar.gz" zzz@192.168.242.128:~/

# W-2 跑完把产物拉回来
scp -r "zzz@192.168.242.128:~/stage0/20261019_stage1_l0/episodes" "C:\Users\asus\WorkBuddy\论文解析复现\LEFS\experiments\20261019_stage1_l0\"
```

### 【VM】

```bash
# V-1 + V-2  解包并自测（不需要臂、不需要 can0），期望 PASS 8/8
mkdir -p ~/stage0 && tar -xzf ~/stage1_bundle.tar.gz -C ~/stage0 &&
cd ~/stage0/20261019_stage1_l0 && source ~/stage0/20261019_stage0/.venv-py3.12/bin/activate &&
python stage1_l0.py selftest &&
python stage1_cmd.py hold --dry-run --duration 3 --out cmd/smoke

# V-3  目录：同上 —— 确认 can0 在（阶段 0 已做过，重开机要做）
ip -details link show can0 | head -3

# V-4  目录：同上 —— L0-a 压测（臂上电、静止）
python stage1_l0.py record --iface can0 --seconds 660 --out raw/l0a_static.jsonl

# V-5  目录：同上 —— 解析（--hz 只是输出采样率，原始层是 200 Hz，随时可改 --hz 200 重跑，不丢数据）
python stage1_l0.py parse raw/l0a_static.jsonl --hz 50 --out episodes/l0a_static

# V-6  目录：同上 —— 把判据贴给我
cat episodes/l0a_static/metrics.json

# ---------- L0-b：指令层（必须先切到程序控，见阶段 0 §0.9 第 ② 步）----------

# V-7  目录：同上 —— 先干跑，不连 CAN，验证 CSV 生成链路
python stage1_cmd.py hold --dry-run --duration 3 --out cmd/smoke
head -2 cmd/smoke.csv          # 期望 t_wall,j1..j6 六列 rad

# V-8  目录：同上 —— 真下发之前，先只保持当前姿态 10 s（最温和的一次运动）
#      人要站在急停旁；--limits 有就用真的，没有会明确警告只做增量限制
python stage1_cmd.py hold --iface can0 --duration 10 \
    --limits ../20261019_stage0/env/joint_limits.json --out cmd/l0b_hold

# V-9  目录：同上 —— 低速正弦，验 q_cmd → q 的延迟与跟踪
python stage1_cmd.py sine --iface can0 --duration 60 --hz 100 \
    --limits ../20261019_stage0/env/joint_limits.json \
    --amp-deg 5 --freq-hz 0.25 --out cmd/l0b_sine5

# V-10 目录：同上 —— 边下发边记录，用同一份 CSV 对齐
#      （record 与 cmd 必须几乎同时起，时间基准都是 epoch 秒，差几秒也只是前段无 q_cmd）
python stage1_l0.py record --iface can0 --seconds 65 --out raw/l0b_sine5.jsonl &
python stage1_cmd.py sine --iface can0 --duration 60 --out cmd/l0b_sine5
python stage1_l0.py parse raw/l0b_sine5.jsonl --hz 50 \
    --qcmd cmd/l0b_sine5.csv --out episodes/l0b_sine5
```

> **V-10 的判据**（这一条能直接判掉 R19）：
> - `q` 跟着 `q_cmd` 动 → 指令生效，模式没问题；
> - `q` 纹丝不动而 `q_cmd` 在扫 → 指令被忽略，回阶段 0 §0.9 第 ② 步查模式，**不要**去调训练代码；
> - `q` 动但明显滞后 → 记下互相关峰值偏移，那就是这条链路的真实延迟，L1 的时序对齐用它。

> V-2 里 `source ...venv` 那一行是**有状态的**，新开终端就要重跑，否则 `python` 报 `command not found`（Ubuntu 26.04 没有 `python` 别名，只有 `python3`）。

---

## 8 出错对照

| 现象 | 含义 | 处置 |
| --- | --- | --- |
| `ModuleNotFoundError: can` | 没激活阶段 0 的 venv | 回 V-2 的 `source` 那行 |
| `CanInitializationError` / `Cannot find device "can0"` | can0 没 up | `sudo ip link set can0 up type can bitrate 1000000` |
| `[record] 一帧都没收到` | 臂没上电，或只收到 `0x24` 心跳 | 回阶段 0 §3 的三步定位表 |
| `[record] 以下高速反馈 ID 一帧未收到：['0x255']` | 该关节的反馈没出来 | 先看是否 `0x255` 单独缺失（固件问题），再看 `0x2A7` 是否也缺 |
| 报告里 `有效样本率 ≥99%：否` | 真丢帧 / 六轴不同步 | 看「各序列到达间隔」表，找出是哪一路 |
| `比值 = 1000.x` 且 `pos251_unit_suspect: true` | 单位选错了 | 加 `--pos251-unit mrad` 重解析（原始层不用重录） |
| `pos251_all_zero: true` | 该路位置没标定 | 记录在案，`q` 继续取 `0x2A5~0x2A7` |
| `stale:q_cmd` 满屏 | CSV 与 CAN 不是同一时间基准 | CSV 必须用 epoch 秒（`msg.timestamp` 同源） |
| 满屏 `no_q_cmd` | 没给 `--qcmd` | L0-a 的正常现象，不算故障 |

---

## 9 还没做的（按优先级）

| # | 事项 | 属于 | 前置 |
| --- | --- | --- | --- |
| 1 | `stage1_cmd.py`（下发 + 记录 q_cmd） | L0-b | 臂上电 |
| 2 | `--limits` 从 `0x473` 回读覆盖硬编码区间 | L1 | 阶段 0 的 `--query-limits` 出结果 |
| 3 | `cur` 符号定标 | L2 | 已知负重数据 |
| 4 | 原始层分卷（>1 GB 时按 100 MB 滚动） | 长期 | 现在 16 MB/10 min，还不需要 |
