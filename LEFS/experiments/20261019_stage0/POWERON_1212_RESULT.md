# 阶段 0 · 上电首采判读（2026-10-19 12:12）

> 来源：`bash env/poweron_collect.sh`（`stage0_probe 1.3-20261019`）
> 产物：`env/bus_probe.json`、`env/sdk_probe.json`、`env/prepower_snapshot/`（上电前失败证据已保住）
> 配套：[`STAGE0_ACTIONS.md`](STAGE0_ACTIONS.md) / [`manifest.json`](manifest.json)

---

## 1 一句话结论

**硬件侧通了。** 11:40 那次「`distinct_ids=1`、只有 `0x24`、TX 全部 `SEND_MESSAGE_FAILED`」的现象，上电后完全消失——这反证了当时的归因是对的：适配器没问题、波特率没问题，就是臂没应答导致 CAN 无 ACK。

现在 9 项门：**6 通过 + 1 待重跑 + 1 待定 + 1 待称重**。

---

## 2 关键实测数字

| 项 | 实测值 | 含义 |
| --- | --- | --- |
| 总帧数 / 错帧 | 31396 / **0**（10 s） | 约 3140 帧·s⁻¹，物理层干净 |
| `distinct_ids` | 25 | 11:40 只有 1 个 |
| `0x251~0x256` | 6/6，每路 n=1999 | 关节高速反馈齐 |
| `0x261~0x266` | 6/6，每路 n≈400 | 低速通道**不是 0**（机型差异判断要改） |
| `0x2A1~0x2A8` | 8/8，每路 n=1999 | **8 个 ID，不是 4 个** |
| 高速/状态周期 | median **5.00 ms**（p95 5.2 / max 13.5） | **200 Hz**，不是文档口径的 50 Hz |
| 低速周期 | median 25.0 ms | 40 Hz |
| 固件 | `S-V1.8-8` | 一次读到，不再返回 `-0x4AF` |
| 使能 | `all=True`，六轴全 True | 自使能，符合 FIRST_ARM 纪律 |
| 首帧守卫 | `opened=True, discarded=10`（baseline 1 + accumulate 9） | 真帧，**R18 的零值假阳性没复现** |

### 三个口径被推翻

**① 200 Hz 不是 50 Hz。** `stage1_l0.py` 原来按 `NOMINAL_DT=0.020` 算期望帧数、判丢帧。真机是 5 ms，差 4 倍——**真丢一半帧，旧判据也看不出来**。已改 `0.005`（v1.2），重跑 `selftest` 仍 `PASS 8/8`。

**② `0x2A8` 是夹爪，不是第七对关节。** 状态类 ID 共 8 个，但关节角仍只在 `0x2A5~0x2A7`。L0 已把它单列为 `GRIPPER_ID`，不再当成未知 ID 或第七轴。

**③ 低速通道 `0x261~0x266` 有数据。** 归档里记的是「1.5 kg 机型多为 0」，实测每路 40 Hz 正常在发。这条影响后面要不要用电流/母线做安全监测——**能用，别当没有**。

---

## 3 三件必须现在处理的事

### 3.1 `ctrl_mode` 是 `TEACHING_MODE(0x2)`，不是 `CAN_CTRL(0x1)`

```
ctrl_mode     : TEACHING_MODE(0x2)
arm_status    : NORMAL(0x0)
motion_status : REACH_TARGET_POS_SUCCESSFULLY(0x0)
teach_status  : STOP_RECORDING(0x2)
```

示教模式下 CAN 运动指令**很可能不被执行**。FACTR2 要的是程序控。若这一条不解决，L1 阶段的 `q_cmd` 会被静默忽略，训练出来的是 `q_obs → q_obs` 的恒等映射：**loss 很低、验证集很漂亮、机器人完全不动**。这是这类复现里最贵的坑，必须在 L1 之前堵掉。

### 3.2 我们把它留在了从站过渡态

`0xFC` 是「切从站（motion output）」，官方流程要 **四步**：

```
MasterSlaveConfig(0xFC,0,0,0)            # 切从站
sleep 0.3
MotionCtrl_1(0x02, 0, 0)                 # 复位 ← 臂可能掉落
sleep 0.6
MotionCtrl_2(0x01, 0x01, 100, 0x00)      # 位置-速度模式
```

脚本只发了第一步就停了，0.2 s 后读 `ctrl_mode` 得 `None`。**这是脚本的设计错误，不是硬件问题**：阶段 0 只做静态采集，本不需要切模式，却用一个「复位期间臂可能掉落」的动作去换一个只读就能得到的结论。

已改为**默认不发**，只有 `--set-slave` 显式开启。同时把归因也改了——原来写「帧发出去没回读 = 臂未上电」，但同一份 json 里固件/使能/守卫全部读到了，那种归因会把人带去查电源和接线。

### 3.3 限位一项没查成，原因是接口名写错

`hasattr(piper, "GetCurrentMotorAngleLimitMaxVel")` 为 `False` → `if` 不成立 → **无异常、无日志、静默跳过**，报告只显示「未查询」。

0.6.1 的正确接口：

```python
piper.SearchAllMotorMaxAngleSpd()          # 发 0x472
raw = piper.GetAllMotorAngleLimitMaxSpd()  # 读 0x473
```

而且 `max_joint_spd` 的单位**随固件换档**：V1.5-8 之前 `0.001 rad/s`，之后 `0.01 rad/s`。本机 `S-V1.8-8` 属后者，原代码 `/1000.0` 会小 10 倍。这三处都已在 `stage0_probe 1.4` 修掉，并保留 `spd_raw` 原值 + `raw_repr` 原文，解析失败也不会再无据可查。

---

## 4 两个「不致命但要知道」的观察

**`can0` 显示 `ERROR-WARNING`，`error_frames=0`。** 这个窗口内没有错帧，`ERROR-WARNING` 大概率是上电前那几百次 TX 无 ACK 累积的 TEC 残留。判断方法：`ip -details -statistics link show can0` 看 `berr-counter`——TEC 回落就无事，持续高位就查 120Ω 终端与线序。

**`sample_point=0.75`。** 这是 `ip link` 的默认值；CiA 在 1 Mbps 推荐 0.875。当前 0 错帧说明可用，但裕度不如推荐值，长线或多节点时先怀疑这里。

**另外 3.2% 的帧是未知 ID**：`0x1c0~0x1c3`（载荷全 0）与 `0x212=04010f02`，合计约 1010 帧/10 s。不影响解码，但会污染「丢帧率」分母。L0 已按「全记 + 标 `F_UNKNOWN_ID` + 解析层不计入关节」处理。

---

## 5 剩下三步

### 第一步：重跑限位（1 分钟，无副作用）

```bash
# 【VM】cwd：~/stage0/20261019_stage0  （先激活 venv）
python scripts/stage0_probe.py --version          # 期望 1.4-20261019
python scripts/stage0_probe.py --phase sdk --allow-write --query-limits
```

拿到 `hardware_limits` 后，六个关节的真实 `min/max/max_spd` 直接写进 L0 的 `JOINT_MIN/MAX`，**不再用 URDF 的标称值当安全上限**。

### 第二步：确认模式（有副作用，人必须在旁边）

先把臂摆到**不容易砸东西的中位姿态、示教/夹爪不承重**，手放急停上：

```bash
python scripts/stage0_probe.py --phase sdk --allow-write --set-slave
```

它走完整四步，并回读 `ctrl_mode`。若想更保守：**先断电重启**回到干净上电状态，再跑这一条。

### 第三步：称末端（5 分钟，不用电）

夹爪 + 相机 + 线缆一起拆下来称 kg。1.5 kg 是含工具的上限，后面每加一个传感器都从这里扣。

---

## 6 门状态一览

| # | 门项 | 状态 | 证据 |
| --- | --- | --- | --- |
| 1 | 固件版本 | ✅ 通过 | `S-V1.8-8` |
| 2 | piper_sdk | ✅ 通过 | `0.6.1` + wheel sha256 |
| 3 | 官方实现 | ✅ 通过 | `0df02c5b` |
| 4 | `source_commit.txt` | ✅ 通过 | 本目录 |
| 5 | 从站模式 | ⚠️ 待定 | 发过 `0xFC` 未 reset |
| 6 | 首帧守卫 | ✅ 通过 | `discarded=10`，真帧非零值 |
| 7 | 铭牌 / 序列号 | ✅ 通过 | `MA100101000019006400985` |
| 8 | 真实限位 | ❌ 待重跑 | 接口名错，已修 |
| 9 | 末端总重 | ⬜ 未评估 | — |

**「阶段 0 通过」的判据**：第 5 项转为「通过」且第 8、9 项补齐。按上面三步做，一次可达。
