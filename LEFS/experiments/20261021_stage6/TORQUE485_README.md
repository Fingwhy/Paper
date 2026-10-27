# 力矩传感器接入（VYJN1-5Nm + CYPY-I-MBUS）

本机（Windows）目录：`LEFS/experiments/20261021_stage6/`
依赖：`pip install pyserial pymodbus`

## 硬件链路

```
VYJN1-5Nm（1.5 mV/V 应变桥，模拟量）
   ↓ 四线（EX+ / EX− / SIG+ / SIG−）
CYPY-I-MBUS 五位显示仪（自带 5 V±5% 激励，220V AC 供电）
   ↓ RS485：仪表 A(5脚) / B(6脚)
USB-485 转接线 → COM3
```

传感器本体输出的是 **1.5 mV/V 的模拟信号**，任何串口都读不了，485 一定是从仪表出来的。

## 已确认的通讯参数（实测，非手册默认）

| 项 | 值 |
|---|---|
| 串口 | COM3（FTDI，VID:PID 0403:6001） |
| 波特率 / 校验 | 115200 / N 8 1 |
| 从站地址 | 1 |
| 通讯方式 | Modbus 从站（不主动发数据） |

## 寄存器

| 地址 | 内容 | 说明 |
|---|---|---|
| 0x54 | 小数点位置 | 本机 = 3（三位小数） |
| 0x55 | 符号 | 0 = 正，1 = 负 |
| 0x56 | 显示值 | **只有幅值 0~60000** |

一次读 `0x54` 起 3 个寄存器，符号必须单独取 —— 只读 0x56 会把负力矩读成大正数。

## 常用命令

```bash
# 0) 换过参数/换过机器时，先扫一遍
python torque485.py --port COM3 --max-addr 8 scan
python torque485.py --port COM3 sniff          # 判断是不是 CNT 连续发射模式

# 1) 看寄存器全貌
python torque485.py --port COM3 --baud 115200 --addr 1 --count 96 dump

# 2) 连续读数（打印 原始值/符号/力矩）
python torque485.py --port COM3 --baud 115200 --addr 1 --count-reads 20 cypy

# 3) 变化监视：只打印数值改变的时刻，最后给统计
#    用途：判断「读数很稳」是真稳，还是信号根本没进仪表
python torque485.py --port COM3 --baud 115200 --addr 1 --seconds 120 watch

# 4) 落盘记录（拧盖同步对账用这个）
python torque485.py --port COM3 --baud 115200 --addr 1 --interval 0 --seconds 60 --out run1.jsonl log

# 5) 标定：交互式输入已知力矩，做线性拟合，结果存 json
python torque485.py --port COM3 --baud 115200 --addr 1 --span 5 --save calib.json calib
```

`--span 5` 是传感器量程；`--full-display` 是「满量程对应的显示值」，默认 5.0
（即 1 count = 0.001 N·m）。**这两个只有标定后才能当真。**

## 已知性能（实测）

| 指标 | 值 |
|---|---|
| 零位抖动 | ±0.006 N·m |
| 解析度 | 0.001 N·m（1 count） |
| 手拧观测范围 | −0.170 ~ +0.335 N·m |
| 最大轮询率 | **≈ 50 Hz**（间隔 0 时 49.9 Hz，0 丢帧） |
| 静止读数 | raw=5 → 0.005 N·m（托一下归零，是真实微小重力矩） |

对比：J6 电流单帧慢漂折合 **0.852 N·m**，本传感器 **0.006 N·m**，好约 140 倍。

## 待做

1. **标定**：砝码 × 力臂，至少两个点（建议 0.2 / 0.5 / 1.0 N·m）。
2. **对账 K6**：传感器串在 J6 与夹爪之间，同一动作同时记录「传感器力矩」和「J6 电流」，
   验证 `K6 = 10.7882`（全项目唯一没有外部基准的数字）。
