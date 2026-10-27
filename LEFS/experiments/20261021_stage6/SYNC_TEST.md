# 力矩精度对照实验（传感器 × 电流 × NEXT）

**目的**：把三个力矩量放在同一条时间轴上比较——这是论文核心主张的直接检验。

| 量 | 来源 | 含义 |
|---|---|---|
| `τ_sens` | **传感器实测**（COM3 → CYPY 仪表） | 真值（噪声 0.006 N·m） |
| `τ_m = K6·i6` | 机械臂电流 | 电流推算的电机力矩（K6 = 10.7882） |
| `τ̂_ext = τ_m − τ̂_f` | NEXT 模型 | 论文的「外力矩估计」 |

---

## 一、机械安装（关键）

传感器串在 **J6 法兰与夹爪之间**，让它的测量轴与 **J6 转轴同轴**。

```
机械臂末端法兰 → [力矩传感器] → 夹爪 → 物体
                      ↑
                 测的就是 J6 这一轴的扭矩
```

**为什么必须同轴**：VYJN1-5Nm 是单轴反扭矩传感器，只测绕自己轴线的扭矩。
装偏了测到的不是 J6 力矩，三个量就没法比。

**量程匹配**：传感器 ±5 N·m；拧盖实测力矩 0~4 N·m —— 用掉 80% 量程，正好。

---

## 二、两边同时记录

传感器在 Windows（COM3），机械臂在 VM（can0），分两台机器记，再用互相关对齐。

### VM 侧（先启动，记 60 s）

```bash
cd ~/stage0/20261019_stage1_l0
mkdir -p ~/stage0/20261021_stage6/raw
python3 -c "import time;open('/home/zzz/stage0/20261021_stage6/raw/SYNC1.t0','w').write(str(time.time()))"
python3 stage1_l0.py record --iface can0 --seconds 60 \
    --out ~/stage0/20261021_stage6/raw/SYNC1.jsonl
```

### Windows 侧（紧接着启动）

```bash
cd "C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6"
python sensor_rec.py --port COM3 --seconds 60 --out runs/sync1_sensor.jsonl
```

两边起点差几秒没关系，后面用互相关自动找。

### 把臂侧解析成 50 Hz 样本（VM）

```bash
cd ~/stage0/20261019_stage1_l0
rm -rf ~/stage0/20261021_stage6/episodes/SYNC1
python3 stage1_l0.py parse ~/stage0/20261021_stage6/raw/SYNC1.jsonl \
    --hz 50 --pos251-unit mrad \
    --out ~/stage0/20261021_stage6/episodes/SYNC1
cat ~/stage0/20261021_stage6/raw/SYNC1.t0
```

**`--pos251-unit mrad` 不能省**：NEXT 训练用的是弧度，漏了这参数 q 会变成角度，
NEXT 预测全废（脚本会打印警告）。

再把 `episodes/SYNC1/samples.jsonl` 拷回 Windows。

---

## 三、对比

```bash
cd "C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6"

# 只比 K·I（不需要 NEXT）
python compare_torque.py \
    --sensor runs/sync1_sensor.jsonl \
    --samples ../20261020_stage4/episodes/SYNC1/samples.jsonl \
    --arm-t0 <SYNC1.t0 的内容> \
    --out runs/cmp_sync1.json --svg runs/cmp_sync1.svg

# 加上 NEXT 预测
python compare_torque.py ... --ckpt ../20261020_stage4/runs/next_v9_h75/model.pt
```

输出：

- 对齐结果（臂侧起点 epoch、互相关滞后、相关系数）
- 符号判定（接线反向会让 τ_sens 整体反号，脚本取 |r| 最大的符号）
- 两行统计：`K·I vs 传感器`、`NEXT τ̂_ext vs 传感器`（r / MAE / bias）
- **线性拟合 `τ_sens = K·i6 + b`** → `K` 就是实测的力矩常数，`b` 是摩擦
- SVG 三曲线叠图

---

## 四、三个可选实验，按价值排序

### A. 拧瓶盖（最能说明问题）

跑现有拧盖动作（`s6_k6_calib.py` / `stage1_cmd.py`），力矩在 0~4 N·m 变化、有 stick-slip。

- **看点 1**：拟合出的 `K` 与 10.7882 差多少 → 这是全项目唯一没有外部基准的数字，
  自这次起变成实测
- **看点 2**：`b`（摩擦）是否随姿态/速度变 → 现在用的是常数 0.86
- **看点 3**：`τ̂_ext` 与传感器差多少。**注意这里 NEXT 是分布外**
  （J1≈150°、J6≈170°），预期很差——这本身就是结论：论文方法不能外推到新位形

### B. 静态扭矩阶梯（最干净的 K6 标定）

保持姿态不动，**用手或用杠杆依次施加 0 / 1 / 2 / 3 / 4 N·m**，每个级别停 3 秒反转方向再来一遍。

- 正反两组给出真实的摩擦（截距）和 K6（斜率），而且**不依赖任何运动模型**
- 比原来的正反转差分强得多：那时 0.18 N·m 只需要 0.017 A，淹没在 0.03 A 噪声里；
  现在 4 N·m 需要 0.37 A，信号足够

### C. 称重姿态（验证质心解算）

在标准称重姿态（`POSE_FWD`）下挂重物，传感器测到的是 `m·g·d`
（`d` = 质心到 J6 轴线的垂直距离）。

- 这不是「重量」本身，而是**独立的一路测量**
- 用途：把已解出的 `(m, Φ)` 代进去预测 `τ6`，与传感器实测对比 ——
  对得上说明质心可信，对不上说明 Φ 错了

---

## 五、已做过的验证

用一段真实拧盖数据合成「已知真值」的传感器轨迹（真值 K6=10.7882、摩擦 0.860、
噪声 0.006 N·m、人为制造 +2.5 s 启动偏差），跑完整流程：

| 项 | 结果 |
|---|---|
| 对齐 | 找到 −2.500 s，臂侧起点 epoch 与真值**完全一致** |
| 反解 K6 | **10.7832**（真值 10.7882，偏 −0.05%） |
| 反解摩擦 | **0.8599**（真值 0.860） |
| 拟合残差 | **0.0055 N·m** ≈ 传感器噪声 0.006 |

⇒ 对齐、符号判定、K 反解三处逻辑都正确。

---

## 六、三个容易踩的坑

1. **传感器接线反了**：τ_sens 整体反号，相关系数会很差。脚本自动取 |r| 最大的符号，
   但要在报告里写清楚是哪个符号。
2. **`samples.jsonl` 没带 `--pos251-unit mrad`**：q 是角度、NEXT 要弧度，预测全废。
   脚本检测到 |q|>10 会警告。
3. **`K·I vs 传感器` 的 MAE 不等于「误差」**：`K·I` 包含摩擦项，传感器不包含，
   所以固定差一个常数（约 0.86 N·m）。**要看拟合后的斜率与残差，不要看 MAE。**
