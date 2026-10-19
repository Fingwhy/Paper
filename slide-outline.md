# Slide Outline

## Meta
- Topic: FACTR 2 论文解析 + AgileX Piper 标准版复现流程报告
- Scenario: 个人研读汇报 / 实验室组会 / 复现前评审（20 页，16:9）
- Content Source: provided（工作区：论文原文、13 张插图、中文解析报告、Piper 手册）
- Research: skip — 工作区材料已覆盖论文全部数据与 Piper 技术规格，无需外部检索
- Style: 深空蓝技术报告 — 深墨蓝 #0B1522 封面/章节页，浅灰 #F5F6F8 内容页，青色 #00C2A8 主强调，琥珀 #FFB020 标关键数据，细网格与坐标刻度作装饰母题
- Slide Count: 20
- Generated At: 2026-10-18T23:55:00+08:00

## Source Materials
- `paper/2606.12406v2.pdf` + `paper/paper_fulltext.md`：论文全文（22 页）
- `paper/paper_cn_summary.md`、`FACTR2_论文解析报告.docx`：中文解析与全部数值表
- `paper/figures/`：fig01 总览、fig02 部署、fig04/fig09 用户研究、fig06 任务进度、fig08 数据分布、fig10 相位切分
- `piper_docs/PiPER_Quick_Start_EN_v1.0.pdf`：6 DOF、1.5 kg、626.75 mm、±0.1 mm、4.2 kg、CAN、DC24V、关节限位与最大速度、piper_sdk / ROS1 / ROS2

## Slide-by-Slide Outline
1. **Slide 1 — Cover** — 论文标题 + 中文副题：让商品级机械臂学会感知外力 | L02 | Image: no | Chart: no
2. **Slide 2 — 速览** — 三个数字概括全文：0.547 Nm / 0.018 Nm / +17 pt | L11 | Image: no | Chart: yes
3. **Slide 3 — 背景** — 接触富操作需要力觉，但力觉一直很贵 | L08 | Image: no | Chart: no
4. **Slide 4 — 方法总览** — FACTR 2 = NEXT + FIRST 一张图讲完 | L14 | Image: fig01 | Chart: no
5. **Slide 5 — NEXT 原理** — 把「建模」换成「学习」：τ_ext = τ_m − f_θ(x) | L05 | Image: no | Chart: no
6. **Slide 6 — NEXT 规格** — 小网络、1 分钟训练、100 Hz 部署 | L06 | Image: fig02 | Chart: no
7. **Slide 7 — FIRST** — 用力矩把示教切三段，再把「接触前」多喂几遍 | L05 | Image: fig10 | Chart: no
8. **Slide 8 — 实验一** — 接触段 0.547 Nm，比最强基线低 62.8% | L17 | Image: no | Chart: yes
9. **Slide 9 — 实验二** — 20 人用户研究：没有力传感器也有「手感」 | L05 | Image: fig09 | Chart: no
10. **Slide 10 — 实验三** — 五个长时序任务，平均进度 44% → 85% | L17 | Image: fig06 | Chart: yes
11. **Slide 11 — 消融** — 只上采样接触前最好；改采样优于改损失 | L17 | Image: no | Chart: yes
12. **Slide 12 — 洞见与局限** — 可迁移的五条洞见，与作者承认的五条边界 | L08 | Image: no | Chart: no
13. **Slide 13 — Section Break** — PART 02：在你的 Piper 上跑起来 | L03 | Image: no | Chart: no
14. **Slide 14 — 路线图** — 六步复现：从 CAN 到策略 | L13 | Image: no | Chart: no
15. **Slide 15 — 硬件与信号链** — 先确认你的 Piper 能吐出什么信号 | L07 | Image: no | Chart: no
16. **Slide 16 — 数据采集** — 10 分钟，把工作空间走一遍 | L06 | Image: fig08 | Chart: no
17. **Slide 17 — 训练与验收** — 1 分钟训练后，先过「贴零」这一关 | L17 | Image: no | Chart: no
18. **Slide 18 — FIRST 与策略** — 采样器改三行，策略只多一路输入 | L08 | Image: no | Chart: no
19. **Slide 19 — 风险清单** — 五个最容易踩的坑与对策 | L15 | Image: no | Chart: no
20. **Slide 20 — Closing** — 力的感知，从「买传感器」变成「跑一次训练」 | L20 | Image: no | Chart: no

## Visual Rhythm Notes
- Dark slides: 1（封面）、13（章节页）、20（结尾）；其余浅灰内容页
- Chart-bearing slides: 2（KPI 组）、8（误差对比条形）、10（任务进度）、11（消融对比）
- 论文原图承担视觉重量：4、6、7、9、10、16
- 深色页使用坐标网格 / 刻度线母题，浅色页使用细发丝线 + 编号圆点，保持工程报告气质
