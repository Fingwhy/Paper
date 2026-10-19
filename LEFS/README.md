# Learning External Force Sensing（LEFS）

FACTR 2 论文解析与 AgileX PiPER 标准版复现存档。GitHub 仓库 `Fingwhy/Paper` 根目录下只有这一个文件夹，本 README 即它的入口。

每篇论文一份完整存档：原文全文、提取的图表、中文解析报告，以及可重跑的排版脚本；配套机械臂的官方资料与「在这台机器上怎么复现」的施工图单独成册。

---

## 目录

| 内容 | 主题 | 交付物 |
| --- | --- | --- |
| **FACTR 2** 论文解析 | 让无力度传感器的普通机械臂学会「感知」外力 | [DOCX](./FACTR2_论文解析报告.docx) · [PDF](./FACTR2_论文解析报告.pdf) |
| **FACTR 2 在 PiPER 上的复现说明** | 逐项对照论文设定与 PiPER 标准版，给出 L0–L4 复现路线与验收指标 | [DOCX](./PiPER_FACTR2_复现说明报告.docx) · [PDF](./PiPER_FACTR2_复现说明报告.pdf) |
| **PiPER 标准版 官方参数与使用说明** | 完整规格、接口、上电流程、SDK、CAN 协议速查（中文整理） | [DOCX](./piper_docs/PiPER_标准版_官方参数与使用说明.docx) · [PDF](./piper_docs/PiPER_标准版_官方参数与使用说明.pdf) · [MD](./piper_docs/PiPER_标准版_官方参数与使用说明.md) |
| **汇报幻灯片** | 论文解析 + PiPER 复现路线，20 页 16:9，可直接用于组会 / 复现评审 | [PPTX](./FACTR2_论文解析与Piper复现.pptx) · [大纲源](./slide-outline.md) |

---

# FACTR 2

> **FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning**
> Steven Oh\*, Jason Jingzhou Liu\*, Tony Tao\*, Philip Han, Kenneth Shaw, Satoshi Funabashi, Ruslan Salakhutdinov, Deepak Pathak
> CMU · Waseda ｜ arXiv:[2606.12406v2](https://arxiv.org/abs/2606.12406) [cs.RO] ｜ 2026-06-10 投稿，2026-08-12 修订
> 项目页：https://jasonjzliu.com/factr2

一句话：用「学出来的外部关节力矩」替代昂贵的力/力矩传感器——NEXT 只需 10 分钟自由运动数据 + 1 分钟训练即可估计外力；FIRST 再用该信号把示教切成自由运动 / 接触前 / 接触三段并重采样，5 个长时序接触任务的任务进度提升 17 个百分点以上。

## 报告结构（论文解析）

1. 论文速览：一句话结论与三条贡献
2. 研究背景：为什么「力」这么贵
3. 方法总览：FACTR 2 = NEXT + FIRST
4. 实验与结果：外部力矩精度 / 遥操作用户研究 / 五个任务主结果 / 相位消融 / 三组对照
5. 关键结论与可迁移的洞见
6. 局限与作者自述的边界
7. 复现要点：给动手实现者的清单
8. 交付文件清单与延伸阅读

## 核心数据速查

| 指标 | 结果 |
| --- | --- |
| 接触段外力矩 L1 误差（Franka） | NEXT **0.547 Nm** vs. FILIC 4.395 / 扰动观测器 1.471 |
| Piper 自由空间误差 | **0.018 Nm**（FILIC 0.435 / DO 0.248） |
| 五个长时序任务平均任务进度 | Base 44% → **FIRST 85%**（TA-VLA 68%） |
| 数据成本 | 10 分钟自由运动 + 1 分钟训练 |

---

# FACTR 2 × PiPER 标准版（复现说明）

论文的低成本平台就是 AgileX Piper，因此「机型对不上」不是问题；真正的差距在**真值、第二台臂、数据与算力**。报告把这件事拆成可验收的五层：

| 层级 | 目标 | 周期 | 验收指标 |
| --- | --- | --- | --- |
| L0 信号链 | CAN → 时间戳 → 落盘 | 1~2 天 | 10 分钟零丢帧，50 Hz ≥ 49 Hz |
| L1 NEXT（自由空间） | 训练 LSTM 复现表 5 量级 | 2~4 天 | 留出集 L1 **≤ 0.05 N·m**（论文 0.018） |
| L2 接触检测 + FIRST | 阈值标定、相位切分、加权采样 | 1~2 天 | 检出率 ≥ 90%，误报 ≤ 1 次/分 |
| L3 力反馈遥操作 | 主从联动 + τ_feedback | 3~5 天 | 主观评分更高、施加关节力矩更低 |
| L4 策略学习 | FIRST vs Base | 数周 | 单任务进度提升 ≥ 10 pt |

三条必须提前接受的结论：

- **Piper 上没有接触段真值**，论文自己也没在 Piper 上给接触误差；想在 Piper 上报接触精度，必须自建标定台。
- **电流只有 50 Hz**（`0x251~0x256`，20 ms 一帧），与论文 100 Hz 不同，管线建议直接跑 50 Hz（`H=25`）。
- **末端 XT30 只允许松灵自有 CAN 设备**，外部力传感器必须走独立总线，并计入 1.5 kg 负载预算。

完整版见 [PiPER_FACTR2_复现说明报告.pdf](./PiPER_FACTR2_复现说明报告.pdf)。

---

# PiPER 标准版（硬件存档）

## 参数速查

| 项 | 指标 |
| --- | --- |
| 自由度 / 负载 | 6 / 1.5 kg（含末端工具） |
| 本体重量 / 臂展 | 4.2 kg / 626.75 mm |
| 重复定位精度 | ±0.1 mm |
| 关节范围 | J1 ±154°，J2 0~195°，J3 -175~0°，J4 -102~102°，J5 ±75°，J6 ±120° |
| 关节最大速度 | J1 180，J2 195，J3 180，J4~J6 225 °/s |
| 供电 / 功耗 | DC 24 V（≤26 V，≥10 A）/ 最大 ≤120 W，综合 ≤40 W |
| 通信 | CAN 2.0B 标准帧，1 Mbps，Motorola(MSB) |
| 反馈 | 高速 `0x251~0x256`（20 ms，含电流 0.001 A）· 低速 `0x261~0x266`（100 ms）· 状态 `0x2A1~0x2A8` |
| 控制 | 位置-速度模式 / MIT 模式（`0x15A~0x15F`，含 T_ref） |
| 环境 / 防护 | -20~50 ℃，25%~85% 非冷凝 / IP22 |

> 关节限位官方改过三版（J4 ±106° → -102~102°，J6 ±100° → ±120°）。**不要硬编码**，用 `0x472` 查询 / `0x473` 回读。

## 官方文件（本地存档）

| 文件 | 说明 |
| --- | --- |
| [PiPER_Quick_Start_EN_v1.0.pdf](./piper_docs/PiPER_Quick_Start_EN_v1.0.pdf) | 官方快速上手手册（含完整规格表与夹爪参数） |
| [PiPER_用户手册_中文_v1.1.0.pdf](./piper_docs/PiPER_用户手册_中文_v1.1.0.pdf) | 官方用户手册（57 页，最新版） |
| [PiPER_CAN协议_API手册_zh_v1.5.xlsx](./piper_docs/PiPER_CAN协议_API手册_zh_v1.5.xlsx) | 官方 CAN 协议全表（复现核心） |
| [PiPER_标准版_官方参数与使用说明.md](./piper_docs/PiPER_标准版_官方参数与使用说明.md) | 上述资料的中文重排（带工程备注） |

原始文件来自 AgileX 官方发布（经 Weston Robot 官方镜像下载，附 SHA256 可校验）；`new.agilex.ai` 域名证书于 2026-10 过期，故使用镜像。

---

## 目录结构

```
LEFS/                                    # 仓库根目录下唯一的文件夹
├── FACTR2_论文解析报告.docx / .pdf        # 论文解析报告（19 页）
├── PiPER_FACTR2_复现说明报告.docx / .pdf  # 复现说明报告（17 页，PiPER 视角）
├── FACTR2_论文解析与Piper复现.pptx        # 汇报幻灯片（20 页）
├── slide-outline.md                     # 幻灯片大纲 / 版式与图表分配
├── piper_docs/                          # PiPER 官方资料 + 中文整理版
│   ├── PiPER_标准版_官方参数与使用说明.md / .docx / .pdf
│   ├── PiPER_Quick_Start_EN_v1.0.pdf    # 官方快速上手
│   ├── PiPER_用户手册_中文_v1.1.0.pdf     # 官方用户手册（57 页）
│   ├── PiPER_CAN协议_API手册_zh_v1.5.xlsx # 官方 CAN 协议
│   └── *.txt                            # 上述 PDF 的文本抽取（便于检索）
└── paper/
    ├── 2606.12406v2.pdf                 # arXiv 官方原文（22 页）
    ├── arxiv_abs.html                   # arXiv 摘要页存档
    ├── paper_fulltext.md                # 全文文本（按页切分）
    ├── paper_cn_summary.md              # 论文中文摘要
    ├── piper_repro_report.md            # 复现说明报告的 Markdown 源（唯一内容源）
    ├── captions.txt                     # 图表标题抽取
    ├── figures/                         # 13 张论文插图（PNG + 文档用 JPEG）
    ├── pages/ · preview/                # 原文 22 页 / 报告 19 页的整页渲染
    ├── build_docx.py                    # 解析报告排版脚本
    ├── build_piper_report.py            # 复现报告 + 官方文档排版脚本（md → docx / pdf）
    ├── docx_kit.py                      # 从 build_docx.py 抽出的可复用排版组件
    └── extract.py · crop_figs.py · prep_figs.py · make_figs.py
```

## 重新生成

```bash
# 依赖：Python 3.13 + pymupdf + python-docx + pillow + openpyxl
# 在 LEFS/ 目录下执行（即本 README 所在目录），脚本内路径均相对于当前工作目录

python paper/build_piper_report.py          # 复现说明报告（DOCX + PDF）
python paper/build_piper_report.py piper    # PiPER 官方参数与使用说明（DOCX + PDF）
python paper/build_docx.py                  # 论文解析报告（DOCX）
```

文案集中在各自的 Markdown 源里，改完重跑即可；`build_piper_report.py` 用同一份 Markdown 同时产出 DOCX（python-docx）与 PDF（PyMuPDF Story），避免两版内容漂移。

若想从头走一遍论文抽取流程：

```bash
python paper/extract.py      # 原文 → 全文 Markdown / 图表标题 / 整页渲染
python paper/crop_figs.py    # 从 PDF 无损裁切矢量图
python paper/prep_figs.py    # 压缩为文档嵌入用的 JPEG
```

## 说明

- 原文 PDF 与所有插图版权归论文作者所有；PiPER 手册与协议版权归 AgileX Robotics 所有。此处仅作个人研读存档，引用请以官方版本为准。
- 报告中的论文数值均取自原文表格；个别仅以柱状图给出的基线数值为估读，已在图表注释中标注。
- 「复现说明报告」中标注「建议 / 工程经验」的内容为复现补充，非论文或厂家原文。
