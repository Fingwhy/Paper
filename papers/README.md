# 论文译文汇总

三篇论文的中文译本与配套材料。每篇一个目录，命名规则 `年份_来源ID_主题/`。

| # | 目录 | 论文 | 来源 | 我的产出 |
|---|---|---|---|---|
| 1 | [`2023_LPOR_硅光子时分复用控制/`](2023_LPOR_硅光子时分复用控制/) | Time-Multiplexed Control of Programmable Silicon Photonic Circuits Enabled by Monolithic CMOS Electronics<br><sub>Zanetto 等，米兰理工</sub> | [DOI 10.1002/lpor.202300124](https://doi.org/10.1002/lpor.202300124) | 中文全译本（6 页）+ 学生版复现指南 + 可跑仿真脚本 |
| 2 | [`2025_arXiv2505.09069_电感式六轴力矩传感器/`](2025_arXiv2505.09069_电感式六轴力矩传感器/) | 电感式六轴力/力矩传感器（论文解析） | [arXiv:2505.09069](https://arxiv.org/abs/2505.09069) | 中文译文 PDF + 带公式的在线稿 |
| 3 | [`2026_arXiv2606.12406_FACTR2_六轴力控/`](2026_arXiv2606.12406_FACTR2_六轴力控/) | FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning<br><sub>CMU × Waseda，本项目主复现对象</sub> | [arXiv:2606.12406](https://arxiv.org/abs/2606.12406) | 中文译文 PDF + 抽取的公式/图片 + 全文文本 |

## 各目录里有什么

**统一约定**：`*_中文译文.pdf` 是成品（读这个）；`中文译稿.html` 是源文件（改排版用这个）；
其余是支撑材料（图、公式、抽取脚本）。

```
papers/
├── 2023_LPOR_硅光子时分复用控制/
│   ├── LPOR2023_硅光子时分复用控制_中文译文.pdf   成品，6 页 A4
│   ├── 中文译稿.html                              源文件（改完可重新导 PDF）
│   ├── 学生版复现指南.md                          四个阶段 + 验收点 + 踩坑清单
│   └── code/sim_core.py                           零依赖，直接跑出对照表
├── 2025_arXiv2505.09069_电感式六轴力矩传感器/
│   ├── 电感式六轴力矩传感器_中文译文.pdf
│   ├── 中文译稿.html                              公式由 assets/tex-svg.js 本地渲染
│   ├── images/                                    译稿引用的 8 张图
│   ├── extract3.py                                网页→结构化文本的抽取脚本
│   └── assets/tex-svg.js                          MathJax 运行时（离线可看公式）
└── 2026_arXiv2606.12406_FACTR2_六轴力控/
    ├── FACTR2_六轴力控_中文译文.pdf
    ├── 中文译稿.html
    ├── fulltext.txt                               原文纯文本，方便全文搜索
    ├── figs/  eqs/                                13 张插图 + 11 个公式 PNG
    └── ...                                        （原文 PDF 不在此处，见下）
```

## 几点说明

- **原文 PDF 不放这里**：`arXiv:2606.12406` 的原文（22 页，26 MB）在
  [`LEFS/paper/2606.12406v2.pdf`](../LEFS/paper/2606.12406v2.pdf)，
  与这里曾经那份的 md5 完全一致，删掉重复件省了 26 MB。
- **LPOR 那篇的译文有数值缺口**：原文里的掺杂浓度、增益因子、MUX 面积与导通电阻、
  电源/基准电压、存储电容个数，在 Wiley 网页上是 MathML 公式渲染的，文本抽取后只剩空括号。
  译文里逐处标注了「数值缺失」，没有编造。补齐需对照原文 PDF。
- **LPOR 那篇没有 arXiv 预印本**。arXiv:2510.18463 是同一课题组后来的相干加法器工作，
  不是这篇，别搞混。
- 译文仅供阅读；引用、转述请以期刊原文为准。

## 重新生成 PDF

译文 HTML 改完后用 Edge 无头模式打印（本机没装 Chrome 也能用）：

```bash
"/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
  --headless=new --disable-gpu --no-pdf-header-footer \
  --print-to-pdf="输出.pdf" --virtual-time-budget=8000 "file:///绝对路径/中文译稿.html"
```
