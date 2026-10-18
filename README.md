# 论文解析复现

个人论文研读与复现记录仓库。每篇论文一份完整存档：原文全文、提取的图表、中文解析报告，以及可重跑的排版脚本。

---

## 目录

| 论文 | 主题 | 报告 |
| --- | --- | --- |
| [FACTR 2](./) | 让无力度传感器的普通机械臂学会「感知」外力 | [DOCX](./FACTR2_论文解析报告.docx) · [PDF](./FACTR2_论文解析报告.pdf) |

---

# FACTR 2

> **FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning**
> Steven Oh\*, Jason Jingzhou Liu\*, Tony Tao\*, Philip Han, Kenneth Shaw, Satoshi Funabashi, Ruslan Salakhutdinov, Deepak Pathak
> CMU · Waseda ｜ arXiv:[2606.12406v2](https://arxiv.org/abs/2606.12406) [cs.RO] ｜ 2026-06-10 投稿，2026-08-12 修订
> 项目页：https://jasonjzliu.com/factr2

一句话：用「学出来的外部关节力矩」替代昂贵的力/力矩传感器——NEXT 只需 10 分钟自由运动数据 + 1 分钟训练即可估计外力；FIRST 再用该信号把示教切成自由运动 / 接触前 / 接触三段并重采样，5 个长时序接触任务的任务进度提升 17 个百分点以上。

## 目录结构

```
.
├── FACTR2_论文解析报告.docx      # 中文解析报告（19 页，含 13 张原图，可编辑）
├── FACTR2_论文解析报告.pdf       # 同上，PDF 版便于分发
└── paper/
    ├── 2606.12406v2.pdf         # arXiv 官方原文（22 页，含附录与参考文献）
    ├── paper_fulltext.md        # 全文文本，按 PDF 分页切分，保留图表标题
    ├── paper_cn_summary.md      # 中文摘要（Markdown，便于二次编辑 / 入库）
    ├── arxiv_abs.html           # 摘要页原件
    ├── captions.txt             # 全部图表标题汇总
    ├── figures/                 # 13 张论文插图（PNG，从原文提取或无损裁切）
    │   └── doc/                 # 压缩后的 JPEG（供文档嵌入）
    ├── pages/                   # 原文 22 页整页渲染（PNG）
    ├── preview/                 # 报告 19 页整页渲染（PNG）
    ├── extract.py               # 抽取全文 / 图表标题 / 图片 / 整页渲染
    ├── crop_figs.py             # 矢量图无损裁切
    ├── prep_figs.py             # 图片压缩（入文档用）
    └── build_docx.py            # 报告排版脚本（改文案后可重新生成）
```

## 报告结构

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
| 接触段外力矩 L1 误差 | NEXT **0.547 Nm** vs. FILIC 4.395 / 扰动观测器 1.471 |
| Piper 自由空间误差 | **0.018 Nm** |
| 五个长时序任务平均任务进度 | Base 44% → **FIRST 85%**（TA-VLA 68%） |
| 数据成本 | 10 分钟自由运动 + 1 分钟训练 |

## 重新生成报告

```bash
# 依赖：Python 3.13 + pymupdf + pillow + python-docx
# 在仓库根目录执行，脚本内的路径均相对于当前工作目录
python paper/build_docx.py
```

文案集中在 `paper/build_docx.py` 中，改完直接重跑即可；图片取自 `paper/figures/doc/`。
若想从头走一遍抽取流程，依次执行：

```bash
python paper/extract.py      # 原文 → 全文 Markdown / 图表标题 / 整页渲染
python paper/crop_figs.py    # 从 PDF 无损裁切矢量图
python paper/prep_figs.py    # 压缩为文档嵌入用的 JPEG
```

## 说明

- 原文 PDF 与所有插图版权归论文作者所有，此处仅作个人研读存档；引用请以 arXiv 官方版本为准。
- 报告中的数值均取自原文表格；个别仅以柱状图给出的基线数值为估读，已在图表注释中标注。
- 后续若加入其它论文，可按 `papers/<论文简称>/` 分目录存放，本目录保持独立。
