# Paper

论文解析与复现存档。两个顶层目录：

| 目录 | 内容 | 入口 |
| --- | --- | --- |
| [`LEFS/`](LEFS/) | **FACTR 2 论文解析 + AgileX PiPER 标准版复现**（主体工程，L0–L4 逐阶段实验档案） | [LEFS/README.md](LEFS/README.md) |
| [`papers/`](papers/) | **论文中文译本汇总**（3 篇：硅光子时分复用控制 / 电感式六轴力矩传感器 / FACTR 2） | [papers/README.md](papers/README.md) |

## 快速导航

- 想知道 **FACTR 2 到底讲了什么** → [`LEFS/FACTR2_论文解析报告.pdf`](LEFS/FACTR2_论文解析报告.pdf)
- 想照着**在这台机器上复现** → [`LEFS/PiPER_FACTR2_复现指南.pdf`](LEFS/PiPER_FACTR2_复现指南.pdf)
- 想看**当前进度** → [`LEFS/PROGRESS.md`](LEFS/PROGRESS.md)
- 想要**某篇论文的中文全文** → [`papers/README.md`](papers/README.md)
- 想跟着**练一遍 LPOR 那篇的建模** → [`papers/2023_LPOR_硅光子时分复用控制/学生版复现指南.md`](papers/2023_LPOR_硅光子时分复用控制/学生版复现指南.md)

## 关于本仓库

- `LEFS/experiments/` 下按阶段留证，每阶段一份报告 + 一份 manifest；原始帧与模型权重不入库（见 `.gitignore`），可由报告里的命令重跑。
- `papers/` 下每篇一个目录，`*_中文译文.pdf` 是成品，`中文译稿.html` 是源文件。
- 原文 PDF 统一放在 `LEFS/paper/`，不重复存放到 `papers/` 里。
- 译文仅供阅读，引用请以期刊 / arXiv 原文为准。
