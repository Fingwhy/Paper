# Paper

本仓库收录三篇论文的中文译本，以及各自的解析和复现材料。一篇论文一个文件夹，文件夹名即论文标题。

## FACTR2_学习外部力感知改进普通机械臂策略学习

**FACTR 2: Learning External Force Sensing for Commodity Robot Arms Improves Policy Learning**
Steven Oh\*, Jason Jingzhou Liu\*, Tony Tao\*, Philip Han, Kenneth Shaw, Satoshi Funabashi, Ruslan Salakhutdinov, Deepak Pathak
卡内基梅隆大学 · 早稻田大学 ｜ arXiv:2606.12406v2 [cs.RO]

论文研究如何让没有力/力矩传感器的普通机械臂获得外力感知：先从关节电流和位姿学习外部关节力矩（NEXT），
再用这个力矩信号把示教轨迹按自由运动、接触前、接触三种状态切分并重采样（FIRST），
在五个长时序接触任务上提升了成功率。

文件夹内容：

- `FACTR2_学习外部力感知改进普通机械臂策略学习_中文译文.pdf` —— 论文全文中文译本
- `中文译稿.html`、`fulltext.txt` —— 译文的排版源文件、原文纯文本
- `figs/`、`eqs/` —— 从原文抽取的 13 张插图与 11 个公式
- `解析.md` —— 论文方法、论文数据与本次复现结果的对照
- `解析与复现/` —— 深入材料：
  - `FACTR2_论文解析报告.pdf` —— 22 页论文的逐节解析
  - `PiPER_FACTR2_复现指南.pdf` —— 按 L0–L4 分级的复现操作手册
  - `PiPER_FACTR2_复现说明报告.pdf` —— 论文设定与 AgileX PiPER 标准版硬件的逐项对照
  - `PROGRESS.md` —— 复现进度
  - `experiments/` —— 各阶段实验报告与判据
  - `paper/` —— 原文 PDF 与抽取的图表
  - `piper_docs/` —— PiPER 机械臂官方资料整理

## 基于电感传感器的新型六轴力-力矩传感器

**A Novel 6-axis Force/Torque Sensor Using Inductance Sensors**
Hyun-Bin Kim, Kyung-Soo Kim（韩国科学技术院机械工程系）

arXiv:2505.09069v1

论文提出一种基于电感测量的六轴力/力矩传感器。线圈与作为靶标的金属板非接触，弹性体受载后靶标位移改变线圈电感，
六个电感数字转换器读取该变化，经非线性标定得到六维力与力矩。信号处理电路集成在 PCB 上，无需外部数据采集设备。

实测指标：直径 85 mm、重 280 g，力量程 890 / 1435 N，力矩量程 27 / 45 N·m，
采样率 4 kHz（实验取 1 kHz），力分辨率 0.035 N，满量程误差约 1%。

文件夹内容：

- `基于电感传感器的新型六轴力-力矩传感器_中文译文.pdf` —— 论文全文中文译本
- `中文译稿.html`、`images/` —— 译文的排版源文件与 8 张插图
- `extract3.py` —— 网页正文与图表的抽取脚本
- `assets/tex-svg.js` —— 离线渲染译稿公式所需的运行时
- `解析.md` —— 传感原理、关键指标来源、原文内部数据不一致之处

## 基于单片CMOS电子学的可编程硅光子电路时分复用控制

**Time-Multiplexed Control of Programmable Silicon Photonic Circuits Enabled by Monolithic CMOS Electronics**
Francesco Zanetto 等（米兰理工大学电子、信息与生物工程系）

Laser & Photonics Reviews, 2023 ｜ DOI: 10.1002/lpor.202300124

论文在不改动标准工艺的前提下，于商用 220 nm 硅光平台上以侧栅结构实现 MOSFET，
进而把模拟多路复用器和采样保持电路单片集成到光子芯片中。
可编程光子架构中大量探测器与热光移相器由此按时间顺序被依次寻址，所需的电连线数量大幅减少。

实测指标：nMOS 阈值电压 2.45 V、反亚阈值斜率 350 mV/dec；16 选 1 多路复用器建立时间约 500 ns；
采样保持电路在 100 Hz 以上刷新时 MZI 输出退化 0.03 dB；110 个模块的光路由器在 10 ms 内完成配置，
10 Gbit/s 信号路由的平均功率代价 0.3 dB。

文件夹内容：

- `基于单片CMOS电子学的可编程硅光子电路时分复用控制_中文译文.pdf` —— 论文全文中文译本（6 页）
- `中文译稿.html` —— 译文的排版源文件
- `学生版复现指南.md` —— 器件、多路复用器、采样保持、控制环路四阶段的建模路线与验收判据
- `code/sim_core.py` —— 对应的仿真脚本，仅依赖 Python 标准库
- `解析.md` —— 方法链条、指标核查、论文中互相矛盾的数据

## 文件命名约定

| 文件名 | 含义 |
|---|---|
| `*_中文译文.pdf` | 论文全文译文，阅读用 |
| `解析.md` | 论文要点、数据核查、复现结论 |
| `中文译稿.html` | 译文的排版源文件，修改后用浏览器重新导出 PDF |
| 其他 PDF | 各篇自有的解析报告或复现指南 |

## 说明

- **原文 PDF 只在 FACTR 2 文件夹内保留一份**（`解析与复现/paper/`）；另两篇的原文未随仓库分发，
  见各篇小节中的 DOI 或 arXiv 编号。
- **部分数值未取到**：硅光子那篇正文中的掺杂浓度、增益因子、MUX 占片面积、导通电阻、电源与基准电压等，
  在期刊网页上以数学公式渲染，文本抽取后为空。译文在这些位置逐处标注「数值缺失」，未作推测填补。
- 译文供阅读使用，引用与转述请以期刊或 arXiv 原文为准。

## 重新生成译文 PDF

本机未安装 Chrome，可使用 Edge 的无头模式打印：

```bash
"/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
  --headless=new --disable-gpu --no-pdf-header-footer \
  --print-to-pdf="输出.pdf" --virtual-time-budget=8000 "file:///绝对路径/中文译稿.html"
```
