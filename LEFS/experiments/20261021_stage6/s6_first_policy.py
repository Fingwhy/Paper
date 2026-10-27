#!/usr/bin/env python
"""FIRST 第 2、3 步：重采样 + flow-matching 策略训练，并在分相上比较验证损失。

运行机器与目录（Windows 本机）
------------------------------
    C:/Users/asus/WorkBuddy/论文解析复现/LEFS/experiments/20261021_stage6/
    前置：python s6_first_dataset.py --save data/first_ds.npz

论文原文
--------
式 (5)   flow matching：x_α = (1−α)x0 + α·x1，v_θ 回归 (x1 − x0)
§5 重采样  p_t = w(s_t) / Σ_j w(s_j)，w_PC, w_C > w_F
附录 D.7  与「加权回归」（改损失权重、不改采样分布）对照

三个实验臂
----------
    base   均匀采样 + 均匀损失        （普通行为克隆）
    wregr  均匀采样 + 相位加权损失    （论文 D.7 的对照）
    first  按相位加权采样 + 均匀损失  （FIRST）

评价指标
--------
1. **分相 flow-matching 验证损失**（论文正文报的就是这个）
2. **分相动作重建误差**（用 Euler 积分从噪声还原动作块，再算 MSE / MAE，单位 deg）
   后者更直观：损失是速度场空间的，不是动作空间的。

口径偏离（必须写明）
------------------
* 无视觉：论文 o_t = (I_1:M, q_t, τ̂_ext,t)，本实验只有本体感受。所以是「机制验证」，
  不是「任务成功率复现」——没有相机、没有遥操作示范、没有仿真环境，任务成功率无从谈起。
* 动作只有 J6 一轴（原始记录只落 qcmd6），见 s6_first_dataset.py 的说明。
* 数据集极小：11 段 × 12.34 s。所以用「留段交叉验证 + 多种子」，报告均值±标准差，
  不做单次留出，否则结果完全被随机种子支配（这一条在 L1 上已经吃过教训）。
"""
from __future__ import annotations

import argparse
import itertools
import json
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
PHASE_NAMES = {0: "Free", 1: "PreContact", 2: "Contact"}


# ------------------------------------------------------------------ 模型
class VelocityNet(nn.Module):
    """v_θ(x_α, α, obs)：预测从噪声指向专家动作块的向量场。"""

    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 256, depth: int = 3):
        super().__init__()
        layers, d = [], obs_dim + act_dim + 1
        for _ in range(depth):
            layers += [nn.Linear(d, hidden), nn.SiLU()]
            d = hidden
        layers += [nn.Linear(d, act_dim)]
        self.net = nn.Sequential(*layers)

    def forward(self, x_a, alpha, obs):
        return self.net(torch.cat([obs, x_a, alpha], dim=1))


# ------------------------------------------------------------------ 训练
def train_one(args, tr, va, phases_tr, w):
    """训练一个策略。w 是三个相位的权重 [w_F, w_PC, w_C]。"""
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    obs_tr = torch.as_tensor(tr["obs"], dtype=torch.float32)
    act_tr = torch.as_tensor(tr["act"], dtype=torch.float32)
    ph_tr = torch.as_tensor(phases_tr, dtype=torch.long)
    obs_va = torch.as_tensor(va["obs"], dtype=torch.float32)
    act_va = torch.as_tensor(va["act"], dtype=torch.float32)
    ph_va = torch.as_tensor(va["phase"], dtype=torch.long)

    sw = torch.as_tensor(np.asarray(w, dtype=np.float32)[phases_tr])
    p = sw / sw.sum()

    net = VelocityNet(obs_tr.shape[1], act_tr.shape[1], args.hidden, args.depth)
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-6)
    n = len(obs_tr)
    for _ in range(args.epochs):
        if args.arm == "first":
            idx = torch.from_numpy(np.random.choice(n, args.batch, p=p.numpy()))
        else:
            idx = torch.randint(0, n, (args.batch,))
        x1 = act_tr[idx]
        obs = obs_tr[idx]
        alpha = torch.rand(len(idx), 1)
        x0 = torch.randn_like(x1)
        xa = (1 - alpha) * x0 + alpha * x1
        target = x1 - x0
        pred = net(xa, alpha, obs)
        loss = ((pred - target) ** 2).mean(dim=1)
        if args.arm == "wregr":
            loss = loss * sw[idx]
        loss = loss.mean()
        opt.zero_grad(); loss.backward()
        nn.utils.clip_grad_norm_(net.parameters(), 1.0)
        opt.step()

    net.eval()
    with torch.no_grad():
        # ① 速度场空间的分相损失（与训练目标一致）
        g = torch.Generator().manual_seed(1234)
        fm = {}
        alpha = torch.rand(len(obs_va), 1, generator=g)
        x0 = torch.randn(act_va.shape, generator=g)
        xa = (1 - alpha) * x0 + alpha * act_va
        per = ((net(xa, alpha, obs_va) - (act_va - x0)) ** 2).mean(dim=1)
        for v in (0, 1, 2):
            m = ph_va == v
            fm[PHASE_NAMES[v]] = float(per[m].mean()) if m.any() else float("nan")

        # ② 动作空间的还原误差：从纯噪声 Euler 积分 20 步
        x = torch.randn(act_va.shape, generator=g)
        for s in range(20):
            a0, a1 = s / 20, (s + 1) / 20
            x = x + net(x, torch.full((len(x), 1), a0), obs_va) / 20
        err = (x - act_va).abs().mean(dim=1)        # 每个样本的平均绝对误差（deg）
        mae = {}
        for v in (0, 1, 2):
            m = ph_va == v
            mae[PHASE_NAMES[v]] = float(err[m].mean()) if m.any() else float("nan")
        mae["All"] = float(err.mean())
    return fm, mae


# ------------------------------------------------------------------ 数据
def load_ds(path: Path):
    z = np.load(path, allow_pickle=True)
    obs, act, ph, ep = z["obs"], z["act"], z["phase"], z["ep"]
    names = [str(x) for x in z["names"]]
    # 标准化（用全体统计量，避免折间泄漏口径不一致；量级差异大，必须做）
    mu, sd = obs.mean(0), obs.std(0) + 1e-6
    obs = (obs - mu) / sd
    amu, asd = act.mean(0), act.std(0) + 1e-6
    act = (act - amu) / asd
    return obs.astype(np.float32), act.astype(np.float32), ph, ep, names


def subset(obs, act, ph, ep, eps):
    m = np.isin(ep, eps)
    return dict(obs=obs[m], act=act[m], phase=ph[m])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ds", default=str(HERE / "data" / "first_ds.npz"))
    ap.add_argument("--out", default=str(HERE / "runs" / "first_policy.json"))
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--hidden", type=int, default=256)
    ap.add_argument("--depth", type=int, default=3)
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--seeds", type=int, default=2)
    a = ap.parse_args()

    obs, act, ph, ep, names = load_ds(Path(a.ds))
    n_ep = len(names)
    print("数据集：%d 段 / %d 样本  obs %d 维 → act %d 维"
          % (n_ep, len(obs), obs.shape[1], act.shape[1]))
    comp = {PHASE_NAMES[v]: float((ph == v).mean()) for v in (0, 1, 2)}
    print("相构成：" + "  ".join("%s %.1f%%" % (k, 100 * v) for k, v in comp.items()))
    print()

    # 留段交叉验证：按段号轮转分折
    folds = []
    for f in range(a.folds):
        va_eps = [i for i in range(n_ep) if i % a.folds == f]
        tr_eps = [i for i in range(n_ep) if i % a.folds != f]
        folds.append((tr_eps, va_eps))

    # 实验臂 × 权重
    # 注意：本数据集 Contact 已占 81.3%（拧盖全程都在接触），再给 C 加权没有意义，
    # 所以网格只扫 Pre-Contact 的权重。论文的场景是自由空间占多数，恰好相反。
    grid = [("base", (1.0, 1.0, 1.0))]
    for wpc in (3, 5, 10, 20):
        for arm in ("wregr", "first"):
            grid.append((arm, (1.0, float(wpc), 1.0)))

    results = {}
    runs_raw = {}
    t_start = time.time()
    for arm, w in grid:
        key = f"{arm}_F{w[0]:g}_PC{w[1]:g}_C{w[2]:g}"
        fm_acc, mae_acc = [], []
        for fi, (tr_eps, va_eps) in enumerate(folds):
            tr = subset(obs, act, ph, ep, tr_eps)
            va = subset(obs, act, ph, ep, va_eps)
            for s in range(a.seeds):
                a.arm, a.seed = arm, 1000 + s
                fm, mae = train_one(a, tr, va, tr["phase"], w)
                fm_acc.append(fm); mae_acc.append(mae)
        # 逐次记录：折与种子在两个实验臂之间完全一致，可以做配对检验
        runs_raw[key] = [{"fm": f, "mae": m} for f, m in zip(fm_acc, mae_acc)]
        def agg(d):
            out = {}
            for k in d[0]:
                vals = np.array([x[k] for x in d], float)
                vals = vals[~np.isnan(vals)]
                out[k] = float(vals.mean()) if len(vals) else float("nan")
                out[k + "_sd"] = float(vals.std()) if len(vals) else float("nan")
            return out
        results[key] = dict(arm=arm, w=w, fm=agg(fm_acc), mae=agg(mae_acc),
                            runs=len(fm_acc))
        print("%-26s FM[F %.5f PC %.5f C %.5f]  MAE[F %.3f PC %.3f C %.3f All %.3f]  (%.0fs)"
              % (key, results[key]["fm"]["Free"], results[key]["fm"]["PreContact"],
                 results[key]["fm"]["Contact"], results[key]["mae"]["Free"],
                 results[key]["mae"]["PreContact"], results[key]["mae"]["Contact"],
                 results[key]["mae"]["All"], time.time() - t_start), flush=True)

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(phase_composition=comp, n_episodes=n_ep, n_samples=int(len(obs)),
                   folds=a.folds, seeds=a.seeds, epochs=a.epochs,
                   results=results, runs=runs_raw), open(out, "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
    print("\n已保存", out)

    # 配对比较：同一折同一种子下，各臂相对 base 的分相差异
    base = runs_raw["base_F1_PC1_C1"]
    print("\n配对比较（base 为参照，n=%d 次配对；差值为负表示更好）" % len(base))
    print("%-22s %-26s %-26s %-26s" % ("配置", "ΔFM PreContact", "ΔFM Contact", "ΔFM Free"))
    for k, rr in runs_raw.items():
        if k == "base_F1_PC1_C1":
            continue
        row = []
        for ph_name in ("PreContact", "Contact", "Free"):
            dif = np.array([r["fm"][ph_name] - b["fm"][ph_name] for r, b in zip(rr, base)])
            dif = dif[~np.isnan(dif)]
            if len(dif) < 3:
                row.append("n/a"); continue
            se = dif.std(ddof=1) / np.sqrt(len(dif))
            t = dif.mean() / se if se > 0 else 0.0
            row.append("%+.4f (%+.1f%%, t=%+.1f)" % (dif.mean(),
                       100 * dif.mean() / np.mean([b["fm"][ph_name] for b in base]), t))
        print("%-22s %-26s %-26s %-26s" % (k.replace("_F1_", " w").replace("_C1", ""), *row))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
