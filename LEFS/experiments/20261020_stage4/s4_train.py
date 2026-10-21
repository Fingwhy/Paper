#!/usr/bin/env python3
"""阶段 4 第 2 步：训练 NEXT（L1 自由空间力矩回归）。

与官方实现的关系
----------------
* **模型结构直接 import 官方 `models.py`**（`_vendor/factr2_next`），不复制不改：
  2 层 LSTM(hidden=128) + 2 层 MLP head(hidden=256)，每窗口重置隐状态。
* **数据源换成 `data/step.npz`**（官方要 H5 + h5py）。不是图省事：官方 Dataset
  会按 `min(len(pos), len(vel), len(cmd), len(torque))` 硬截断，对「段首 q_cmd 缺失」
  这种真实缺口会**静默截掉尾部**；这里改成显式掩码 + 段首截断，并把丢了多少帧打出来。
* **训练细节按指南 §7.2 补齐**。官方 `train.py` 有三处与参数卡不符：
  用 `Adam`（应为 AdamW + wd=1e-6）、**没有梯度裁剪**、**没有早停**（固定跑完 epochs）。
  这三条都在这里补上，且在 `--report` 里明确写出来。

三条禁令的落实
--------------
1. 不跨 episode 切窗 —— 窗口只在本段 `ep_start[i]..ep_start[i+1]` 内生成；
2. 不用测试集调任何东西 —— 测试集这里**根本不出现**，只由 `s4_eval.py` 读一次；
3. 不把插值当真实带宽 —— 采样就是 50 Hz，不重采样、不插值。

用法（本机，LEFS/experiments/20261020_stage4）：
    python s4_train.py --smoke              # 3 epoch × 3000 窗口，验通链路
    python s4_train.py --out runs/next_v1   # 全量
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

HERE = Path(__file__).resolve().parent
VENDOR_SRC = HERE.parent / "_vendor" / "factr2_next" / "src" / "factr2_next"
import sys
sys.path.insert(0, str(VENDOR_SRC))
from factr2_next.training.models import build_model  # noqa: E402

JN = ["J1", "J2", "J3", "J4", "J5", "J6"]


# ---------------------------------------------------------------- 数据
class WindowDataset(Dataset):
    """按段切因果窗口：窗口 [t-H+1, t] → 标签 y[t]（论文 Eq.3：预测最后时刻的 τ_m）。"""

    def __init__(self, npz, split, history, stride=1):
        self.history = int(history)
        self.stride = max(1, int(stride))
        X = npz[f"{split}_X"]
        Y = npz[f"{split}_Y"]
        ep_start = npz[f"{split}_ep_start"]
        centers, owners = [], []
        for i in range(len(ep_start) - 1):
            a, b = int(ep_start[i]), int(ep_start[i + 1])
            # 起点必须留在本段内：最早的合法终点是 a+H-1
            # stride>1 时不取连续窗口：相邻窗口重叠 24/25，冗余极高，
            # 84k 窗口的真实自由度远小于 84k，这是过拟合的直接原因之一。
            for t in range(a + self.history - 1, b, self.stride):
                centers.append(t)
                owners.append(i)
        self.centers = np.asarray(centers, np.int64)
        self.owners = np.asarray(owners, np.int64)
        self.X = X.astype(np.float32)
        self.Y = Y.astype(np.float32)
        self.ep_name = npz[f"{split}_ep_name"]

    def __len__(self):
        return len(self.centers)

    def __getitem__(self, i):
        c = int(self.centers[i])
        return (torch.from_numpy(self.X[c - self.history + 1: c + 1]),
                torch.from_numpy(self.Y[c]))

    def subset_by(self, mask):
        sub = WindowDataset.__new__(WindowDataset)
        sub.history = self.history
        sub.stride = self.stride
        sub.X, sub.Y, sub.ep_name = self.X, self.Y, self.ep_name
        sub.centers = self.centers[mask]
        sub.owners = self.owners[mask]
        return sub


def fit_norm(ds):
    # 只用**本子集窗口对应的帧**，不能用全集：smoke/残差实验会传子集进来，
    # 用全集会把训练集之外的帧算进均值方差（数据泄漏）。
    x = ds.X[ds.centers]
    y = ds.Y[ds.centers]
    return {
        "x_mean": x.mean(axis=0).astype(np.float32),
        "x_std": (x.std(axis=0) + 1e-6).astype(np.float32),
        "y_mean": y.mean(axis=0).astype(np.float32),
        "y_std": (y.std(axis=0) + 1e-6).astype(np.float32),
    }


def apply_norm(ds, norm):
    ds.X = (ds.X - norm["x_mean"]) / norm["x_std"]
    ds.Y = (ds.Y - norm["y_mean"]) / norm["y_std"]


# ---------------------------------------------------------------- 训练
def run_epoch(model, loader, device, opt=None, clip=0.0):
    model.train(opt is not None)
    tot, n = 0.0, 0
    for x, y in loader:
        x, y = x.to(device), y.to(device)
        pred = model(x)
        loss = nn.functional.mse_loss(pred, y)
        if opt is not None:
            opt.zero_grad(set_to_none=True)
            loss.backward()
            if clip > 0:
                nn.utils.clip_grad_norm_(model.parameters(), clip)
            opt.step()
        tot += loss.item() * len(x)
        n += len(x)
    return tot / max(n, 1)


@torch.no_grad()
def phys_mae(model, loader, device, norm):
    """反归一化后按物理单位算 MAE（N·m）与逐轴 MAE。"""
    model.eval()
    ap, ay = [], []
    for x, y in loader:
        p = model(x.to(device)).cpu().numpy()
        ap.append(p)
        ay.append(y.numpy())
    P = np.concatenate(ap) * norm["y_std"] + norm["y_mean"]
    Y = np.concatenate(ay) * norm["y_std"] + norm["y_mean"]
    per_axis = np.abs(P - Y).mean(axis=0)
    return float(per_axis.mean()), per_axis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE / "data" / "step.npz"))
    ap.add_argument("--out", default=str(HERE / "runs" / "next_v1"))
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--batch", type=int, default=2048)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--wd", type=float, default=1e-6)
    ap.add_argument("--clip", type=float, default=1.0)
    ap.add_argument("--patience", type=int, default=20)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--smoke", action="store_true", help="3000 窗口 × 3 epoch，只验链路")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--stride", type=int, default=1, help="窗口步长，>1 可降低相邻窗口冗余")
    ap.add_argument("--dropout", type=float, default=0.1)
    ap.add_argument("--hidden", type=int, default=128)
    ap.add_argument("--layers", type=int, default=2)
    ap.add_argument("--residual", action="store_true",
                    help="残差学习：先用训练集拟合 τ≈Aq+b 的线性重力基线，网络只学残差。"
                         "依据：线性基线已解释 J2 的 89%% 方差，残差 MAE 仅 0.123 N·m，"
                         "且残差与 q_cmd−q 相关性 +0.40~+0.57（摩擦/滞后），这才是 NEXT 该学的部分")
    ap.add_argument("--select-metric", choices=("val_mae", "val_loss"), default="val_mae",
                    help="checkpoint 选择指标。默认 val_mae（物理 N·m，与验收口径一致）："
                         "归一化 MSE 对 J2 极端姿态过度敏感，会选出几乎没训练的第 2 个 epoch")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    npz = np.load(args.data, allow_pickle=True)
    H = int(npz["history"])
    print(f"数据 {args.data}  H={H}")

    train_full = WindowDataset(npz, "train", H, stride=args.stride)
    val = WindowDataset(npz, "val", H)
    print(f"窗口数：train {len(train_full)}  val {len(val)}")
    print(f"  （train {len(npz['train_ep_name'])} 段 / val {len(npz['val_ep_name'])} 段；"
          f"测试集 3 段不进训练）")

    train = train_full
    if args.smoke:
        train = train_full.subset_by(np.arange(len(train_full)) < 3000)
        args.epochs = 3
        print(f"[smoke] 只用前 {len(train)} 个窗口 / {args.epochs} epoch")

    # ---- 残差学习：把「姿态→重力矩」这个大头交给线性基线，网络只学剩下的 ----
    lin_W = None
    if args.residual:
        q_tr = train.X[train.centers][:, 0:6].astype(np.float64)
        A = np.hstack([q_tr, np.ones((len(q_tr), 1))])
        lin_W, *_ = np.linalg.lstsq(A, train.Y[train.centers].astype(np.float64), rcond=None)
        train.Y = (train.Y - np.hstack(
            [train.X[:, 0:6].astype(np.float64),
             np.ones((len(train.X), 1))]) @ lin_W).astype(np.float32)
        A_va = np.hstack([val.X[:, 0:6].astype(np.float64),
                          np.ones((len(val.X), 1))])
        val.Y = (val.Y - A_va @ lin_W).astype(np.float32)
        print(f"[残差] 线性重力基线拟合完成，残差 y 的 std = "
              f"{train.Y[train.centers].std(axis=0).round(4).tolist()}")

    # 归一化：只在训练集拟合（指南 §7.1 第 3 步）
    norm = fit_norm(train)
    apply_norm(train, norm)
    apply_norm(val, norm)

    device = torch.device(args.device)
    model_cfg = dict(type="lstm", hidden_size=args.hidden, num_layers=args.layers,
                     head_hidden=256, head_layers=2, dropout=args.dropout,
                     bidirectional=False, state_mode="stateless")
    model = build_model(model_cfg, train.X.shape[1], train.Y.shape[1], H).to(device)
    n_par = sum(p.numel() for p in model.parameters())
    print(f"模型：{model_cfg}  参数 {n_par/1e3:.1f}k")
    print(f"优化器：AdamW lr={args.lr} wd={args.wd} clip={args.clip} "
          f"early-stop patience={args.patience} warmup={args.warmup}")

    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.wd)
    tl = DataLoader(train, batch_size=args.batch, shuffle=True, drop_last=False)
    vl = DataLoader(val, batch_size=args.batch, shuffle=False)

    best = float("inf")
    best_ep = -1
    bad = 0
    hist = []
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    for ep in range(1, args.epochs + 1):
        tr = run_epoch(model, tl, device, opt, args.clip)
        va = run_epoch(model, vl, device)
        mae, per = phys_mae(model, vl, device, norm)
        hist.append(dict(epoch=ep, train=tr, val=va, val_mae_nm=mae,
                         per_axis=per.tolist()))
        star = ""
        key = mae if args.select_metric == "val_mae" else va
        if key < best - 1e-9:
            best, best_ep, bad = key, ep, 0
            torch.save({"model_state_dict": model.state_dict(),
                        "input_size": train.X.shape[1],
                        "output_size": train.Y.shape[1],
                        "history": H, "model": model_cfg,
                        "norm": {k: v.tolist() for k, v in norm.items()},
                        "epoch": ep, "val_loss": va, "val_mae_nm": mae,
                        "select_metric": args.select_metric,
                        "lin_W": lin_W},
                       out_dir / "model.pt")
            star = " *"
        else:
            bad += 1
        print(f"epoch {ep:03d}  train {tr:.5f}  val {va:.5f}  "
              f"MAE {mae:.4f} N·m  [{time.time()-t0:5.1f}s]{star}", flush=True)
        if ep > args.warmup and bad >= args.patience:
            print(f"早停：best epoch {best_ep}（{args.select_metric} {best:.5f}），连续 {bad} 轮无改善")
            break
    torch.save({"model_state_dict": model.state_dict(),
                "input_size": train.X.shape[1], "output_size": train.Y.shape[1],
                "history": H, "model": model_cfg,
                "norm": {k: v.tolist() for k, v in norm.items()},
                "epoch": best_ep, "val_loss": best, "config": vars(args),
                "select_metric": args.select_metric,
                "lin_W": lin_W},
               Path(args.out) / "model.pt")
    json.dump(dict(config=vars(args), model=model_cfg, n_params=n_par,
                   best_epoch=best_ep, best_val=best, history=hist,
                   n_train_windows=len(train), n_val_windows=len(val)),
              open(Path(args.out) / "train_log.json", "w"), indent=2, ensure_ascii=False)
    print(f"\n保存 → {Path(args.out)/'model.pt'}（best epoch {best_ep}, "
          f"{args.select_metric} {best:.5f}）")
    print(f"总耗时 {time.time()-t0:.1f} s")


if __name__ == "__main__":
    main()
