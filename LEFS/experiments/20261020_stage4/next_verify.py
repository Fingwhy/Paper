#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""next_verify.py —— 用训练好的 NEXT 模型去预测拧盖姿态下的 J6 力矩

目的
----
回答一个问题：**NEXT 模型在拧盖姿态下还能用吗？**

做法
----
1. 读拧盖数据（171.5 Hz、度），重采样到训练用的 50 Hz、并转成弧度；
2. 按 H 帧滑窗喂进模型，得到预测的 τ_m（6 轴）；
3. 与实测 τ_m = K·I 对比；
4. 同时在**训练集内**的一段数据上做同样的事，作为「脚本正确性」的对照。

如果训练集内准、拧盖姿态差得很远 ⇒ 是分布外（模型没见过这个姿态），
不是脚本 bug。

用法（本机，venv 里有 torch）：
    cd LEFS/experiments/20261020_stage4
    python next_verify.py --ckpt runs/kfold/kf0 --twist ../20261021_stage6/data/4.棘轮全量预开全打开_轮2.csv
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "_vendor", "factr2_next", "src", "factr2_next"))

import torch                                                   # noqa: E402
from factr2_next.training.models import build_model            # noqa: E402

# 标定得到的 K（N·m/A）；J1 未标定，用 SDK 标称值，只作参考
K_JOINT = [1.18125, 4.7185, 4.7185, 0.7446, 0.7446, 10.7882]
HZ_TRAIN = 50.0            # 训练数据的采样率
DT_TRAIN = 1.0 / HZ_TRAIN


# ------------------------------------------------------------------ 模型
def load_model(ckpt_dir, device="cpu"):
    path = os.path.join(ckpt_dir, "model.pt")
    ck = torch.load(path, map_location=device, weights_only=False)
    cfg = ck["model"]
    H = int(ck["history"])
    m = build_model(cfg, int(ck["input_size"]), int(ck["output_size"]), H)
    m.load_state_dict(ck["model_state_dict"])
    m.eval()
    norm = {k: np.array(v, dtype=np.float32) for k, v in ck["norm"].items()}
    return m, H, norm, ck


def predict(model, H, norm, X_raw, lin_W=None, device="cpu"):
    """X_raw: (N, 18) 原始物理量（rad, rad/s, rad）。

    按论文 Eq.3：窗口 [t-H+1, t] 预测第 t 帧；段内不足 H 的丢弃。

    **残差模式**（--residual）：训练时 Y 先减了线性重力基线 [q,1]@lin_W，
    所以预测时必须加回来，否则输出只是"残差"，会接近 0。
    """
    X = (X_raw - norm["x_mean"]) / norm["x_std"]
    N = len(X)
    if N < H:
        return np.zeros((0, 6))
    idx = np.arange(H - 1, N)
    win = np.stack([X[i - H + 1:i + 1] for i in idx])           # (M, H, 18)
    out = []
    with torch.no_grad():
        for s in range(0, len(win), 512):
            b = torch.from_numpy(win[s:s + 512].astype(np.float32)).to(device)
            out.append(model(b).cpu().numpy())
    P = np.concatenate(out) * norm["y_std"] + norm["y_mean"]
    if lin_W is not None:
        A = np.hstack([X_raw[idx][:, 0:6].astype(np.float64),
                       np.ones((len(idx), 1))])
        P = P + (A @ np.asarray(lin_W, dtype=np.float64)).astype(np.float32)
    return P, idx


# ------------------------------------------------------------------ 数据
def load_twist_csv(path):
    """读 twistlog csv，返回 t(s)、q(rad)、dq(rad/s)、qcmd(rad)、i(A)。"""
    with open(path, encoding="utf-8") as f:
        rd = list(csv.reader(f))
    hdr, rows = rd[0], rd[1:]
    ix = {h: i for i, h in enumerate(hdr)}
    need = ["t_s"] + [f"q{i}" for i in range(1, 7)] + [f"dq{i}" for i in range(1, 7)]
    for k in need:
        if k not in ix:
            sys.exit(f"[FAIL] 缺列 {k}（需要 twistlog 32 列格式）")
    R2D = math.pi / 180.0
    t = np.array([float(r[ix["t_s"]]) for r in rows])
    q = np.array([[float(r[ix[f"q{i}"]]) * R2D for i in range(1, 7)] for r in rows])
    dq = np.array([[float(r[ix[f"dq{i}"]]) * R2D for i in range(1, 7)] for r in rows])
    # 只有 qcmd6；其余轴锁死，指令≈实测
    qc = q.copy()
    if "qcmd6" in ix:
        qc[:, 5] = np.array([float(r[ix["qcmd6"]]) * R2D for r in rows])
    i = np.array([[float(r[ix[f"i{i}"]]) for i in range(1, 7)] for r in rows])
    return t, q, dq, qc, i


def resample_50hz(t, *arrays):
    """线性插值到 50 Hz 等间隔网格（训练就是这个采样率）。"""
    t0, t1 = t[0], t[-1]
    n = int(round((t1 - t0) / DT_TRAIN)) + 1
    grid = t0 + np.arange(n) * DT_TRAIN
    out = [np.stack([np.interp(grid, t, a[:, k]) for k in range(a.shape[1])], axis=1)
           if a.ndim == 2 else np.interp(grid, t, a)
           for a in arrays]
    return grid, out


def load_train_episode(pattern, seg):
    """从训练集里取一段，返回 (X_raw, Y_raw) 供 sanity check。"""
    p = None
    roots = [os.path.join(HERE, "episodes"),
             os.path.join(HERE, "..", "20261019_stage3", "episodes"),
             os.path.join(HERE, "..", "20261019_stage1_l0", "episodes")]
    for root in roots:
        for f in glob.glob(os.path.join(root, "*", "samples.jsonl")):
            if seg in os.path.basename(os.path.dirname(f)):
                p = f
                break
        if p:
            break
    if p is None:
        return None
    qs, dqs, cs, taus = [], [], [], []
    with open(p, encoding="utf-8") as f:
        for line in f:
            d = json.loads(line)
            q, dq = d.get("q"), d.get("dq")
            cur, qc = d.get("cur"), d.get("q_cmd")
            if not q or not dq or not cur or not qc:
                continue
            if any(v is None for v in list(q) + list(dq) + list(cur) + list(qc)):
                continue
            qs.append(q)
            dqs.append(dq)
            cs.append(qc)
            taus.append([cur[k] * K_JOINT[k] for k in range(6)])
    if len(qs) < 100:
        return None
    X = np.hstack([np.array(qs), np.array(dqs), np.array(cs)]).astype(np.float32)
    Y = np.array(taus, dtype=np.float32)
    return X, Y


# ------------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser(description="用 NEXT 模型预测拧盖姿态下的 τ_m")
    ap.add_argument("--ckpt", default="runs/kfold/kf0", help="模型目录（含 model.pt）")
    ap.add_argument("--twist", default=None, help="拧盖 csv（32 列全量格式）")
    ap.add_argument("--seg", default="S1_1_mid", help="训练集内用于对照的段名")
    ap.add_argument("--device", default="cpu")
    a = ap.parse_args()

    model, H, norm, ck = load_model(a.ckpt, a.device)
    lin_W = ck.get("lin_W")
    print(f"[ckpt] 残差模式 = {ck.get('config', {}).get('residual')}"
          f"   lin_W = {'有 ' + str(np.array(lin_W).shape) if lin_W is not None else '无'}")
    print(f"[ckpt] {a.ckpt}  H={H}  input={ck['input_size']}  "
          f"epoch={ck.get('epoch')}  val_loss={ck.get('val_loss'):.4f}")
    print(f"[ckpt] 标签归一化 y_std（6 轴）= "
          f"{[round(float(v), 4) for v in norm['y_std']]}  N·m")
    print(f"       ⇒ 训练集里 J6 力矩的 std 只有 {float(norm['y_std'][5]):.4f} N·m")

    # ---------- 对照 1：训练集内
    tr = load_train_episode(None, a.seg)
    if tr is None:
        print(f"\n[skip] 找不到训练段 {a.seg}")
    else:
        X, Y = tr
        P, idx = predict(model, H, norm, X, lin_W, a.device)
        if len(P):
            Yt = Y[idx]
            ae = np.abs(P - Yt).mean(axis=0)
            print(f"\n[对照·训练集内] 段 {a.seg}  窗口 {len(P)}")
            print(f"  {'轴':<4}{'实测均值':>10}{'预测均值':>10}{'MAE':>9}")
            for k in range(6):
                print(f"  J{k+1:<3}{Yt[:, k].mean():>10.3f}{P[:, k].mean():>10.3f}{ae[k]:>9.3f}")
            print(f"  总 MAE = {ae.mean():.4f} N·m")
            print(f"  实测 Y 的范围: {Yt.min():.2f} ~ {Yt.max():.2f} N·m")

    # ---------- 对照 2：拧盖姿态
    if not a.twist:
        print("\n[skip] 没给 --twist")
        return
    if not os.path.exists(a.twist):
        sys.exit(f"[FAIL] 文件不存在 {a.twist}")
    t, q, dq, qc, i = load_twist_csv(a.twist)
    dur = t[-1] - t[0]
    hz = len(t) / dur
    print(f"\n[拧盖] {os.path.basename(a.twist)}  {len(t)} 帧 / {dur:.2f}s = {hz:.1f} Hz")
    print(f"       q6: {math.degrees(q[0,5]):.1f}° → {math.degrees(q[-1,5]):.1f}°")

    tg, (qg, dqg, qcg, ig) = resample_50hz(t, q, dq, qc, i)
    print(f"       重采样到 50 Hz: {len(tg)} 点（{len(tg)*DT_TRAIN:.2f}s）")

    Xr = np.hstack([qg, dqg, qcg]).astype(np.float32)
    P, idx = predict(model, H, norm, Xr, lin_W, a.device)
    if not len(P):
        sys.exit("[FAIL] 窗口不足")
    Ymeas = np.stack([ig[idx, k] * K_JOINT[k] for k in range(6)], axis=1)

    print(f"\n[拧盖] 有效窗口 {len(P)}")
    print(f"  {'轴':<4}{'实测 K·I':>10}{'NEXT 预测':>11}{'差':>9}{'判定':>8}")
    for k in range(6):
        m_, p_ = Ymeas[:, k].mean(), P[:, k].mean()
        print(f"  J{k+1:<3}{m_:>10.3f}{p_:>11.3f}{p_-m_:>9.3f}"
              f"{'  ← 差得远' if abs(p_-m_)>0.5 else ''}")

    print(f"\n  J6 细看（拧盖的关键轴）")
    e = P[:, 5] - Ymeas[:, 5]
    print(f"    实测 τ6: {Ymeas[:,5].min():+.2f} ~ {Ymeas[:,5].max():+.2f} N·m  "
          f"均值 {Ymeas[:,5].mean():+.2f}")
    print(f"    预测 τ6: {P[:,5].min():+.2f} ~ {P[:,5].max():+.2f} N·m  "
          f"均值 {P[:,5].mean():+.2f}")
    print(f"    误差: 均值 {e.mean():+.2f}  std {e.std():.2f}  最大 {np.abs(e).max():.2f} N·m")
    print(f"    训练集 J6 标签的 std = {float(norm['y_std'][5]):.4f} N·m")
    print(f"    拧盖时 τ6 的跨度 = {Ymeas[:,5].max()-Ymeas[:,5].min():.2f} N·m "
          f"= 训练 std 的 {(Ymeas[:,5].max()-Ymeas[:,5].min())/float(norm['y_std'][5]):.0f} 倍")

    if abs(e.mean()) > 0.3:
        print("\n[结论] 模型在拧盖姿态下**明显失效**：预测与实测差 "
              f"{abs(e.mean()):.2f} N·m（J6）。这不是脚本问题——"
              "见上面「训练集内」的 MAE 对照。")
    else:
        print("\n[结论] 模型在拧盖姿态下的偏差在 0.3 N·m 内，值得进一步验证。")


if __name__ == "__main__":
    main()
