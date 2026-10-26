#!/usr/bin/env python3
"""对比规划轨迹与真机实际轨迹：打印每关节误差 + 输出一张对比图。"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")  # 只保存文件，不弹窗
import matplotlib.pyplot as plt


HERE = os.path.dirname(os.path.abspath(__file__))
PLAN_PATH = os.path.join(HERE, "plan_trajectory.npy")
ACTUAL_PATH = os.path.join(HERE, "actual_trajectory.npy")
OUT_PATH = os.path.join(HERE, "sim2real_compare.png")

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def main():
    plan = np.load(PLAN_PATH)
    actual = np.load(ACTUAL_PATH)
    assert plan.shape == actual.shape, f"shape mismatch: {plan.shape} vs {actual.shape}"

    diff = actual - plan
    x = np.arange(plan.shape[0])
    max_err_deg = np.abs(diff).max(axis=0) * 180.0 / np.pi

    fig, axes = plt.subplots(3, 2, figsize=(11, 9))
    axes = axes.ravel()

    # 前 5 个子图：每个关节画「规划 vs 实际」两条曲线
    for k in range(5):
        ax = axes[k]
        ax.plot(x, plan[:, k], "o-", label="planned")
        ax.plot(x, actual[:, k], "s--", label="actual")
        ax.set_title(f"{JOINT_NAMES[k]}  (max err {max_err_deg[k]:.2f} deg)")
        ax.set_xlabel("waypoint")
        ax.set_ylabel("angle (rad)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # 第 6 个子图：每个关节的最大误差柱状图
    ax = axes[5]
    ax.bar(JOINT_NAMES, max_err_deg, color="tab:red", alpha=0.75)
    ax.set_title("max abs error per joint (deg)")
    ax.set_ylabel("deg")
    ax.tick_params(axis="x", rotation=30)
    ax.grid(True, axis="y", alpha=0.3)

    fig.suptitle("Sim2real: planned vs actual joint trajectory", fontsize=13)
    fig.tight_layout()
    fig.savefig(OUT_PATH, dpi=150)

    print("saved:", OUT_PATH)
    print("per-joint max error (deg):", dict(zip(JOINT_NAMES, np.round(max_err_deg, 2))))
    print("overall max error (deg): %.2f" % max_err_deg.max())


if __name__ == "__main__":
    main()
