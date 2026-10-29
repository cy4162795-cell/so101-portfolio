#!/usr/bin/env python
"""Trim leading/trailing idle frames from so101_v8_full -> so101_v8_trim.

为什么需要：演示数据每条开头约 5 秒、结尾约 3 秒手臂静止不动，ACT 学会
了「开头保持不动」，rollout 时一上来就卡住。本脚本按每条的关节运动速度
自适应地剪掉开头/结尾的死时间，只保留「已经在动」的帧。
"""

from __future__ import annotations

import glob

import av
import numpy as np
import pandas as pd

from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset

SRC = "yang/so101_v8_full"
DST = "yang/so101_v8_trim"
ROOT = "/opt/lerobot-hf/lerobot"

THRESHOLD = 1.5  # 6 个关节每步变化量之和，超过它算「在动」
MARGIN_START = 3  # 运动开始前保留的帧数
MARGIN_END = 15  # 运动结束后保留的帧数


def main() -> None:
    base = f"{ROOT}/{SRC}"
    df = pd.concat(
        [pd.read_parquet(f) for f in sorted(glob.glob(f"{base}/data/chunk-*/file-*.parquet"))],
        ignore_index=True,
    ).sort_values("index").reset_index(drop=True)
    action = np.stack(df["action"].to_numpy())  # (N, 6)
    tasks = pd.read_parquet(f"{base}/meta/tasks.parquet")
    task_by_index = {int(row.task_index): idx for idx, row in tasks.iterrows()}

    # 逐条计算要保留的帧区间
    keep = np.zeros(len(df), dtype=bool)
    ranges: dict[int, tuple[int, int]] = {}
    for e in sorted(df.episode_index.unique()):
        pos = np.where(df.episode_index.to_numpy() == e)[0]
        q = action[pos]
        vel = np.abs(np.diff(q, axis=0)).sum(axis=1)
        rolling = pd.Series(vel).rolling(5, min_periods=1).mean().to_numpy()
        moving = rolling > THRESHOLD
        idx = np.where(moving)[0]
        if len(idx) == 0:
            s, t = 0, len(q) - 1
        else:
            s = max(0, int(idx[0]) - MARGIN_START)
            t = min(len(q) - 1, int(idx[-1]) + MARGIN_END)
        ranges[e] = (s, t)
        keep[pos[s] : pos[t] + 1] = True

    print(f"保留帧数: {keep.sum()} / {len(df)}，每条区间: {ranges}")

    src = LeRobotDataset(SRC, video_backend="pyav")
    enc = RGBEncoderConfig(
        vcodec="h264", pix_fmt="yuv420p", g=2, crf=23, preset="veryfast", video_backend="pyav"
    )
    dst = LeRobotDataset.create(
        repo_id=DST,
        fps=30,
        features=src.features,
        root=f"{ROOT}/{DST}",
        robot_type="so_follower",
        use_videos=True,
        video_backend="pyav",
        rgb_encoder=enc,
    )

    video_files = sorted(glob.glob(f"{base}/videos/observation.images.front/chunk-*/file-*.mp4"))
    global_idx = 0
    written = 0
    for vf in video_files:
        container = av.open(vf)
        stream = container.streams.video[0]
        for frame in container.decode(stream):
            if global_idx >= len(df):
                break
            row = df.iloc[global_idx]
            if keep[global_idx]:
                img = frame.to_ndarray(format="rgb24")  # (H, W, 3) uint8
                dst.add_frame(
                    {
                        "observation.state": np.asarray(row["observation.state"], dtype=np.float32),
                        "observation.images.front": np.asarray(img, dtype=np.uint8),
                        "action": np.asarray(row["action"], dtype=np.float32),
                        "task": task_by_index[int(row["task_index"])],
                    }
                )
                written += 1
            # 每条录完就保存一次
            nxt = global_idx + 1
            if nxt >= len(df) or df.iloc[nxt]["episode_index"] != row["episode_index"]:
                dst.save_episode()
            global_idx += 1
        container.close()

    dst.finalize()
    print(f"完成，写出帧数 {written}，数据集 {DST}")


if __name__ == "__main__":
    main()
