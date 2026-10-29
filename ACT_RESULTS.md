# SO-101 ACT 模仿学习抓取（2026-10-29 里程碑）

## 结果

- 首次真机「抓取 + 放置」成功：靠近 → 张开夹爪 → 合爪夹起 → 移到目标区 → 松开。
- 成功率：仍低，如实记录（多次尝试中 1 次成功，其余在抓取落点上偏 20~30°）。
- 演示视频：[demos/act_grasp_success_20261029.mp4](demos/act_grasp_success_20261029.mp4)

## 数据与方法

- 硬件：SO-101 主从臂遥操作，单目 USB 相机 640×480@30fps。
- 数据：50 条抓取示范，用 [lerobot/trim_so101.py](lerobot/trim_so101.py) 自适应剪除每条开头/结尾的静止段，得到 `yang/so101_v8_trim`（50 条 / 24,358 帧，抓、放段保留完整）。
- 训练：ACT，batch 8，从零 24,000 步 → `--resume=true` 续训到 48,000 步。
- 部署：`n_action_steps=100`、`temporal_ensemble_coeff=null`、`max_relative_target=10`、`reset_to_initial_position=true`。

## 关键教训

1. 演示数据开头/结尾的静止发呆会让 ACT 学成「原地不动」——先剪死时间。
2. 时间集成在精细抓取阶段会把「对准/合爪/抬/放」平均成接近不动，本任务弃用。
3. 夹爪过载/夹不紧：调高 `Max_Torque_Limit`，并让 `Overload_Torque` 高于扭矩上限（见 so_follower.py 的 gripper 配置）。
4. 剩余问题：抓取瞬间手臂落点比示范平均低 20~30°，根因是 50 条示范的抓取姿态不一致，ACT 用 L1 损失平均后落在方块附近而非正确位置。

## 下一步

- 用 Diffusion Policy 做对比（`--policy.type=diffusion --batch_size=4 --steps=80000`），验证是否能改善精细抓取的落点稳定性。
