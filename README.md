# SO-101 机械臂：运动控制 + 真机 sim2real（陈洋 · 求职证据包）

用桌面六自由度机械臂（SO-101，Feetech 串口伺服）从零搭起一套「建模 → 逆解 → 轨迹规划 → 真机回放 → 误差验证」的完整闭环，并额外补齐 C++ 版运动学库、ROS 2 双通道节点和 RRT 全局路径规划。目标是证明：不是「看教程跑通」，而是能独立完成真机工程落地的每一个环节。

## 一、量化结果（面试可直接引用）

| 模块 | 指标 | 数值 |
| --- | --- | --- |
| 正运动学 FK | 末端位置 vs 真机尺量 | 误差约 2 cm |
| 逆运动学 IK（DLS + 关节限位） | 可及目标收敛误差 | 约 0.1 mm |
| 笛卡尔直线轨迹规划 | 离线最大偏离直线 | 0.09 mm |
| 真机 sim2real | 关节最大跟踪误差 | 4.46°（elbow_flex） |
| 真机 sim2real | 各关节最大误差 | shoulder_pan 1.74° / shoulder_lift 2.52° / elbow_flex 4.46° / wrist_flex 2.06° / wrist_roll 0.29° |
| C++ 正运动学库 | FK 末端与 Python 版一致性 | tip ≈ (0.1785, −0.0184, 0.3106) m |

## 二、目录结构

```
.
├── kinematics/                  # Python：URDF 解析 / FK / 数值雅可比 / DLS IK / 直线轨迹 / sim2real 对比
│   ├── FK_IK.py                 # 核心库（运动学 + 逆解 + 规划 + 可视化）
│   ├── workspace.py             # 蒙特卡洛工作空间 / 多起点 IK 验证
│   ├── line.py                  # 直线插值辅助
│   ├── plan_trajectory.npy      # 规划轨迹 (20, 5)
│   ├── actual_trajectory.npy    # 真机实际轨迹 (20, 5)
│   ├── compare_sim2real.py      # 对比脚本（输出误差 + 对比图）
│   └── sim2real_compare.png     # 规划 vs 真机对比图
├── cpp/                         # C++：Eigen 正运动学库 + ROS 2 节点（topic + service）+ Feetech 串口读写
│   ├── fk.hpp / fk.cpp          # 正运动学库
│   ├── joint_state_publisher.cpp# ROS 2 节点：/joint_states 上报 + /move_to 服务控真机
│   ├── read_servo.cpp           # Feetech 串口协议最小原型（读角度 / 写 goal / 写 torque）
│   ├── CMakeLists.txt / package.xml
│   └── srv/MoveToTarget.srv
├── rrt/                         # RRT 全局路径规划（手写闭环）
│   ├── rrt.py                   # RRT：采样 / 生长 / 碰撞检测 / 回溯 / 目标偏向
│   ├── rrt_visualize.py         # 可视化
│   └── rrt_demo.png
└── urdf/
    └── so101_new_calib.urdf     # SO-101 模型（6 revolute + 1 fixed 夹爪 TCP）
```

## 三、各模块说明

### 1. 运动学与轨迹规划（kinematics/）

- **URDF 解析**：程序化读取关节 origin（xyz + rpy）与 axis，按 parent→child 从 `base_link` 建到 `gripper_frame_link`（夹爪 TCP 固定偏移），正确处理树形链路。
- **FK**：`origin_to_T`（rpy→齐次变换）⊕ `Rz(θ)` 逐关节累积。
- **IK**：数值雅可比 + 阻尼最小二乘（DLS）+ 关节限位；冗余关节取最小范数解，不可及目标顶限位保护。
- **轨迹规划**：笛卡尔直线 `line_waypoint` → 逐点 IK（热启动），20 waypoint 直线最大偏离 0.09 mm。
- **sim2real**：把规划关节序列写回真机，读回实际角度对比，得到上表误差。

### 2. C++ 与 ROS 2（cpp/）

- 用 Eigen / CMake 独立写正运动学库，FK 输出与 Python 版逐位一致。
- ROS 2 节点同时具备两个通道：`/joint_states`（10 Hz topic，上报真机 6 关节弧度）+ `/move_to`（service，真机动作闭环，返回 success）。
- 手写 Feetech 串口协议（读地址 56 / 写 goal 地址 42 / 写 torque 地址 40），处理响应帧堆积、校验和、失败重试等真机脏问题。

### 3. RRT 全局路径规划（rrt/）

- 从零手写：撒点 → 就近生长 → 点碰撞检测 → 到达判断 → 回溯 → 目标偏向（10%）→ 失败分支保护。

## 四、快速开始

- 规划（生成 `plan_trajectory.npy`，不依赖真机）：
  ```bash
  python kinematics/FK_IK.py
  ```
- 真机回放（需 SO-101 接在 `/dev/ttyACM1`，读实际角度存 `actual_trajectory.npy`）：见 `cpp/` 与 `kinematics/` 内的运行说明。
- 对比（输出误差 + `sim2real_compare.png`）：
  ```bash
  python kinematics/compare_sim2real.py
  ```
- RRT 可视化：
  ```bash
  python rrt/rrt_visualize.py
  ```

## 五、技术栈

Python · NumPy · matplotlib · ROS 2 Jazzy（rclpy/rclcpp）· C++17 · Eigen3 · CMake · Feetech SCServo 串口协议

