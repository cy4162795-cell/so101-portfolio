# =============================================================================
# SO-101 机械臂运动学工具包
# 功能：URDF 解析、正运动学(FK)、雅可比矩阵、逆运动学(IK)、轨迹规划、可视化
# =============================================================================

import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import os
from collections import defaultdict


# =============================================================================
# 一、基础数学工具
# =============================================================================

def rpy_to_R(roll, pitch, yaw):
    """
    把 URDF 的 rpy（Roll-Pitch-Yaw）转成 3×3 旋转矩阵。

    参数:
        roll, pitch, yaw: 绕 X/Y/Z 轴的旋转角（弧度）

    返回:
        3×3 旋转矩阵

    说明:
        URDF 规定：先绕 X 转 roll，再绕 Y 转 pitch，最后绕 Z 转 yaw。
        矩阵形式：R = Rz @ Ry @ Rx（先转的放右边）。
    """
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0,              1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    return Rz @ Ry @ Rx


def origin_to_T(xyz, rpy):
    """
    把 URDF 的 xyz（平移）+ rpy（旋转）拼成 4×4 齐次变换矩阵。

    参数:
        xyz: 长度为 3 的平移向量 (x, y, z)
        rpy: 长度为 3 的旋转角 (roll, pitch, yaw)

    返回:
        4×4 齐次变换矩阵

    结构:
        T = [ R  w ]
            [ 0  1 ]
        R: 3×3 旋转矩阵
        w: 3×1 平移向量
    """
    roll, pitch, yaw = rpy
    R = rpy_to_R(roll, pitch, yaw)
    w = np.array(xyz, dtype=float).reshape(3, 1)
    T = np.block([
        [R, w],
        [np.zeros((1, 3)), np.array([[1.0]])]
    ])
    return T


def R_z(theta):
    """
    绕 Z 轴旋转 theta 弧度的 4×4 齐次变换矩阵。

    参数:
        theta: 旋转角（弧度）

    返回:
        4×4 旋转矩阵

    注意:
        只适用于旋转轴为 Z 轴的情况。当前 URDF 所有关节 axis 都是 (0,0,1)，
        但实际旋转轴方向由 origin 的 rpy 决定。
    """
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])


def joint_T(xyz, rpy, theta):
    """
    单个关节的完整变换 = origin（固定安装） × 旋转（关节运动）。

    参数:
        xyz: 关节安装位置
        rpy: 关节安装姿态
        theta: 关节角度（弧度）

    返回:
        4×4 变换矩阵

    注意:
        这是辅助函数，主流程用 fk_gripper_frame。
    """
    o = origin_to_T(xyz, rpy)
    t = R_z(theta)
    return o @ t


# =============================================================================
# 二、URDF 解析
# =============================================================================

def parse_urdf(path):
    """
    解析 URDF 文件，提取每个关节的关键信息。

    参数:
        path: URDF 文件路径

    返回:
        关节字典列表，每个字典包含:
            - name:   关节名
            - type:   关节类型 (revolute / fixed / ...)
            - xyz:    安装位置 (3,)
            - rpy:    安装姿态 (3,)
            - axis:   旋转轴 (3,)
            - parent: 父连杆名
            - child:  子连杆名
            - lower:  角度下限（fixed 关节为 None）
            - upper:  角度上限（fixed 关节为 None）
    """
    root = ET.parse(path).getroot()
    joints = []

    for j in root.findall("joint"):
        # 基本信息
        name = j.attrib["name"]
        jtype = j.attrib.get("type", "fixed")

        # 父子连杆
        parent = j.find("parent").attrib["link"]
        child = j.find("child").attrib["link"]

        # 关节限位（fixed 关节没有 <limit>）
        limit_el = j.find("limit")
        if limit_el is not None:
            lower = float(limit_el.attrib.get("lower", "-inf"))
            upper = float(limit_el.attrib.get("upper", "inf"))
        else:
            lower = None
            upper = None

        # 安装位置和姿态
        origin = j.find("origin")
        xyz = np.array([float(v) for v in origin.attrib["xyz"].split()])
        rpy = np.array([float(v) for v in origin.attrib["rpy"].split()])

        # 旋转轴（默认 Z 轴）
        axis_el = j.find("axis")
        axis = (np.array([float(v) for v in axis_el.attrib["xyz"].split()])
                if axis_el is not None else np.array([0.0, 0.0, 1.0]))

        joints.append({
            "name": name, "type": jtype,
            "xyz": xyz, "rpy": rpy, "axis": axis,
            "parent": parent, "child": child,
            "upper": upper, "lower": lower
        })

    return joints


# =============================================================================
# 三、运动链构建（DFS）
# =============================================================================

def build_chain(joints, start="base_link", target="gripper_frame_link"):
    """
    用深度优先搜索，找一条从 start 连杆到 target 连杆的关节链。

    参数:
        joints: 关节列表
        start:  起始连杆名
        target: 目标连杆名

    返回:
        关节列表，按从 start 到 target 的顺序排列

    原理:
        1. 建立 "父连杆 → 子关节列表" 的索引
        2. DFS 从 start 出发，逐个尝试子关节
        3. 找到目标就返回 True，路径保留；走不通就 pop 回溯
    """
    # 建立索引：父连杆名 → 子关节列表
    by_parent = defaultdict(list)
    for j in joints:
        by_parent[j["parent"]].append(j)

    path = []

    def dfs(link):
        if link == target:
            return True
        for j in by_parent.get(link, []):
            path.append(j)
            if dfs(j["child"]):
                return True
            path.pop()   #删除最后一个元素 回溯
        return False

    dfs(start)
    return path


# =============================================================================
# 四、正运动学（FK）
# =============================================================================

def fk_gripper_frame(joints, q):
    """
    正运动学：给定关节角度，算末端 gripper_frame_link 的 4×4 位姿。

    参数:
        joints: 关节列表
        q:      关节角度列表（长度 = 主动关节数）

    返回:
        4×4 齐次变换矩阵（末端在 base_link 坐标系中的位姿）

    步骤:
        1. build_chain 拿到关节链
        2. 从单位矩阵开始，逐个关节累乘变换
        3. 每个关节：origin_to_T（固定安装） × R_z(qval)（关节旋转）
    """
    chain = build_chain(joints)
    T = np.eye(4)
    for j, qval in zip(chain, list(q) + [0]):
        Ti = origin_to_T(j["xyz"], j["rpy"])
        if j["type"] == "revolute":
            Ti = Ti @ R_z(qval)
        T = T @ Ti
    return T


# =============================================================================
# 五、雅可比矩阵（数值法）
# =============================================================================

def jacobian(joints, q, eps=1e-4):
    """
    数值法计算雅可比矩阵，形状 (3, n)。

    参数:
        joints: 关节列表
        q:      关节角度
        eps:    数值微分步长

    返回:
        3×n 雅可比矩阵

    原理:
        对每个关节 i，单独加一个扰动 eps，看末端位置变化多少。
        J[:, i] = (tip_i - tip_0) / eps
    """
    tip_0 = fk_gripper_frame(joints, q)[:3, 3]
    q = np.array(q, dtype=float)
    n = len(q)

    J = np.zeros((3, n))
    for i in range(n):
        q_1 = q.copy()
        q_1[i] = q_1[i] + eps
        tip_i = fk_gripper_frame(joints, q_1)[:3, 3]
        J[:, i] = (tip_i - tip_0) / eps

    return J


# =============================================================================
# 六、逆运动学（DLS）
# =============================================================================

def ik(joints, target, q_0, lam=0.1, step=200, eps=1e-4):
    """
    阻尼最小二乘法（DLS）逆运动学。

    参数:
        joints: 关节列表
        target: 目标位置 (x, y, z)
        q_0:    初始关节角度
        lam:    阻尼因子（越大越稳，越小越快）
        step:   最大迭代次数
        eps:    收敛阈值

    返回:
        (q, e): 解出的关节角度，最终误差向量

    公式:
        dq = J.T @ inv(J J.T + λ² I) @ e
        q += dq
        q = clip(q, lower, upper)
    """
    q = np.array(q_0, dtype=float)

    # 从链条里提取 revolute 关节的限位（顺序和 q 一致）
    chain = build_chain(joints)
    revolute_joints = [j for j in chain if j["type"] == "revolute"]
    lower = [j["lower"] if j["lower"] is not None else -np.inf for j in revolute_joints]
    upper = [j["upper"] if j["upper"] is not None else  np.inf for j in revolute_joints]

    for i in range(step):
        tip = fk_gripper_frame(joints, q)[:3, 3]
        e = target - tip

        # 收敛判断
        if np.linalg.norm(e) < eps:
            reached = True
            break

        # DLS 更新
        J = jacobian(joints, q, eps=1e-4)
        dq = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3)) @ e
        q += dq

        # 关节限位裁剪
        q = np.clip(q, lower, upper)

    return q, e


# =============================================================================
# 七、轨迹规划（直线 + 热启动）
# =============================================================================

def line_waypoint(A, B, N=50):
    """
    生成从 A 到 B 的直线上的 N 个 waypoint（线性插值）。

    参数:
        A: 起点 (3,)
        B: 终点 (3,)
        N: waypoint 数量

    返回:
        (N, 3) 数组，每一行是一个 3D 位置

    原理:
        waypoint_i = A + (B - A) * t_i
        t_i = i / (N-1)，从 0 到 1
    """
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    t = np.linspace(0, 1, N)[:, None]   # (N, 1)，用于广播
    return A + (B - A) * t               # (N, 3)


def plan_trajectory(A, B, N, q_start):
    """
    规划从 A 到 B 的直线轨迹，返回每个 waypoint 的关节角度。

    参数:
        A:       起点 (x, y, z)
        B:       终点 (x, y, z)
        N:       waypoint 数量
        q_start: 初始关节角度

    返回:
        关节角度列表，每个元素是长度 n 的数组

    原理:
        对每个 waypoint 调用 IK，用上一步的解作为下一步的初始值（热启动），
        保证解的连续性，收敛也更快。
    """
    waypoints = line_waypoint(A, B, N)
    qs = []
    q = np.array(q_start, dtype=float)

    for wp in waypoints:
        q, _ = ik(joints, wp, q)     # 用上一步的 q 作为初始值
        qs.append(q.copy())           # 复制一份存进列表

    return qs


# =============================================================================
# 八、可视化
# =============================================================================

def fk_link_positions(joints, q):
    """
    返回整条臂的关节点坐标，用于画骨架。

    参数:
        joints: 关节列表
        q:      关节角度

    返回:
        (7, 3) 数组：base + 6 个连杆的坐标
    """
    chain = build_chain(joints)
    pts = [np.zeros(3)]              # base_link 在原点
    T = np.eye(4)

    for j, qval in zip(chain, list(q) + [0.0]):
        Ti = origin_to_T(j["xyz"], j["rpy"])
        if j["type"] == "revolute":
            Ti = Ti @ R_z(qval)
        T = T @ Ti
        pts.append(T[:3, 3].copy())

    return np.array(pts)


def visualize_trajectory(A, B, N=20, q_start=None):
    """
    可视化直线轨迹：末端走过的路径 + 首/中/尾三帧的机械臂骨架。

    参数:
        A, B:    起点和终点
        N:       waypoint 数量
        q_start: 初始关节角度
    """
    qs = plan_trajectory(A, B, N, q_start)
    tips = np.array([fk_gripper_frame(joints, q)[:3, 3] for q in qs])

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    # 末端轨迹
    ax.plot(tips[:, 0], tips[:, 1], tips[:, 2], "b-o", lw=2, ms=3, label="tip trajectory")

    # 起点 / 终点
    ax.scatter(*np.array(A, dtype=float), c="g", s=90, marker="*", label="start A")
    ax.scatter(*np.array(B, dtype=float), c="r", s=90, marker="*", label="end B")

    # 画首 / 中 / 尾三帧的机械臂骨架
    for idx in [0, N // 2, N - 1]:
        pts = fk_link_positions(joints, qs[idx])
        color = plt.cm.viridis(idx / (N - 1))
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", lw=3, ms=4, color=color)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("SO-101 straight-line trajectory")
    ax.set_box_aspect([1, 1, 1])
    ax.legend()
    plt.show()


# =============================================================================
# 九、主程序
# =============================================================================

if __name__ == "__main__":
    HERE = os.path.dirname(os.path.abspath(__file__))
    # 1. 解析 URDF
    joints = parse_urdf(os.path.join(HERE, "..", "urdf", "so101_new_calib.urdf"))

    # 2. 初始关节角度
    q = [0.07363107781851078, -1.2256506495206274, 1.3560390164909069,
         -1.2256506495206274, -1.2256506495206274]

    # 3. 规划一条从 A 到 B 的直线轨迹
    p_t = plan_trajectory(
        A=(0.30,0,0.20),
        B=(0.25,0.15,0.25),
        N=20,
        q_start=[0.0736, -1.2257, 1.356, -1.2257, 1.6828],
    )

    # 4. 打印结果并保存
    np.save(os.path.join(HERE, "plan_trajectory.npy"), p_t)
    np.set_printoptions(precision=4, suppress=True)
    print("轨迹 (每个 waypoint 一行的 5 个关节角度):")
    print(np.array(p_t))

    # 5. 可视化（取消注释即可看到 3D 图）
    # visualize_trajectory(
    #     A=(0.30, 0, 0.20), B=(0.25, 0.15, 0.25), N=20,
    #     q_start=[0.0736, -1.2257, 1.356, -1.2257, 1.6828]
    # )
