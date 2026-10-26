#!/usr/bin/env python3
"""SO-101 工作空间：蒙特卡洛采样算可达范围，或对单点做多起点 IK 验证。"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import FK_IK as fk


URDF = os.path.join(HERE, "..", "urdf", "so101_new_calib.urdf")
N_SAMPLES = 200000
SEED = 0


def build_fk():
    """解析 URDF、构建运动链，返回 FK 函数、5 关节限位和 joints 列表。"""
    joints = fk.parse_urdf(URDF)
    chain = fk.build_chain(joints)
    T_fixed = [fk.origin_to_T(j["xyz"], j["rpy"]) for j in chain]

    def fk_tip(q):
        T = np.eye(4)
        qi = 0
        for j, Tf in zip(chain, T_fixed):
            Ti = Tf.copy()
            if j["type"] == "revolute":
                qval = float(q[qi]) if qi < 5 else 0.0
                Ti = Ti @ fk.R_z(qval)
                qi += 1
            T = T @ Ti
        return T[:3, 3]

    rev = [j for j in chain if j["type"] == "revolute"][:5]
    lower = np.array([j["lower"] for j in rev])
    upper = np.array([j["upper"] for j in rev])
    return fk_tip, lower, upper, joints


def check_point(x, y, z, n_starts=40, tol=0.01, seed=1):
    """多起点 IK 验证单点是否可达：随机换 n_starts 个初始角度解 IK，取最小残余误差。"""
    _, lower, upper, joints = build_fk()
    rng = np.random.default_rng(seed)
    point = np.array([x, y, z], float)
    best = 1e9
    for _ in range(n_starts):
        q0 = rng.uniform(lower, upper)
        q, e = fk.ik(joints, point, q0)
        best = min(best, float(np.linalg.norm(e)))
    reachable = best <= tol
    print(f"point ({x:.3f}, {y:.3f}, {z:.3f}) | r={np.hypot(x, y):.3f}m | "
          f"min IK residual {best*100:.2f} cm -> {'可达' if reachable else '不可达'}")
    return reachable


def sample_workspace():
    fk_tip, lower, upper, _ = build_fk()
    rng = np.random.default_rng(SEED)
    Q = rng.uniform(lower, upper, size=(N_SAMPLES, 5))
    P = np.array([fk_tip(q) for q in Q])

    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    r = np.sqrt(x**2 + y**2)

    print("=== SO-101 工作空间边界（蒙特卡洛采样） ===")
    print(f"x: [{x.min():.3f}, {x.max():.3f}] m")
    print(f"y: [{y.min():.3f}, {y.max():.3f}] m")
    print(f"z: [{z.min():.3f}, {z.max():.3f}] m")
    print(f"r=sqrt(x^2+y^2): [{r.min():.3f}, {r.max():.3f}] m")

    print("\n=== 按高度 z 的最大可达半径 ===")
    print(f"{'z (m)':<10}{'r_max (m)':>12}")
    for zc in np.arange(0.00, 0.50, 0.05):
        mask = np.abs(z - zc) < 0.025
        if mask.sum() == 0:
            continue
        print(f"{zc:<10.2f}{r[mask].max():>12.3f}")


def main():
    if len(sys.argv) == 5 and sys.argv[1] == "--check":
        check_point(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    else:
        sample_workspace()


if __name__ == "__main__":
    main()
