import os
import random
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 固定随机种子，方便复现同一张图
random.seed(3)

START = (0.0, 0.0)      # 起点
GOAL = (10.0, 10.0)     # 终点
STEP = 0.6              # 每轮最多长多长
MAX_ITER = 3000         # 最多撒多少个点
GOAL_RADIUS = 0.8       # 离终点多近算「到了」

# 障碍物：几个圆 (圆心x, 圆心y, 半径)
obstacles = [(4.0, 4.0, 1.2), (6.5, 6.5, 1.0), (3.0, 7.5, 1.0), (7.0, 3.0, 1.0)]

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def collide(a, b):
    """判断线段 a-b 是否碰到任何一个障碍圆（沿线段采 20 个点检查，教学版够用）"""
    n = 20
    for i in range(n + 1):
        x = a[0] + (b[0] - a[0]) * i / n
        y = a[1] + (b[1] - a[1]) * i / n
        for (cx, cy, r) in obstacles:
            if math.hypot(x - cx, y - cy) < r:
                return True
    return False

nodes = [START]
parent = {START: None}

goal_reached = False
for _ in range(MAX_ITER):
    if goal_reached:
        break
    # 随机撒点；每 10 次里有一次直接撒到终点，让它更快摸到门
    if random.random() < 0.1:
        x_rand = GOAL
    else:
        x_rand = (random.uniform(-0.5, 10.5), random.uniform(-0.5, 10.5))
    # 找树上离它最近的点
    x_near = min(nodes, key=lambda p: dist(p, x_rand))
    d = dist(x_near, x_rand)
    if d < 1e-6:
        continue
    # 朝 x_rand 方向走固定一步
    x_new = (x_near[0] + (x_rand[0] - x_near[0]) / d * STEP,
             x_near[1] + (x_rand[1] - x_near[1]) / d * STEP)
    # 碰撞检测：撞了就作废这一轮
    if collide(x_near, x_new):
        continue
    parent[x_new] = x_near
    nodes.append(x_new)
    # 够近了就把终点也接进来，宣布到达
    if dist(x_new, GOAL) < GOAL_RADIUS:
        parent[GOAL] = x_new
        nodes.append(GOAL)
        goal_reached = True

# 从终点顺「爸爸」链走回起点
path = []
if goal_reached:
    p = GOAL
    while p is not None:
        path.append(p)
        p = parent[p]
    path = path[::-1]

fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
for ax in axes:
    for (cx, cy, r) in obstacles:
        ax.add_patch(plt.Circle((cx, cy), r, color='gray', alpha=0.5))
    ax.plot(START[0], START[1], 'go', markersize=10, label='start')
    ax.plot(GOAL[0], GOAL[1], 'r*', markersize=14, label='goal')
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 11)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)

ax = axes[0]
for p, q in parent.items():
    if q is not None:
        ax.plot([q[0], p[0]], [q[1], p[1]], 'b-', lw=0.6, alpha=0.4)
ax.set_title('Tree growth (blue edges = collision-free)')

ax = axes[1]
for p, q in parent.items():
    if q is not None:
        ax.plot([q[0], p[0]], [q[1], p[1]], 'b-', lw=0.6, alpha=0.4)
if path:
    px = [p[0] for p in path]
    py = [p[1] for p in path]
    ax.plot(px, py, 'r-', lw=2.5, label='path')
ax.legend()
ax.set_title('Backtrack from goal -> red path')

plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rrt_demo.png'), dpi=130)
print('树节点数:', len(nodes))
print('路径点数:', len(path))
