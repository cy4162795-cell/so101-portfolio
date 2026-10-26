import random
import math
import numpy as np
import matplotlib.pyplot as plt

obstacles = [(3.0, 3.0, 1.0), (-3.0, 2.0, 1.0)]
start = (0.0,0.0)
nodes = [start]
def in_obstacles(p):
    for (cx,cy,r) in obstacles:
        if math.hypot(p[0]-cx,p[1]-cy)<r:
            return True
    return False


def dist(a,b):
    d = math.hypot(a[0]-b[0],a[1]-b[1])
    return d 

goal = (15.0, 15.0)
parent = {start: None}

found = False
for i in range(2000):
    # 目标偏向：每 10 轮抽 1 轮直接撒到终点，其余在起点~终点附近撒
    if random.random() < 0.1:
        x_rand = goal
    else:
        x_rand = (random.uniform(-1, 16), random.uniform(-1, 16))
    x_near = min(nodes, key=lambda p: dist(p, x_rand))
    d = dist(x_near, x_rand)
    if d < 1e-6:      # 随机点恰好落在已有点上，防除以 0
        continue
    step = 0.5
    x_new = (x_near[0] + (x_rand[0] - x_near[0]) / d * step,
             x_near[1] + (x_rand[1] - x_near[1]) / d * step)
    if in_obstacles(x_new):
        continue

    parent[x_new] = x_near
    nodes.append(x_new)

    if dist(x_new, goal) < 0.8:
        parent[goal] = x_new
        found = True
        print("第", i+1, "轮摸到终点了！")
        break
    # 使用 f-string 并限制保留 2 位小数
    print(f"第 {i+1:2d} 轮 | 随机点 ({x_rand[0]:.2f}, {x_rand[1]:.2f}) | 最近点 ({x_near[0]:.2f}, {x_near[1]:.2f})| 新点 ({x_new[0]:.2f}, {x_new[1]:.2f})")

if not found:
    print("循环跑完了还是没摸到终点：把 range 调大，或检查 goal 与撒点范围是否匹配")
    exit()

path = []
p = goal
while p is not None:
    path.append(p)
    p = parent[p]
path.reverse()
print(f"路径点：{np.array(path)}")
fig, ax = plt.subplots()
for (cx, cy, r) in obstacles:
    ax.add_patch(plt.Circle((cx, cy), r, color='gray', alpha=0.5))

for child, dad in parent.items():
    if dad is not None:
        ax.plot([dad[0], child[0]], [dad[1], child[1]], 'b-', lw=0.5, alpha=0.4)

px = [p[0] for p in path]
py = [p[1] for p in path]
ax.plot(px, py, 'r-', lw=2)

ax.plot(start[0], start[1], 'go', markersize=10)
ax.plot(goal[0], goal[1], 'r*', markersize=14)
ax.set_aspect('equal')
plt.show()

