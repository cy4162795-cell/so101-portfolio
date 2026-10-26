import numpy as np
import xml.etree.ElementTree as ET
from collections import defaultdict
# #线性插值  waypoint_i = A + (B − A) · t_i
#   t_i = i / (N − 1)   （t 从 0 到 1）
def line_waypoint(A,B,N=50):
    A = np.array(A,dtype=float)
    B = np.array(B,dtype=float)
    t = np.linspace(0,1,N)[:,None]
    waypoint = A + (B - A)*t
    return waypoint
A=(0.30,0,0.20)
B=(0.25,0.15,0.25)
N=10 
np.set_printoptions(precision=4,suppress=True)
print(f"结果是：{line_waypoint(A,B ,N )}")