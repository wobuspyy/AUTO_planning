import numpy as np
import matplotlib.pyplot as plt
import math
import heapq
import time

class Node:
    def __init__(self, x, y, cost , pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

class Para:
    def __init__(self, minx, miny, maxx, maxy, xw, yw, reso, motion):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xw = xw
        self.yw = yw
        self.reso = reso
        self.motion = motion

def cal_index(node , p):
    return (node.y - p.miny)*p.xw + (node.x - p.minx)

def get_env():
    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    for i in range(10): # My Test
        ox.append(60-i)
        oy.append(40)

    return ox, oy

def get_motion():
    #定义可能探索的方向
    motion = [[-1, 0], [-1, 1], [0, 1], [1, 1],
              [1, 0], [1, -1], [0, -1], [-1, -1]]

    return motion

def calc_obsmap(ox, oy, rr, P):# 判断哪些是障碍
    obsmap = [[False for _ in range(P.yw)] for _ in range(P.xw)]
    
    for x in range(P.xw):
        xx = x + P.minx
        for y in range(P.yw):
            yy = y + P.miny
            for oxx, oyy in zip(ox, oy):
                if math.hypot(oxx-xx, oyy-yy) <= rr/P.reso:
                    obsmap[x][y] = True
    return obsmap

def calc_parameters(ox, oy, rr, reso):
    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    xw = maxx - minx
    yw = maxy - miny
    motion = get_motion()
    P = Para(minx, miny, maxx, maxy, xw, yw, reso, motion)
    obsmap = calc_obsmap(ox, oy, rr, P)
    return P, obsmap

def h(node1, node2):
    return math.hypot(node1.x-node2.x, node1.y-node2.y)

def fvalue(node1, node2):
    return node1.cost + h(node1, node2)

def u_cost(u):
    return math.hypot(u[0], u[1])

def check_node(node, P, obsmap):
    if node.x < P.minx or node.y < P.miny or node.x > P.maxx or node.y > P.maxy:
        return False
    if obsmap[node.x - P.minx][node.y - P.miny]:
        return False
    return True

def extract_path(closed_set, node_start, node_goal, P):
    pathx = [node_goal.x]
    pathy = [node_goal.y]
    n_ind = cal_index(node_goal, P)

    while True:
        node = closed_set[n_ind]
        pathx.append(node.x)
        pathy.append(node.y)
        n_ind = node.pind

        if node == node_start:
            break
    pathx = [x * P.reso for x in reversed(pathx)]
    pathy = [y * P.reso for y in reversed(pathy)]
    return pathx, pathy

def astar_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    node_start = Node(round(sx/reso), round(sy/reso), 0.0, -1)
    node_goal = Node(round(gx/reso), round(gy/reso), 0.0, -1)

    ox = [x/reso for x in ox]
    oy = [y/reso for y in oy]

    P, obsmap = calc_parameters(ox, oy, rr, reso)
    
    open_set, closed_set = dict(), dict()
    open_set[cal_index(node_start, P)] = node_start

    q_priority = []
    heapq.heappush(q_priority, (fvalue(node_start, node_goal) , cal_index(node_start, P)))

    while True:
        if not q_priority:
            break
        _, ind = heapq.heappop(q_priority)
        n_curr = open_set[ind]
        closed_set[ind] = n_curr
        open_set.pop(ind)

        for i in range(len(P.motion)): #检测不同的运动方向
            node = Node(n_curr.x + P.motion[i][0], n_curr.y + P.motion[i][1], n_curr.cost+u_cost(P.motion[i]), ind)
            if not check_node(node, P, obsmap):
                continue
            n_ind = cal_index(node, P)
            if n_ind not in closed_set:
                if n_ind in open_set:
                    if open_set[n_ind].cost > node.cost:
                        open_set[n_ind].cost = node.cost
                        open_set[n_ind].pind = ind
                else:
                    open_set[n_ind] = node
                    heapq.heappush(q_priority, ( fvalue(node, node_goal), cal_index(node, P)))

    pathx, pathy = extract_path(closed_set, node_start, node_goal, P)
    return pathx, pathy, closed_set

def cal_total_length(pathx, pathy):
    length = 0
    for i in range(len(pathx) - 1):
        length = length + math.hypot(pathx[i+1] - pathx[i] , pathy[i+1] - pathy[i])
    return length

def get_checked_point(closed_set):
    # open_set[cal_index(node_start, P)] = node_start
    checked_x = []
    checked_y = []
    for i, j in closed_set.items():
        checked_x.append(j.x)
        checked_y.append(j.y)
    return checked_x, checked_y
def main():
    sx = 10
    sy = 10
    gx = 50
    gy = 50

    robot_radius = 2.0
    grid_resolution = 1.0
    ox, oy = get_env()

    time_start = time.time()
    pathx, pathy, closed_set = astar_planning(sx, sy, gx, gy, ox, oy, grid_resolution, robot_radius)
    time_end = time.time()
    print("规划时间：", time_end - time_start)
    length = cal_total_length(pathx, pathy) # 判断规划的路径的总长度
    print("length:", length)

    # 查看总共搜索了多少点
    checked_x, checked_y = get_checked_point(closed_set)
    print("查看点的数量", len(checked_x))
    plt.plot(ox, oy, 'sk')
    plt.plot(pathx, pathy)
    # plt.plot(checked_x, checked_y)
    plt.plot(sx, sy, 'sb')
    plt.plot(gx, gy, 'sr')
    plt.show()

if __name__ == "__main__":
    main()