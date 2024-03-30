"""
基于python实现的hybrid_a_star算法
把汽车视为一个箭头 碰撞模型为圆
"""

import numpy as np
import reeds_shepp as rsCurve
import matplotlib.pyplot as plt
import math
import heapq
from heapdict import heapdict
import draw


def get_obstacle():
    ox, oy = [], []
    for i in range(30):
        ox.append(i)
        oy.append(0.0)
    for i in range(30):
        ox.append(30.0)
        oy.append(i)
    for i in range(31):
        ox.append(i)
        oy.append(30.0)
    for i in range(31):
        ox.append(0.0)
        oy.append(i)
    for i in range(15):
        ox.append(15)
        oy.append(i)
    # for i in range(60):
    #     ox.append(i)
    #     oy.append(0.0)
    # for i in range(60):
    #     ox.append(60.0)
    #     oy.append(i)
    # for i in range(61):
    #     ox.append(i)
    #     oy.append(60.0)
    # for i in range(61):
    #     ox.append(0.0)
    #     oy.append(i)
    # for i in range(30):
    #     ox.append(30)
    #     oy.append(i)
    return ox, oy


class Cost:
    reverse = 10
    directionChange = 150
    steerAngle = 1
    steerAngleChange = 5
    hybridCost = 50


class Car:
    maxSteerAngle = 0.4
    steerPresion = 10
    radius = 2


class Para:
    def __init__(self, ox, oy, xyResolution, yawResolution):
        minx = round(min(ox) / xyResolution)
        miny = round(min(oy) / xyResolution)
        maxx = round(max(ox) / xyResolution)
        maxy = round(max(oy) / xyResolution)
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xw = maxx - minx
        self.yw = maxy - miny
        self.xyResolution = xyResolution
        self.yawResolution = yawResolution


class Node:
    def __init__(self, ind, cost, pind):
        self.ind = ind  # list [x y yaw]
        self.cost = cost
        self.pind = pind  # list[parent.x  parent.y  parent.yaw]


def set_holonomic_motion():
    motion = [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
    return motion


def cal_index_h(node, P):
    return (node.ind[0] - P.minx) * P.xw + (node.ind[1] - P.miny)


def cal_cost_h(motion):
    return math.hypot(motion[0], motion[1])


def node_is_valid(node, ox, oy, P):
    if (
        node.ind[0] < P.minx
        or node.ind[0] > P.maxx
        or node.ind[1] < P.miny
        or node.ind[1] > P.maxy
    ):  # 不在网格范围内就算False
        return False
    for oxx, oyy in zip(ox, oy):  # 只要小于碰撞半径，就是Fasle
        if math.hypot(oxx - node.ind[0], oyy - node.ind[1]) < Car.radius:
            return False
    return True


def cal_holonomic_cost(g, ox, oy, P):
    goal_node = Node(g, 0.0, g)
    # 基本思想，从goal到处走，计算所有能到达的点的cost，到不了那就是有障碍，记为inf
    holonomic_motion = set_holonomic_motion()
    open_set_h, closed_set_h = dict(), dict()

    open_set_h[cal_index_h(goal_node, P)] = goal_node
    priority_h = []

    heapq.heappush(priority_h, (goal_node.cost, cal_index_h(goal_node, P)))

    # 从goal_node开始遍历所有能遍历的节点，计算出他们的cost，到达不了的地方设置为inf（意味着有obstacle）
    circle = 0
    while 1:
        circle = circle + 1
        # print("circle : ", circle)

        if not open_set_h:  # open_set_h空了
            break
        _, current_index = heapq.heappop(priority_h)
        current_node = open_set_h[current_index]
        open_set_h.pop(current_index)
        closed_set_h[current_index] = current_node

        # 遍历下一个节点 8种可能的方向
        for i in range(len(holonomic_motion)):
            next_node = Node(
                [
                    current_node.ind[0] + holonomic_motion[i][0],
                    current_node.ind[1] + holonomic_motion[i][1],
                ],
                current_node.cost + cal_cost_h(holonomic_motion[i]),
                current_index,
            )
            if not node_is_valid(
                next_node, ox, oy, P
            ):  # 看下一个节点是否有效，没效果直接跳过
                continue
            # tag
            next_node_index = cal_index_h(next_node, P)

            if next_node_index not in closed_set_h:
                if next_node_index in open_set_h:
                    if next_node.cost < open_set_h[next_node_index].cost:
                        open_set_h[next_node_index].cost = next_node.cost
                        open_set_h[next_node_index].pind = next_node.pind
                else:
                    open_set_h[next_node_index] = next_node
                    heapq.heappush(priority_h, (next_node.cost, next_node_index))

    holonomic_cost = [[np.inf for i in range(P.maxy)] for i in range(P.maxx)]
    for key in closed_set_h.keys():
        holonomic_cost[closed_set_h[key].ind[0]][closed_set_h[key].ind[1]] = (
            closed_set_h[key].cost
        )
    return holonomic_cost


def index(node):
    return tuple([node.ind[0], node.ind[1], node.ind[2]])


def check_reed_shepp(start_node, goal_node, ox, oy, P):
    rs_path = rsCurve.calc_all_paths(
        start_node.ind[0],
        start_node.ind[1],
        start_node.ind[2],
        goal_node.ind[0],
        goal_node.ind[1],
        goal_node.ind[2],
        maxc=Car.maxSteerAngle,
        step_size=2,
    )
    # 返回的rs_path是所有可能的情况，计算每一种路径是否会碰撞，选择第一个安全的路径
    for i in range(len(rs_path)):
        tag = 1
        x = rs_path[i].x
        y = rs_path[i].y
        yaw = rs_path[i].yaw
        for xx, yy in zip(x, y):  # 找出所有的xx， yy
            node = Node([xx, yy], 0.0, [xx, yy])
            if not node_is_valid(node, ox, oy, P):  # 只要有一个不是就退出
                tag = 0
                break
        if tag == 1:  # 相当于都ok
            return x, y, yaw
    return [], [], []


def get_simulate_motion():
    direction = 1
    simulate_motion = []
    for i in np.arange(
        Car.maxSteerAngle,
        -(Car.maxSteerAngle + Car.maxSteerAngle / Car.steerPresion),
        -Car.maxSteerAngle / Car.steerPresion,
    ):
        simulate_motion.append([i, direction])
        simulate_motion.append([i, -direction])
    return simulate_motion


def cal_cost(current_node, simulate_motion, step):
    # 计算simulate节点的cost
    new_cost = 0
    if simulate_motion[1] == -1:
        new_cost = new_cost + Cost.directionChange
        new_cost = new_cost + step * Cost.reverse
    else:
        new_cost = new_cost + step
    new_cost = new_cost + Cost.steerAngleChange * simulate_motion[0]
    cost = current_node.cost + new_cost
    return cost


def bak_track(start_node, current_node, closed_set):
    x, y, yaw = [], [], []
    while True:
        if current_node.ind == start_node.ind:  # 找到开始点
            break
        else:  # 没找到开始点，继续遍历
            x.append(current_node.ind[0])
            y.append(current_node.ind[1])
            yaw.append(current_node.ind[2])
            current_node = closed_set[current_node.pind]
    x.reverse()
    y.reverse()
    yaw.reverse()
    return x, y, yaw


def hybrid_a_star(s, g, ox, oy, P, step=1):
    x = []
    y = []
    yaw = []
    # 获得启发式地图（基于obstacle）类似于A star
    holonomic_cost = cal_holonomic_cost(g, ox, oy, P)
    # 已经获取了holonomic cost图
    # print(holonomic_cost)

    # 获得基于动力学的cost(Reed-shepp曲线)
    """
    写一个大循环 隔几次就用RS曲线试一下能不能
    如果不能  就开始simulate 用simulate出来的next-node 计算cost 和启发式cost取最大值 
    """
    open_set, closed_set = dict(), dict()
    start_node = Node(s, 0.0, s)
    goal_node = Node(g, 0.0, g)

    open_set[index(start_node)] = start_node
    cost_priority = heapdict()

    cost_priority[index(start_node)] = max(
        start_node.cost,
        Cost.hybridCost * holonomic_cost[start_node.ind[0]][start_node.ind[1]],
    )
    circle_number = 0

    simulate_motion = get_simulate_motion()
    # print(simulate_motion)
    while True:
        circle_number = circle_number + 1
        if not open_set:  # 一旦open-set空了，结束循环
            return [], [], []

        current_node_index = cost_priority.popitem()[0]
        current_node = open_set[current_node_index]

        open_set.pop(current_node_index)
        closed_set[current_node_index] = current_node

        # 看能否用RS曲线
        if circle_number == 0:  # 每n次查找一下RS曲线
            x, y, yaw = check_reed_shepp(
                current_node, goal_node, ox, oy, P
            )  # 如果用RS曲线找到，那就结束
            if x:  # x非空，说明找到了
                # 要把当前已经找到的路径保存下来
                # path = path(start_node --> current_node) + path(current_node --> goal_node)
                # 当前路径 start_node --> current_node
                x_, y_, yaw_ = bak_track(start_node, current_node, closed_set)
                return x_ + x, y_ + y, yaw_ + yaw
        # 如果不为空，那么下面继续simulate
        if (
            round(current_node.ind[0]) == goal_node.ind[0]
            and round(current_node.ind[1]) == goal_node.ind[1]
        ):
            print("WE FOUND THE PATH")
            break
        for i in range(len(simulate_motion)):
            yaw_now = (
                current_node.ind[2] + simulate_motion[i][0] * simulate_motion[i][1]
            )
            x_now = current_node.ind[0] + math.cos(yaw_now) * step
            y_now = current_node.ind[1] + math.sin(yaw_now) * step
            next_node = Node(
                [x_now, y_now, yaw_now],
                cal_cost(current_node, simulate_motion[i], step),
                current_node_index,
            )
            # 检查next-node是否可行
            if not node_is_valid(next_node, ox, oy, P):
                continue  # 要是这个节点不行，就直接跳过这一个，查看下一个

            # 到了这里说明next_node是满足碰撞要求的
            next_node_index = index(next_node)

            if next_node_index not in closed_set:
                if next_node_index not in open_set:
                    open_set[next_node_index] = next_node
                    cost_priority[next_node_index] = max(
                        next_node.cost,
                        Cost.hybridCost
                        * holonomic_cost[round(next_node.ind[0])][
                            round(next_node.ind[1])
                        ],
                    )
                # 以上的holonomic cost之所以选取round，是取整 离散化
                else:  # next-node在open-set里面
                    if next_node.cost < open_set[next_node_index].cost:
                        open_set[next_node_index] = next_node
                        cost_priority[next_node_index] = max(
                            next_node.cost,
                            Cost.hybridCost
                            * holonomic_cost[round(next_node.ind[0])][
                                round(next_node.ind[1])
                            ],
                        )

    # 这一段逻辑关系还需要处理一下
    # 怎么把RS曲线的路径和simulate的路径合在一起？
    return x, y, yaw


def main():
    print("strat!")
    # 设置参数

    s = [10, 10, np.deg2rad(90)]
    g = [20, 20, np.deg2rad(90)]

    ox, oy = get_obstacle()
    P = Para(ox, oy, xyResolution=1, yawResolution=np.deg2rad(15))

    plt.plot(ox, oy, "sk")
    plt.arrow(s[0], s[1], math.cos(s[2]), math.sin(s[2]), color="red", width=0.2)
    plt.arrow(g[0], g[1], math.cos(g[2]), math.sin(g[2]), color="blue", width=0.2)
    # plt.show()

    x, y, yaw = hybrid_a_star(s, g, ox, oy, P)

    for i in range(len(x)):
        draw.Car(x[i], y[i], yaw[i], 1.0, 3)
        plt.pause(0.1)
    plt.show()


if __name__ == "__main__":
    main()
