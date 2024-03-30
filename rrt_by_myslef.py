import matplotlib.pyplot as plt
import math
import numpy as np
import random
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def get_obstacle():
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

def generate_random_node(xw, yw): # 返回一个随机节点
    random_x = random.randint(1, xw-1)
    random_y = random.randint(1, yw-1)
    node = Node(random_x, random_y)

    return node 
def node_in_list(node, node_list):
    for i in node_list:
        if i.x == node.x and i.y == node.y:
            return True
    return False

def get_near_node_index(node_list, xw, yw):
    random_node = generate_random_node(xw, yw)

    while 1:
        if not(random_node.x <= 0 or random_node.x >= xw or random_node.y <= 0 or random_node.y >= yw or node_in_list(random_node, node_list)):
            break
        else:
            random_node = generate_random_node(xw, yw)
        
    dlist = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2
                for node in node_list]
    minind = dlist.index(min(dlist))
    # print("random_node x{} y{}".format(random_node.x, random_node.y))
    return minind, random_node
def check_node(near_node, random_node, ox, oy, rr):# 有问题
    # 终于知道问题在哪里了，只能在最后return true，要不然障碍没判断完就结束了
    A = random_node.y - near_node.y
    B = near_node.x - random_node.x
    C = random_node.x * near_node.y - near_node.x * random_node.y

    if near_node.x == random_node.x: # 竖线
        for oxx, oyy in zip(ox, oy):
            maxy = max(near_node.y, random_node.y)
            miny = min(near_node.y, random_node.y)
            if oyy > maxy or oyy < miny: # 一开始逻辑关系写错了
                distance_to_point_1 = math.hypot(oxx - near_node.x, oyy - near_node.y)
                distance_to_point_2 = math.hypot(oxx - random_node.x, oyy - random_node.y)
                if distance_to_point_1 <= rr or distance_to_point_2 <= rr:
                    return False
            else:
                distance_to_line = abs(near_node.x - oxx)
                if distance_to_line <= rr:
                    return False
                
    elif random_node.y == near_node.y: # 横线
        for oxx, oyy in zip(ox, oy):
            maxx = max(near_node.x, random_node.x)
            minx = min(near_node.x, random_node.x)
            if oxx > maxx or oxx < minx: # 不在线段内
                distance_to_point_1 = math.hypot(oxx - near_node.x, oyy - near_node.y)
                distance_to_point_2 = math.hypot(oxx - random_node.x, oyy - random_node.y)
                if distance_to_point_1 <= rr or distance_to_point_2 <= rr:
                    return False
            else: # 在线段内
                distance_to_line = abs(near_node.y - oyy)
                if distance_to_line <= rr:
                    return False
    else:
        k = (random_node.y - near_node.y)/(random_node.x - near_node.x)
        b = near_node.y - k * near_node.x
        k_ = -1/k
        for oxx, oyy in zip(ox, oy):
            maxy = max(near_node.y, random_node.y)
            miny = min(near_node.y, random_node.y)
            if maxy == near_node.y:
                maxy_x = near_node.x
                miny_x = random_node.x
            else:
                maxy_x = random_node.x
                miny_x = near_node.x

            maxy_ = maxy + k_ * (oxx - maxy_x)
            miny_ = miny + k_ * (oxx - miny_x)
            if oyy > miny_ and oyy < maxy_: # 投影在线段上
                distance_to_line = abs(A*oxx + B*oyy + C)/math.sqrt(A**2 + B**2)
                if distance_to_line <= rr:
                    return False
            else: # 投影不在线段上，最近的点为端点
                distance_to_point_1 = math.hypot(oxx - near_node.x, oyy - near_node.y)
                distance_to_point_2 = math.hypot(oxx - random_node.x, oyy - random_node.y)
                if distance_to_point_1 <= rr or distance_to_point_2 <= rr:
                    return False
    return True

def get_near_node_index_true(node_list, xw, yw, ox, oy, rr):
    while 1:
        minind, random_node = get_near_node_index(node_list, xw, yw)
        near_node = node_list[minind]
        print("near_node x:{} y:{}\nrandom_node x:{} y:{}\n".format(near_node.x, near_node.y, random_node.x, random_node.y))
        if check_node(near_node, random_node, ox, oy, rr): # 如果合适
            print("Succuss   near_node x:{} y:{}\nrandom_node x:{} y:{}\n".format(near_node.x, near_node.y, random_node.x, random_node.y))
            return minind, near_node, random_node
        
def get_new_node(goal_node, near_node, random_node, ox, oy, rr, number, step=4):
    if number % 2 == 0 and check_node(goal_node, near_node, ox, oy, rr):
        goal_node.parent = near_node
        new_node = goal_node
    if math.hypot(near_node.x - random_node.x, near_node.y - random_node.y) < step:
        random_node.parent = near_node
        new_node = random_node
    else:
        if near_node.x == random_node.x: # 竖线
            new_node = Node(round(near_node.x), round(random_node.y) + step)
            new_node.parent = near_node  
        else: # 斜线
            k = (random_node.y - near_node.y)/(random_node.x - near_node.x)
            new_node = Node(round(near_node.x + step/(1+k**2)**0.5), round(near_node.y + step*k/(1+k**2)**0.5))
            if check_node(near_node, new_node, ox, oy, rr): # 新节点过来安全检查
                new_node.parent = near_node
            else: # 新结点没过安全检查
                new_node = Node(round(random_node.x), round(random_node.y) + step)
                new_node.parent = near_node  
    print("new_node x {} y {}".format(new_node.x, new_node.y))
    return new_node

def RRT_planning(start_node, goal_node, xw, yw, ox, oy, rr, step=10):
    print("RRT_planning start")
    node_list = [start_node]
    max_itr = 500
    number = 0
    while 1:
        number = number + 1
        minind, near_node, random_node = get_near_node_index_true(node_list, xw, yw, ox, oy, rr) # ERROR
        random_node.parent = near_node
        node_list.append(random_node)

        # plt.plot(ox, oy, 'sk')
        # plt.plot(near_node.x, near_node.y, 'sr')
        # plt.plot(random_node.x, random_node.y, 'sb')
        # plt.show()

        if node_in_list(goal_node, node_list): # end condition 
            print("找到了目标节点 RRT搜索结束")
            break
    return node_list

def cal_path(node_list, start_node, goal_node):
    print("cal_path start")
    if not node_in_list(goal_node, node_list):
        print("Fail To Find The Path!")
        return [],[]
    else:
        pathx, pathy = [goal_node.x], [goal_node.y]
        current_node = node_list[-1]
        while not (start_node.x == current_node.x and start_node.y == current_node.y): # 
            father_node = current_node.parent
            pathx.append(father_node.x)
            pathy.append(father_node.y)
            current_node = father_node

    pathx.reverse()
    pathy.reverse()
    print(pathx)
    print(pathy)
    return pathx, pathy

def opt_tra(pathx, pathy, ox, oy, rr):
    pathx_, pathy_ = [pathx[0]],[pathy[0]]
    length = len(pathx)
    for i in range(length):
        for j in range(i+1, length):
            node_1 = Node(pathx[i], pathy[i])
            node_2 = Node(pathx[j], pathy[j])
            if check_node(node_1, node_2, ox, oy, rr):
                temp_node = node_2
            else:
                i = j - 1
                break
        pathx_.append(temp_node.x)
        pathy_.append(temp_node.y)
    return pathx_, pathy_

def main():
    print("main strat")
    #set param
    s_x, s_y = 10, 10 # start_node
    g_x, g_y = 50, 50 # goal_node
    xw = 60
    yw = 60
    rr = 4
    step = 10 # 最小步长

    ox, oy = get_obstacle()
    start_node = Node(s_x, s_y)
    goal_node = Node(g_x, g_y)

    node_list = RRT_planning(start_node, goal_node, xw, yw, ox, oy, rr) #目前问题是陷入了死循环

    pathx, pathy = cal_path(node_list, start_node, goal_node)

    pathx_, pathy_ = opt_tra(pathx, pathy, ox, oy, rr)
    pathx_, pathy_ = opt_tra(pathx_, pathy_, ox, oy, rr)
    # 运行一次有时候还是会有不好的点，运行两次就会好很多

    plt.plot(ox, oy, 'sk')
    # plt.plot(start_node.x, start_node.y, 'sb')
    # plt.plot(goal_node.x, goal_node.y, 'sr')
    plt.plot(pathx, pathy, 'b')
    plt.plot(pathx_, pathy_, 'r')
    plt.show()

if __name__ == "__main__":
    main()