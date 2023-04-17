import time
from random_numbers import lst
import numpy as np
from heapq import heappush, heappop
from math import radians, cos, sin, asin, sqrt

#读取需要的文件
file_path_nodes="California Road Network's Nodes (Node ID, Longitude, Latitude).txt"
file_path_edges="California Road Network's Edges (Edge ID, Start Node ID, End Node ID, L2 Distance).txt"
nodes = np.loadtxt(file_path_nodes,delimiter=' ')
edges = np.loadtxt(file_path_edges,delimiter=' ')

# 计算经纬度之间的近似距离
def calc_distance(lat1, lon1, lat2, lon2):
    # 将经纬度转换为弧度
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # Haversine 公式
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 # 地球半径，单位为千米
    return c * r/100

def A_star(start, end, nodes, edges):

    # 初始化起点的距离信息
    g = {start: 0} # 起点到自身的距离为 0
    h = calc_distance(nodes[start][1], nodes[start][2], nodes[end][1], nodes[end][2]) # 起点到终点的近似距离
    f = g[start] + h # 起点到终点的估价函数值
    open_list = [(start, f)] # open_list 用于存储待扩展的节点
    closed_list = set() # closed_list 用于存储已扩展过的节点
    parent = {start: None} # parent 用于存储节点的父节点，用于最终构造路径

    while open_list:
        # 取出 open_list 中估价函数值最小的节点
        curr_node, curr_f = heappop(open_list)

        if curr_node == end: # 如果当前节点是终点，则返回路径和路径长度
            path = [end]
            while parent[path[0]] is not None:
                path.insert(0, parent[path[0]])
            return path, g[end]

        closed_list.add(curr_node) # 将当前节点加入 closed_list 中

        # 遍历与当前节点相邻的节点
        for edge in edges:
            if edge[1] == curr_node: # 如果当前节点是边的起点，则该边的终点是当前节点的相邻节点
                neighbor_node = edge[2]
            elif edge[2] == curr_node: # 如果当前节点是边的终点，则该边的起点是当前节点的相邻节点
                neighbor_node = edge[1]
            else:
                continue
            if neighbor_node in closed_list: # 如果相邻节点已经扩展过，则跳过
                continue
            neighbor_node = int(neighbor_node)
            # 计算起点到相邻节点的距离
            dist_from_start_to_neighbor = g[curr_node] + edge[3]

            if neighbor_node not in (node[0] for node in open_list): # 如果相邻节点不在 open_list 中，将其加入 open_list
                parent[neighbor_node] = curr_node
                g[neighbor_node] = dist_from_start_to_neighbor
                h = calc_distance(nodes[neighbor_node][1], nodes[neighbor_node][2], nodes[end][1], nodes[end][2])
                f = dist_from_start_to_neighbor + h
                heappush(open_list, (neighbor_node, f))
            else: # 如果相邻节点已经在 open_list 中
                # 如果当前路径比之前的路径更优，则更新相邻节点的距离信息
                if dist_from_start_to_neighbor < g[neighbor_node]:
                    parent[neighbor_node] = curr_node
                    g[neighbor_node] = dist_from_start_to_neighbor
                    h = calc_distance(nodes[neighbor_node][1], nodes[neighbor_node][2], nodes[end][1], nodes[end][2])
                    f = dist_from_start_to_neighbor + h
                    for i, (node, f_value) in enumerate(open_list):
                        if node == neighbor_node:
                            open_list[i] = (neighbor_node, f)
                            break
        # 当前节点的相邻节点都已经扩展过，将当前节点从 open_list 中移除
        open_list = [(node, f_value) for (node, f_value) in open_list if node != curr_node]

    return None, None # 如果无法从起点到达终点，返回 None

sum_time = 0
sum_path = 0
n=0
for i in lst:
    n+=1
    start_time = time.time()
    path,one_path = A_star(i[0],i[1],nodes,edges)
    end_time = time.time()
    one_time = end_time-start_time
    sum_time+=one_time
    sum_path+=one_path
    print("当前搜索次数：", n, "，总用时：", sum_time, "，路径总权值：", sum_path)

print("平均运行时间：", sum_time / 1000, "，平均路径权值：", sum_path / 1000)



