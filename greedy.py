import heapq
import math
import time
import numpy as np
from random_numbers import lst
from math import radians, cos, sin, asin, sqrt

#读取需要的文件
file_path_nodes="California Road Network's Nodes (Node ID, Longitude, Latitude).txt"
file_path_edges="California Road Network's Edges (Edge ID, Start Node ID, End Node ID, L2 Distance).txt"
nodes = np.loadtxt(file_path_nodes,delimiter=' ')
edges = np.loadtxt(file_path_edges,delimiter=' ')


def calc_distance(lat1, lon1, lat2, lon2):
    """计算两个坐标点之间的距离"""
    # 将经纬度转换为弧度
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # Haversine 公式
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 # 地球半径，单位为千米
    return c * r/100

def Greedy_best_first_search(start, end, nodes, edges):
    visited = set()
    heap = [(calc_distance(nodes[start][1], nodes[start][2], nodes[end][1], nodes[end][2]), start)]
    parent = {}
    dist = {}

    for node in nodes:
        parent[node[0]] = None
        dist[node[0]] = math.inf
    dist[start] = 0

    while heap:
        (f, current_node) = heapq.heappop(heap)
        visited.add(current_node)

        if current_node == end:
            path = []
            node = end
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            return path, dist[end]

        for edge in edges:
            if current_node == edge[1]:
                neighbor_node = edge[2]
            elif current_node == edge[2]:
                neighbor_node = edge[1]
            else:
                continue
            neighbor_node = int(neighbor_node)

            if neighbor_node not in visited:
                new_cost = dist[current_node] + edge[3]
                if new_cost < dist[neighbor_node]:
                    dist[neighbor_node] = new_cost
                    heapq.heappush(heap, (calc_distance(nodes[neighbor_node][1], nodes[neighbor_node][2], nodes[end][1], nodes[end][2]), neighbor_node))
                    parent[neighbor_node] = current_node

    return None, None

sum_time = 0
sum_path = 0
n =0
# 起点和终点随机给定
for i in lst:
    n+=1
    start_time = time.time()
    path_lst,shortest_path = Greedy_best_first_search(i[0],i[1],nodes,edges)
    end_time = time.time()
    one_time = end_time - start_time
    sum_time+=one_time
    sum_path+=shortest_path
    print("当前搜索次数：",n,"，总用时：",sum_time,"，路径总权值：",sum_path)


print("平均运行时间：",sum_time/1000,"，平均路径权值：",sum_path/1000)


