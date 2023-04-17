import heapq
import numpy as np
import time
from random_numbers import lst

#读取需要的文件
file_path_nodes="California Road Network's Nodes (Node ID, Longitude, Latitude).txt"
file_path_edges="California Road Network's Edges (Edge ID, Start Node ID, End Node ID, L2 Distance).txt"
nodes = np.loadtxt(file_path_nodes,delimiter=' ')
edges = np.loadtxt(file_path_edges,delimiter=' ')

def dijkstra(graph, start, end):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    queue = [(0, start)]
    previous_nodes = {node: None for node in graph}
    while queue:
        current_distance, current_node = heapq.heappop(queue)
        if current_node == end:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            return distances[end], path[::-1]
        if current_distance > distances[current_node]:
            continue
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))
    return float('inf'), []

# 将节点列表转换为字典
graph = {}
for node in nodes:
    graph[node[0]] = []

# 将边列表添加到图中
for edge in edges:
    graph[edge[1]].append((edge[2], edge[3]))
    graph[edge[2]].append((edge[1], edge[3]))


sum_time = 0
sum_path = 0
n = 0
# 起点和终点随机给定
for i in lst:
    n+=1
    start_time = time.time()
    shortest_distance, shortest_path = dijkstra(graph, i[0],i[1])
    end_time = time.time()
    one_time = end_time - start_time
    sum_time+=one_time
    sum_path+=shortest_distance
    print("当前搜索次数：", n, "，总用时：", sum_time, "，路径总权值：", sum_path)

print("平均运行时间：", sum_time / 1000, "，平均路径权值：", sum_path / 1000)

