# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018
# Modified by Shang-Tse Chen (stchen@csie.ntu.edu.tw) on 03/03/2022

"""
This is the main entry point for HW1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)
import heapq
from collections import deque
from functools import lru_cache

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "fast": fast,
    }.get(searchMethod)(maze)


def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    start = maze.getStart()
    goals = set(maze.getObjectives())
    queue = deque([(start, goals, [])])
    visited = set()

    while queue:
        current, remaining_goals, path = queue.popleft()

        if current in remaining_goals:
            remaining_goals = set(remaining_goals) - {current}

        if not remaining_goals:
            return path + [current]

        if (current, tuple(remaining_goals)) in visited:
            continue
        visited.add((current, tuple(remaining_goals)))

        for neighbor in maze.getNeighbors(*current):
            if (neighbor, tuple(remaining_goals)) not in visited:
                queue.append((neighbor, remaining_goals, path + [current]))
    return []


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    start = maze.getStart()  # 取得起點
    goals = maze.getObjectives()  # 取得所有終點
    if not goals:
        return []

    working_queue = [(0, start, [])]  # (cost, current_position, path)
    visited = set()

    while working_queue:
        cost, current, path = heapq.heappop(working_queue)
        if current in visited:
            continue
        visited.add(current)

        if current in goals:
            return path + [current]  # 找到最短路徑

        for neighbor in maze.getNeighbors(current[0], current[1]):
            if neighbor not in visited:
                new_cost = cost + 1 + _manhattan_distance(neighbor, goals[0])  # A* 評估函數
                heapq.heappush(working_queue, (new_cost, neighbor, path + [current]))
    return []
def heuristic_corner(position, unvisited_corners):
    return min(_manhattan_distance(position, corner) for corner in unvisited_corners)

def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here
    start = maze.getStart()
    corners = set(maze.getObjectives())  # 角落為目標點
    visited = set()

    working_queue = [(0, start, corners, [])]  # (cost, 當前位置, 剩餘角落, 路徑)

    while working_queue:
        cost, current, remaining_corners, path = heapq.heappop(working_queue)

        if current in remaining_corners:
            remaining_corners = set(remaining_corners) - {current}

        if not remaining_corners:
            return path + [current]  # 所有角落都經過

        if (current, tuple(remaining_corners)) in visited:
            continue
        visited.add((current, tuple(remaining_corners)))

        for neighbor in maze.getNeighbors(*current):
            if (neighbor, tuple(remaining_corners)) not in visited:
                new_cost = cost + 1 + heuristic_corner(neighbor,remaining_corners)
                heapq.heappush(working_queue, (new_cost, neighbor, remaining_corners, path + [current]))
    return []


def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    start = maze.getStart()
    goals = set(maze.getObjectives())
    visited = set()

    # 優先佇列 (cost, 當前位置, 剩餘目標, 路徑)
    working_queue = [(0, start, goals, [])]

    while working_queue:
        cost, current, remaining_goals, path = heapq.heappop(working_queue)
        if current in remaining_goals:
            remaining_goals = set(remaining_goals) - {current}

        if not remaining_goals:
            return path + [current]  # 所有目標已達成

        if (current, tuple(remaining_goals)) in visited:
            continue
        visited.add((current, tuple(remaining_goals)))

        for neighbor in maze.getNeighbors(*current):
            if (neighbor, tuple(remaining_goals)) not in visited:
                new_cost = cost + 1 + _admissible_heuristic_for_points(remaining_goals)
                heapq.heappush(working_queue, (new_cost, neighbor, remaining_goals, path + [current]))

    return []


def prim_mst_cost(objectives):
    if not objectives:
        return 0

    mst_cost = 0
    connected = set([objectives[0]])
    edge_candidates = {obj: (_manhattan_distance(objectives[0], obj), objectives[0]) for obj in objectives if obj != objectives[0]}
    heap = [(dist, start, end) for end, (dist, start) in edge_candidates.items()]
    heapq.heapify(heap)

    while len(connected) < len(objectives):
        cost, _, next_vertex = heapq.heappop(heap)
        if next_vertex in connected:
            continue
        mst_cost += cost
        connected.add(next_vertex)
        for obj in objectives:
            if obj not in connected:
                new_dist = _manhattan_distance(next_vertex, obj)
                if new_dist < edge_candidates.get(obj, (float('inf'),))[0]:
                    edge_candidates[obj] = (new_dist, next_vertex)
                    heapq.heappush(heap, (new_dist, next_vertex, obj))

    return mst_cost

def greedy_best_first(maze, start, goal):
    open_list = [(_manhattan_distance(start, goal), start)]
    came_from = {start: None}

    while open_list:
        open_list.sort(key=lambda x: x[0])
        current_distance, current_node = open_list.pop(0)

        if current_node == goal:
            path = []
            while current_node:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1] 

        for neighbor in maze.getNeighbors(*current_node):
            if neighbor not in came_from:
                open_list.append((_manhattan_distance(neighbor, goal), neighbor))
                came_from[neighbor] = current_node

    return None 

def fast(maze):
    start = maze.getStart()
    objectives = maze.getObjectives()
    path = []
    current = start

    while objectives:
        # Find the closest dot
        closest_dot = min(objectives, key=lambda dot: _manhattan_distance(current, dot))
        
        # Find path to the closest dot using Greedy Best-First Search
        path_to_next_dot = greedy_best_first(maze, current, closest_dot)
        if path_to_next_dot is None:
            raise ValueError("No path to the objective was found.")
        
        # Skip the first node (current position) when extending the path
        path.extend(path_to_next_dot[1:])
        
        # Update the current position and remove the reached dot from the objectives
        current = closest_dot
        objectives.remove(closest_dot)

    return path

def _admissible_heuristic_for_points(points_set: set[tuple[int, int]]) -> int:
    """根據給定的 points 計算啟發式函數值。"""
    if not points_set:
        return 0
    # 將 set 轉換為 list
    points = list(points_set)

    # 計算相鄰點之間的曼哈頓距離總和
    total_distance = 0
    for i in range(1, len(points)):
        total_distance += _manhattan_distance(points[i], points[i - 1])

    return total_distance


def _manhattan_distance(a, b):
    """
    計算兩點之間的曼哈頓距離
    @param a: 第一個點的座標, tuple(row, col)
    @param b: 第二個點的座標, tuple(row, col)
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # 曼哈頓距離