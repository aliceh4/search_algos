# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)

import queue
import heapq
from copy import deepcopy

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    queue = []
    visited = set() # use a set to keep track of visited states
    queue.append([maze.start])
    goal = maze.waypoints[0] # only one waypoint...
    while queue: # while our queue is not empty
        cur_path = queue.pop(0)
        cur_row, cur_col = cur_path[-1]
        if (cur_row, cur_col) in visited:
            continue
        visited.add((cur_row, cur_col))
        if ((cur_row, cur_col) == goal):
            return cur_path
        for item in maze.neighbors(cur_row, cur_col):
            if item not in visited:
                queue.append(cur_path + [item]) # will keep on appending to cur_path
    return []
     

def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    start = maze.start
    end = maze.waypoints[0]
    return get_cost(maze, start, end)


class ctor:
    def __init__(self, row, col, cost, tcost):
        self.row = row
        self.col = col
        self.position = (row, col)
        self.sofarcost = 0
        self.cost = cost  # heuristic
        self.tcost = tcost  # f = g + h（total）
        self.prev = None
        self.not_visited = []
        self.objective_left = []

    def __lt__(self, other):
        return self.tcost < other.tcost

def astar_corner(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    return astar_multiple(maze)

def astar_multiple(maze):
    """
    Runs A star for part 4 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    num_of_state = 0
    start = maze.start
    goals_left = maze.waypoints
    goals_left = list(goals_left)
    goals_left.insert(0, start)
    edge_list = {}
    heuristic_list = {}
    for i in goals_left:
        for j in goals_left:
            if i != j:
                construct_path = get_cost(maze, i, j)
                edge_list[(i, j)] = construct_path
                heuristic_list[(i, j)] = len(construct_path)
                num_of_state += 10
    not_visited_list = {}
    visited = {}    
    cur_path = queue.PriorityQueue()
    mst_weights = get_MST(maze, goals_left, heuristic_list)
    start_r, start_c = maze.start
    start_state = ctor(start_r, start_c, 0, mst_weights)
    start_state.not_visited = maze.waypoints

    cur_path.put(start_state)
    not_visited_list[(start_r, start_c)] = len(start_state.not_visited)

    while len(goals_left):
        cur_state = cur_path.get()
        if not cur_state.not_visited:
            break
        for n in cur_state.not_visited:
            n_row, n_col = n
            n_cost = cur_state.cost + \
                heuristic_list[(cur_state.position, n)] - 1
            next_state = ctor(n_row, n_col, n_cost, 0)
            next_state.prev = cur_state
            next_state.not_visited = deepcopy(cur_state.not_visited)
            if n in next_state.not_visited:
                listx = list(next_state.not_visited)
                listx.remove(n)
                next_state.not_visited = tuple(listx)

            visited[(n_row, n_col)] = 0
            not_visited_list[n] = len(next_state.not_visited)
            mst_weights = get_MST(maze, cur_state.not_visited, heuristic_list)
            next_state.tcost = n_cost + mst_weights
            a = len(goals_left) - 1
            if a:
                next_state.tcost += len(next_state.not_visited)
            cur_path.put(next_state)
    ret_path1 = print_path(maze, edge_list, cur_state, visited)
    return ret_path1

def get_cost(maze, start, end):
    pq = queue.PriorityQueue()
    visited = {}
    result_row, result_col = end
    start_row, start_col = start
    cost = abs(start_row-result_row) + abs(start_col - result_col)
    pq.put((cost, [(start_row, start_col)]))
    while not pq.empty():
        cur_path = pq.get()[1]
        cur_row, cur_col = cur_path[-1]
        if (cur_row, cur_col) in visited:
            continue
        cur_cost = abs(cur_row - result_row) + \
            abs(cur_col - result_col) + len(cur_path) - 1
        visited[(cur_row, cur_col)] = cur_cost
        if (cur_row, cur_col) == (result_row, result_col):
            return cur_path
        for item in maze.neighbors(cur_row, cur_col):
            new_cost = abs(item[0] - result_row) + \
                abs(item[1] - result_col) + len(cur_path) - 1
            if item not in visited:
                pq.put((new_cost, cur_path + [item]))
            else:
                if visited[item] > new_cost:
                    visited[item] = new_cost
                    pq.put((new_cost, cur_path + [item]))
    return 


def get_MST(maze, goals, heuristic_list):
    # Prim
    if not len(goals):
        return 0
    start = goals[0]
    visited = {}
    visited[start] = True
    MST_edges = []
    mst_weights = 0
    while len(visited) < len(goals):
        qe = queue.PriorityQueue()
        for v in visited:
            for n in goals:
                if visited.get(n) == True:
                    continue
                new_edge = (v, n)
                new_cost = heuristic_list[new_edge]-2
                qe.put((new_cost, new_edge))
        add_edge = qe.get()
        MST_edges.append(add_edge[1])
        mst_weights += add_edge[0]
        visited[add_edge[1][1]] = True
    return mst_weights

def print_path(maze, path, state, visited):
    ret_path = []
    goals_list = []
    while state:
        goals_list.append(state.position)
        state = state.prev
    total_dot = len(goals_list)-1
    for i in range(total_dot):
        ret_path += path[(goals_list[i], goals_list[i+1])][:-1]
    start = maze.start
    ret_path.append(start)
    ret_path = ret_path[::-1]
    return ret_path

def fast(maze):
    """
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []
    
            
