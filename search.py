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
    visited = set() # use a set to keep track of visited states (don't need a dict yet)
    queue.append([maze.start])
    goal = maze.waypoints[0] # only one waypoint
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
    return get_length(maze, start, end)


class Node:
    """
    Create a Node class to make our data easier to keep track of/organize
    """
    def __init__(self, row, col, cost, total_cost):
        self.position = (row, col)
        self.cost = cost  # h
        self.total_cost = total_cost  # f = g + h（total）
        self.prev = None # our previous Node
        self.not_visited = [] # list of not visited nodes

    def __lt__(self, other):
        return self.total_cost < other.total_cost

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
    start = maze.start
    goals_left = maze.waypoints
    goals_left = list(goals_left) # convert to list so we can insert... tuple can't do insert
    goals_left.insert(0, start)
    edge_list = {}
    heuristic_list = {}

    # iterate through our waypoints (including start now)
    for i in goals_left:
        for j in goals_left:
            if i != j:
                construct_path = get_length(maze, i, j) # get length from one waypoint to another
                edge_list[(i, j)] = construct_path # edge list for waypoint i to waypoint j
                heuristic_list[(i, j)] = len(construct_path) # heuristic_list will be the length of the path

    # initialize variables
    not_visited_list = {}
    visited = {}
    cur_path = queue.PriorityQueue() # will contain nodes
    mst_weights = get_MST(maze, goals_left, heuristic_list)
    startx, starty = maze.start
    start_state = Node(startx, starty, 0, mst_weights) # create new node
    start_state.not_visited = maze.waypoints

    cur_path.put(start_state)
    not_visited_list[(startx, startx)] = len(start_state.not_visited)

    # keep on iterating until there are no more goals left
    while len(goals_left):
        cur_state = cur_path.get() # get the highest priority item
        if not cur_state.not_visited: # if we have visited everything
            break
        for n in cur_state.not_visited:
            n_row, n_col = n
            n_cost = cur_state.cost + heuristic_list[(cur_state.position, n)] - 1 # subtract starting position
            next_state = Node(n_row, n_col, n_cost, 0)
            next_state.prev = cur_state
            next_state.not_visited = deepcopy(cur_state.not_visited)
            if n in next_state.not_visited:
                listx = list(next_state.not_visited) # turn to list so we can manipulate
                listx.remove(n) # remove from not visited
                next_state.not_visited = tuple(listx)

            visited[(n_row, n_col)] = 0
            not_visited_list[n] = len(next_state.not_visited)
            mst_weights = get_MST(maze, cur_state.not_visited, heuristic_list)
            next_state.total_cost = n_cost + mst_weights # update the total cost for next_state
            goal_count = len(goals_left) - 1
            if goal_count:
                next_state.total_cost += len(next_state.not_visited)
            cur_path.put(next_state)

    final_path = get_path(maze, edge_list, cur_state)
    return final_path

"""
The following are helper functions for astar_multiple
"""

def get_length(maze, start, end):
    """
    Returns the path to be taken from start to end (used in astar_single)
    """
    pq = queue.PriorityQueue()
    visited = {}
    result_row, result_col = end
    start_row, start_col = start
    cost = manhattan_distance(start_row, start_col, result_row, result_col)
    pq.put((cost, [(start_row, start_col)]))
    while not pq.empty():
        cur_path = pq.get()[1]
        cur_row, cur_col = cur_path[-1]
        if (cur_row, cur_col) in visited:
            continue
        cur_cost = manhattan_distance(cur_row, cur_col, result_row, result_col) + len(cur_path) - 1
        visited[(cur_row, cur_col)] = cur_cost
        if (cur_row, cur_col) == (result_row, result_col):
            return cur_path
        for item in maze.neighbors(cur_row, cur_col):
            new_cost = manhattan_distance(item[0], item[1], result_row, result_col) + len(cur_path) - 1
            if item not in visited:
                pq.put((new_cost, cur_path + [item]))
            else:
                if visited[item] > new_cost:
                    visited[item] = new_cost
                    pq.put((new_cost, cur_path + [item]))
    return []

def manhattan_distance(startx, starty, goalx, goaly):
    return abs(startx - starty) + abs(starty - goaly)


def get_MST(maze, goals, heuristic_list):
    # use Prims method
    if not len(goals): # if there are no goals
        return 0
    start = goals[0] # our first goal is our start
    visited = {}
    visited[start] = True # now we have visited this position
    edges = []
    weights = 0
    while len(visited) < len(goals):
        q = queue.PriorityQueue()
        for v in visited:
            for g in goals:
                if visited.get(g) == True:
                    continue
                new_edge = (v, g) # create a new edge
                new_cost = heuristic_list[new_edge] - 2 # don't forget to subtract 2
                q.put((new_cost, new_edge))
        add_edge = q.get()
        edges.append(add_edge[1])
        weights += add_edge[0]
        visited[add_edge[1][1]] = True
    return weights

def get_path(maze, path, state):
    """
    maze, edge_list, cur_state, visited
    """
    position_list = []
    while state: # keep on moving from current state to end
        position_list.append(state.position)
        state = state.prev
    total_len = len(position_list) - 1

    final_path = []
    for i in range(total_len):
        final_path += path[(position_list[i], position_list[i+1])][:-1] # reverse order b/c starting from back
    final_path.append(maze.start)
    return final_path[::-1] # reverse so we return from start to end

def fast(maze):
    """
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO?? 
    return []
    
            
