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
    # reference: https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
    queue = []
    visited = set() # use a set to keep track of visited states
    queue.append([maze.start])
    goal = maze.waypoints[0] # only one waypoint
    while queue: # while our queue is not empty
        path = queue.pop(0)
        row, col = path[len(path) - 1] # get last position in path
        if (row, col) in visited:
            continue
        visited.add((row, col))
        if ((row, col) == goal):
            return path
        for n in maze.neighbors(row, col):
            if n not in visited:
                queue.append(path + [n]) # will keep on appending to cur_path
    # return empty list if unsuccessful
    return []
     
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

def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    start = maze.start
    end = maze.waypoints[0] # NOTE: only one waypoint
    return get_path(maze, start, end)

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
    # initialize our goals list and add start (treat start as a waypoint)
    start = maze.start
    goals_left = list(maze.waypoints) # convert to list so we can insert... tuple can't do insert
    goals_left.insert(0, start)

    # iterate through our waypoints and update our edge list and heuristic list
    edge_list = {}
    heuristic_list = {}
    for i in goals_left:
        for j in goals_left:
            if i != j:
                new_path = get_path(maze, i, j) # get length from one waypoint to another
                edge_list[(i, j)] = new_path # edge list for waypoint i to waypoint j
                heuristic_list[(i, j)] = len(new_path) # heuristic_list will be the length of the path according to single astar search

    # initialize variables
    path = queue.PriorityQueue() # will contain nodes
    weights = get_MST(maze, goals_left, heuristic_list)
    
    startx, starty = maze.start
    sstate = Node(startx, starty, 0, weights) # create new node
    sstate.not_visited = maze.waypoints
    path.put(sstate)

    # keep on iterating until there are no more goals left
    while len(goals_left):
        cur_state = path.get() # get the highest priority item
        if len(cur_state.not_visited) == 0:
            break
        for n in cur_state.not_visited:
            n_cost = cur_state.cost + heuristic_list[(cur_state.position, n)] - 1 # calculate the cost using our already initialized heuristic list
            n_row, n_col = n

            # initialize our 'next' variable
            next_state = Node(n_row, n_col, n_cost, 0)
            next_state.not_visited = deepcopy(cur_state.not_visited)
            next_state.prev = cur_state

            if n in next_state.not_visited:
                listx = list(next_state.not_visited) # turn to list so we can manipulate
                listx.remove(n) # remove from not visited
                next_state.not_visited = tuple(listx)

            weights = get_MST(maze, cur_state.not_visited, heuristic_list)
            next_state.total_cost = n_cost + weights + len(next_state.not_visited) # f = g + h
            path.put(next_state)

    # format final output correctly
    position_list = []
    while cur_state: # keep on moving from current state to end
        position_list.append(cur_state.position)
        cur_state = cur_state.prev

    final_path = []
    for i in range(len(position_list) - 1):
        final_path += edge_list[(position_list[i], position_list[i+1])][:-1] # reverse order b/c starting from back
    final_path.append(maze.start)
    return final_path[::-1] # reverse so we return from start to end

"""
The following are helper functions for astar functions
"""

def get_path(maze, start, end):
    """
    Returns the path to be taken from start to end (used in astar_single)

    Note that our heuristic here is Manhattan Distance
    """
    pq = queue.PriorityQueue()
    visited = {}
    result_row, result_col = end
    start_row, start_col = start
    cost = manhattan_distance(start_row, start_col, result_row, result_col)
    pq.put((cost, [(start_row, start_col)]))
    while not pq.empty():
        cur_path = pq.get()[1]
        cur_row, cur_col = cur_path[len(cur_path) - 1]

        # If we've already visited this location
        if (cur_row, cur_col) in visited:
            continue

        cur_cost = manhattan_distance(cur_row, cur_col, result_row, result_col) + len(cur_path) - 1 # f = h + g
        visited[(cur_row, cur_col)] = cur_cost

        # We have reached our goal, so return path
        if (cur_row, cur_col) == (result_row, result_col):
            return cur_path

        # look at neighbors and continue iterating
        for item in maze.neighbors(cur_row, cur_col):
            new_cost = manhattan_distance(item[0], item[1], result_row, result_col) + len(cur_path) - 1
            if item not in visited:
                pq.put((new_cost, cur_path + [item]))
            else:
                # need to replace original in visited if new_cost is lower
                if visited[item] > new_cost:
                    visited[item] = new_cost
                    pq.put((new_cost, cur_path + [item]))
    return []

def manhattan_distance(startx, starty, goalx, goaly):
    return abs(startx - starty) + abs(starty - goaly)


def get_MST(maze, goals, heuristic_list):
    if len(goals) == 0: # if there are no goals
        return 0

    start = goals[0] # our first goal is our start
    visited = {}
    visited[start] = 1 # now we have visited this position
    weights = 0

    while len(visited) != len(goals): # check if we have visited every node or not
        q = queue.PriorityQueue()
        for v in visited:
            for g in goals:
                if visited.get(g) == 1:
                    continue
                new_edge = (v, g) # create a new "edge" between two waypoints
                new_cost = heuristic_list[new_edge] - 2
                q.put((new_cost, new_edge))
        w, e = q.get() # get highest priority "edge"
        weights += w # add the weight
        visited[e[1]] = 1
    return weights

def fast(maze):
    """
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO?
    return []
    