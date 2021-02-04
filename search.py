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

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    queue = []
    visited = set()
    queue.append([maze.start])
    goal = maze.waypoints[0]
    while queue:
        cur_path = queue.pop(0)
        cur_row, cur_col = cur_path[-1]
        if (cur_row, cur_col) in visited:
            continue
        visited.add((cur_row, cur_col))
        if ((cur_row, cur_col) == goal):
            return cur_path
        for item in maze.neighbors(cur_row, cur_col):
            if item not in visited:
                queue.append(cur_path + [item])
    return []
     

def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    goalx, goaly = maze.waypoints[0]
    startx, starty = maze.start
    pq = queue.PriorityQueue()
    visited = {}

    # our heuristic is manhattan distance
    cost = abs(startx - goalx) + abs(starty - goaly)

    pq.put((cost, [(startx, starty)]))
    while not pq.empty():
        cur_path = pq.get()[1]
        x, y = cur_path[-1]
        if (x, y) in visited:
            continue
        cur_cost = abs(x - goalx) + abs(y - goaly) + len(cur_path) - 1
        visited[(x, y)] = cur_cost
        if (x, y) == (goalx, goaly):
            return cur_path
        for item in maze.neighbors(x, y):
            new_cost = abs(item[0] - goalx) + abs(item[1] - goaly) + len(cur_path) - 1
            if item not in visited:
                pq.put((new_cost, cur_path + [item]))
            else:
                # if a node that’s already in the explored set found, test to see if the new h(n)+g(n) is smaller than the old one.
                if visited[item] > new_cost:
                    visited[item] = new_cost
                    pq.put((new_cost, cur_path + [item]))
    return []

def astar_corner(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    return []

def astar_multiple(maze):
    """
    Runs A star for part 4 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []

def fast(maze):
    """
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []
    
            
