# Standard library imports
from typing import Callable

# Local application imports
from . import Vector2D, PriorityQueue, Cell, CellType
from pathfinder.graphics import Scene
import time

def reconstruct_path(goal: Vector2D, prev_node: dict) -> list:
    """Travels down 'prev_node' dictionary starting from 'goal' to retrieve final path"""
    path = []
    prev = prev_node[goal]  # remove 'goal' from path
    
    while prev != None:
        path.append(prev)
        prev = prev_node[prev]
    
    path = path[:-1]  # remove 'start' from path
    path.reverse()
    return path

def dijkstra(start: Vector2D, goal: Vector2D):
    return 0

def a_star(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> (list, list):
    """
    descripption bla bla. Always optimal etc
    Uses modified version of A* algorithm (LINK) that doesn't remove node from frontier if a better path is found. This saves on
    computation while still behaving similarly to the original A*. Also uses heuristic as a tie-breaker
    """
    # if h(n) = 0, A* becomes Dijkstra's Algorithm
    # if h(n) <= cost from n to goal, A* always optimal. The lower, the slower (more explored paths)
    # if h(n) = cost from n to goal, which isn't always possible, A* only follows the best path
    # if h(n) >, not guaranteed to find shortest path, but can be faster
    # if h(n) >> g(n), A* basically becomes Greedy Best-First-Search

    frontier = PriorityQueue()  # nodes to be explored
    dist_from_goal = heuristic(start, goal)  # estimate of cost to reach goal from current node
    frontier.put(start, dist_from_goal, dist_from_goal)

    prev_node = dict()  # maps n to node that precedes it in cheapest currently-known path from start to n
    cost_so_far = dict()  # maps n to cost of cheapest currently-known path from start to n
    explored = []  # keeps track of previously explored nodes

    prev_node[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:  # solution found!
            return (reconstruct_path(goal, prev_node), explored)

        explored.append(current)

        for neighbor in grid.get_neighbors(current):
            path_cost =  cost_so_far[current] + grid.cost(neighbor)  # cost of path to 'neighbor' going through 'current'

            if (neighbor not in cost_so_far) or (path_cost < cost_so_far[neighbor]): # if this path reaches node 'neighbor' faster than some previous path
                cost_so_far[neighbor] = path_cost
                expected_cost = heuristic(neighbor, goal)
                priority = path_cost + expected_cost
                frontier.put(neighbor, priority, expected_cost)  # 'expected_cost' is used as a tie breaker to reduce paths to explore

                prev_node[neighbor] = current

    # If frontier empty but goal was never reached, no solution was found
    return ([], explored)

def greedy_bfs(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], int]) -> list:
    """
    A case of A* where cost_so_far is negligible in relation to heuristic. At every step, finds next best step based on heuristic.
    # Keeps track of previously-explored paths to prevent hanging
    """
    prev_node = dict()

    prev_node[start] = None
    current = start
    neighbors = []

    neighbors = PriorityQueue()
    while current != goal: # or all have been visited
        grid.set_cell(current, Cell(val = CellType.searched))

        for neighbor in grid.get_neighbors(current):
            if neighbor not in prev_node:
                dist_from_goal = heuristic(neighbor, goal)
                neighbors.put(neighbor, dist_from_goal)
        if neighbors.empty():
            return None
        else:
            best_neighbor = neighbors.get()
        neighbors.clear()

        prev_node[best_neighbor] = current
        current = best_neighbor

    return reconstruct_path(goal, prev_node)
