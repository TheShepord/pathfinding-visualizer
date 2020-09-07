# implements the pathfinding algorithms used in pathfinding-visualizer.py

# Standard library imports
from typing import Callable

# Local application imports
from . import Vector2D, Queue, PriorityQueue, Stack, Cell, CellType
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

def a_star(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> (list, list):
    """
    descripption bla bla. Always optimal etc
    Uses modified version of the original A* algorithm (LINK). These changes are:
    - Not removing node from frontier if a better path to it is found.
        Motivation: saves on computation (search in priority queue) while still behaving similarly to the original A*.
    - Not pre-setting cost_so_far of all nodes to INF, instead adding nodes as they're explored.
        Motivation: space complexity increases O(width x height / cell_dimensions) in a square grid
    - Using the heuristic as a tie-breaker for equally-valued nodes.
        Motivation: prevents exploring equally-appealing nodes when one is (according to heuristic) "closer" to goal
    """
    # if h(n) = 0, A* becomes Dijkstra's Algorithm
    # if h(n) <= cost from n to goal, A* always optimal. The lower, the slower (more explored paths)
    # if h(n) = cost from n to goal, which isn't always possible, A* only follows the best path
    # if h(n) >, not guaranteed to find shortest path, but can be faster
    # if h(n) >> g(n), A* basically becomes Greedy Best-First-Search

    frontier = PriorityQueue()  # nodes to be explored
    expected_cost = heuristic(start, goal)  # estimate of cost to reach goal from current node

    prev_node = dict()  # maps n to node that precedes it in cheapest currently-known path from start to n
    cost_so_far = dict()  # maps n to cost of cheapest currently-known path from start to n
    explored = []  # keeps track of previously explored nodes, to be drawn later

    frontier.put(start, expected_cost, expected_cost)
    prev_node[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:  # solution found!
            return (reconstruct_path(goal, prev_node), explored)

        explored.append(current)

        for neighbor in grid.get_unexplored_neighbors(current):
            path_cost =  cost_so_far[current] + grid.cost(neighbor)  # cost of path to 'neighbor' going through 'current'

            if (neighbor not in cost_so_far) or (path_cost < cost_so_far[neighbor]): # if this path reaches node 'neighbor' faster than some previous path
                cost_so_far[neighbor] = path_cost
                expected_cost = heuristic(neighbor, goal)
                priority = path_cost + expected_cost
                frontier.put(neighbor, priority, expected_cost)  # 'expected_cost' is used as a tie breaker to reduce paths to explore

                prev_node[neighbor] = current

    # If frontier empty but goal was never reached, no solution was found
    return ([], explored)

def dijkstra(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> (list, list):
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

    prev_node = dict()  # maps n to node that precedes it in cheapest currently-known path from start to n
    cost_so_far = dict()  # maps n to cost of cheapest currently-known path from start to n
    explored = []  # keeps track of previously explored nodes, to be drawn later

    current_cost = 0

    frontier.put(start, current_cost)
    prev_node[start] = None
    cost_so_far[start] = current_cost

    while not frontier.empty():
        current = frontier.get()

        if current == goal:  # solution found!
            return (reconstruct_path(goal, prev_node), explored)

        explored.append(current)

        for neighbor in grid.get_unexplored_neighbors(current):
            path_cost =  cost_so_far[current] + grid.cost(neighbor)  # cost of path to 'neighbor' going through 'current'

            if (neighbor not in cost_so_far) or (path_cost < cost_so_far[neighbor]): # if this path reaches node 'neighbor' faster than some previous path
                cost_so_far[neighbor] = path_cost
                
                frontier.put(neighbor, path_cost)  # 'expected_cost' is used as a tie breaker to reduce paths to explore
                prev_node[neighbor] = current

    # If frontier empty but goal was never reached, no solution was found
    return ([], explored)

def greedy_bfs(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> list:
    """
    Greedy Best-First Search
    Essentially A* but cost_so_far is negligible in relation to heuristic.
    """
    frontier = PriorityQueue()  # nodes to be explored
    prev_node = dict()  # maps n to node that precedes it in cheapest currently-known path from start to n
    explored = []  # keeps track of previously explored nodes, to be drawn later

    frontier.put(start, heuristic(start, goal))
    prev_node[start] = None

    while not frontier.empty():
        current = frontier.get()

        if current == goal:  # solution found!
            return (reconstruct_path(goal, prev_node), explored[1:])  # [1: to remove 'start']

        grid.set_cell(current, Cell(val = CellType.searched))
        explored.append(current)
        
        for neighbor in grid.get_unexplored_neighbors(current):
            frontier.put(neighbor, heuristic(neighbor, goal))
            prev_node[neighbor] = current

    
    # If frontier empty but goal was never reached, no solution was found
    return ([], explored[1:])  # [1: to remove 'start']


def breadth_fs(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> (list, list):
    """Iterative Breadth-First Search. Nodes are explored using a queue."""
    frontier = Queue()  # nodes to be explored
    prev_node = dict()  # maps n to node that precedes it in cheapest currently-known path from start to n
    explored = []  # keeps track of previously explored nodes, to be drawn later

    prev_node[start] = None
    frontier.put(start)

    while not frontier.empty():
        current = frontier.get()
        grid.set_cell(current, Cell(val = CellType.searched))
        
        for neighbor in grid.get_unexplored_neighbors(current):
            prev_node[neighbor] = current
            frontier.put(neighbor)
            explored.append(neighbor)
            if neighbor == goal:
                return (reconstruct_path(goal, prev_node), explored)

            grid.set_cell(neighbor, Cell(val = CellType.searched))

    # If frontier empty but goal was never reached, no solution was found
    return ([], explored)
    
def depth_fs(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> (list, list):
    """Iterative Depth-First Search. Nodes are explored using a stack."""
    frontier = Stack()
    prev_node = dict()
    explored = []

    frontier.put(start)
    prev_node[start] = None

    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            return (reconstruct_path(goal, prev_node), explored[1:])  # [1:] to remove start from list

        grid.set_cell(current, Cell(val = CellType.searched))
        explored.append(current)

        for neighbor in grid.get_unexplored_neighbors(current):
            prev_node[neighbor] = current
            frontier.put(neighbor)

                # grid.set_cell(neighbor, Cell(val = CellType.searched))
    
    # If frontier empty but goal was never reached, no solution was found
    return ([], explored[1:])  # [1:] to remove start from list
