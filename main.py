import sys
import heapq
from collections import namedtuple
import math
from typing import Callable, NamedTuple
import numpy as np

from PyQt5.QtWidgets import QWidget, QApplication, QGraphicsScene, QGraphicsView, QVBoxLayout, QMenuBar, QAction
from PyQt5.QtCore import QObject, pyqtSignal, QRect
from PyQt5.QtGui import QPen, QColor, QBrush


class Config:
    CELL_LENGTH = 40  # size of each cell to be displayed on-screen
    NUM_CELLS_X = 20
    NUM_CELLS_Y = 15
    HEURISTIC_WEIGHT = 1  # scalar multiplier for heuristic return value
    DIAGONALS = True  # can pathfinding move diagonally?

class Pallete:
    searched = QColor(0, 206, 209)
    path = QColor(255, 255, 0)
    start = QColor(255, 20, 147)
    goal = QColor(0, 250, 154)

class Cell(NamedTuple):
    weight: int
    blocked: bool


class Vector2D(NamedTuple):
    x: int
    y: int


class PriorityQueue:
    def __init__(self):
        self.heap = []
    
    def __contains__(self, item: Vector2D) -> bool:
        for i in self.heap:
            if i[2] == item:
                return True
        return False

    def empty(self) -> bool:
        return len(self.heap) == 0

    def put(self, item: Vector2D, priority: float, tie_breaker: float = 0) -> None:
        heapq.heappush(self.heap, (priority, tie_breaker, item))

    def get(self) -> Vector2D:
        return heapq.heappop(self.heap)[2]
    
    def clear(self) -> None:
        self.heap.clear()


def manhattan_distance(node: Vector2D, goal: Vector2D) -> float:
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    return Config.HEURISTIC_WEIGHT*(dx + dy)

def euclidean_distance(node: Vector2D, goal: Vector2D) -> float:
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    return Config.HEURISTIC_WEIGHT*math.sqrt(dx*dx + dy*dy)

def chebyshev_distance (node: Vector2D, goal: Vector2D) -> float:
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    diagonal_cost = 1

    return Config.HEURISTIC_WEIGHT*(dx + dy) + (diagonal_cost - 2*Config.HEURISTIC_WEIGHT)*min(dx, dy)

def octile_distance (node: Vector2D, goal: Vector2D) -> float:
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    diagonal_cost = math.sqrt(2)

    return Config.HEURISTIC_WEIGHT*(dx + dy) + (diagonal_cost - 2*Config.HEURISTIC_WEIGHT)*min(dx, dy)


def reconstruct_path(goal: Vector2D, prev_node: dict) -> list:
    path = []

    prev = prev_node[goal]
    while prev != None:
        path.insert(0, prev)
        prev = prev_node[prev]

    return path

class Communicate(QObject):
    
    cell_traversed = pyqtSignal(Vector2D, QColor)


# class Grid(QObject):


class Scene(QGraphicsScene):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.c = Communicate()
        self.c.cell_traversed.connect(self.color_cell)

        self.initUI()
        
        # initialize 2D grid of cell data
        self.grid = [[Cell(weight=1, blocked=False)]*Config.NUM_CELLS_X]*Config.NUM_CELLS_Y

        self.steps = np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])

    def get_neighbors(self, cell: Vector2D) -> list:
        result = []
        
        curr_cell = np.array([cell.x, cell.y])
            
        for step in self.steps:
            neighbor = step + curr_cell
            neighbor = Vector2D(x = neighbor[0], y = neighbor[1])

            # if neighbor is within bounds and isn't blocked, add to results
            if (0 <= neighbor.x < Config.NUM_CELLS_X) and (0 <= neighbor.y < Config.NUM_CELLS_Y) \
                and (self.grid[neighbor.x][neighbor.y].blocked == False):

                result.append(neighbor)
                # self.c.cell_traversed.emit(neighbor, Pallete.searched)
        return result

    def cost(self, cell: Vector2D) -> float:
        return self.grid[cell.x][cell.y].weight

    def set_diagonal(self) -> None:
        if Config.DIAGONALS:
            self.steps = np.array([[0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]])
        else:
            np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])

    def initUI(self) -> None:

        width = Config.CELL_LENGTH * Config.NUM_CELLS_X
        height = Config.CELL_LENGTH * Config.NUM_CELLS_Y
        self.setSceneRect(0, 0, width, height)
        self.setItemIndexMethod(QGraphicsScene.NoIndex)

        # draw cells
        for x in range(0, Config.NUM_CELLS_X + 1):
            col = x * Config.CELL_LENGTH
            self.addLine(col, 0, col, height)
        
        for y in range(0, Config.NUM_CELLS_Y + 1):
            row = y * Config.CELL_LENGTH
            self.addLine(0, row, width, row)

    def color_cell(self, cell: Vector2D, color: QColor) -> None:
        row = cell.y * Config.CELL_LENGTH
        col = cell.x * Config.CELL_LENGTH
        
        pen = QPen(color, 1)
        brush = QBrush(color)
        self.addRect(col, row, Config.CELL_LENGTH, Config.CELL_LENGTH, pen, brush)
        

def dijkstra(start: Vector2D, goal: Vector2D):
    return 0

def A_Star(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> list:
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
    dist_from_goal = heuristic(start, goal)
    frontier.put(start, dist_from_goal, dist_from_goal)
    # start = Node(fScore = hVal, gScore = 0, coord = start_coord, prev_node = None)'

    prev_node = dict()  # maps n to node that precedes it in cheapest currently-known path from start to n
    cost_so_far = dict()  # maps n to cost of cheapest currently-known path from start to n

    prev_node[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
                
        grid.color_cell(current, Pallete.searched)

        if current == goal:
            return reconstruct_path(goal, prev_node)

        for neighbor in grid.get_neighbors(current):
            path_cost =  cost_so_far[current] + grid.cost(neighbor)  # cost of path to 'neighbor' going through 'current'

            # if this path reaches node 'neighbor' faster than some previous path
            if (neighbor not in cost_so_far) or (path_cost < cost_so_far[neighbor]):
                cost_so_far[neighbor] = path_cost
                expected_cost = heuristic(neighbor, goal)
                priority = path_cost + expected_cost
                frontier.put(neighbor, priority, expected_cost)  # 'expected_cost' is used as a tie breaker to reduce paths to explore

                prev_node[neighbor] = current

    return []

def Greedy_BFS(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], int]) -> list:
    prev_node = dict()

    prev_node[start] = None
    current = start
    neighbors = []

    neighbors = PriorityQueue()
    while current != goal: # or all have been visited
        grid.color_cell(current, Pallete.searched)

        for neighbor in grid.get_neighbors(current):
            dist_from_goal = heuristic(neighbor, goal)
            neighbors.put(neighbor, dist_from_goal)
        best_neighbor = neighbors.get()
        neighbors.clear()

        prev_node[best_neighbor] = current
        current = best_neighbor

    return reconstruct_path(goal, prev_node)



# class View(QtWidgets.QGraphicsView):

class Layout(QVBoxLayout):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        menubar = QMenuBar()
        runButton = menubar.addMenu('RUN')
        runButton.addAction("A*")
        # runButton.addAction("Greedy BFS")

        # self.connect(runButton, SIGNAL("execute()"), execute)
        runButton.triggered[QAction].connect(lambda: self.execute(A_Star))
        # runButton.triggered[QAction].connect(lambda: self.execute(Greedy_BFS))
        
        self.addWidget(menubar)

        self.scene = Scene()
        view = QGraphicsView(self.scene)
        self.addWidget(view)

    def execute(self, pathfinder: Callable) -> None:
        start = Vector2D(2,8)
        goal = Vector2D(11, 3)

        result = pathfinder(start, goal, self.scene, manhattan_distance)

        [self.scene.color_cell(cell, Pallete.path) for cell in result]
        self.scene.color_cell(start, Pallete.start)
        self.scene.color_cell(goal, Pallete.goal)


if __name__ == '__main__':
    
    app = QApplication(sys.argv)

    layout = Layout()

    w = QWidget()
    w.setLayout(layout)
    w.show()
    
    sys.exit(app.exec_())