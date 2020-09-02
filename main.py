import sys
import heapq
from collections import namedtuple
import math
from typing import Callable, NamedTuple
import numpy as np

from PyQt5.QtWidgets import QWidget, QApplication, QGraphicsScene, QGraphicsView, QVBoxLayout, QMenuBar, QAction, QAbstractScrollArea
from PyQt5.QtCore import QObject, pyqtSignal, QRect, Qt, QMargins, QPoint
from PyQt5.QtGui import QPen, QColor, QBrush, QResizeEvent


class Config:
    """Global system settings"""
    CELL_LENGTH = 40  # size of each cell to be displayed on-screen
    NUM_CELLS_X = 20
    NUM_CELLS_Y = 15
    HEURISTIC_WEIGHT = 1  # scalar multiplier for heuristic return value
    DIAGONALS = False  # can pathfinding move diagonally?

class Pallete:
    """Color to draw each object"""
    searched = QColor(0, 206, 209)
    path = QColor(255, 255, 0)
    start = QColor(255, 20, 147)
    goal = QColor(0, 250, 154)

class Cell(NamedTuple):
    """aaa"""
    weight: int
    blocked: bool


class Vector2D(NamedTuple):
    """2D vector representing cartesian x and y positions"""
    x: int
    y: int


class PriorityQueue:
    """Priority queue implemented with Python's heap queue algorithm"""
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
    """Returns manhattan distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    return Config.HEURISTIC_WEIGHT*(dx + dy)

def euclidean_distance(node: Vector2D, goal: Vector2D) -> float:
    """Returns euclidean distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    return Config.HEURISTIC_WEIGHT*math.sqrt(dx*dx + dy*dy)

def chebyshev_distance (node: Vector2D, goal: Vector2D) -> float:
    """Returns chebyshev distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    diagonal_cost = 1

    return Config.HEURISTIC_WEIGHT*(dx + dy) + (diagonal_cost - 2*Config.HEURISTIC_WEIGHT)*min(dx, dy)

def octile_distance (node: Vector2D, goal: Vector2D) -> float:
    """Returns octile distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    diagonal_cost = math.sqrt(2)

    return Config.HEURISTIC_WEIGHT*(dx + dy) + (diagonal_cost - 2*Config.HEURISTIC_WEIGHT)*min(dx, dy)


def reconstruct_path(goal: Vector2D, prev_node: dict) -> list:
    """Travels down 'prev_node' dictionary starting from 'goal' to retrieve final path"""
    path = []

    prev = prev_node[goal]
    while prev != None:
        path.append(prev)
        prev = prev_node[prev]
    path = path.reverse()
    return path

# class Communicate(QObject):
    
#     cell_traversed = pyqtSignal(Vector2D, QColor)


# class Grid(QObject):


class Scene(QGraphicsScene):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # self.c = Communicate()
        # self.c.cell_traversed.connect(self.color_cell)

        self.draw_grid()
        self.init_start_goal()
        
        # initialize 2D grid of cell data
        self.grid = [[Cell(weight=1, blocked=False)]*Config.NUM_CELLS_X]*Config.NUM_CELLS_Y

        self.set_diagonal()


    def get_neighbors(self, cell: Vector2D) -> list:
        """Return neighbors to Vector2D inside the scene"""
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
        """Return weight of traversing 'cell'"""
        return self.grid[cell.x][cell.y].weight

    def set_diagonal(self) -> None:
        if Config.DIAGONALS:
            self.steps = np.array([[0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]])
        else:
            np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])

    def draw_grid(self) -> None:
        self.clear()

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
        row = cell.y * Config.CELL_LENGTH + 1
        col = cell.x * Config.CELL_LENGTH + 1
        
        pen = QPen(color, 1)
        brush = QBrush(color)
        self.addRect(col, row, Config.CELL_LENGTH - 2, Config.CELL_LENGTH - 2, pen, brush)

    def init_start_goal(self):
        height = Config.NUM_CELLS_Y // 2

        self.start = Vector2D(1, height)
        self.goal = Vector2D(Config.NUM_CELLS_X - 1, height)

        self.color_cell(self.start, )


class View(QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # self.setSizeIncrement(Config.CELL_LENGTH, Config.CELL_LENGTH)

        self.setViewportMargins(QMargins(0, 0, 0, 0))
        self.centerOn(0,0)

        self.setMinimumSize(Config.CELL_LENGTH*5,Config.CELL_LENGTH*5)
        self.setBaseSize(Config.CELL_LENGTH*10,Config.CELL_LENGTH*10)




def dijkstra(start: Vector2D, goal: Vector2D):
    return 0

def pathfind_a_star(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], float]) -> list:
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

    grid.draw_grid()

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

def pathfind_greedy_bfs(start: Vector2D, goal: Vector2D, grid: Scene, heuristic: Callable[[Vector2D, Vector2D], int]) -> list:
    grid.draw_grid()

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

class Layout(QVBoxLayout):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        menubar = QMenuBar()
        algorithm_button = menubar.addMenu("Algorithm")
        action_a_star = QAction("A*", self)
        action_a_star.triggered.connect(lambda: self.execute(pathfind_a_star))
        algorithm_button.addAction(action_a_star)
        action_greedy_bfs = QAction("Greedy BFS", self)
        action_greedy_bfs.triggered.connect(lambda: self.execute(pathfind_greedy_bfs))
        algorithm_button.addAction(action_greedy_bfs)
        
        self.heuristic = manhattan_distance

        heuristics_button = menubar.addMenu("Heuristic")
        action_manhattan = QAction("Manhattan", self)
        action_manhattan.triggered.connect(lambda: self.set_heuristic(manhattan_distance))
        heuristics_button.addAction(action_manhattan)
        action_euclidean = QAction("Euclidean", self)
        action_euclidean.triggered.connect(lambda: self.set_heuristic(euclidean_distance))
        heuristics_button.addAction(action_euclidean)
        action_chebyshev = QAction("Chebyshev", self)
        action_chebyshev.triggered.connect(lambda: self.set_heuristic(chebyshev_distance))
        heuristics_button.addAction(action_chebyshev)
        action_octile = QAction("Octile", self)
        action_octile.triggered.connect(lambda: self.set_heuristic(octile_distance))
        heuristics_button.addAction(action_octile)


        self.addWidget(menubar)

        self.scene = Scene()
        view = View(self.scene)
        self.addWidget(view)

        
    
    def execute(self, pathfinder: Callable) -> None:
        result = pathfinder(start, goal, self.scene, self.heuristic)

        [self.scene.color_cell(cell, Pallete.path) for cell in result]
        self.scene.color_cell(start, Pallete.start)
        self.scene.color_cell(goal, Pallete.goal)
        # layout.scene.draw_grid()

    def set_heuristic(self, heuristic):
        self.heuristic = heuristic

# def resize_overload(*args, **kwargs):
#     def wrapper():
#         func()
#     return wrapper
        

class Window(QAbstractScrollArea):
    def __init__(self, layout, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setLayout(layout)
        
        self.setSizeIncrement(Config.CELL_LENGTH, Config.CELL_LENGTH)

        self.setViewportMargins(QMargins(0, 0, 0, 0))


        # self.resize(layout.sizeHint())

        # self.setMinimumSize(Config.CELL_LENGTH*5,Config.CELL_LENGTH*5)
        # self.setBaseSize(Config.CELL_LENGTH*10,Config.CELL_LENGTH*10)
        
        # self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
    # @resize_overload
    def resizeEvent(self, event):

        # get largest multiple of the new width / new height that is divisible by CELL_LENGTH
        nearest_w = (event.size().width() // Config.CELL_LENGTH) #* Config.CELL_LENGTH
        nearest_h = (event.size().height() // Config.CELL_LENGTH)#* Config.CELL_LENGTH
        print(nearest_w, nearest_h)

        # self.resize(nearest_w,nearest_h)
        Config.NUM_CELLS_X = nearest_w
        Config.NUM_CELLS_Y = nearest_h - 1
        layout.scene.draw_grid()
        

        # self.resize()
        # class Config:
        #     CELL_LENGTH = 40  # size of each cell to be displayed on-screen
        #     NUM_CELLS_X = 20
        #     NUM_CELLS_Y = 15
        #     HEURISTIC_WEIGHT = 1  # scalar multiplier for heuristic return value
        #     DIAGONALS = True  # can pathfinding move diagonally?
        


if __name__ == '__main__':
    
    app = QApplication(sys.argv)

    layout = Layout()
    
    w = Window(layout)
    w.show()
    
    sys.exit(app.exec_())