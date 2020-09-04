# Standard library imports
from time import sleep
import threading

# Third party imports
import numpy as np
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsRectItem, QGraphicsObject
from PyQt5.QtCore import QRect, QRectF, Qt, QMargins, QPropertyAnimation
from PyQt5.QtGui import QPen, QColor, QBrush, QMouseEvent, QPainter

# Local application imports
from . import Config, CellType, Cell, Vector2D, PriorityQueue

def in_bounds(coord: Vector2D) -> bool:
    """Returns whether a given coordinate is within the bounds of the screen"""
    return (0 <= coord.x < Config.NUM_CELLS_X) and (0 <= coord.y < Config.NUM_CELLS_Y)

class RectObject(QGraphicsObject):
    """"""
    def __init__(self, x, y, w, h, pen, brush):
        super().__init__()

        self.rect = QRectF(x, y, w, h)
        self.pen = pen
        self.brush = brush
        self.setTransformOriginPoint(x + w/2, y + h/2)
    
    def boundingRect(self):
        return self.rect
    
    def paint(self, painter: QPainter, styles, widget=None):
        painter.setPen(self.pen)
        painter.setBrush(self.brush)
        painter.drawRect(self.rect)

class Scene(QGraphicsScene):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # self.c = Communicate()
        # self.c.cell_traversed.connect(self.color_cell)
        # initialize 2D grid of cell data
        self.cells = { (i,j): Cell(val = CellType.empty) for i in range(Config.NUM_CELLS_X) for j in range(Config.NUM_CELLS_Y) }
        self.animations = dict()

        self.setItemIndexMethod(QGraphicsScene.NoIndex)
        self.draw_grid()
        self.init_start_goal()
        self.set_diagonal()
        self.repaint_cells()


    def init_start_goal(self):
        """Initialize start and goal positions"""
        height = (Config.NUM_CELLS_Y // 2) - 2
        self.start = Vector2D(1, height)
        self.goal = Vector2D(Config.NUM_CELLS_X - 1, height)

        self.set_cell(self.start, Cell(val = CellType.start))
        self.set_cell(self.goal, Cell(val = CellType.goal))
        
        # print([x for x in self.cells if Cell(val = CellType.start) in x])
        
        # self.color_cell(self.start, Pallete.start)
        # self.color_cell(self.goal, Pallete.goal)
    
    def set_diagonal(self) -> None:
        """Generates array of possible moves based on Config.DIAGONALS"""
        if Config.DIAGONALS:
            self.neighbor_steps = np.array([[0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]])
        else:
            self.neighbor_steps = np.array([[0, 1], [0, -1], [1, 0], [-1, 0]])
                
    def get_neighbors(self, cell: Vector2D) -> list:
        """Return neighbors to Vector2D inside the scene"""
        result = []
        
        curr_cell = np.array([cell.x, cell.y])
            
        for step in self.neighbor_steps:
            neighbor = step + curr_cell
            neighbor = Vector2D(x = neighbor[0], y = neighbor[1])

            if in_bounds(neighbor) and not self.is_barrier(neighbor):
                result.append(neighbor)

        return result

    def cost(self, coord: Vector2D) -> float:
        """Return weight of traversing cell at 'coord'"""
        return self.cells[coord].weight
    
    def set_cell(self, coord: Vector2D, new_cell: Cell) -> None:
        """Set value of cell at cells[x][y]"""
        self.cells[coord] = new_cell

    def is_barrier(self, coord: Vector2D):
        """Return whether cell at 'coord' is barrier"""
        return self.cells[coord].val == CellType.barrier
    
    def cell_type(self, coord: Vector2D):
        """Return type of cell at 'coord'"""
        return self.cells[coord].val

    def draw_grid(self) -> None:
        """Draws """
        width = Config.CELL_LENGTH * Config.NUM_CELLS_X
        height = Config.CELL_LENGTH * Config.NUM_CELLS_Y
        self.setSceneRect(0, 0, width, height)

        # draw cells
        for x in range(0, Config.NUM_CELLS_X + 1):
            col = x * Config.CELL_LENGTH
            self.addLine(col, 0, col, height)
        
        for y in range(0, Config.NUM_CELLS_Y + 1):
            row = y * Config.CELL_LENGTH
            self.addLine(0, row, width, row)

    def resize_update(self) -> None:
        """Resizes grid and keeps 'start' and 'goal' cells inside of view"""
        self.clear()
        self.draw_grid()

        if self.start.x >= Config.NUM_CELLS_X - 1:
            self.start = Vector2D(Config.NUM_CELLS_X - 2, self.start.y)
        if self.start.y >= Config.NUM_CELLS_Y - 1:
            self.start = Vector2D(self.start.x, Config.NUM_CELLS_Y - 2)
        if self.goal.x >= Config.NUM_CELLS_X - 1:
            self.goal = Vector2D(Config.NUM_CELLS_X - 2, self.goal.y)
        if self.goal.y >= Config.NUM_CELLS_Y - 1:
            self.goal = Vector2D(self.goal.x, Config.NUM_CELLS_Y - 2)

        self.cells = { (i,j): Cell(val = CellType.empty) for i in range(Config.NUM_CELLS_X) for j in range(Config.NUM_CELLS_Y) }
        self.set_cell(self.start, Cell(val = CellType.start))
        self.set_cell(self.goal, Cell(val = CellType.goal))
        
        self.repaint_cells()

    def color_cell(self, coord: Vector2D, animate: bool = False) -> None:
        """Colors cell using the specified color. If 'animate' true, drawing is animated"""
        row = coord.y * Config.CELL_LENGTH + 1  # +1 so as to not paint over grid lines
        col = coord.x * Config.CELL_LENGTH + 1
        color = self.cells[coord].val
        pen = QPen(color, 1)
        brush = QBrush(color)

        if animate:
            threading.Thread (
                target=self.animate_rect, \
                args=(col, row, Config.CELL_LENGTH - 2, Config.CELL_LENGTH - 2, pen, brush), \
                daemon=True \
            ).start()

        else:
            self.addRect(col, row, Config.CELL_LENGTH - 2, Config.CELL_LENGTH - 2, pen, brush)  # -2 so as to not paint over grid lines

    def animate_rect(self, x: int, y: int, w: int, h: int, pen: QPen, brush: QBrush, duration: float = 0.125, n_steps: float = 16) -> list:
        """Creates RectObject that transposes from dimensions 0 x 0 to 'w' x 'h' in 'n_steps' steps over the course of 'duration' seconds"""
        scale = 0.0

        time_step = duration / n_steps
        scale_step = 1.0 / n_steps

        while scale <= 1.0:
            rect = RectObject(x + w/2*(1-scale), y + h/2*(1-scale), w*scale, h*scale, pen, brush)
            self.addItem(rect)
            sleep(time_step)
            scale += scale_step

    def repaint_cells(self) -> None:
        """Repaints all cells"""
        self.set_cell(self.start, Cell(val = CellType.start))
        self.set_cell(self.goal, Cell(val = CellType.goal))
        for x in range(Config.NUM_CELLS_X):
            for y in range(Config.NUM_CELLS_Y):
                self.color_cell(Vector2D(x,y))

    def clear_path(self) -> None:
        """Removes searched and explored-type cells from grid"""
        for x in range(Config.NUM_CELLS_X):
            for y in range(Config.NUM_CELLS_Y):
                if self.cells[Vector2D(x,y)].val == CellType.path \
                or self.cells[Vector2D(x,y)].val == CellType.searched:
                    self.set_cell(Vector2D(x,y), Cell(val = CellType.empty))
                    self.color_cell(Vector2D(x,y))

    def draw_explored(self, explored: list, animate: bool = False):
        for current_cell in explored:
            if self.cell_type(current_cell) not in (CellType.goal, CellType.start):
                self.set_cell(current_cell, Cell(val = CellType.searched))
                self.color_cell(current_cell, animate)
                sleep(0.03125)

    def draw_path(self, path: list, prev_thread: threading.Thread = None, animate: bool = False) -> None:
        if prev_thread:
            prev_thread.join()

        for current_cell in path:
            self.set_cell(current_cell, Cell(val = CellType.path))
            self.color_cell(current_cell, animate)
            sleep(0.03125)


class View(QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setViewportMargins(QMargins(0, 0, 0, 0))
        self.centerOn(0,0)

        self.setMinimumSize(Config.CELL_LENGTH*5,Config.CELL_LENGTH*5)

        self.painting_cell = CellType.barrier  # What type of cell was last clicked? Default is barrier.
        self.setMouseTracking(False)  # Only track mouse movements while mousekey down


    def mousePressEvent(self, event):
        """If user clicked, they're either drawing a barrier, removing a barrier, wanting to move start, or wanting to move goal"""
        x = event.x() // Config.CELL_LENGTH
        y = event.y() // Config.CELL_LENGTH

        if self.scene().cell_type(Vector2D(x,y)) == CellType.empty:
            # If user clicked on an empty cell, they're trying to draw barriers
            self.painting_cell = CellType.barrier
            
        elif self.scene().cell_type(Vector2D(x,y)) == CellType.barrier:
            # If user clicked on an barrier, they're trying to remove barriers
            self.painting_cell = CellType.empty
        elif self.scene().cell_type(Vector2D(x,y)) == CellType.start:
            # If user clicked on start, they're trying to move start
            self.painting_cell = CellType.start
        elif self.scene().cell_type(Vector2D(x,y)) == CellType.goal:
            # If user clicked on goal, they're trying to move goal
            self.painting_cell = CellType.goal
        
        if self.painting_cell in (CellType.barrier, CellType.empty):
            if self.scene().start != Vector2D(x, y) and self.scene().goal != Vector2D(x, y):
                self.scene().set_cell(Vector2D(x, y), Cell(val = self.painting_cell))
                self.scene().color_cell(Vector2D(x,y), True)
    
    
    def mouseMoveEvent(self, event):
        """If user holding down mouse button, they're either drawing barriers, removing barriers, moving start, or moving goal"""
        x = event.x() // Config.CELL_LENGTH
        y = event.y() // Config.CELL_LENGTH
        
        if self.painting_cell in (CellType.barrier, CellType.empty):
            # drawing/removing barriers
            if self.scene().start != Vector2D(x, y) and self.scene().goal != Vector2D(x, y):
                self.scene().set_cell(Vector2D(x, y), Cell(val = self.painting_cell))
                self.scene().color_cell(Vector2D(x,y), True)

        elif self.painting_cell == CellType.start:
            # Moving start
            if self.scene().goal != Vector2D(x,y) and in_bounds(Vector2D(x,y)):  # start and goal can't overlap, and new location must be in bounds
                self.scene().set_cell(self.scene().start, Cell(val = CellType.empty))
                self.scene().color_cell(self.scene().start)
                self.scene().start = Vector2D(x, y)
                self.scene().set_cell(self.scene().start, Cell(val = CellType.start))
                self.scene().color_cell(self.scene().start)

        elif self.painting_cell == CellType.goal:
            # Moving goal
            if self.scene().start != Vector2D(x,y) and in_bounds(Vector2D(x,y)):  # start and goal can't overlap, and new location must be in bounds
                self.scene().set_cell(self.scene().goal, Cell(val = CellType.empty))
                self.scene().color_cell(self.scene().goal)
                self.scene().goal = Vector2D(x, y)
                self.scene().set_cell(self.scene().goal, Cell(val = CellType.goal))
                self.scene().color_cell(self.scene().goal)
