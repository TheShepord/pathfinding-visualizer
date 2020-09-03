# Third party imports
import numpy as np
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView
from PyQt5.QtCore import QRect, Qt, QMargins, QPropertyAnimation
from PyQt5.QtGui import QPen, QColor, QBrush, QMouseEvent

# Local application imports
from . import Config, CellType, Cell, Vector2D, PriorityQueue

class Scene(QGraphicsScene):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # self.c = Communicate()
        # self.c.cell_traversed.connect(self.color_cell)
        # initialize 2D grid of cell data
        self.cells = { (i,j): Cell(val = CellType.empty) for i in range(Config.NUM_CELLS_X) for j in range(Config.NUM_CELLS_Y) }
    
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

            # if neighbor is within bounds and isn't blocked, add to results
            if (0 <= neighbor.x < Config.NUM_CELLS_X) and (0 <= neighbor.y < Config.NUM_CELLS_Y) \
                and (not self.is_blocked(neighbor)):

                result.append(neighbor)
                # self.c.cell_traversed.emit(neighbor, Pallete.searched)
        return result

    def cost(self, coord: Vector2D) -> float:
        """Return weight of traversing cell at 'coord'"""
        return self.cells[coord].weight
    
    def set_cell(self, coord: Vector2D, new_cell: Cell) -> None:
        """Set value of cell at cells[x][y]"""
        self.cells[coord] = new_cell

    def is_blocked(self, coord: Vector2D):
        """Return whether cell at 'coord' is blocked"""
        return self.cells[coord].val == CellType.blocked

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

    def color_cell(self, coord: Vector2D, color: QColor, animate: bool = False) -> None:
        """Colors cell using the specified color. If 'animate' true, drawing is animated"""
        row = coord.y * Config.CELL_LENGTH + 1  # +1 so as to not paint over grid lines
        col = coord.x * Config.CELL_LENGTH + 1
        
        pen = QPen(color, 1)
        brush = QBrush(color)
        # if color == CellType.goal:
        #     print(coord)

        draw_delay = 500

        
        
        self.addRect(col, row, Config.CELL_LENGTH - 2, Config.CELL_LENGTH - 2, pen, brush)  # -2 so as to not paint over grid lines

    
    def repaint_cells(self):
        """Repaints all cells"""
        self.set_cell(self.start, Cell(val = CellType.start))
        self.set_cell(self.goal, Cell(val = CellType.goal))
        for x in range(Config.NUM_CELLS_X):
            for y in range(Config.NUM_CELLS_Y):
                self.color_cell(Vector2D(x,y), self.cells[Vector2D(x,y)].val)

    # def animate_path(self):


class View(QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # self.setSizeIncrement(Config.CELL_LENGTH, Config.CELL_LENGTH)

        self.setViewportMargins(QMargins(0, 0, 0, 0))
        self.centerOn(0,0)

        self.setMinimumSize(Config.CELL_LENGTH*5,Config.CELL_LENGTH*5)


        self.mouse_painting_blocked = False
        # self.setMouseTracking(True)

        # self.resize(Config.CELL_LENGTH*Config.NUM_CELLS_X,Config.CELL_LENGTH*Config.NUM_CELLS_Y)

    def mousePressEvent(self, event):
        x = event.x() // Config.CELL_LENGTH
        y = event.y() // Config.CELL_LENGTH

        self.mouse_painting_blocked = not self.scene().is_blocked(Vector2D(x, y))

        # if not self.mouse_painting_blocked:
        #     print(self.scene().cells[x][y])
        
        if self.scene().start != Vector2D(x, y) and self.scene().goal != Vector2D(x, y):
            if self.mouse_painting_blocked:
                self.scene().set_cell(Vector2D(x, y), Cell(val = CellType.blocked))
                # self.scene().color_cell(Vector2D(x, y), Pallete.blocked)
            else:
                self.scene().set_cell(Vector2D(x, y), Cell(val = CellType.empty))
                # self.scene().color_cell(Vector2D(x, y), Pallete.empty)
        self.scene().repaint_cells()
    
    
    def mouseMoveEvent(self, event):
        x = event.x() // Config.CELL_LENGTH
        y = event.y() // Config.CELL_LENGTH
        
        if self.scene().start != Vector2D(x, y) and self.scene().goal != Vector2D(x, y):
            if self.mouse_painting_blocked:
                self.scene().set_cell(Vector2D(x, y), Cell(val = CellType.blocked))
            else:
                self.scene().set_cell(Vector2D(x, y), Cell(val = CellType.empty))

        self.scene().repaint_cells()