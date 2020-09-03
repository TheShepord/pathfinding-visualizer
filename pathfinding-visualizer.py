# Standard library imports
import sys
from typing import Callable
import numpy as np

# Third party imports
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QMenuBar, QAction, QAbstractScrollArea
from PyQt5.QtGui import QResizeEvent

# Local application imports
from pathfinder import Config, Cell, CellType
import pathfinder.heuristics as heuristics
import pathfinder.algorithms as algorithm
from pathfinder.graphics import Scene, View


# def flatten(list_of_lists: list) -> list:
#     """Returns flattened list of lists"""
#     return [y for x in list_of_lists for y in x]


class Layout(QVBoxLayout):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        menubar = QMenuBar()
        algorithm_button = menubar.addMenu("Algorithm")
        action_a_star = QAction("A*", self)
        action_a_star.triggered.connect(lambda: self.execute(algorithm.a_star))
        algorithm_button.addAction(action_a_star)
        action_greedy_bfs = QAction("Greedy BFS", self)
        action_greedy_bfs.triggered.connect(lambda: self.execute(algorithm.greedy_bfs))
        algorithm_button.addAction(action_greedy_bfs)
        
        self.heuristic = heuristics.manhattan

        heuristics_button = menubar.addMenu("Heuristic")
        action_manhattan = QAction("Manhattan", self)
        action_manhattan.triggered.connect(lambda: self.set_heuristic(heuristics.manhattan))
        heuristics_button.addAction(action_manhattan)
        action_euclidean = QAction("Euclidean", self)
        action_euclidean.triggered.connect(lambda: self.set_heuristic(heuristics.euclidean))
        heuristics_button.addAction(action_euclidean)
        action_chebyshev = QAction("Chebyshev", self)
        action_chebyshev.triggered.connect(lambda: self.set_heuristic(heuristics.chebyshev))
        heuristics_button.addAction(action_chebyshev)
        action_octile = QAction("Octile", self)
        action_octile.triggered.connect(lambda: self.set_heuristic(heuristics.octile))
        heuristics_button.addAction(action_octile)

    
        self.addWidget(menubar)

        self.scene = Scene()
        view = View(self.scene)
        self.addWidget(view)
        
    
    def execute(self, pathfinder: Callable) -> None:
        result = pathfinder(self.scene.start, self.scene.goal, self.scene, self.heuristic)

        if result != None:
            [self.scene.set_cell(current, Cell(val = CellType.path)) for current in result]

        self.scene.repaint_cells()
        # [self.scene.color_cell(cell, Pallete.path) for cell in result]
        # self.scene.color_cell(start, Pallete.start)
        # self.scene.color_cell(goal, Pallete.goal)
        # layout.scene.draw_grid()

    def set_heuristic(self, heuristic):
        self.heuristic = heuristic

# def resize_overload(*args, **kwargs):
#     def wrapper():
#         func()
#     return wrapper
        

class Window(QWidget):
    def __init__(self, layout, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setLayout(layout)
        
        self.setSizeIncrement(Config.CELL_LENGTH, Config.CELL_LENGTH)

        # self.resize(layout.sizeHint())

        # self.setMinimumSize(Config.CELL_LENGTH*5,Config.CELL_LENGTH*5)
        # self.setBaseSize(Config.CELL_LENGTH*10,Config.CELL_LENGTH*10)
        
        # self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
    # @resize_overload
    def resizeEvent(self, event):

        # gets nearest width/height divisible by cell length
        nearest_w = event.size().width() // Config.CELL_LENGTH #-1
        nearest_h = event.size().height() // Config.CELL_LENGTH #-2
        
        Config.NUM_CELLS_X = nearest_w
        Config.NUM_CELLS_Y = nearest_h
        layout.scene.resize_update()



if __name__ == '__main__':
    
    app = QApplication(sys.argv)

    layout = Layout()
    
    w = Window(layout)
    w.show()
    
    sys.exit(app.exec_())