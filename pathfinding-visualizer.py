# Standard library imports
import sys
from typing import Callable
import numpy as np
from time import sleep
import threading

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

        # Adding menubar
        self.menubar = QMenuBar()

        # Algorithms widget
        self.algorithm_button = menubar.addMenu("Algorithm")
        self.action_a_star = QAction("A*", self)
        self.action_a_star.triggered.connect(lambda: self.execute(algorithm.a_star))
        self.algorithm_button.addAction(self.action_a_star)
        self.action_greedy_bfs = QAction("Greedy BFS", self)
        self.action_greedy_bfs.triggered.connect(lambda: self.execute(algorithm.greedy_bfs))
        self.algorithm_button.addAction(self.action_greedy_bfs)
        
        self.heuristic = heuristics.manhattan # Default heuristic

        # Heuristics widget
        self.heuristics_button = menubar.addMenu("Heuristic")
        self.action_manhattan = QAction("Manhattan", self)
        self.action_manhattan.triggered.connect(lambda: self.set_heuristic(heuristics.manhattan))
        self.heuristics_button.addAction(self.action_manhattan)
        self.action_euclidean = QAction("Euclidean", self)
        self.action_euclidean.triggered.connect(lambda: self.set_heuristic(heuristics.euclidean))
        self.heuristics_button.addAction(self.action_euclidean)
        self.action_chebyshev = QAction("Chebyshev", self)
        self.action_chebyshev.triggered.connect(lambda: self.set_heuristic(heuristics.chebyshev))
        self.heuristics_button.addAction(self.action_chebyshev)
        self.action_octile = QAction("Octile", self)
        self.action_octile.triggered.connect(lambda: self.set_heuristic(heuristics.octile))
        self.heuristics_button.addAction(self.action_octile)
        
        self.addWidget(menubar)

        self.scene = Scene()
        view = View(self.scene)
        self.addWidget(view)
        
    
    def execute(self, pathfinder: Callable) -> None:
        """Executes selected pathfinding algorithm using currently-selected heuristic"""
        self.scene.clear_path()
        result, explored = pathfinder(self.scene.start, self.scene.goal, self.scene, self.heuristic)

        if explored != []:
            # Draws explored tiles. Multi-threading used for tiles to draw sequentially
            draw_explored_thread = threading.Thread(target=self.scene.draw_explored, args=(explored, True))
            draw_explored_thread.start()

        if result != []:
            # Draws path. Multi-threading used for path to draw sequentially
            threading.Thread(target=self.scene.draw_path, args=(result, draw_explored_thread, True)).start()

    def set_heuristic(self, heuristic):
        self.heuristic = heuristic


class Window(QWidget):
    def __init__(self, layout, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setLayout(layout)
        
        self.setSizeIncrement(Config.CELL_LENGTH, Config.CELL_LENGTH)

    def resizeEvent(self, event):

        # gets nearest width/height divisible by cell length
        nearest_w = event.size().width() // Config.CELL_LENGTH
        nearest_h = event.size().height() // Config.CELL_LENGTH - 1
        
        Config.NUM_CELLS_X = nearest_w
        Config.NUM_CELLS_Y = nearest_h
        layout.scene.resize_update()



if __name__ == '__main__':
    
    app = QApplication(sys.argv)

    layout = Layout()
    
    w = Window(layout)
    w.show()
    
    sys.exit(app.exec_())