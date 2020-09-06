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
        self.algorithm_button = self.menubar.addMenu("Algorithm")
        self.action_a_star = QAction("A*", self)
        self.action_a_star.triggered.connect(lambda: self.execute(algorithm.a_star))
        self.algorithm_button.addAction(self.action_a_star)
        self.action_greedy_bfs = QAction("Greedy BFS", self)
        self.action_greedy_bfs.triggered.connect(lambda: self.execute(algorithm.greedy_bfs))
        self.algorithm_button.addAction(self.action_greedy_bfs)
        self.action_breadth_fs = QAction("Breadth-First Search", self)
        self.action_breadth_fs.triggered.connect(lambda: self.execute(algorithm.breadth_fs))
        self.algorithm_button.addAction(self.action_breadth_fs)
        self.action_depth_fs = QAction("Depth-First Search", self)
        self.action_depth_fs.triggered.connect(lambda: self.execute(algorithm.depth_fs))
        self.algorithm_button.addAction(self.action_depth_fs)
        
        self.heuristic = heuristics.manhattan # Default heuristic

        # Heuristics widget
        self.heuristics_button = self.menubar.addMenu("Heuristic")
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
        
        self.addWidget(self.menubar)

        self.scene = Scene()
        view = View(self.scene)
        self.addWidget(view)

        self.draw_explored = threading.Thread()
        self.draw_result = threading.Thread()
        
    def execute(self, pathfinder: Callable) -> None:
        """Executes given pathfinding algorithm using currently-selected heuristic, drawing tiles to screen"""
        if not self.draw_explored.is_alive() and not self.draw_result.is_alive():
            result, explored = pathfinder(self.scene.start, self.scene.goal, self.scene, self.heuristic)

            self.scene.clear_path()  # Remove currently-drawn path tiles
            if explored != []:
                # Draws explored tiles. Multi-threading used for tiles to draw sequentially
                self.draw_explored = threading.Thread( \
                    target=self.scene.draw_cell_sequence, \
                    args=(explored, CellType.searched, True), \
                    daemon = True
                )
                self.draw_explored.start()

            if result != []:
                # Draws path. Multi-threading used for path to draw sequentially
                # self.draw_explored is fed as 'prev_thread' so that path is drawn after explored tiles
                self.draw_result = threading.Thread( \
                    target=self.scene.draw_cell_sequence, \
                    args=(result, CellType.path, True, self.draw_explored), \
                    daemon = True
                )
                self.draw_result.start()
            
            self.scene.set_cell(self.scene.start, Cell(val = CellType.start))
            self.scene.set_cell(self.scene.goal, Cell(val = CellType.goal))

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