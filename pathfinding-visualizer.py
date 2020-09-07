# Implements the main application window and communicates with pathfinder modules

# Standard library imports
import sys
from typing import Callable
import numpy as np
from time import sleep, time
import threading

# Third party imports
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QMenuBar, QAction, QAbstractScrollArea, QHBoxLayout, QGraphicsTextItem
from PyQt5.QtGui import QResizeEvent

# Local application imports
from pathfinder import Config, Cell, CellType
import pathfinder.heuristics as heuristics
import pathfinder.algorithms as algorithm
from pathfinder.graphics import Scene, View


class Layout(QVBoxLayout):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.heuristic = heuristics.manhattan # Default heuristic
        
        self.init_menubar()
        self.connect_actions()

        # Generates QGraphicsScene and attaches it to QGraphicsView
        self.scene = Scene()
        view = View(self.scene)
        self.addWidget(view)

        # Threads to be used for drawing tiles sequentially
        self.draw_explored = threading.Thread()
        self.draw_result = threading.Thread()
    
    def init_menubar(self) -> None:
        # Adding menubar
        self.menubar = QMenuBar()

        # Algorithms widget
        self.algorithm_button = self.menubar.addMenu("Algorithm")
        self.action_a_star = QAction("A* Search", self)
        self.action_dijkstra= QAction("Dijkstra", self)
        self.action_greedy_bfs = QAction("Greedy BFS", self)
        self.action_breadth_fs = QAction("Breadth-First Search", self)
        self.action_depth_fs = QAction("Depth-First Search", self)

        # Heuristics widget
        self.heuristics_button = self.menubar.addMenu("Heuristic")
        self.action_manhattan = QAction("Manhattan", self)
        self.action_euclidean = QAction("Euclidean", self)
        self.action_chebyshev = QAction("Chebyshev", self)
        self.action_octile = QAction("Octile", self)

        # Options widget
        self.options_button = self.menubar.addMenu("Options")
        self.action_diagonals = QAction("Use diagonals", self)

        self.addWidget(self.menubar)

    def connect_actions(self) -> None:
        # ===Algorithm Widget===
        # Algorithm: A* Search
        self.action_a_star.triggered.connect(lambda: self.execute(algorithm.a_star))
        self.algorithm_button.addAction(self.action_a_star)
        # Algorithm: Dijkstra
        self.action_dijkstra.triggered.connect(lambda: self.execute(algorithm.dijkstra))
        self.algorithm_button.addAction(self.action_dijkstra)
        # Algorithm: Greedy Best-First Search
        self.action_greedy_bfs.triggered.connect(lambda: self.execute(algorithm.greedy_bfs))
        self.algorithm_button.addAction(self.action_greedy_bfs)
        # Algorithm: Breadth-First Search
        self.action_breadth_fs.triggered.connect(lambda: self.execute(algorithm.breadth_fs))
        self.algorithm_button.addAction(self.action_breadth_fs)
        # Algorithm: Depth-First Search
        self.action_depth_fs.triggered.connect(lambda: self.execute(algorithm.depth_fs))
        self.algorithm_button.addAction(self.action_depth_fs)
        
        # ===Heuristics Widget===
        # Heuristic: Manhattan
        self.action_manhattan.setCheckable(True)
        self.action_manhattan.setChecked(True)  # starts checked, default heuristic
        self.action_manhattan.triggered.connect(
            lambda: self.heuristic_button_clicked(heuristics.manhattan, self.action_manhattan)
        )
        self.heuristics_button.addAction(self.action_manhattan)
        # Heuristic: Euclidean
        self.action_euclidean.setCheckable(True)
        self.action_euclidean.triggered.connect(
            lambda: self.heuristic_button_clicked(heuristics.euclidean, self.action_euclidean)
        )
        self.heuristics_button.addAction(self.action_euclidean)
        # Heuristic: Chebyshev
        self.action_chebyshev.setCheckable(True)
        self.action_chebyshev.triggered.connect(
            lambda: self.heuristic_button_clicked(heuristics.chebyshev, self.action_chebyshev)
        )
        self.heuristics_button.addAction(self.action_chebyshev)
        # Heuristic: Octile
        self.action_octile.setCheckable(True)
        self.action_octile.triggered.connect(
            lambda: self.heuristic_button_clicked(heuristics.octile, self.action_octile)
        )
        self.heuristics_button.addAction(self.action_octile)
        
        # ===Options Widget===
        # Option: use diagonals
        self.action_diagonals.setCheckable(True)
        self.action_diagonals.triggered.connect(self.diagonals_button_clicked)
        self.options_button.addAction(self.action_diagonals)
    
    def execute(self, pathfinder: Callable) -> None:
        """Executes given pathfinding algorithm using currently-selected heuristic, drawing tiles"""
        if not self.draw_explored.is_alive() and not self.draw_result.is_alive():
            self.scene.clear_path()  # Remove currently-drawn path tiles
            start = time()
            result, explored = pathfinder(self.scene.start, self.scene.goal, self.scene, self.heuristic)
            end = time()

            # self.text.setPlainText(f"Time taken: {round(end - start, 4)}s\nNodes visited: {len(explored)}\nNodes in path: {len(result)}")
            # self.scene.addItem(self.text)
            log = f"Time taken: {round(end - start, 4)}s\nNodes visited: {len(explored)}\nNodes in path: {len(result)}"
            self.scene.set_text_log(log)
            
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
            self.scene.color_cell(self.scene.start)
            self.scene.color_cell(self.scene.goal)

    def heuristic_button_clicked(self, heuristic: Callable, caller: QAction):
        """When heuristic button clicked, set self.heuristic to heuristic and untick others"""
        self.heuristic = heuristic

        for action in self.heuristics_button.actions():
            if action != caller:
                action.setChecked(False)

    def diagonals_button_clicked(self):
        """When diagonals button clicked, activate/deactivate diagonals"""
        if self.action_diagonals.isChecked():
            Config.DIAGONALS = True
            self.scene.set_diagonal()
        else:
            Config.DIAGONALS = False
            self.scene.set_diagonal()

class Window(QWidget):
    def __init__(self, layout, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setLayout(layout)
        
        self.setSizeIncrement(Config.CELL_LENGTH, Config.CELL_LENGTH)

    def resizeEvent(self, event):
        """QWidget.resizeEvent overload, changes grid dimensions"""
        # gets nearest width/height divisible by cell length
        nearest_w = event.size().width() // Config.CELL_LENGTH
        nearest_h = event.size().height() // Config.CELL_LENGTH - 1
        
        Config.NUM_CELLS_X = nearest_w
        Config.NUM_CELLS_Y = nearest_h
        layout.scene.resize_update()

if __name__ == '__main__':
    # QApplication is necessary for PyQt to run
    app = QApplication(sys.argv)

    layout = Layout()
    w = Window(layout)  # Give Window the specified layout
    w.show()

    # Initialize PyQt event loop
    sys.exit(app.exec_())