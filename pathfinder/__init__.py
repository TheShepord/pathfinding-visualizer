# implements various data structures and configuration settings used by pathfinding-visualizer.py

# Standard library imports
import heapq
from typing import NamedTuple

# Third party imports
from PyQt5.QtGui import QColor


class Config:
    """Global settings"""
    CELL_LENGTH = 40  # size of each cell to be displayed on-screen
    NUM_CELLS_X = 20
    NUM_CELLS_Y = 10
    HEURISTIC_WEIGHT = 1  # scalar multiplier for heuristic return value
    DIAGONALS = False  # can pathfinding move diagonally?


class CellType:
    """Color to draw each object"""
    searched = QColor(0, 206, 209)
    path = QColor(255, 255, 0)
    start = QColor(255, 20, 147)
    goal = QColor(0, 250, 154)
    barrier = QColor(47, 79, 79)
    empty = QColor(255, 255, 255)


class Cell(NamedTuple):
    """aaa"""
    val: CellType
    weight: int = 1


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


class Node:
    def __init__(self, val):
        self.val = val
        self.link = None


class Stack:
    """LIFO data structure implemented in linked-list fashion"""
    def __init__(self):
        self.top = None
    
    def put(self, val):
        if self.top:
            new_top = Node(val)
            new_top.link = self.top
            self.top = new_top
        else:
            self.top = Node(val)
        
    def get(self):
        if self.top:
            top_val = self.top.val
            self.top = self.top.link
            return top_val
        else:
            return None
    
    def empty(self):
        if self.top:
            return False
        else:
            return True


class Queue:
    """FIFO data structure implemented in linked-list fashion"""
    def __init__(self):
        self.head = None
        self.tail = None
    
    def put(self, val):
        if self.tail:
            prev_tail = self.tail
            self.tail = Node(val)
            prev_tail.link = self.tail
        else:
            new_node = Node(val)
            self.tail = new_node
            self.head = new_node

    def get(self):
        if self.head:
            head_val = self.head.val
            self.head = self.head.link
            if self.head == None:
                self.tail = None
            return head_val
        else:
            return None
    
    def empty(self):
        if self.head:
            return False
        else:
            return True