# implements the heuristics used by certain pathfinding algorithms

# Standard library imports
from math import sqrt

# Local application imports
from . import Vector2D, Config

def manhattan(node: Vector2D, goal: Vector2D) -> float:
    """Returns manhattan distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    return Config.HEURISTIC_WEIGHT*(dx + dy)

def euclidean(node: Vector2D, goal: Vector2D) -> float:
    """Returns euclidean distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    return Config.HEURISTIC_WEIGHT*sqrt(dx*dx + dy*dy)

def chebyshev(node: Vector2D, goal: Vector2D) -> float:
    """Returns chebyshev distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    diagonal_cost = 1

    return Config.HEURISTIC_WEIGHT*(dx + dy) + (diagonal_cost - 2*Config.HEURISTIC_WEIGHT)*min(dx, dy)

def octile(node: Vector2D, goal: Vector2D) -> float:
    """Returns octile distance between two points, used as a heuristic."""
    dx = abs(node.x - goal.x)
    dy = abs(node.y - goal.y)

    diagonal_cost = sqrt(2)

    return Config.HEURISTIC_WEIGHT*(dx + dy) + (diagonal_cost - 2*Config.HEURISTIC_WEIGHT)*min(dx, dy)
