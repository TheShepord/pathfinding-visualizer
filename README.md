# pathfinding-visualizer

[![Github license](https://img.shields.io/github/license/mashape/apistatus.svg?style=flat-square)](https://github.com/TheShepord/pathfinding-visualizer/blob/master/LICENSE)

Grid-based visualizer for popular pathfinding algorithms. Written in Python, with PyQt5 for rendering and UI. As of latest commit, supports A*, Dijkstra, Greedy BFS, Breadth-First Search and Depth-First Search.

## Implementation

### Algorithms
* A* Search
  * Searches nodes based on f(n) = cost_so_far(n) + heuristic(n)
  * Weighted, always optimal given an admissible heurisitic
* Dijkstra's algorithm
  * Searches nodes by progressively moving further from the starting node
  * Weighted, always optimal
* Greedy Best-First Search
  * At every step, explore the best node according to chosen heuristic
  * Weighted, not always optimal
* Breadth-First Search
  * Nodes are progressively explored by adding neighbors to a queue
  * Unweighted, always optimal given an unweighted graph
* Depth-First Search
  * Nodes are progressively explored by pushing neighbors to a stack
  * Unweighted, not always optimal

### Heuristics
* Manhattan: distance between two nodes measured along axes
* Euclidean: straight-line distance between two nodes
* Chebyshev: distance between two nodes measured along axes, but diagonal movement costs 1
* Octile: distance between two nodes measured along axes, diagonal movement costs &radic;2

## Usage

`$ cd <path of project install>`
`$ python3 pathfinding-visualizer.py`

Left-click on empty tiles to begin drawing barriers, left-click on barriers to begin erasing barriers. Left-click on start or goal to drag them around. Select different algorithms and heuristics using the toolbar.

## Upcoming Features

* Support for visualizing and modifying individual cell weights
* Legend for different node colors
* In-app display of each algorithm's characteristics
* Create your own heuristic formulas
* More algorithm options

## About

*License*: This project is released under the [MIT License](https://github.com/TheShepord/pathfinding-visualizer/blob/master/LICENSE).

*References*: